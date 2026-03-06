#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <cctype>
#include <unordered_map>
#include <chrono>
#include <sstream>

// Zebra Scanner SDK
#include "zebra-scanner/CsBarcodeTypes.h"
#include "zebra-scanner/CsIEventListenerXml.h"
#include "zebra-scanner/Cslibcorescanner_xml.h"
#include "zebra-scanner/CsUserDefs.h"

using namespace std;


// =============================================================
// 시간 유틸리티
// =============================================================
static inline uint64_t GetNowMs()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}


// =============================================================
// 키보드 입력 감지
// =============================================================
int _kbhit(void) {
    struct termios oldt, newt;
    int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) { ungetc(ch, stdin); return 1; }
    return 0;
}

int _getch(void) {
    struct termios oldattr, newattr;
    int ch;
    tcgetattr(STDIN_FILENO, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
    return ch;
}


// =============================================================
// XML 태그 추출
// =============================================================
static inline std::string ExtractTag(const std::string& xml, const std::string& tag)
{
    std::string open  = "<"  + tag + ">";
    std::string close = "</" + tag + ">";
    size_t s = xml.find(open);
    size_t e = xml.find(close);
    if (s == std::string::npos || e == std::string::npos || e <= s) return "";
    s += open.size();
    return xml.substr(s, e - s);
}


// =============================================================
// "0xNN 0xNN ..." → ASCII 변환 (16진수)
// =============================================================
static inline bool LooksLikeHexBytes(const std::string& s)
{
    return (s.find("0x") != std::string::npos || s.find("0X") != std::string::npos);
}

static inline int HexNibble(char c)
{
    if ('0' <= c && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    if ('a' <= c && c <= 'f') return 10 + (c - 'a');
    return -1;
}

static std::string HexStringToAscii(const std::string& hex)
{
    std::string out;
    out.reserve(hex.size() / 4);
    size_t i = 0;
    while (i < hex.size())
    {
        if (i + 3 < hex.size() &&
            hex[i] == '0' && (hex[i+1] == 'x' || hex[i+1] == 'X'))
        {
            int n1 = HexNibble(hex[i+2]);
            int n2 = HexNibble(hex[i+3]);
            if (n1 >= 0 && n2 >= 0) {
                out.push_back((char)((n1 << 4) | n2));
                i += 4;
                continue;
            }
        }
        i++;
    }
    return out;
}


// =============================================================
//     "NN NN NN ..." raw hex (0x 없는 형식) → ASCII 변환
//     예) "30 30 30 31 35 52 31 35" → "0015R15"
// =============================================================
static inline bool IsHexChar(char c)
{
    return isdigit((unsigned char)c) ||
           ('a' <= tolower((unsigned char)c) && tolower((unsigned char)c) <= 'f');
}

static inline bool LooksLikeRawHexBytes(const std::string& s)
{
    if (s.empty()) return false;
    std::istringstream iss(s);
    std::string token;
    int count = 0;
    while (iss >> token) {
        // 토큰이 정확히 2자리 16진수여야 함
        if (token.size() != 2) return false;
        if (!IsHexChar(token[0]) || !IsHexChar(token[1])) return false;
        count++;
    }
    return (count >= 2);
}

static std::string RawHexToAscii(const std::string& s)
{
    std::string out;
    std::istringstream iss(s);
    std::string token;
    while (iss >> token) {
        if (token.size() == 2 && IsHexChar(token[0]) && IsHexChar(token[1])) {
            int hi = HexNibble(token[0]);
            int lo = HexNibble(token[1]);
            if (hi >= 0 && lo >= 0)
                out.push_back((char)((hi << 4) | lo));
        }
    }
    return out;
}


// =============================================================
//     통합 변환 함수
//     우선순위: 0x-HEX → raw HEX → 원본
// =============================================================
static std::string DataLabelToString(const std::string& raw)
{
    // 1순위: "0xNN 0xNN" 형식
    if (LooksLikeHexBytes(raw)) {
        std::string ascii = HexStringToAscii(raw);
        if (!ascii.empty()) return ascii;
    }

    // 2순위: "NN NN" raw hex 형식 (0x 없는 2자리 hex)
    if (LooksLikeRawHexBytes(raw)) {
        std::string ascii = RawHexToAscii(raw);
        if (!ascii.empty()) return ascii;
    }

    // 3순위: 원본 그대로
    return raw;
}


// =============================================================
// 동작 튜닝 파라미터
// =============================================================
static const uint64_t kRearmCooldownMs  = 10;   // 재트리거 최소 간격 [ms]
static const uint64_t kSameDataBlockMs  = 50;   // 같은 바코드 중복 차단 시간 [ms]
static const uint64_t kKeepAlivePullMs  = 100;  // keep-alive pull 주기 [ms]


// =============================================================
// 전역 상태 (스캐너별)
// =============================================================
static unsigned short           g_numScanners = 0;
static vector<unsigned int>     g_scannerIds;

// 재트리거 / 중복 방지 테이블
static std::unordered_map<int, uint64_t>     g_lastRearmMs;
static std::unordered_map<int, std::string>  g_lastData;
static std::unordered_map<int, uint64_t>     g_lastDataMs;


// =============================================================
// 트리거 제어 함수들
// =============================================================
static void PullTrigger(int scannerId)
{
    std::string inXml =
        "<inArgs><scannerID>" + std::to_string(scannerId) + "</scannerID></inArgs>";
    std::string outXml;
    StatusID status;
    ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, inXml, outXml, &status);
}

static void ReleaseTrigger(int scannerId)
{
    std::string inXml =
        "<inArgs><scannerID>" + std::to_string(scannerId) + "</scannerID></inArgs>";
    std::string outXml;
    StatusID status;
    ::ExecCommand(CMD_DEVICE_RELEASE_TRIGGER, inXml, outXml, &status);
}

// 쿨다운 포함 재트리거 (Release 없이 Pull만 재요청 → 꼬임 방지)
static void ReArmTrigger(int scannerId)
{
    uint64_t now  = GetNowMs();
    uint64_t& last = g_lastRearmMs[scannerId];
    if (now - last < kRearmCooldownMs) return;
    last = now;
    PullTrigger(scannerId);
}

static void ReleaseTriggerAll()
{
    for (auto id : g_scannerIds)
        ReleaseTrigger((int)id);
}


// =============================================================
// EventSink 클래스
// [FIX] Windows: CCmdTarget + DISPATCH_MAP + AfxConnectionAdvise
//       Linux  : IEventListenerXml 가상 함수 오버라이드
// =============================================================
class EventSink : public IEventListenerXml
{
public:
    EventSink()  { cout << "[DEBUG] EventSink Created"   << endl; }
    ~EventSink() { cout << "[DEBUG] EventSink Destroyed" << endl; }

    // ── 바코드 이벤트 ──────────────────────────────────────────
    // [FIX] Windows: OnScanDataEvent(short, BSTR)  → BSTR을 wstring→string 변환
    //       Linux  : OnBarcodeEvent(short, string&) → 이미 UTF-8 std::string
    void OnBarcodeEvent(short eventType, std::string& pscanData) override
    {
        const std::string& xml = pscanData;

        std::string data  = ExtractTag(xml, "datalabel");
        std::string dtype = ExtractTag(xml, "datatype");
        std::string id    = ExtractTag(xml, "scannerID");

        if (data.empty()) {
            std::cout << "[SCAN XML] " << xml << std::endl;
            return;
        }

        // [핵심 변경] 통합 변환 함수 사용
        std::string printable = DataLabelToString(data);

        // scannerID 파싱
        int sid = -1;
        try { sid = id.empty() ? -1 : std::stoi(id); }
        catch (...) { sid = -1; }

        // 같은 바코드 중복 차단
        if (sid >= 0) {
            uint64_t now = GetNowMs();
            if (g_lastData[sid] == printable &&
                (now - g_lastDataMs[sid]) < kSameDataBlockMs)
            {
                return;  // 너무 빠른 중복 → 무시
            }
            g_lastData[sid]   = printable;
            g_lastDataMs[sid] = now;
        }

        // 출력
        std::cout
            << "[BARCODE] scannerID=" << (id.empty()    ? "?" : id)
            << " datatype="           << (dtype.empty() ? "?" : dtype)
            << " data="               << printable
            << std::endl;

        // 다음 스캔을 위해 재트리거 (쿨다운 포함)
        if (sid >= 0) ReArmTrigger(sid);
    }

    // ── 나머지 순수 가상 함수 (미사용, 빈 구현) ────────────────
    void OnImageEvent(short, int, short, char*, int, std::string&) override {}
    void OnVideoEvent(short, int, char*, int, std::string&) override {}
    void OnScannerNotification(short, std::string&) override {}
    void OnBinaryDataEvent(short, int, short, unsigned char*, std::string&) override {}
    void OnPNPEvent(short eventType, std::string ppnpData) override
    {
        if (eventType == SCANNER_ATTACHED)
            cout << "[EVENT] Scanner Attached" << endl;
        else if (eventType == SCANNER_DETACHED)
            cout << "[EVENT] Scanner Detached" << endl;
    }
    void OnCommandResponseEvent(short, std::string&) override {}
    void OnIOEvent(short, unsigned char) override {}
    void OnScanRMDEvent(short, std::string&) override {}
    void OnDisconnect() override { cout << "[EVENT] Scanner Disconnected" << endl; }
};


// =============================================================
// 전역 EventSink
// =============================================================
static EventSink* g_sink = nullptr;


// =============================================================
// CoreScanner 제어 함수들
// =============================================================

// 1. 스캐너 초기화
static bool OpenScanner()
{
    g_sink = new EventSink();
    StatusID status;
    ::Open(g_sink, SCANNER_TYPE_ALL, &status);

    if (status == STATUS_OK) {
        cout << "[OK] Scanner Opened & Event Sink Connected" << endl;
        return true;
    }
    cout << "[ERROR] Open() failed. Status=" << status << endl;
    return false;
}

// 2. 연결된 스캐너 목록 가져오기
static bool GetScanners()
{
    StatusID status;
    string outXml;
    ::GetScanners(&g_numScanners, &g_scannerIds, outXml, &status);

    if (status == STATUS_OK && g_numScanners > 0) {
        cout << "[OK] GetScanners count=" << g_numScanners << " IDs: ";
        for (int i = 0; i < g_numScanners; ++i) {
            cout << g_scannerIds[i];
            if (i != g_numScanners - 1) cout << ", ";
        }
        cout << endl;
        return true;
    }
    cout << "[ERROR] GetScanners failed. Status=" << status
         << " num=" << g_numScanners << endl;
    return false;
}

// 3. 바코드 이벤트만 구독
static bool RegisterBarcodeEvent()
{
    string inXml =
        "<inArgs><cmdArgs>"
        "<arg-int>1</arg-int>"
        "<arg-int>" + to_string(SUBSCRIBE_BARCODE) + "</arg-int>"
        "</cmdArgs></inArgs>";

    string outXml;
    StatusID status;
    ::ExecCommand(CMD_REGISTER_FOR_EVENTS, inXml, outXml, &status);

    if (status == STATUS_OK) {
        cout << "[OK] REGISTER_FOR_EVENTS (BARCODE)" << endl;
        return true;
    }
    cout << "[ERROR] REGISTER_FOR_EVENTS failed. Status=" << status << endl;
    return false;
}

// 4. 스캔 활성화
static bool EnableAll()
{
    if (g_numScanners == 0) return false;

    for (auto id : g_scannerIds) {
        string inXml =
            "<inArgs><scannerID>" + to_string(id) + "</scannerID></inArgs>";
        string outXml;
        StatusID status;
        ::ExecCommand(CMD_DEVICE_SCAN_ENABLE, inXml, outXml, &status);

        if (status != STATUS_OK) {
            cout << "[ERROR] SCAN_ENABLE failed id=" << id
                 << " status=" << status << endl;
            return false;
        }
    }
    cout << "[OK] All scanners enabled" << endl;
    return true;
}

// 5. 최초 1회 트리거 ON
static bool PullTriggerAllOnce()
{
    if (g_numScanners == 0) return false;

    for (auto id : g_scannerIds) {
        string inXml =
            "<inArgs><scannerID>" + to_string(id) + "</scannerID></inArgs>";
        string outXml;
        StatusID status;
        ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, inXml, outXml, &status);

        if (status == STATUS_OK)
            cout << "[OK] PULL_TRIGGER ON (id=" << id << ")" << endl;
        else
            cout << "[ERROR] PULL_TRIGGER failed id=" << id
                 << " status=" << status << endl;
    }
    return true;
}

// 6. 종료 처리
static void Cleanup()
{
    cout << "[INFO] Cleaning up..." << endl;

    ReleaseTriggerAll();

    StatusID status;
    ::Close(0, &status);

    if (g_sink) {
        delete g_sink;
        g_sink = nullptr;
    }
}


// =============================================================
// MAIN 함수
// =============================================================
int main()
{
    cout << "=== Barcode Scanner (Linux) - press 'q' to quit ===\n";

    if (!OpenScanner())            return -1;
    if (!GetScanners())          { Cleanup(); return -1; }
    if (!EnableAll())            { Cleanup(); return -1; }
    if (!RegisterBarcodeEvent()) { Cleanup(); return -1; }

    // 최초 1회 트리거 ON
    PullTriggerAllOnce();

    // Keep-Alive 타이머
    uint64_t lastKeepAlive = GetNowMs();

    while (true)
    {
        // Keep-Alive: 주기적으로 PULL_TRIGGER 유지 (스캔 모드 지속)
        uint64_t now = GetNowMs();
        if (now - lastKeepAlive >= kKeepAlivePullMs)
        {
            lastKeepAlive = now;
            for (auto id : g_scannerIds)
                PullTrigger((int)id);
        }

        if (_kbhit()) {
            int c = _getch();
            if (c == 'q' || c == 'Q') break;
        }

        usleep(1000);
    }

    Cleanup();
    cout << "[INFO] Program exit." << endl;
    return 0;
}

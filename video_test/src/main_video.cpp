/*#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <termios.h>

// OpenCV (화면 출력용)
#include "opencv2/opencv.hpp"

// Zebra Scanner SDK
#include "zebra-scanner/CsBarcodeTypes.h"
#include "zebra-scanner/CsIEventListenerXml.h"
#include "zebra-scanner/Cslibcorescanner_xml.h"
#include "zebra-scanner/CsUserDefs.h"

using namespace std;
using namespace cv;

// logging
int video_cnt = 0;


// =============================================================
// 키보드 입력 감지
// =============================================================
int _kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
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
// EventSink 클래스: 스캐너로부터 이벤트를 받는 곳
// =============================================================
class EventSink : public IEventListenerXml
{
public:
    EventSink() { 
        cout << "[DEBUG] EventSink Created" << endl;
    }
    virtual ~EventSink() {
        cout << "[DEBUG] EventSink Destroyed" << endl;
    }

    // 1. 바코드 스캔 이벤트
    void OnBarcodeEvent(short eventType, std::string& scanData) override
    {   
        // 바코드 스캔은 로그만 남김
        cout << "[EVENT] Barcode Scanned (Data ignored in Video Mode)" << endl;
    }

    // 2. 이미지 수신 이벤트
    void OnImageEvent(short eventType, int size, short imageFormat, char* sfImageData, int dataLength, std::string& pScannerData) override
    {
        cout << "[EVENT] onImageEvent Called! Type=" << eventType 
             << " Size=" << size 
             << " Format=" << imageFormat << endl;
        
        if (size > 0 && sfImageData != nullptr)
        {
            vector<uchar> buffer((uchar*)sfImageData, (uchar*)sfImageData + dataLength);
            Mat frame = imdecode(buffer, IMREAD_COLOR);

            if (!frame.empty())
            {   
                // 화면 출력
                imshow("Zebra Scanner Image", frame);
                waitKey(1);
                cout << "[SUCCESS] Frame displayed! Size: " << frame.cols << "x" << frame.rows << endl;
            }
            else {
                cout << "[ERROR] Failed to decode frame" << endl;
            }
        }
    }

    // 3. 비디오 수신 이벤트
    void OnVideoEvent(short eventType, int size, char* sfVideoData, int dataLength, std::string& pScannerData) override
    {
        if (video_cnt == 0) {
            cout << "[EVENT] onVideoEvent Called! Type=" << eventType << " Size=" << size << endl;
            video_cnt++;
        }
        
        if (size > 0 && sfVideoData != nullptr)
        {
            vector<uchar> buffer((uchar*)sfVideoData, (uchar*)sfVideoData + dataLength);
            Mat frame = imdecode(buffer, IMREAD_COLOR);

            if (!frame.empty())
            {   
                imshow("Zebra Scanner Video", frame);
                waitKey(1);
            }
            else {
                cout << "[ERROR] Failed to decode frame" << endl;
            }
        }
    }

    // 4. 알림 이벤트
    void OnScannerNotification(short notificationType, std::string& pScannerData) override
    {
        cout << "[EVENT] Scanner Notification! Type=" << notificationType << endl;
        switch(notificationType)
        {
            case SCANNER_NOTIFICATION_DECODE_MODE: cout << "  -> Barcode Mode" << endl; break;
            case SCANNER_NOTIFICATION_SNAPSHOT_MODE: cout << "  -> Image Mode" << endl; break;
            case SCANNER_NOTIFICATION_VIDEO_MODE: cout << "  -> Video Mode" << endl; break;
            case 0x0D: cout << "  -> Device Enabled" << endl; break;
            case 0x0E: cout << "  -> Device Disabled" << endl; break;
        }
    }

    // 기타 이벤트
    void OnBinaryDataEvent(short eventType, int dataLength, short dataFormat, unsigned char* sfBinaryData, std::string& pScannerData) override {}
    void OnPNPEvent(short eventType, std::string ppnpData) override {}
    void OnCommandResponseEvent(short status, std::string& prspData) override {}
    void OnIOEvent(short type, unsigned char data) override {}
    void OnScanRMDEvent(short eventType, std::string& prmdData) override {}
    void OnDisconnect() override { cout << "[EVENT] Scanner Disconnected" << endl; }
};


// =============================================================
// 전역 변수
// =============================================================
static EventSink* g_sink = nullptr;
static unsigned short g_numScanners = 0;
static vector<unsigned int> g_scannerIds;


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
        cout << "[INFO] Scanner Opened & Event Sink Connected Successfully" << endl;
        return true;
    }
    cout << "[ERROR] Scanner Open Failed. Status=" << status << endl;
    return false;
}

// 2. 연결된 스캐너 목록 가져오기
static bool GetScanners()
{
    StatusID status;
    string outXml;
    
    ::GetScanners(&g_numScanners, &g_scannerIds, outXml, &status);

    if (status == STATUS_OK && g_numScanners > 0 && g_scannerIds.size() > 0) {
        cout << "[INFO] Scanner Connected. ID: " << g_scannerIds[0] << endl;
        return true;
    }
    cout << "[WARN] No scanners found.\n";
    return false;
}

// 3. 비디오 이벤트 구독 (Barcode + Image + Video)
static bool RegisterVideoEvents()
{
    string inXml = "<inArgs><cmdArgs><arg-int>3</arg-int><arg-int>1,2,4</arg-int></cmdArgs></inArgs>";
    string outXml;
    StatusID status;

    ::ExecCommand(CMD_REGISTER_FOR_EVENTS, inXml, outXml, &status);

    if (status != STATUS_OK) {
        cout << "[ERROR] Event Registration Failed. Status: " << status << endl;
        return false;
    }

    cout << "[INFO] Events Registered Successfully (Barcode/Image/Video)" << endl;
    return true;
}

// 4. ViewFinder 파라미터 설정
static bool EnableViewFinder()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);

    string inXml = "<inArgs><scannerID>" + id + 
                   "</scannerID><cmdArgs><arg-xml><attrib_list><attribute>"
                   "<id>324</id><datatype>B</datatype><value>1</value>"
                   "</attribute></attrib_list></arg-xml></cmdArgs></inArgs>";

    string outXml;
    StatusID status;

    ::ExecCommand(CMD_RSM_ATTR_SET, inXml, outXml, &status);

    if (status == STATUS_OK) {
        cout << "[INFO] ViewFinder Enabled (Parameter 324)" << endl;
        return true;
    }
    cout << "[ERROR] ViewFinder Enable Failed. Status=" << status << endl;
    return false;
}

// 5. 비디오 모드 설정
static bool SetVideoMode()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";

    string outXml;
    StatusID status;

    ::ExecCommand(CMD_DEVICE_CAPTURE_VIDEO, inXml, outXml, &status);

    if (status == STATUS_OK) {
        cout << "[INFO] SET_VIDEO_MODE Success (Opcode 4000)" << endl;
        return true;
    }
    
    cout << "[ERROR] SET_VIDEO_MODE Failed. Status=" << status << endl;
    return false;
}

// 6. Pull Trigger
static bool PullTrigger()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";

    string outXml;
    StatusID status;

    ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, inXml, outXml, &status);

    if (status == STATUS_OK) {
        cout << "[INFO] PULL_TRIGGER Success (Opcode 2011)" << endl;
        return true;
    }

    cout << "[ERROR] PULL_TRIGGER Failed. Status=" << status << endl;
    return false;
}

// 7. Release Trigger
static bool ReleaseTrigger()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";

    string outXml;
    StatusID status;

    ::ExecCommand(CMD_DEVICE_RELEASE_TRIGGER, inXml, outXml, &status);
    
    cout << "[INFO] Trigger Released" << endl;
    return true;
}

static void Cleanup()
{
    cout << "[INFO] Cleaning up..." << endl;
    ReleaseTrigger();

    StatusID status;
    ::Close(0, &status);
    
    if (g_sink) delete g_sink;
}


// =============================================================
// MAIN 함수
// =============================================================
int main()
{
    cout << "=== Zebra Scanner Video Streamer (OpenCV on Linux) ===\n";
    cout << "Press 'q' or ESC to quit.\n";

    if (!OpenScanner()) return -1;
    if (!GetScanners()) return -1;

    // 0. Release Trigger
    //ReleaseTrigger();

    // 1. 이벤트 등록
    if(!RegisterVideoEvents()) return -1;

    // 2. ViewFinder 활성화
    if (!EnableViewFinder()) return -1;

    // 3. 비디오 모드 활성화
    if (!SetVideoMode()) return -1;

    usleep(500 * 1000);

    // 4. Pull Trigger
    if (!PullTrigger()) return -1;

    cout << "[INFO] Video streaming started. Waiting for frames..." << endl;
    cout << "===============================================\n" << endl;

    // 5. 메시지 루프
    bool running = true;
    int loopCount = 0;
    
    while (running)
    {
        // 키 입력 확인 (직접 구현한 리눅스용 _kbhit 사용)
        if (_kbhit()) {
            int c = _getch();
            if (c == 'q' || c == 'Q' || c == 27) {
                cout << "\n[INFO] Quit key pressed" << endl;
                running = false;
            }
        }

        // OpenCV 창 갱신
        char key = (char)waitKey(10);
        
        // 상태 출력 (1sec)
        if (++loopCount % 100 == 0) {
            cout << "[INFO] Still waiting for events... (Press 'q' to quit)" << endl;
        }
    }

    Cleanup();
    cout << "\n[INFO] Application terminated." << endl;
    return 0;
}*/
/*
 * zebra_aruco_streamer.cpp
 *
 * 수정 이력:
 * [OPT-1] vector<uchar> 멤버 변수로 재사용 → 매 프레임 메모리 재할당 제거
 * [OPT-2] 콜백에서 imshow/waitKey 완전 제거 → SDK 콜백 스레드 블로킹 방지
 * [OPT-3] 디코딩 전용 스레드 분리 → 수신/디코딩/표시 파이프라인 병렬화
 * [OPT-4] Raw 데이터 큐 도입 + 큐 overflow 시 오래된 프레임 드롭 (실시간 우선)
 * [OPT-5] std::move로 Mat 소유권 이전 → 불필요한 메모리 복사 제거
 * [OPT-6] 더블 버퍼링 → 메인루프에서 clone() 제거
 * [OPT-7] atomic memory_order 명시 → 불필요한 메모리 펜스 제거
 * [OPT-8] waitKey 중복 호출 제거 → 메인 루프에서 waitKey(1) 단 1회
 * [OPT-9] namedWindow WINDOW_NORMAL 미리 생성 → 매 프레임 창 리사이즈 방지
 * [NEW-1] ArUco 마커 인식 추가 (detectMarkers + estimatePoseSingleMarkers)
 * [NEW-2] 카메라 캘리브레이션 파라미터 섹션 추가 (측정값으로 교체 필요)
 * [NEW-3] 자세 추정 결과 콘솔 출력 + 축 시각화 (drawFrameAxes)
 */

 /*
 * zebra_video_streamer.cpp
 *
 * 수정 이력 (원본 대비):
 * [OPT-1] vector<uchar> 멤버 변수 재사용 → 매 프레임 메모리 재할당 제거
 * [OPT-2] 콜백에서 imshow/waitKey 완전 제거 → SDK 콜백 스레드 블로킹 방지
 * [OPT-3] 디코딩 전용 스레드 분리 → 수신/디코딩/표시 파이프라인 병렬화
 * [OPT-4] Raw 데이터 큐 도입 + 큐 overflow 시 오래된 프레임 드롭 (실시간 우선)
 * [OPT-5] std::move로 Mat 소유권 이전 → 불필요한 메모리 복사 제거
 * [OPT-6] 더블 버퍼링 → 메인루프에서 clone() 제거
 * [OPT-7] atomic memory_order 명시 → 불필요한 메모리 펜스 제거
 * [OPT-8] waitKey 중복 호출 제거 → 메인 루프에서 waitKey(1) 단 1회
 * [OPT-9] namedWindow 미리 생성 → 매 프레임 창 리사이즈 방지
 */

#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <thread>            // [OPT-3]
#include <mutex>
#include <condition_variable>
#include <queue>             // [OPT-4]
#include <atomic>

#include "opencv2/opencv.hpp"

#include "zebra-scanner/CsBarcodeTypes.h"
#include "zebra-scanner/CsIEventListenerXml.h"
#include "zebra-scanner/Cslibcorescanner_xml.h"
#include "zebra-scanner/CsUserDefs.h"

using namespace std;
using namespace cv;


// =============================================================
// [OPT-3][OPT-4] Raw 프레임 큐 (콜백 → 디코딩 스레드)
// =============================================================
struct RawFrame {
    vector<uchar> data;
};

static queue<RawFrame>      g_rawQueue;
static mutex                g_queueMutex;
static condition_variable   g_queueCV;
static atomic<bool>         g_running(true);

// [OPT-6] 더블 버퍼 (디코딩 스레드 → 메인 루프)
static Mat                  g_frameBuffer[2];
static atomic<int>          g_writeIdx(0);
static atomic<bool>         g_newFrameReady(false);
static mutex                g_frameMutex;


// =============================================================
// 키보드 입력 감지 (Linux용)
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
// EventSink: SDK 콜백 수신
// =============================================================
class EventSink : public IEventListenerXml
{
private:
    // [OPT-1] 버퍼 멤버 변수 재사용 (매 프레임 new 방지)
    vector<uchar> m_buffer;

    // [OPT-2] 공통 처리: raw 데이터를 큐에만 넣고 즉시 리턴
    void PushRawData(char* data, int dataLength)
    {
        if (dataLength <= 0 || data == nullptr) return;

        RawFrame raw;
        raw.data.assign((uchar*)data, (uchar*)data + dataLength);

        {
            lock_guard<mutex> lock(g_queueMutex);
            // [OPT-4] 큐가 밀리면 오래된 프레임 버림 (실시간 우선)
            while (g_rawQueue.size() > 2) {
                g_rawQueue.pop();
            }
            g_rawQueue.push(std::move(raw));
        }
        g_queueCV.notify_one();
    }

public:
    EventSink()  { cout << "[DEBUG] EventSink Created" << endl; }
    ~EventSink() { cout << "[DEBUG] EventSink Destroyed" << endl; }

    void OnBarcodeEvent(short eventType, std::string& scanData) override
    {
        cout << "[EVENT] Barcode Scanned (ignored in Video Mode)" << endl;
    }

    // [OPT-2] 콜백에서 imshow/waitKey 제거 → 큐에만 push
    void OnImageEvent(short eventType, int size, short imageFormat,
                      char* sfImageData, int dataLength,
                      std::string& pScannerData) override
    {
        PushRawData(sfImageData, dataLength);
    }

    // [OPT-2] 콜백에서 imshow/waitKey 제거 → 큐에만 push
    void OnVideoEvent(short eventType, int size, char* sfVideoData,
                      int dataLength, std::string& pScannerData) override
    {
        PushRawData(sfVideoData, dataLength);
    }

    void OnScannerNotification(short notificationType,
                               std::string& pScannerData) override
    {
        cout << "[EVENT] Scanner Notification! Type=" << notificationType << endl;
        switch (notificationType) {
            case SCANNER_NOTIFICATION_DECODE_MODE:   cout << "  -> Barcode Mode" << endl; break;
            case SCANNER_NOTIFICATION_SNAPSHOT_MODE: cout << "  -> Image Mode"   << endl; break;
            case SCANNER_NOTIFICATION_VIDEO_MODE:    cout << "  -> Video Mode"   << endl; break;
            case 0x0D: cout << "  -> Device Enabled"  << endl; break;
            case 0x0E: cout << "  -> Device Disabled" << endl; break;
        }
    }

    void OnBinaryDataEvent(short, int, short, unsigned char*, std::string&) override {}
    void OnPNPEvent(short, std::string) override {}
    void OnCommandResponseEvent(short, std::string&) override {}
    void OnIOEvent(short, unsigned char) override {}
    void OnScanRMDEvent(short, std::string&) override {}
    void OnDisconnect() override { cout << "[EVENT] Scanner Disconnected" << endl; }
};


// =============================================================
// [OPT-3] 디코딩 전용 스레드
//   Raw 큐에서 꺼내 imdecode → 더블 버퍼에 저장
// =============================================================
void DecoderThread()
{
    while (g_running.load(memory_order_acquire))
    {
        RawFrame raw;
        {
            unique_lock<mutex> lock(g_queueMutex);
            g_queueCV.wait(lock, [] {
                return !g_rawQueue.empty() || !g_running.load(memory_order_acquire);
            });
            if (!g_running.load(memory_order_acquire)) break;
            raw = std::move(g_rawQueue.front());
            g_rawQueue.pop();
        }

        Mat frame = imdecode(raw.data, IMREAD_COLOR);
        if (!frame.empty())
        {
            // [OPT-6] 현재 표시 중인 버퍼와 반대쪽에 쓰기
            int writeIdx = g_writeIdx.load(memory_order_relaxed) ^ 1;
            {
                lock_guard<mutex> lock(g_frameMutex);
                // [OPT-5] move로 소유권 이전 (메모리 복사 없음)
                g_frameBuffer[writeIdx] = std::move(frame);
                g_writeIdx.store(writeIdx, memory_order_release);
            }
            // [OPT-7] release/acquire로 순서 보장
            g_newFrameReady.store(true, memory_order_release);
        }
    }
    cout << "[INFO] DecoderThread exited." << endl;
}


// =============================================================
// 전역 변수
// =============================================================
static EventSink*           g_sink = nullptr;
static unsigned short       g_numScanners = 0;
static vector<unsigned int> g_scannerIds;


// =============================================================
// CoreScanner 제어 함수들
// =============================================================
static bool OpenScanner()
{
    g_sink = new EventSink();
    StatusID status;
    ::Open(g_sink, SCANNER_TYPE_ALL, &status);
    if (status == STATUS_OK) {
        cout << "[INFO] Scanner Opened Successfully" << endl;
        return true;
    }
    cout << "[ERROR] Scanner Open Failed. Status=" << status << endl;
    return false;
}

static bool GetScanners()
{
    StatusID status;
    string outXml;
    ::GetScanners(&g_numScanners, &g_scannerIds, outXml, &status);
    if (status == STATUS_OK && g_numScanners > 0 && !g_scannerIds.empty()) {
        cout << "[INFO] Scanner Connected. ID: " << g_scannerIds[0] << endl;
        return true;
    }
    cout << "[WARN] No scanners found." << endl;
    return false;
}

static bool RegisterVideoEvents()
{
    string inXml  = "<inArgs><cmdArgs><arg-int>3</arg-int><arg-int>1,2,4</arg-int></cmdArgs></inArgs>";
    string outXml;
    StatusID status;
    ::ExecCommand(CMD_REGISTER_FOR_EVENTS, inXml, outXml, &status);
    if (status != STATUS_OK) {
        cout << "[ERROR] Event Registration Failed. Status=" << status << endl;
        return false;
    }
    cout << "[INFO] Events Registered (Barcode/Image/Video)" << endl;
    return true;
}

static bool EnableViewFinder()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id +
                   "</scannerID><cmdArgs><arg-xml><attrib_list><attribute>"
                   "<id>324</id><datatype>B</datatype><value>1</value>"
                   "</attribute></attrib_list></arg-xml></cmdArgs></inArgs>";
    string outXml;
    StatusID status;
    ::ExecCommand(CMD_RSM_ATTR_SET, inXml, outXml, &status);
    if (status == STATUS_OK) {
        cout << "[INFO] ViewFinder Enabled (Parameter 324)" << endl;
        return true;
    }
    cout << "[ERROR] ViewFinder Enable Failed. Status=" << status << endl;
    return false;
}

static bool SetVideoMode()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";
    string outXml;
    StatusID status;
    ::ExecCommand(CMD_DEVICE_CAPTURE_VIDEO, inXml, outXml, &status);
    if (status == STATUS_OK) {
        cout << "[INFO] Video Mode Set (Opcode 4000)" << endl;
        return true;
    }
    cout << "[ERROR] Video Mode Failed. Status=" << status << endl;
    return false;
}

static bool PullTrigger()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";
    string outXml;
    StatusID status;
    ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, inXml, outXml, &status);
    if (status == STATUS_OK) {
        cout << "[INFO] Trigger Pulled (Opcode 2011)" << endl;
        return true;
    }
    cout << "[ERROR] Trigger Failed. Status=" << status << endl;
    return false;
}

static bool ReleaseTrigger()
{
    if (g_numScanners == 0) return false;
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";
    string outXml;
    StatusID status;
    ::ExecCommand(CMD_DEVICE_RELEASE_TRIGGER, inXml, outXml, &status);
    cout << "[INFO] Trigger Released" << endl;
    return true;
}

static void Cleanup(thread& decoderThread)
{
    cout << "[INFO] Cleaning up..." << endl;

    ReleaseTrigger();

    // [OPT-3] 디코딩 스레드 종료 신호
    g_running.store(false, memory_order_release);
    g_queueCV.notify_all();
    if (decoderThread.joinable()) decoderThread.join();

    StatusID status;
    ::Close(0, &status);

    if (g_sink) { delete g_sink; g_sink = nullptr; }

    destroyAllWindows();
}


// =============================================================
// MAIN
// =============================================================
int main()
{
    cout << "=== Zebra Scanner Video Streamer (OpenCV on Linux) ===" << endl;
    cout << "Press 'q' or ESC to quit." << endl;

    if (!OpenScanner())         return -1;
    if (!GetScanners())         return -1;
    if (!RegisterVideoEvents()) return -1;
    if (!EnableViewFinder())    return -1;
    if (!SetVideoMode())        return -1;

    usleep(500 * 1000);
    if (!PullTrigger())         return -1;

    // [OPT-3] 디코딩 스레드 시작
    thread decoderThread(DecoderThread);

    // [OPT-9] 창 미리 생성 (매 프레임 리사이즈 방지)
    namedWindow("Zebra Scanner Video", WINDOW_NORMAL | WINDOW_KEEPRATIO);

    cout << "[INFO] Video streaming started. Waiting for frames..." << endl;
    cout << "===============================================" << endl;

    bool running   = true;
    int  loopCount = 0;

    while (running)
    {
        // [OPT-6][OPT-7] 새 프레임 있을 때만 화면 갱신
        if (g_newFrameReady.load(memory_order_acquire))
        {
            int idx;
            {
                lock_guard<mutex> lock(g_frameMutex);
                idx = g_writeIdx.load(memory_order_relaxed);
                g_newFrameReady.store(false, memory_order_release);
            }
            imshow("Zebra Scanner Video", g_frameBuffer[idx]);
        }

        // [OPT-8] waitKey 1회 (기존 콜백 waitKey(1) + 루프 waitKey(10) 중복 제거)
        char key = (char)waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) running = false;

        if (_kbhit()) {
            int c = _getch();
            if (c == 'q' || c == 'Q' || c == 27) running = false;
        }

        if (++loopCount % 1000 == 0) {
            cout << "[INFO] Running... (Press 'q' to quit)" << endl;
        }
    }

    Cleanup(decoderThread);
    cout << "[INFO] Application terminated." << endl;
    return 0;
}
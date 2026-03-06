#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <mutex>
#include <atomic>

#include "opencv2/opencv.hpp"
#include "zebra-scanner/CsBarcodeTypes.h"
#include "zebra-scanner/CsIEventListenerXml.h"
#include "zebra-scanner/Cslibcorescanner_xml.h"
#include "zebra-scanner/CsUserDefs.h"

using namespace std;
using namespace cv;

// 프레임 공유 버퍼 (콜백 → 메인)
static Mat g_latestFrame;
static mutex g_frameMutex;
static atomic<bool> g_newFrameReady(false);

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

class EventSink : public IEventListenerXml
{
public:
    // 콜백에서는 디코딩만 하고 프레임 저장 후 즉시 리턴
    void OnVideoEvent(short eventType, int size, char* sfVideoData, int dataLength, std::string& pScannerData) override
    {
        if (size <= 0 || sfVideoData == nullptr) return;

        vector<uchar> buffer((uchar*)sfVideoData, (uchar*)sfVideoData + dataLength);
        Mat frame = imdecode(buffer, IMREAD_COLOR);

        if (!frame.empty()) {
            lock_guard<mutex> lock(g_frameMutex);
            g_latestFrame = frame;  // move 대신 직접 대입 (얕은 복사, 헤더만)
            g_newFrameReady.store(true);
        }
    }

    void OnImageEvent(short eventType, int size, short imageFormat, char* sfImageData, int dataLength, std::string& pScannerData) override
    {
        if (size <= 0 || sfImageData == nullptr) return;

        vector<uchar> buffer((uchar*)sfImageData, (uchar*)sfImageData + dataLength);
        Mat frame = imdecode(buffer, IMREAD_COLOR);

        if (!frame.empty()) {
            lock_guard<mutex> lock(g_frameMutex);
            g_latestFrame = frame;
            g_newFrameReady.store(true);
        }
    }

    void OnBarcodeEvent(short eventType, std::string& scanData) override {}
    void OnScannerNotification(short notificationType, std::string& pScannerData) override {
        cout << "[EVENT] Scanner Notification! Type=" << notificationType << endl;
    }
    void OnBinaryDataEvent(short eventType, int dataLength, short dataFormat, unsigned char* sfBinaryData, std::string& pScannerData) override {}
    void OnPNPEvent(short eventType, std::string ppnpData) override {}
    void OnCommandResponseEvent(short status, std::string& prspData) override {}
    void OnIOEvent(short type, unsigned char data) override {}
    void OnScanRMDEvent(short eventType, std::string& prmdData) override {}
    void OnDisconnect() override { cout << "[EVENT] Disconnected" << endl; }
};

static EventSink* g_sink = nullptr;
static unsigned short g_numScanners = 0;
static vector<unsigned int> g_scannerIds;

static bool OpenScanner() {
    g_sink = new EventSink();
    StatusID status;
    ::Open(g_sink, SCANNER_TYPE_ALL, &status);
    if (status == STATUS_OK) { cout << "[INFO] Scanner Opened" << endl; return true; }
    cout << "[ERROR] Open Failed. Status=" << status << endl;
    return false;
}

static bool GetScanners() {
    StatusID status;
    string outXml;
    ::GetScanners(&g_numScanners, &g_scannerIds, outXml, &status);
    if (status == STATUS_OK && g_numScanners > 0) {
        cout << "[INFO] Scanner ID: " << g_scannerIds[0] << endl;
        return true;
    }
    cout << "[WARN] No scanners found." << endl;
    return false;
}

static bool RegisterVideoEvents() {
    string inXml = "<inArgs><cmdArgs><arg-int>3</arg-int><arg-int>1,2,4</arg-int></cmdArgs></inArgs>";
    string outXml;
    StatusID status;
    ::ExecCommand(CMD_REGISTER_FOR_EVENTS, inXml, outXml, &status);
    if (status != STATUS_OK) { cout << "[ERROR] Event Reg Failed" << endl; return false; }
    cout << "[INFO] Events Registered" << endl;
    return true;
}

static bool EnableViewFinder() {
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id +
                   "</scannerID><cmdArgs><arg-xml><attrib_list><attribute>"
                   "<id>324</id><datatype>B</datatype><value>1</value>"
                   "</attribute></attrib_list></arg-xml></cmdArgs></inArgs>";
    string outXml; StatusID status;
    ::ExecCommand(CMD_RSM_ATTR_SET, inXml, outXml, &status);
    if (status == STATUS_OK) { cout << "[INFO] ViewFinder Enabled" << endl; return true; }
    cout << "[ERROR] ViewFinder Failed. Status=" << status << endl;
    return false;
}

static bool SetVideoMode() {
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";
    string outXml; StatusID status;
    ::ExecCommand(CMD_DEVICE_CAPTURE_VIDEO, inXml, outXml, &status);
    if (status == STATUS_OK) { cout << "[INFO] Video Mode Set" << endl; return true; }
    cout << "[ERROR] Video Mode Failed. Status=" << status << endl;
    return false;
}

static bool PullTrigger() {
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";
    string outXml; StatusID status;
    ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, inXml, outXml, &status);
    if (status == STATUS_OK) { cout << "[INFO] Trigger Pulled" << endl; return true; }
    cout << "[ERROR] Trigger Failed. Status=" << status << endl;
    return false;
}

static bool ReleaseTrigger() {
    string id = to_string(g_scannerIds[0]);
    string inXml = "<inArgs><scannerID>" + id + "</scannerID></inArgs>";
    string outXml; StatusID status;
    ::ExecCommand(CMD_DEVICE_RELEASE_TRIGGER, inXml, outXml, &status);
    cout << "[INFO] Trigger Released" << endl;
    return true;
}

static void Cleanup() {
    ReleaseTrigger();
    StatusID status;
    ::Close(0, &status);
    if (g_sink) delete g_sink;
}

int main()
{
    cout << "=== Zebra Scanner Video Streamer ===" << endl;
    cout << "Press 'q' or ESC to quit." << endl;

    if (!OpenScanner()) return -1;
    if (!GetScanners()) return -1;
    if (!RegisterVideoEvents()) return -1;
    if (!EnableViewFinder()) return -1;
    if (!SetVideoMode()) return -1;

    usleep(500 * 1000);
    if (!PullTrigger()) return -1;

    cout << "[INFO] Streaming started..." << endl;

    namedWindow("Zebra Scanner Video", WINDOW_AUTOSIZE);

    bool running = true;
    while (running)
    {
        // 새 프레임이 있을 때만 화면 갱신
        if (g_newFrameReady.load())
        {
            Mat displayFrame;
            {
                lock_guard<mutex> lock(g_frameMutex);
                displayFrame = g_latestFrame.clone();
                g_newFrameReady.store(false);
            }
            imshow("Zebra Scanner Video", displayFrame);
        }

        // waitKey는 메인 루프에서 한 번만, 짧게
        char key = (char)waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {
            running = false;
        }

        // 키보드 fallback
        if (_kbhit()) {
            int c = _getch();
            if (c == 'q' || c == 'Q' || c == 27) running = false;
        }
    }

    Cleanup();
    destroyAllWindows();
    cout << "[INFO] Done." << endl;
    return 0;
}
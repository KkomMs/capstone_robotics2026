#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <mutex>

// OpenCV
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

// mutex
std::mutex g_frameMutex;
Mat g_currentFrame;


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
// Checkerboard 파라미터
// =============================================================
Size boardSize(10, 7);
float squareSize = 0.025f;   // [m]


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
                std::lock_guard<std::mutex> lock(g_frameMutex);
                g_currentFrame = frame;
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
    cout << "=== Zebra Scanner Video - Camera Calibration (Linux) ===\n";

    // ======== 스캐너 비디오 받아오기 ========
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
    // =====================================

    // 캘리브레이션 데이터 준비
    vector<Point3f> objPoints;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            objPoints.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;
    int captureCount = 0;
    Size imageSize;

    cout << "\n[INFO] Capture the checkerboard by clicking on the screen and pressing 'c'." << endl;
    cout << "[INFO] After capturing, press 'ESC' to start calibration." << endl;
    cout << "===============================================\n" << endl;

    bool running = true;

    while (running)
    {
        // 콜백에서 받은 최신 프레임을 안전하게 복사
        Mat frame;
        {
            std::lock_guard<std::mutex> lock(g_frameMutex);
            if (!g_currentFrame.empty()) {
                g_currentFrame.copyTo(frame);
            }
        }

        if (!frame.empty()) {
            imageSize = frame.size();
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            vector<Point2f> corners;

            // 체커보드 찾기
            bool found = findChessboardCorners(
                frame, boardSize, corners,
                CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK
            );

            Mat displayFrame = frame.clone();
            if (found) {
                // 서브픽셀 정밀화
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
                drawChessboardCorners(displayFrame, boardSize, corners, found);
            }
            putText(displayFrame, "Captured: " + to_string(captureCount),
                    Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
            putText(displayFrame, found ? "Chessboard: FOUND" : "Chessboard: NOT FOUND",
                    Point(10, 65), FONT_HERSHEY_SIMPLEX, 0.7,
                    found ? Scalar(0, 255, 0) : Scalar(0, 0, 255), 2);
            imshow("Zebra Calibration", displayFrame);
        }

        char key = (char)waitKey(10);

        if (key == 27) { // ESC
            running = false;
        } else if (key == 'c' || key == 'C') {
            if (!frame.empty()) {
                Mat gray;
                cvtColor(frame, gray, COLOR_BGR2GRAY);
                vector<Point2f> corners;
                bool found = findChessboardCorners(
                    frame, boardSize, corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK
                );
                if (found) {
                    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                        TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
                    imagePoints.push_back(corners);
                    objectPoints.push_back(objPoints);
                    captureCount++;
                    cout << "[Capture Success] Total: " << captureCount << " saved." << endl;
                } else {
                    cout << "[Warning] The checkerboard was not fully recognized." << endl;
                }
            }
        } else if (key == 'q' || key == 'Q') {
            running = false;
        }

        if (_kbhit()) {
            int c = _getch();
            if (c == 27 || c == 'q' || c == 'Q') {
                running = false;
            }
        }
    }

    // 스캐너 연결 종료
    Cleanup();
    destroyAllWindows();

    // 캘리브레이션 연산 수행
    if (captureCount >= 5) {
        cout << "\n[INFO] Calibration operation in progress..." << endl;
        Mat cameraMatrix, distCoeffs;
        vector<Mat> rvecs, tvecs;

        double rms = calibrateCamera(objectPoints, imagePoints, imageSize,
                                     cameraMatrix, distCoeffs, rvecs, tvecs);

        cout << "\n=== Calibration complete ===" << endl;
        cout << "RMS Error: " << rms << endl;
        cout << "[Camera Matrix]\n" << cameraMatrix << endl;
        cout << "[Distortion Coefficients]\n" << distCoeffs << endl;

        FileStorage fs("../data/camera_calibration.xml", FileStorage::WRITE);
        if (fs.isOpened()) {
            fs << "cameraMatrix" << cameraMatrix;
            fs << "distCoeffs"   << distCoeffs;
            fs.release();
            cout << "\n[Success] '../data/camera_calibration.xml' saving complete." << endl;
        } else {
            cout << "\n[ERROR] Failed to open file for writing." << endl;
        }
    } else {
        cout << "\n[ERROR] Calibration canceled: insufficient captures (need >= 5, got "
             << captureCount << ")." << endl;
    }

    cout << "[INFO] Program exit." << endl;
    return 0;
}
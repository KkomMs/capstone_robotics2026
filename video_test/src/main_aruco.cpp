#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <cmath>

// OpenCV (4.5.4)
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

// Zebra Scanner SDK
#include "zebra-scanner/CsBarcodeTypes.h"
#include "zebra-scanner/CsIEventListenerXml.h"
#include "zebra-scanner/Cslibcorescanner_xml.h"
#include "zebra-scanner/CsUserDefs.h"

using namespace std;
using namespace cv;

// Camera Calibration
cv::Mat g_cameraMatrix;
cv::Mat g_distCoeffs;

// logging
int video_cnt = 0;


// =============================================================
// ArUco 마커 설정
// =============================================================
cv::Ptr<cv::aruco::Dictionary> g_dictionary;
cv::Ptr<cv::aruco::DetectorParameters> g_detectorParams;

const float MARKER_SIZE = 0.03f;     // [m]

std::vector<cv::Point3f> objPoints = {
        cv::Point3f(-MARKER_SIZE / 2.0f,  MARKER_SIZE / 2.0f, 0),
        cv::Point3f( MARKER_SIZE / 2.0f,  MARKER_SIZE / 2.0f, 0),
        cv::Point3f( MARKER_SIZE / 2.0f, -MARKER_SIZE / 2.0f, 0),
        cv::Point3f(-MARKER_SIZE / 2.0f, -MARKER_SIZE / 2.0f, 0)
    };


// =============================================================
// 회전 벡터 -> 오일러 각도(Roll, Pitch, Yaw)로 변환
// =============================================================
cv::Vec3d getEulerAngles(const cv::Mat& rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R); // 회전 벡터를 3x3 회전 행렬로 변환

    double sy = std::sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
    bool singular = sy < 1e-6; // 특이점 확인

    double x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2,1), R.at<double>(2,2)); // Roll
        y = std::atan2(-R.at<double>(2,0), sy);               // Pitch
        z = std::atan2(R.at<double>(1,0), R.at<double>(0,0)); // Yaw
    } else {
        x = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = std::atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    // 라디안 값을 degree로 변환하여 반환
    return cv::Vec3d(x, y, z) * (180.0 / CV_PI);
}

// =============================================================
// ArUco 마커 검출 및 화면에 표시
// =============================================================void ArucoDetect(Mat& frame)
/*void ArucoDetect(Mat& frame)
{
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;

    // 마커 감지 및 포즈 추정
    cv::aruco::detectMarkers(frame, g_dictionary, corners, ids, g_detectorParams, rejected);

    // 감지된 마커 표시
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        for (size_t i = 0; i < ids.size(); ++i) {
            cv::Mat rvec, tvec;

            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, g_cameraMatrix, g_distCoeffs, rvec, tvec);
            cv::aruco::drawAxis(frame, g_cameraMatrix, g_distCoeffs, rvec, tvec, MARKER_SIZE / 1.5f);
            
            // 위치 좌표 [m]
            double tx = tvec.at<double>(0,0);
            double ty = tvec.at<double>(1,0);
            double tz = tvec.at<double>(2,0);

            //회전 각도
            cv::Vec3d eulerAngles = getEulerAngles(rvec);

            // 화면에 텍스트 출력
            std::string idText = "ID: " + std::to_string(ids[i]);
            std::string posText = cv::format("XYZ(m): [%.2f, %.2f, %.2f]", tx, ty, tz);
            std::string rotText = cv::format("Rot(deg): [%.1f, %.1f, %.1f]", eulerAngles[0], eulerAngles[1], eulerAngles[2]);

            cv::Point textPos(cvRound(corners[i][0].x), cvRound(corners[i][0].y - 10));
            cv::putText(frame, idText, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, posText, cv::Point(textPos.x, textPos.y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 2);
            cv::putText(frame, rotText, cv::Point(textPos.x, textPos.y - 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 2);
        }
    }
    else {
        putText(frame, "No ArUco markers detected", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    }
}*/
void ArucoDetect(cv::Mat& frame)
{
    if (g_cameraMatrix.empty() || g_distCoeffs.empty() || !g_dictionary || !g_detectorParams) {
        return;
    }

    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;

    cv::aruco::detectMarkers(frame, g_dictionary, corners, ids, g_detectorParams, rejected);

    if (ids.empty()) {
        putText(frame, "No ArUco markers detected", Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
        return;
    }

    cv::aruco::drawDetectedMarkers(frame, corners, ids);

    for (size_t i = 0; i < ids.size(); ++i) {
        try {
            cv::Mat rvec, tvec;
            vector<vector<cv::Point2f>> singleCorner = { corners[i] };

            cv::aruco::estimatePoseSingleMarkers(
                singleCorner, MARKER_SIZE, g_cameraMatrix, g_distCoeffs, rvec, tvec
            );

            cv::Mat rvecMat = (cv::Mat_<double>(3,1) << rvec.at<double>(0,0), rvec.at<double>(0,1), rvec.at<double>(0,2));
            cv::Mat tvecMat = (cv::Mat_<double>(3,1) << tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));

            cv::aruco::drawAxis(frame, g_cameraMatrix, g_distCoeffs, rvecMat, tvecMat, MARKER_SIZE / 1.5f);

            double tx = tvecMat.at<double>(0,0);
            double ty = tvecMat.at<double>(1,0);
            double tz = tvecMat.at<double>(2,0);

            cv::Vec3d eulerAngles = getEulerAngles(rvecMat);

            cv::Point textPos(cvRound(corners[i][0].x), cvRound(corners[i][0].y - 10));
            cv::putText(frame, "ID: " + std::to_string(ids[i]), textPos,
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 2);
            cv::putText(frame, cv::format("XYZ(m): [%.2f, %.2f, %.2f]", tx, ty, tz),
                        cv::Point(textPos.x, textPos.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,0), 2);
            cv::putText(frame, cv::format("Rot(deg): [%.1f, %.1f, %.1f]",
                                          eulerAngles[0], eulerAngles[1], eulerAngles[2]),
                        cv::Point(textPos.x, textPos.y - 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 2);
        } catch (cv::Exception& e) {
            cout << "[ERROR] marker " << i << " : " << e.what() << endl;
            continue;
        }
    }
}


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
                // ArUco 마커 검출
                ArucoDetect(frame);
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
    cout << "=== Zebra Scanner Video - ArUco Detector (Linux) ===\n";
    cout << "Press 'q' or ESC to quit.\n";

    // ArUco 초기화
    g_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    g_detectorParams = cv::aruco::DetectorParameters::create();

    // 캘리브레이션 데이터 불러오기
    cv::FileStorage fs("../data/camera_calibration.xml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "[ERROR] Can't find file 'camera_calibration.xml'." << endl;
        return -1;
    }

    fs["cameraMatrix"] >> g_cameraMatrix;
    fs["distCoeffs"] >> g_distCoeffs;
    fs.release();

    g_cameraMatrix.convertTo(g_cameraMatrix, CV_64F);
    g_distCoeffs.convertTo(g_distCoeffs, CV_64F);

    cout << "[INFO] Data loading complete." << endl;

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
}
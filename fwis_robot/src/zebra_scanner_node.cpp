/**
 * zebra_scanner_node.cpp
 *
 * barcode_bridge_node 초기화 방식 기반으로 ArUco 영상 인식 통합
 *
 * - SDK 초기화: barcode_bridge_node 그대로
 *   (CMD_DEVICE_SCAN_ENABLE + SUBSCRIBE_BARCODE + 150ms keepAlive)
 * - 영상 프레임: OnVideoEvent → DecoderLoop → ArUco 인식 → publish
 * - /alignment_done=true → scanning_=true → OnBarcodeEvent publish
 * - /scan_done=true      → scanning_=false
 */

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <sstream>
#include <regex>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "scanner_interfaces/msg/barcode_event.hpp"
#include "scanner_interfaces/msg/tilt_state.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

#include "zebra-scanner/CsBarcodeTypes.h"
#include "zebra-scanner/CsIEventListenerXml.h"
#include "zebra-scanner/Cslibcorescanner_xml.h"
#include "zebra-scanner/CsUserDefs.h"

using namespace std;

// ── 상수 ─────────────────────────────────────────────────────
static const uint64_t kKeepAlivePullMs = 50;
static const uint64_t kSameDataBlockMs = 300;

// ── 유틸 ─────────────────────────────────────────────────────
static inline uint64_t GetNowMs()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

static string ExtractTag(const string & xml, const string & tag)
{
    const string open  = "<"  + tag + ">";
    const string close = "</" + tag + ">";
    size_t s = xml.find(open);
    size_t e = xml.find(close);
    if (s == string::npos || e == string::npos || e <= s) return "";
    return xml.substr(s + open.size(), e - s - open.size());
}

static int HexNibble(char c)
{
    if ('0' <= c && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    if ('a' <= c && c <= 'f') return 10 + (c - 'a');
    return -1;
}

static bool LooksLikeHexBytes(const string & s)
{
    return s.find("0x") != string::npos || s.find("0X") != string::npos;
}

static string HexStringToAscii(const string & hex)
{
    string out;
    size_t i = 0;
    while (i < hex.size()) {
        if (i + 3 < hex.size() &&
            hex[i] == '0' && (hex[i+1] == 'x' || hex[i+1] == 'X')) {
            int n1 = HexNibble(hex[i+2]);
            int n2 = HexNibble(hex[i+3]);
            if (n1 >= 0 && n2 >= 0) {
                out.push_back((char)((n1 << 4) | n2));
                i += 4; continue;
            }
        }
        i++;
    }
    return out;
}

static bool IsHexChar(char c)
{
    return isdigit((unsigned char)c) ||
           ('a' <= tolower((unsigned char)c) && tolower((unsigned char)c) <= 'f');
}

static bool LooksLikeRawHexBytes(const string & s)
{
    if (s.empty()) return false;
    istringstream iss(s);
    string token;
    int count = 0;
    while (iss >> token) {
        if (token.size() != 2) return false;
        if (!IsHexChar(token[0]) || !IsHexChar(token[1])) return false;
        count++;
    }
    return count >= 2;
}

static string RawHexToAscii(const string & s)
{
    string out;
    istringstream iss(s);
    string token;
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

static string DataLabelToString(const string & raw)
{
    if (LooksLikeHexBytes(raw)) {
        string r = HexStringToAscii(raw);
        if (!r.empty()) return r;
    }
    if (LooksLikeRawHexBytes(raw)) {
        string r = RawHexToAscii(raw);
        if (!r.empty()) return r;
    }
    return raw;
}

struct ParsedBarcode { string serial; int unit; };

static bool ParseBarcode(const string & raw, ParsedBarcode & out)
{
    static const regex re("([A-Za-z0-9]{4})([0-9]{2})");
    smatch m;
    if (!regex_search(raw, m, re)) return false;
    out.serial = m[1].str();
    out.unit   = stoi(m[2].str());
    return out.unit >= 1 && out.unit <= 51;
}

// ─────────────────────────────────────────────────────────────
struct RawFrame { vector<uchar> data; int scanner_id = -1; };

// 전역 포인터
class ZebraScannerNode;
static ZebraScannerNode * g_node_ptr = nullptr;

static unsigned short            g_numScanners = 0;
static vector<unsigned int>      g_scannerIds;
static map<int, string>          g_lastData;
static map<int, uint64_t>        g_lastDataMs;

class ZebraScannerNode : public rclcpp::Node
{
public:
    explicit ZebraScannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("zebra_scanner_node", options),
      scanning_(false)
    {
        scanner_id_left_  = this->get_parameter("scanner_id_left").as_int();
        scanner_id_right_ = this->get_parameter("scanner_id_right").as_int();
        marker_size_      = this->get_parameter("marker_size").as_double();
        const string calib_file = this->get_parameter("calibration_file").as_string();

        aruco_dict_   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        aruco_params_ = cv::aruco::DetectorParameters::create();

        if (!calib_file.empty()) {
            cv::FileStorage fs(calib_file, cv::FileStorage::READ);
            if (fs.isOpened()) {
                fs["cameraMatrix"] >> camera_matrix_;
                fs["distCoeffs"]   >> dist_coeffs_;
                fs.release();
                camera_matrix_.convertTo(camera_matrix_, CV_64F);
                dist_coeffs_.convertTo(dist_coeffs_, CV_64F);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "[ZebraScanner] calibration open failed: %s", calib_file.c_str());
            }
        }

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        pub_left_    = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers_left",  qos);
        pub_right_   = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers_right", qos);
        pub_barcode_ = this->create_publisher<scanner_interfaces::msg::BarcodeEvent>("/barcode/unit_event", 100);

        sub_alignment_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/alignment_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    g_lastData.clear();
                    g_lastDataMs.clear();
                    // 비디오 모드 → 바코드 모드 전환
                    for (auto id : g_scannerIds) {
                        const string sid = to_string(id);

                        const string xml =
                            "<inArgs><scannerID>" + sid + "</scannerID></inArgs>";

                        const string vf_off_xml =
                            "<inArgs><scannerID>" + sid +
                            "</scannerID><cmdArgs><arg-xml><attrib_list><attribute>"
                            "<id>324</id><datatype>B</datatype><value>0</value>"
                            "</attribute></attrib_list></arg-xml></cmdArgs></inArgs>";

                        string out, vf_out;
                        StatusID status;

                        RCLCPP_INFO(this->get_logger(),
                            "[ZebraScanner] scannerID=%s -> barcode mode start", sid.c_str());

                        ::ExecCommand(CMD_DEVICE_RELEASE_TRIGGER, xml, out, &status);
                        usleep(100 * 1000);

                        ::ExecCommand(CMD_RSM_ATTR_SET, vf_off_xml, vf_out, &status);
                        usleep(100 * 1000);

                        ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, xml, out, &status);
                    }

                    scanning_.store(true);
                    RCLCPP_INFO(this->get_logger(), "[ZebraScanner] → 바코드 모드");
                }
            });

        sub_scan_done_ = this->create_subscription<std_msgs::msg::Bool>(
            "/scan_done", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    scanning_.store(false);
                    // 바코드 모드 → 영상 모드 복귀
                    for (auto id : g_scannerIds) {
                        const string sid = to_string(id);
                        const string xml =
                            "<inArgs><scannerID>" + sid + "</scannerID></inArgs>";
                        const string vf_xml =
                            "<inArgs><scannerID>" + sid +
                            "</scannerID><cmdArgs><arg-xml><attrib_list><attribute>"
                            "<id>324</id><datatype>B</datatype><value>1</value>"
                            "</attribute></attrib_list></arg-xml></cmdArgs></inArgs>";
                        string out, vf_out; StatusID status;
                        ::ExecCommand(CMD_DEVICE_RELEASE_TRIGGER, xml,    out,    &status);
                        ::ExecCommand(CMD_RSM_ATTR_SET,           vf_xml, vf_out, &status);
                        ::ExecCommand(CMD_DEVICE_CAPTURE_VIDEO,   xml,    out,    &status);
                        ::ExecCommand(CMD_DEVICE_PULL_TRIGGER,    xml,    out,    &status);
                    }
                    RCLCPP_INFO(this->get_logger(), "[ZebraScanner] → 영상 모드");
                }
            });

        // keepAlive 타이머
        keepalive_timer_ = this->create_wall_timer(
            chrono::milliseconds(kKeepAlivePullMs),
            [this]() {
                for (auto id : g_scannerIds) {
                    const string xml =
                        "<inArgs><scannerID>" + to_string(id) + "</scannerID></inArgs>";
                    string out; StatusID status;
                    ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, xml, out, &status);
                }
            });

        g_node_ptr = this;

        if (!InitScanner()) {
            RCLCPP_ERROR(this->get_logger(), "[ZebraScanner] scanner init failed");
            return;
        }

        running_.store(true);
        decoder_thread_ = thread(&ZebraScannerNode::DecoderLoop, this);

        cv::namedWindow("scanner_left",  cv::WINDOW_NORMAL);
        cv::namedWindow("scanner_right", cv::WINDOW_NORMAL);

        RCLCPP_INFO(this->get_logger(), "[ZebraScanner] ready | left=%d right=%d",
            scanner_id_left_, scanner_id_right_);
    }

    ~ZebraScannerNode() override
    {
        g_node_ptr = nullptr;
        running_.store(false);
        queue_cv_.notify_all();
        if (decoder_thread_.joinable()) decoder_thread_.join();
        for (auto id : g_scannerIds) {
            const string xml =
                "<inArgs><scannerID>" + to_string(id) + "</scannerID></inArgs>";
            string out; StatusID status;
            ::ExecCommand(CMD_DEVICE_RELEASE_TRIGGER, xml, out, &status);
        }
        StatusID status; ::Close(0, &status);
        if (sink_) sink_.reset();
        cv::destroyAllWindows();
    }

    void onBarcodeXml(const string & xml)
    {
        if (!scanning_.load()) return;

        string data  = ExtractTag(xml, "datalabel");
        string dtype = ExtractTag(xml, "datatype");
        string id    = ExtractTag(xml, "scannerID");
        if (data.empty()) return;

        string printable = DataLabelToString(data);

        int sid = -1;
        try { sid = id.empty() ? -1 : stoi(id); } catch (...) {}

        uint64_t now = GetNowMs();
        if (sid >= 0) {
            if (g_lastData[sid] == printable &&
                (now - g_lastDataMs[sid]) < kSameDataBlockMs) return;
            g_lastData[sid]    = printable;
            g_lastDataMs[sid]  = now;
        }

        ParsedBarcode pb;
        if (!ParseBarcode(printable, pb)) {
            RCLCPP_INFO(this->get_logger(), "Ignore raw barcode: %s", printable.c_str());
            return;
        }

        scanner_interfaces::msg::BarcodeEvent msg;
        msg.stamp      = this->now();
        msg.scanner_id = sid;
        msg.raw        = printable;
        msg.serial     = pb.serial;
        msg.unit       = pb.unit;
        msg.left_tick  = 0;
        msg.right_tick = 0;
        pub_barcode_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "[PUB] unit=U%02d serial=%s raw=%s",
            pb.unit, pb.serial.c_str(), printable.c_str());
    }

private:
    class EventSink : public IEventListenerXml
    {
    public:
        void OnBarcodeEvent(short, string & pscanData) override {
            if (g_node_ptr) g_node_ptr->onBarcodeXml(pscanData);
        }

        void OnVideoEvent(short, int size, char * data, int len,
                          string & scanner_data) override
        {
            if (!g_node_ptr || size <= 0 || !data || len <= 0) return;
            int sid = -1;
            const string id_str = ExtractTag(scanner_data, "scannerID");
            if (!id_str.empty()) { try { sid = stoi(id_str); } catch (...) {} }

            RawFrame frame;
            frame.data.assign(reinterpret_cast<uchar*>(data),
                              reinterpret_cast<uchar*>(data) + len);
            frame.scanner_id = sid;
            {
                lock_guard<mutex> lock(g_node_ptr->queue_mutex_);
                while (g_node_ptr->raw_queue_.size() > 3) g_node_ptr->raw_queue_.pop();
                g_node_ptr->raw_queue_.push(move(frame));
            }
            g_node_ptr->queue_cv_.notify_one();
        }

        void OnImageEvent(short et, int size, short, char * data, int len,
                          string & sd) override { OnVideoEvent(et, size, data, len, sd); }

        void OnPNPEvent(short t, string) override {
            if (!g_node_ptr) return;
            if (t == SCANNER_ATTACHED)
                RCLCPP_INFO(g_node_ptr->get_logger(), "[ZebraScanner] scanner attached");
            else if (t == SCANNER_DETACHED)
                RCLCPP_WARN(g_node_ptr->get_logger(), "[ZebraScanner] scanner detached");
        }
        void OnScannerNotification(short, string &) override {}
        void OnBinaryDataEvent(short, int, short, unsigned char *, string &) override {}
        void OnCommandResponseEvent(short, string &) override {}
        void OnIOEvent(short, unsigned char) override {}
        void OnScanRMDEvent(short, string &) override {}
        void OnDisconnect() override {
            if (g_node_ptr)
                RCLCPP_WARN(g_node_ptr->get_logger(), "[ZebraScanner] disconnected");
        }
    };

    bool InitScanner()
    {
        sink_ = std::make_unique<EventSink>();
        StatusID status;

        ::Open(sink_.get(), SCANNER_TYPE_ALL, &status);
        if (status != STATUS_OK) {
            RCLCPP_ERROR(this->get_logger(), "Open() failed status=%d", status);
            return false;
        }

        string outXml;
        ::GetScanners(&g_numScanners, &g_scannerIds, outXml, &status);
        if (status != STATUS_OK || g_numScanners == 0) {
            RCLCPP_ERROR(this->get_logger(), "No scanners found");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Detected scanners: %d", g_numScanners);

        for (auto id : g_scannerIds) {
            const string xml =
                "<inArgs><scannerID>" + to_string(id) + "</scannerID></inArgs>";
            string out;
            ::ExecCommand(CMD_DEVICE_SCAN_ENABLE, xml, out, &status);
        }

        {
            const string reg_xml =
                "<inArgs><cmdArgs>"
                "<arg-int>1</arg-int>"
                "<arg-int>" + to_string(SUBSCRIBE_BARCODE) + "</arg-int>"
                "</cmdArgs></inArgs>";
            string reg_out;
            ::ExecCommand(CMD_REGISTER_FOR_EVENTS, reg_xml, reg_out, &status);
            if (status != STATUS_OK) {
                RCLCPP_ERROR(this->get_logger(), "Register barcode event failed");
                return false;
            }
        }

        {
            const string reg_xml =
                "<inArgs><cmdArgs><arg-int>3</arg-int><arg-int>1,2,4</arg-int></cmdArgs></inArgs>";
            string reg_out;
            ::ExecCommand(CMD_REGISTER_FOR_EVENTS, reg_xml, reg_out, &status);
        }

        for (auto id : g_scannerIds) {
            const string sid = to_string(id);
            const string xml = "<inArgs><scannerID>" + sid + "</scannerID></inArgs>";
            const string vf_xml =
                "<inArgs><scannerID>" + sid +
                "</scannerID><cmdArgs><arg-xml><attrib_list><attribute>"
                "<id>324</id><datatype>B</datatype><value>1</value>"
                "</attribute></attrib_list></arg-xml></cmdArgs></inArgs>";
            string out, vf_out;
            ::ExecCommand(CMD_RSM_ATTR_SET,         vf_xml, vf_out, &status);
            ::ExecCommand(CMD_DEVICE_CAPTURE_VIDEO, xml,    out,    &status);
            ::ExecCommand(CMD_DEVICE_PULL_TRIGGER,  xml,    out,    &status);
        }

        for (auto id : g_scannerIds) {
            const string xml =
                "<inArgs><scannerID>" + to_string(id) + "</scannerID></inArgs>";
            string out;
            ::ExecCommand(CMD_DEVICE_PULL_TRIGGER, xml, out, &status);
        }

        RCLCPP_INFO(this->get_logger(), "[ZebraScanner] SDK init complete");
        return true;
    }

    void DecoderLoop()
    {
        while (running_.load()) {
            RawFrame raw;
            {
                unique_lock<mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this]{
                    return !raw_queue_.empty() || !running_.load();
                });
                if (!running_.load()) break;
                raw = move(raw_queue_.front());
                raw_queue_.pop();
            }
            cv::Mat frame = cv::imdecode(raw.data, cv::IMREAD_COLOR);
            if (frame.empty()) continue;
            DetectAndPublish(frame, raw.scanner_id);
        }
    }

    void DetectAndPublish(cv::Mat & frame, int scanner_id)
    {
        if (!aruco_dict_ || !aruco_params_) return;
        vector<int> ids;
        vector<vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, aruco_dict_, corners, ids, aruco_params_);
        cv::Mat display = frame.clone();
        if (!ids.empty()) cv::aruco::drawDetectedMarkers(display, corners, ids);
        const string win = (scanner_id == scanner_id_left_) ? "scanner_left" : "scanner_right";
        cv::imshow(win, display);
        cv::waitKey(1);
        if (ids.empty()) return;

        ros2_aruco_interfaces::msg::ArucoMarkers msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = (scanner_id == scanner_id_left_) ? "camera_left" : "camera_right";

        for (size_t i = 0; i < ids.size(); ++i) {
            msg.marker_ids.push_back(ids[i]);
            geometry_msgs::msg::Pose pose;
            if (!camera_matrix_.empty() && !dist_coeffs_.empty()) {
                try {
                    cv::Mat rvec, tvec;
                    vector<vector<cv::Point2f>> single = {corners[i]};
                    cv::aruco::estimatePoseSingleMarkers(
                        single, static_cast<float>(marker_size_),
                        camera_matrix_, dist_coeffs_, rvec, tvec);
                    pose.position.x = tvec.at<double>(0,0);
                    pose.position.y = tvec.at<double>(0,1);
                    pose.position.z = tvec.at<double>(0,2);
                    cv::Mat R; cv::Rodrigues(rvec, R);
                    double tr = R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2);
                    double qw = sqrt(max(0.0,1.0+tr))/2.0;
                    double qx = sqrt(max(0.0,1.0+R.at<double>(0,0)-R.at<double>(1,1)-R.at<double>(2,2)))/2.0;
                    double qy = sqrt(max(0.0,1.0-R.at<double>(0,0)+R.at<double>(1,1)-R.at<double>(2,2)))/2.0;
                    double qz = sqrt(max(0.0,1.0-R.at<double>(0,0)-R.at<double>(1,1)+R.at<double>(2,2)))/2.0;
                    qx = copysign(qx, R.at<double>(2,1)-R.at<double>(1,2));
                    qy = copysign(qy, R.at<double>(0,2)-R.at<double>(2,0));
                    qz = copysign(qz, R.at<double>(1,0)-R.at<double>(0,1));
                    pose.orientation.x=qx; pose.orientation.y=qy;
                    pose.orientation.z=qz; pose.orientation.w=qw;
                } catch (const cv::Exception & ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[ZebraScanner] pose failed: %s", ex.what());
                }
            }
            msg.poses.push_back(pose);
        }
        if (scanner_id == scanner_id_left_)       pub_left_->publish(msg);
        else if (scanner_id == scanner_id_right_)  pub_right_->publish(msg);
        else { pub_left_->publish(msg); pub_right_->publish(msg); }
    }

    // ── 멤버 변수 ──────────────────────────────────────────────
    int    scanner_id_left_, scanner_id_right_;
    double marker_size_;

    cv::Ptr<cv::aruco::Dictionary>         aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    unique_ptr<EventSink> sink_;
    atomic<bool>          scanning_{false};

    queue<RawFrame>    raw_queue_;
    mutex              queue_mutex_;
    condition_variable queue_cv_;
    atomic<bool>       running_{false};
    thread             decoder_thread_;

    rclcpp::TimerBase::SharedPtr keepalive_timer_;

    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr pub_left_, pub_right_;
    rclcpp::Publisher<scanner_interfaces::msg::BarcodeEvent>::SharedPtr    pub_barcode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_alignment_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_scan_done_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = make_shared<ZebraScannerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
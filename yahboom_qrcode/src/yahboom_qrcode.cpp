//
// Created by yahboom on 5/30/23.
//
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/wechat_qrcode.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <utility>
#include "std_msgs/msg/header.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rclcpp/time.hpp"
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>
#include "image_color_lab/msg/string_stamped.hpp"

#include <iostream>
#include <vector>

using std::placeholders::_1;
rclcpp::Clock system_clock(RCL_ROS_TIME);
namespace enc = sensor_msgs::image_encodings;
double getContourAreaMax(std::vector<std::vector<cv::Point>> contours, uint *index);
cv::Mat image_processing(const cv::Mat in_image);
//sensor_msgs::msg::CompressedImage cv_to_ros(cv::Mat image, header_msg);
sensor_msgs::msg::CompressedImage cv_to_ros(const cv::Mat& image);
class XgoQrCodePublisherCNode : public rclcpp::Node
{
public:
    XgoQrCodePublisherCNode()
            : Node("minimal_subscriber")
    {
        subscription_lab = this->create_subscription<std_msgs::msg::String>(
                "/lab_set", 100, std::bind(&XgoQrCodePublisherCNode::lab_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "cv_image", 100);
        publisher_compressed_image = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "/image_raw/compressed", 100);
        publisher_mask_image = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "image_mask", 100);

        publisher_identify_image = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "image_identify", 100);

        publisher_dilated_image = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "image_dilated", 100);

        publisher_eroded_image = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "image_eroded", 100);
        publisher_contours_image = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "image_contours", 1);
        publisher_identify_point = this->create_publisher<image_color_lab::msg::StringStamped>(
                "obj_msg", 10);

    }
    void processImageAndPublish(){
        cv::VideoCapture cap("/dev/video0");
        if (!cap.isOpened())
        {
            RCLCPP_ERROR(rclcpp::get_logger("camera"), "Failed to open camera.");
            return;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        std::cout << "A demo program of WeChat QRCode Detector: " << std::endl;
        try {
            detector = cv::makePtr<cv::wechat_qrcode::WeChatQRCode>(detect_prototxt , detect_caffe_model,
                                                                    sr_prototxt, sr_caffe_model);
        } catch (const std::exception& e) {
            std::cout <<
                      "\n---------------------------------------------------------------\n"
                      "Failed to initialize WeChatQRCode.\n"
                      "Please, download 'detector.*' and 'sr.*' from\n"
                      "https://github.com/WeChatCV/opencv_3rdparty/tree/wechat_qrcode\n"
                      "and put them into the current directory.\n"
                      "---------------------------------------------------------------\n";
            std::cout << e.what() << std::endl;
            return;
        }

//        cv::FileStorage fs_read("/home/pi/cartographer_ws2/ost.yaml", cv::FileStorage::READ);
        std::cout <<
                      "\n---------------------------------------------------------------\n"
                      "11111,here.\n"
                      "---------------------------------------------------------------\n";
            std::cout << std::endl;
//        cv::Mat camera_matrix, dist_coeffs;
//        fs_read["camera_matrix"] >> camera_matrix;
//        fs_read["distortion_coefficients"] >> dist_coeffs;
//        fs_read.release();
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) <<587.263338, 0.000000, 172.423957, 0.000000, 650.221263, 252.595890, 0.000000, 0.000000, 1.000000);
        cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << -0.605286, 0.114931, -0.002447, 0.120408, 0.000000);
        
        fov_h = 2 * atan(640 / (2*587.263338));
        
        fov_v = 2 * atan(480 / (2*650.221263));

        std::cout << "init,fov."<< fov_h << std::endl;
        std::cout << "init111,foh. "<< fov_v << std::endl;
        rclcpp::WallRate loop_rate(30);
        while (rclcpp::ok())
        {
            cv::Mat frame;
            cap >> frame;
            if (!frame.empty())
            {
                publish_process(frame);
            }
            loop_rate.sleep();
        }
    }
private:
    void publish_process(const cv::Mat msg)
    {
        //cv image
        cv::Mat image_raw =  msg;
        time_t now = time(0);
        char* dt = ctime(&now);
        cv::putText(image_raw, dt, cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2);
        cv::Size imageSize = image_raw.size();
        int width = imageSize.width;
        int height = imageSize.height;

        cv::line(image_raw, cv::Point((width / 2 - 10), height / 2), cv::Point ((width / 2 + 10), height / 2),  cv::Scalar (0, 255, 255), 2);
        cv::line(image_raw, cv::Point(width / 2, (height / 2 - 10)),cv::Point (width / 2, (height / 2 + 10)), cv::Scalar(0, 255, 255), 2);
//qrcode

        std::string prevstr = "";
        std::vector<cv::String> resDecoded;
        std::vector<cv::Mat> points;
        auto res = detector->detectAndDecode(msg, points);
        for (const auto& t : res) {
            if (t != prevstr)
                std::cout << t << std::endl;

        }
        resDecoded = res;
        rapidjson::Document doc;
        doc.SetObject();
        // 添加键值对到 JSON 对象中
        rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();
        for (int i = 0; i < static_cast<int>(resDecoded.size()); i++)
        {
            std::cout << "decode-" << i+1 << ": " << resDecoded[i] << std::endl;
            cv::Point pt1 = cv::Point((int)points[i].at<float>(0, 0), (int)points[i].at<float>(0, 1));
            cv::Point pt2 = cv::Point((int)points[i].at<float>(1, 0), (int)points[i].at<float>(1, 1));
            cv::Point pt3 = cv::Point((int)points[i].at<float>(2, 0), (int)points[i].at<float>(2, 1));
            cv::Point pt4 = cv::Point((int)points[i].at<float>(3, 0), (int)points[i].at<float>(3, 1));
            if(i == 0){
                
                std::vector<cv::Point> Points;
                Points.push_back(pt1);
                Points.push_back(pt2);
                Points.push_back(pt3);
                Points.push_back(pt4);
                double area = cv::contourArea(Points);
                double qr_size = sqrt(area);
                double distance = qr_size / 0.51;
                std::cout << "11111,here. %f"<< distance << std::endl;
                std::cout << "11111,fov. %f"<< fov_v << std::endl;
                std::cout << "11111,foh. "<< fov_h << std::endl;
                
//                std::cout distance << std::endl;
                doc.AddMember("area", area, allocator);
                doc.AddMember("dist", distance, allocator);
                doc.AddMember("center_x", (pt1.x + pt3.x)/2, allocator);
                doc.AddMember("center_y", (pt1.y + pt3.y)/2, allocator);
                doc.AddMember("point1_x", pt1.x, allocator);
                doc.AddMember("point1_y", pt1.y, allocator);
                doc.AddMember("point2_x", pt2.x, allocator);
                doc.AddMember("point2_y", pt2.y, allocator);
                doc.AddMember("point3_x", pt3.x, allocator);
                doc.AddMember("point3_y", pt3.y, allocator);
                doc.AddMember("point4_x", pt4.x, allocator);
                doc.AddMember("point4_y", pt4.y, allocator);
            }
            line(image_raw, pt1, pt2, cv::Scalar(0, 255, 0), 2);
            line(image_raw, pt2, pt3, cv::Scalar(0, 255, 0), 2);
            line(image_raw, pt3, pt4, cv::Scalar(0, 255, 0), 2);
            line(image_raw, pt4, pt1, cv::Scalar(0, 255, 0), 2);
            cv::putText(image_raw, resDecoded[i], pt1, 0, 2, cv::Scalar(255, 0, 0), 2);
        }

        sensor_msgs::msg::CompressedImage compressed_image = cv_to_ros(image_raw);
        rapidjson::StringBuffer string_Buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(string_Buffer);
        doc.Accept(writer);
        auto objMessage = image_color_lab::msg::StringStamped();
        std_msgs::msg::Header header_msg;
        header_msg.stamp.sec = system_clock.now().seconds();
        header_msg.stamp.nanosec = system_clock.now().nanoseconds();
        objMessage.header = std::move(header_msg);
        objMessage.data = string_Buffer.GetString();

        if (!res.empty())
            prevstr = res.back();
        cv::Mat frame_resize, frame_lab;
        cv::resize(image_raw, frame_resize, image_raw.size(), cv::INTER_NEAREST);
        cv::cvtColor(frame_resize, frame_lab, cv::COLOR_BGR2Lab);
        cv::Mat image_thresh;
        std::lock_guard<std::mutex> lock(mutex_);
        cv::Scalar lower_range(std::get<0>(lab_min), std::get<1>(lab_min), std::get<2>(lab_min));
        cv::Scalar upper_range(std::get<0>(lab_max), std::get<1>(lab_max), std::get<2>(lab_max));
        cv::inRange(frame_lab, lower_range, upper_range, image_thresh);  // 对原图像和掩模进行位运算
        time_t now_thresh = time(0);
        char* dt_thresh = ctime(&now_thresh);
        cv::putText(image_thresh, dt_thresh, cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2);
        sensor_msgs::msg::CompressedImage compressed_image_thresh = cv_to_ros(image_thresh);
        // 对掩膜进行腐蚀操作
//opencv4.2
//        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
//opencv4.7
//        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
//        std::cout << "Size of compressed image02: " << std::endl;
//        cv::Mat eroded;
//        cv::erode(image_thresh, eroded, element);
//        time_t now_eroded = time(0);
//        char* dt_eroded = ctime(&now_eroded);
//
//        cv::putText(eroded, dt_eroded, cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2);
//        std::cout << "Size of compressed image1: " << std::endl;
//        sensor_msgs::msg::CompressedImage compressed_image_eroded = cv_to_ros(eroded);
//        cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//        cv::Mat dilated;
//        cv::dilate(eroded, dilated, element_dilate);  //膨胀
//        time_t now_dilated = time(0);
//        char* dt_dilated = ctime(&now_dilated);
//        cv::putText(dilated, dt_dilated, cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2);
//        std::cout << "Size of compressed image2: " << std::endl;
//        sensor_msgs::msg::CompressedImage compressed_image_dilated = cv_to_ros(dilated);
//        std::vector<std::vector<cv::Point>> contours;
//        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//        cv::drawContours(image_raw, contours, -1, cv::Scalar(0, 0, 255), 3,cv::LINE_AA);
//        sensor_msgs::msg::CompressedImage compressed_image_contours = cv_to_ros(image_raw);
//
//        uint index;
//        double area_max = getContourAreaMax(contours, &index);
//        sensor_msgs::msg::CompressedImage compressed_image_identify;
//        auto objMessage = image_color_lab::msg::StringStamped();
////        objMessage.header =
//        std_msgs::msg::Header header_msg;
//        header_msg.stamp.sec = system_clock.now().seconds();
//        header_msg.stamp.nanosec = system_clock.now().nanoseconds();
//        objMessage.header = std::move(header_msg);
//        // 创建 RapidJSON 的值对象
//        rapidjson::Document doc;
//        if (area_max > 100){
//            std::vector<cv::Point> area_max_contour = contours[index];
//            cv::Point2f center;
//            float radius;
//            cv::minEnclosingCircle(area_max_contour, center, radius);
//            cv::circle(image_raw, center, int(radius), cv::Scalar(0, 255, 255), 2);
//
//            //绘制分割线
//            cv::line(image_raw, cv::Point(0, (height / 2) - 20), cv::Point (width, (height / 2) -20),  cv::Scalar (0, 255, 255), 2);
//            cv::line(image_raw, cv::Point(0, (height / 2) + 20),cv::Point (width, (height / 2) + 20), cv::Scalar(0, 255, 255), 2);
//
//            cv::line(image_raw, cv::Point((width / 2) - 20, 0), cv::Point ((width / 2) - 20, height),  cv::Scalar (0, 255, 255), 2);
//            cv::line(image_raw, cv::Point((width / 2) + 20, 0),cv::Point ((width / 2) + 20, height ), cv::Scalar(0, 255, 255), 2);
//
//            cv::Rect rectLeft(0, 0, (width / 2) - 20, height - 0);
//
//            cv::Rect rectRight((width / 2) + 20, 0, (width / 2) - 20, height - 0);
//
//            cv::Rect rectUpper((width / 2) - 20, 0, 40, (height / 2) - 20);
//
//            cv::Rect rectBelow((width / 2) - 20, (height / 2) + 20, 40, (height / 2) - 20);
//
//            cv::Rect rectMiddle((width / 2) - 20, 0, 40, height);
//
//            cv::Point2f point(50, 30);
//            if (point.x >= static_cast<float>(rectMiddle.x) && point.x < static_cast<float> (rectMiddle.x) + static_cast<float>(rectMiddle.width) && point.y >= static_cast<float>(rectMiddle.y) && point.y < static_cast<float>(rectMiddle.y) + static_cast<float>(rectMiddle.height)) {
//                // 点在矩形区域内
//            } else {
//                // 点不在矩形区域内
//            }
//
//
//            compressed_image_identify = cv_to_ros(image_raw);
//            doc.SetObject();
//            // 添加键值对到 JSON 对象中
//            rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();
//            doc.AddMember("center_x", center.x, allocator);
//            doc.AddMember("center_y", center.y, allocator);
//            doc.AddMember("img_w", width, allocator);
//            doc.AddMember("img_h", height, allocator);
//
//
//
//            if (center.x >= static_cast<float>(rectMiddle.x) && center.x < static_cast<float> (rectMiddle.x) + static_cast<float>(rectMiddle.width) && center.y >= static_cast<float>(rectMiddle.y) && center.y < static_cast<float>(rectMiddle.y) + static_cast<float>(rectMiddle.height)) {
//                // 点在矩形区域内
//
//                if (center.x >= static_cast<float>(rectUpper.x) && center.x < static_cast<float> (rectUpper.x) + static_cast<float>(rectUpper.width) && center.y >= static_cast<float>(rectUpper.y) && center.y < static_cast<float>(rectUpper.y) + static_cast<float>(rectUpper.height)) {
//                    // 点在矩形区域内
//                    RCLCPP_INFO(rclcpp::get_logger("camera"), "Obj at rectUpper");
//                    doc.AddMember("horizontal_movement", 0, allocator);
//                    doc.AddMember("vertical_movement", -1, allocator);
//                } else {
//                    // 点不在矩形区域内
//                    ;
//
//                }
//
//                if (center.x >= static_cast<float>(rectBelow.x) && center.x < static_cast<float> (rectBelow.x) + static_cast<float>(rectBelow.width) && center.y >= static_cast<float>(rectBelow.y) && center.y < static_cast<float>(rectBelow.y) + static_cast<float>(rectBelow.height)) {
//                    // 点在矩形区域内
//                    RCLCPP_INFO(rclcpp::get_logger("camera"), "Obj at rectBelow");
//                    doc.AddMember("horizontal_movement", 0, allocator);
//                    doc.AddMember("vertical_movement", 1, allocator);
//                } else {
//                    // 点不在矩形区域内
//
//                    ;
//                }
//            } else {
//                // 点不在矩形区域内
//                if (center.x >= static_cast<float>(rectLeft.x) && center.x < static_cast<float> (rectLeft.x) + static_cast<float>(rectLeft.width) && center.y >= static_cast<float>(rectLeft.y) && center.y < static_cast<float>(rectLeft.y) + static_cast<float>(rectLeft.height)) {
//                    // 点在矩形区域内
//                    RCLCPP_INFO(rclcpp::get_logger("camera"), "Obj at rectLeft");
//                    doc.AddMember("horizontal_movement", 1, allocator);
//                    doc.AddMember("vertical_movement", 0, allocator);
//                } else {
//                    // 点不在矩形区域内
//                    ;
//                }
//                if (center.x >= static_cast<float>(rectRight.x) && center.x < static_cast<float> (rectRight.x) + static_cast<float>(rectRight.width) && center.y >= static_cast<float>(rectRight.y) && center.y < static_cast<float>(rectRight.y) + static_cast<float>(rectRight.height)) {
//                    // 点在矩形区域内
//                    RCLCPP_INFO(rclcpp::get_logger("camera"), "Obj at rectRight");
//                    doc.AddMember("horizontal_movement", -1, allocator);
//                    doc.AddMember("vertical_movement", 0, allocator);
//                } else {
//                    // 点不在矩形区域内
//                    ;
//                }
//            }
//
//            // 序列化 JSON 对象f
//            rapidjson::StringBuffer string_Buffer;
//            rapidjson::Writer<rapidjson::StringBuffer> writer(string_Buffer);
//            doc.Accept(writer);
//            objMessage.data = string_Buffer.GetString();
//        }else{
//            doc.SetObject();
//            // 添加键值对到 JSON 对象中
//            rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();
//            doc.AddMember("center_x", width/2, allocator);
//            doc.AddMember("center_y", height/2, allocator);
//            doc.AddMember("img_w", width, allocator);
//            doc.AddMember("img_h", height, allocator);
//            doc.AddMember("horizontal_movement", 0, allocator);
//            doc.AddMember("vertical_movement", 0, allocator);
//            // 序列化 JSON 对象f
//            rapidjson::StringBuffer string_Buffer;
//            rapidjson::Writer<rapidjson::StringBuffer> writer(string_Buffer);
//            doc.Accept(writer);
//            objMessage.data = string_Buffer.GetString();
//        }







        publisher_compressed_image ->publish(compressed_image);
        publisher_mask_image ->publish(compressed_image_thresh);
//        publisher_dilated_image ->publish(compressed_image_dilated);
//        publisher_eroded_image ->publish(compressed_image_eroded);
//        publisher_identify_image ->publish(compressed_image_identify);
//        publisher_contours_image ->publish(compressed_image_contours);
        publisher_identify_point ->publish(objMessage);
    }

    void lab_callback(const std_msgs::msg::String::SharedPtr msg) {
        rapidjson::Document document;
        document.Parse(msg->data.c_str());
        if (document.HasMember("l") && document.HasMember("a") && document.HasMember("b")) {
            l_min = document["l"].GetInt();
            a_min = document["a"].GetInt();
            b_min = document["b"].GetInt();
            l_max = document["l_max"].GetInt();
            a_max = document["a_max"].GetInt();
            b_max = document["b_max"].GetInt();
            std::lock_guard<std::mutex> lock(mutex_);
            std::get<0>(lab_min) = l_min;
            std::get<1>(lab_min) = a_min;
            std::get<2>(lab_min) = b_min;
            std::get<0>(lab_max) = l_max;
            std::get<1>(lab_max) = a_max;
            std::get<2>(lab_max) = b_max;
            RCLCPP_INFO(this->get_logger(), "Parsed l_min=%d", l_min);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_lab;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_compressed_image;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_mask_image;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_identify_image;
    rclcpp::Publisher<image_color_lab::msg::StringStamped>::SharedPtr publisher_identify_point;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_dilated_image;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_eroded_image;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_contours_image;
    std::mutex mutex_;
    int64 l_min, a_min, b_min, l_max, a_max, b_max;
    std::tuple<int64, int64, int64> lab_min{
            l_min, a_min, b_min
    };
    std::tuple<int64, int64, int64> lab_max{
            l_max, a_max, b_max
    };

    //      qrcode

    cv::Ptr<cv::wechat_qrcode::WeChatQRCode> detector;

    std::string detect_prototxt = "/root/yahboomcar_ws/src/yahboom_qrcode/src/detect.prototxt";
    std::string detect_caffe_model = "/root/yahboomcar_ws/src/yahboom_qrcode/src/detect.caffemodel";
    std::string sr_prototxt = "/root/yahboomcar_ws/src/yahboom_qrcode/src/sr.prototxt";
    std::string sr_caffe_model = "/root/yahboomcar_ws/src/yahboom_qrcode/src/sr.caffemodel";
    cv::String calibPath = "/root/yahboomcar_ws/src/yahboom_qrcode/src//ost.yaml";
    double fov_h, fov_v;

};


double getContourAreaMax(std::vector<std::vector<cv::Point>> contours, uint *index)
{
    double area_max = 0;
    // Loop over all closed contours
    for (uint i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area > area_max)
        {
            area_max = area;
            *index = i;
        }
    }
    return area_max;
}

cv::Mat image_processing(const cv::Mat in_image)
{
    // Create output image
    cv::Mat out_image;
    // Processing
    out_image = in_image;
    // Show image in a different window
    cv::imshow("out_image",out_image);
    cv::waitKey(3);
    return out_image;
}

sensor_msgs::msg::CompressedImage cv_to_ros2(const cv::Mat& image) // 将cv图像格式转换为ROS压缩图像格式
{
    std::string encoding = "bgr8";
    std::vector<uchar> buffer;
    int bitDepth = enc::bitDepth(encoding);
    sensor_msgs::msg::CompressedImage compressed;
    std_msgs::msg::Header header_msg;
    header_msg.stamp.sec = system_clock.now().seconds();
    header_msg.stamp.nanosec = system_clock.now().nanoseconds();
    compressed.header = std::move(header_msg);
    compressed.format = "bgr8";
    std::vector<int> params;
//    params.push_back(cv::IMWRITE_JPEG_QUALITY); // 参数名称
//    params.push_back(90); // 参数值
    params.resize(9, 0);
    params[0] = cv::IMWRITE_JPEG_QUALITY;
    params[1] = 90;
    params[2] = cv::IMWRITE_JPEG_PROGRESSIVE;
    params[3] = 0;
    params[4] = cv::IMWRITE_JPEG_OPTIMIZE;
    params[5] = 0;
    params[6] = cv::IMWRITE_JPEG_RST_INTERVAL;
    params[7] = 0;
    compressed.format += "; jpeg compressed ";
    if ((bitDepth == 8) || (bitDepth == 16))
    {
        // Target image format
        std::string targetFormat;
        if (enc::isColor(encoding))
        {
            targetFormat = "bgr8";
            compressed.format += targetFormat;
        }
        cv::imencode(".jpg", image, buffer, params);
        compressed.data = buffer;
    }
    return compressed;
}




sensor_msgs::msg::CompressedImage cv_to_ros(const cv::Mat& image) {
    std::string encoding = "bgr8";
    std::vector<uchar> buffer;
    int bitDepth = enc::bitDepth(encoding);
    sensor_msgs::msg::CompressedImage compressed;
    std_msgs::msg::Header header_msg;
    rclcpp::Clock clock(RCL_ROS_TIME);
    auto now = clock.now();
    header_msg.stamp.sec = now.seconds();
    header_msg.stamp.nanosec = now.nanoseconds();
    compressed.header = std::move(header_msg);
    compressed.format = "bgr8";
    std::vector<int> params;
    params.resize(8, 0);
    params[0] = cv::IMWRITE_JPEG_QUALITY;
    params[1] = 80;
    params[2] = cv::IMWRITE_JPEG_PROGRESSIVE;
    params[3] = 0;
    params[4] = cv::IMWRITE_JPEG_OPTIMIZE;
    params[5] = 0;
    params[6] = cv::IMWRITE_JPEG_RST_INTERVAL;
    params[7] = 0;
    compressed.format += "; jpeg compressed ";
    if ((bitDepth == 8) || (bitDepth == 16)) {
        std::string targetFormat;
        if (enc::isColor(encoding)) {
            targetFormat = "bgr8";
            compressed.format += targetFormat;
        }
        cv::imencode(".jpg", image, buffer, params);
        compressed.data = buffer;
    }
    return compressed;
}




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XgoQrCodePublisherCNode>();
    std::thread image_proc_th(&XgoQrCodePublisherCNode::processImageAndPublish, node);
    rclcpp::spin(node);
    // 安全退出线程
    image_proc_th.join();
    rclcpp::shutdown();
    return 0;
}

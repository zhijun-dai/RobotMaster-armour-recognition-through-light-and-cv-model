#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp> // 使用 ROS2 Jazzy 的头文件后缀 .hpp
#include <vision_msgs/msg/detection2_d_array.hpp> // 用于发布检测结果
#include <opencv2/opencv.hpp> // 添加 OpenCV
#include <cstring>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <MvCameraControl.h> // Hikvision MVS SDK 头文件，提供 MV_* 符号
#include "hikvision_interface/msg/hik_image_info.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// MV_CHECK 定义
#ifndef MV_CHECK
#define MV_CHECK(logger, func, ...)                                                                 \
    do {                                                                                              \
        int __ret = func(__VA_ARGS__);                                                                  \
        if (__ret != MV_OK) {                                                                           \
            RCLCPP_ERROR(logger, #func " failed with code %d", __ret);                                   \
        }                                                                                               \
    } while (0)
#endif

namespace hikvision_ros2_driver {

using HikImageInfo = hikvision_interface::msg::HikImageInfo;

class HikvisionDriver : public rclcpp::Node {
public:
    explicit HikvisionDriver(const rclcpp::NodeOptions &options);
    ~HikvisionDriver();

    struct Impl;
    std::unique_ptr<Impl> pImpl;

private:
    // 为图像回调添加成员函数
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

struct HikvisionDriver::Impl {
    rclcpp::Publisher<HikImageInfo>::SharedPtr p_info_pub;
    // 内部原始图像发布器，用于触发本节点订阅回调（/image_raw）
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p_img_in_pub;
    // 添加装甲板检测结果发布器
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr p_armor_detections_pub;
    // C++预处理图像发布器（调试）
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p_armor_preproc_pub;
    // 发布预处理后的 20x28 单通道 canvas，供 Python 模型直接使用
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p_armor_crop_pub;
    std::string camera_name;

    rclcpp::TimerBase::SharedPtr sim_timer;
    bool simulate_mode = false;

    double exposure_time_param_;
    double gain_param_;
    double frame_rate_param_;
    std::string pixel_format_param_;
    void* handle = nullptr;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle;

    // 添加图像订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr p_img_sub;

    // 上一次裁剪尺寸（保留但不使用）
    int last_crop_width = 64;
    int last_crop_height = 64;

    static void image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
};

// 实现图像回调函数，包含装甲板识别逻辑
void HikvisionDriver::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // 假设输入是 BGR
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image.clone(); // 获取 OpenCV Mat 格式图像

    // OpenCV 装甲板识别逻辑（原算法）
    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat kernel7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

    // 预处理：灰度化 + 高斯模糊
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    // 提取高亮区域（固定阈值180）
    cv::Mat brightMask;
    cv::threshold(gray, brightMask, 180, 255, cv::THRESH_BINARY);
    cv::morphologyEx(brightMask, brightMask, cv::MORPH_OPEN, kernel7);
    cv::morphologyEx(brightMask, brightMask, cv::MORPH_CLOSE, kernel7);
    cv::morphologyEx(brightMask, brightMask, cv::MORPH_DILATE, kernel7);
    cv::morphologyEx(brightMask, brightMask, cv::MORPH_DILATE, kernel7);

    // HSV 蓝色掩码（仅调试，可视化使用）
    cv::Mat hsv; cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lowerBlue(160, 200, 200), upperBlue(180, 255, 255);
    cv::Mat blueMask; cv::inRange(hsv, lowerBlue, upperBlue, blueMask);

    // 最终用于找轮廓的掩码
    cv::Mat combinedMask = brightMask;

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combinedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 筛选轮廓
    std::vector<cv::RotatedRect> valid_rects;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 50) continue;
        cv::RotatedRect min_rect = cv::minAreaRect(contour);
        double w = min_rect.size.width, h = min_rect.size.height;
        double aspect_ratio = std::max(w, h) / std::min(w, h);
        if (aspect_ratio < 1.5 || aspect_ratio > 7.0) continue;
        if (std::abs(min_rect.angle) < 30 || std::abs(min_rect.angle) > 150) continue;
        double rect_area = w * h;
        double compactness = area / rect_area;
        if (compactness < 0.3) continue;
        valid_rects.push_back(min_rect);
    }

    // 选择最近的一对灯条
    cv::RotatedRect armor_rrect;
    bool armor_found = false;
    if (valid_rects.size() >= 2) {
        double best_distance = 1e9; int best_i = -1, best_j = -1;
        for (size_t i = 0; i < valid_rects.size(); ++i) {
            for (size_t j = i + 1; j < valid_rects.size(); ++j) {
                double d = cv::norm(valid_rects[i].center - valid_rects[j].center);
                if (d < best_distance) { best_distance = d; best_i = (int)i; best_j = (int)j; }
            }
        }
        if (best_i != -1 && best_j != -1) {
            cv::Point2f pts1[4], pts2[4];
            valid_rects[best_i].points(pts1);
            valid_rects[best_j].points(pts2);
            std::vector<cv::Point2f> all_points;
            for (int k = 0; k < 4; ++k) { all_points.push_back(pts1[k]); all_points.push_back(pts2[k]); }
            armor_rrect = cv::minAreaRect(all_points);

            // 仅用于可视化和裁剪：将旋转矩形的短边扩大为原来的2倍
            if (armor_rrect.size.width < armor_rrect.size.height) {
                armor_rrect.size.width *= 2.0f;
            } else {
                armor_rrect.size.height *= 2.0f;
            }

            armor_found = true;
        }
    }

    // 在 combinedMask 上画出所有灯条和装甲板框，发布到 /armor_preprocessed_cpp
    try {
        cv::Mat mask_vis; cv::cvtColor(combinedMask, mask_vis, cv::COLOR_GRAY2BGR);
        for (const auto& rect : valid_rects) {
            cv::Point2f pts[4]; rect.points(pts);
            for (int i = 0; i < 4; ++i) cv::line(mask_vis, pts[i], pts[(i+1)%4], cv::Scalar(255, 0, 0), 1);
        }
        if (armor_found) {
            cv::Point2f rpts[4]; armor_rrect.points(rpts);
            for (int i = 0; i < 4; ++i) cv::line(mask_vis, rpts[i], rpts[(i+1)%4], cv::Scalar(0, 255, 0), 4);
        }
        auto preproc_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, mask_vis).toImageMsg();
        pImpl->p_armor_preproc_pub->publish(*preproc_msg);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while publishing preprocessed image: %s", e.what());
    }

    // 进一步：在 C++ 端直接完成裁剪+遮挡灯条+二值化+letterbox，并发布到 /armor_crop_canvas
    if (armor_found) {
        try {
            // 1) 在原图上放大灯条 1.5 倍后用黑色遮挡
            cv::Mat masked = img.clone();
            for (const auto &r : valid_rects) {
                cv::RotatedRect er = r; // expanded rect
                er.size.width *= 1.5f;
                er.size.height *= 1.5f;
                cv::Point2f bpts[4]; er.points(bpts);
                std::vector<cv::Point> poly(4);
                for (int i = 0; i < 4; ++i) poly[i] = bpts[i];
                cv::fillConvexPoly(masked, poly, cv::Scalar(0, 0, 0));
            }

            // 2) 使用装甲板的旋转矩形从遮挡后的图中透视裁剪
            cv::Point2f src_pts[4]; armor_rrect.points(src_pts);
            int w_box = std::max(1, static_cast<int>(std::round(armor_rrect.size.width)));
            int h_box = std::max(1, static_cast<int>(std::round(armor_rrect.size.height)));
            std::vector<cv::Point2f> dst_pts = {
                {0.f, 0.f},
                {static_cast<float>(w_box - 1), 0.f},
                {static_cast<float>(w_box - 1), static_cast<float>(h_box - 1)},
                {0.f, static_cast<float>(h_box - 1)}
            };
            cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts.data());
            cv::Mat armor_warp;
            cv::warpPerspective(masked, armor_warp, M, cv::Size(w_box, h_box));

            // 3) 灰度化 + 以均值为阈值的二值化 + 开闭运算净化
            cv::Mat gray2; cv::cvtColor(armor_warp, gray2, cv::COLOR_BGR2GRAY);
            double mean_val = gray2.empty() ? 127.0 : cv::mean(gray2)[0];
            int thr = std::max(1, std::min(254, static_cast<int>(std::round(mean_val))));
            cv::Mat bin_img; cv::threshold(gray2, bin_img, thr, 255, cv::THRESH_BINARY);
            cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(bin_img, bin_img, cv::MORPH_OPEN, k3);
            cv::morphologyEx(bin_img, bin_img, cv::MORPH_CLOSE, k3);

            // 4) letterbox 到 20x28 的单通道 canvas
            const int target_h = 20, target_w = 28;
            int rh = bin_img.rows, rw = bin_img.cols;
            float scale = (rh > 0 && rw > 0) ? std::min(static_cast<float>(target_h) / rh,
                                                        static_cast<float>(target_w) / rw) : 1.f;
            int new_w = std::max(1, static_cast<int>(std::round(rw * scale)));
            int new_h = std::max(1, static_cast<int>(std::round(rh * scale)));
            cv::Mat resized; cv::resize(bin_img, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_AREA);
            cv::Mat canvas = cv::Mat::zeros(target_h, target_w, CV_8UC1);
            int top = (target_h - new_h) / 2;
            int left = (target_w - new_w) / 2;
            resized.copyTo(canvas(cv::Rect(left, top, new_w, new_h)));

            // 5) 发布到 /armor_crop_canvas，供 Python 模型订阅并进行推理
            auto crop_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, canvas).toImageMsg();
            pImpl->p_armor_crop_pub->publish(*crop_msg);
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to produce armor crop canvas: %s", e.what());
        }
    }

    // 发布检测结果（仅保留旋转矩形中心和尺寸作为 bbox）
    auto detections_msg = std::make_unique<vision_msgs::msg::Detection2DArray>();
    detections_msg->header = msg->header;
    if (armor_found) {
        vision_msgs::msg::Detection2D detection; detection.header = msg->header; detection.results.resize(1);
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis; hypothesis.hypothesis.class_id = "armor"; hypothesis.hypothesis.score = 1.0f; detection.results[0] = hypothesis;
        vision_msgs::msg::BoundingBox2D bbox;
        bbox.center.position.x = armor_rrect.center.x;
        bbox.center.position.y = armor_rrect.center.y;
        bbox.size_x = armor_rrect.size.width;
        bbox.size_y = armor_rrect.size.height;
        detection.bbox = bbox;
        detections_msg->detections.push_back(detection);
    }
    pImpl->p_armor_detections_pub->publish(std::move(detections_msg));
}

void HikvisionDriver::Impl::image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
    auto node = reinterpret_cast<HikvisionDriver *>(pUser);

    uint64_t dev_stamp = (uint64_t)pFrameInfo->nDevTimeStampHigh << 32ull | (uint64_t)pFrameInfo->nDevTimeStampLow;
    uint64_t host_stamp = pFrameInfo->nHostTimeStamp;

    // 发布 sensor_msgs::msg::Image
    auto p_img_raw_msg = std::make_unique<sensor_msgs::msg::Image>();
    if (pFrameInfo->nFrameLen > p_img_raw_msg->data.max_size()) {
        RCLCPP_ERROR_ONCE(node->get_logger(), "image bytes exceed max available size for sensor_msgs::msg::Image");
        return;
    }
    p_img_raw_msg->header.frame_id = node->pImpl->camera_name;
    p_img_raw_msg->header.stamp.sec = host_stamp / 1000ull;
    p_img_raw_msg->header.stamp.nanosec = (host_stamp % 1000ull) * 1000000ull;
    p_img_raw_msg->is_bigendian = false;
    p_img_raw_msg->width = pFrameInfo->nWidth;
    p_img_raw_msg->height = pFrameInfo->nHeight;
    if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerRG8) {
        p_img_raw_msg->step = pFrameInfo->nWidth * 1;
        p_img_raw_msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerBG8) {
        p_img_raw_msg->step = pFrameInfo->nWidth * 1;
        p_img_raw_msg->encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGR8) {
        p_img_raw_msg->step = pFrameInfo->nWidth * 1;
        p_img_raw_msg->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGB8) {
        p_img_raw_msg->step = pFrameInfo->nWidth * 1;
        p_img_raw_msg->encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_Mono8) {
        p_img_raw_msg->step = pFrameInfo->nWidth * 1;
        p_img_raw_msg->encoding = sensor_msgs::image_encodings::MONO8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_RGB8_Packed) {
        p_img_raw_msg->step = pFrameInfo->nWidth * 3;
        p_img_raw_msg->encoding = sensor_msgs::image_encodings::RGB8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BGR8_Packed) {
        p_img_raw_msg->step = pFrameInfo->nWidth * 3;
        p_img_raw_msg->encoding = sensor_msgs::image_encodings::BGR8;
    } else {
        RCLCPP_ERROR_ONCE(node->get_logger(), "unsupport pixel format for sensor_msgs::msg::Image: %d", (int)pFrameInfo->enPixelType);
        return; // 如果不支持则不发布
    }
    p_img_raw_msg->data.resize(p_img_raw_msg->height * p_img_raw_msg->step);
    if (pFrameInfo->nFrameLen < p_img_raw_msg->data.size()) {
        RCLCPP_ERROR(node->get_logger(), "nFrameLen < data.size() for sensor_msgs::msg::Image, len=%d", pFrameInfo->nFrameLen);
        return;
    }
    std::copy_n(pData, p_img_raw_msg->data.size(), p_img_raw_msg->data.data());

    // 发布到 /image_raw，触发订阅回调 image_callback
    node->pImpl->p_img_in_pub->publish(std::move(p_img_raw_msg));

    // 发布 HikImageInfo 
    auto p_info_msg = std::make_unique<hikvision_interface::msg::HikImageInfo>();
    {
        uint64_t now_ns = node->now().nanoseconds();
        p_info_msg->header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000ull);
        p_info_msg->header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ull);
    }
    p_info_msg->header.frame_id = node->pImpl->camera_name;
    p_info_msg->dev_stamp.sec = static_cast<uint32_t>(dev_stamp / 1000000000ull);
    p_info_msg->dev_stamp.nanosec = static_cast<uint32_t>(dev_stamp % 1000000000ull);
    p_info_msg->frame_num = 0u;
    p_info_msg->gain = static_cast<float>(node->pImpl->gain_param_);
    p_info_msg->exposure = static_cast<float>(node->pImpl->exposure_time_param_);
    p_info_msg->red = 0u; p_info_msg->green = 0u; p_info_msg->blue = 0u;
    node->pImpl->p_info_pub->publish(std::move(p_info_msg));
}


HikvisionDriver::HikvisionDriver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("hikvision_ros2_driver_node", options), pImpl(std::make_unique<Impl>()) {
    auto logger = get_logger();

    this->declare_parameter<std::string>("camera_name", "camera");
    this->declare_parameter<double>("exposure_time", 5000.0);
    this->declare_parameter<double>("gain", 0.0);
    this->declare_parameter<double>("frame_rate", 30.0);
    this->declare_parameter<std::string>("pixel_format", "PixelFormat_BayerRG8");
    this->declare_parameter<bool>("simulate", false);

    pImpl->camera_name = this->get_parameter("camera_name").as_string();
    pImpl->exposure_time_param_ = this->get_parameter("exposure_time").as_double();
    pImpl->gain_param_ = this->get_parameter("gain").as_double();
    pImpl->frame_rate_param_ = this->get_parameter("frame_rate").as_double();
    pImpl->pixel_format_param_ = this->get_parameter("pixel_format").as_string();
    pImpl->simulate_mode = this->get_parameter("simulate").as_bool();

    pImpl->param_cb_handle = this->add_on_set_parameters_callback([
        this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
            rcl_interfaces::msg::SetParametersResult result; result.successful = true; std::string failed_msg;
            for (const auto &p : params) {
                try {
                    if (p.get_name() == "exposure_time") {
                        double v = p.as_double(); pImpl->exposure_time_param_ = v;
                        if (pImpl->handle) {
                            int ret = MV_CC_SetFloatValue(pImpl->handle, "ExposureTime", (float)v);
                            if (MV_OK != ret) {
                                result.successful = false;
                                failed_msg = "Failed to set ExposureTime";
                                break;
                            }
                        }
                    } else if (p.get_name() == "gain") {
                        double g = p.as_double(); pImpl->gain_param_ = g;
                        if (pImpl->handle) {
                            int ret = MV_CC_SetFloatValue(pImpl->handle, "Gain", (float)g);
                            if (MV_OK != ret) {
                                result.successful = false;
                                failed_msg = "Failed to set Gain";
                                break;
                            }
                        }
                    } else if (p.get_name() == "frame_rate") {
                        double f = p.as_double(); pImpl->frame_rate_param_ = f;
                        if (pImpl->handle) {
                            int ret = MV_CC_SetFloatValue(pImpl->handle, "AcquisitionFrameRate", (float)f);
                            if (MV_OK != ret) {
                                result.successful = false;
                                failed_msg = "Failed to set AcquisitionFrameRate";
                                break;
                            }
                        }
                    }
                } catch (...) {
                    result.successful = false;
                    failed_msg = "Exception while applying parameter";
                }
            }
            if (!result.successful) {
                result.reason = failed_msg;
            }
            return result;
    });
    (void)pImpl->param_cb_handle;

    RCLCPP_INFO(logger, "trying to open camera: '%s'", pImpl->camera_name.c_str());
    RCLCPP_INFO(logger, "Initial parameters - Exposure: %f, Gain: %f, Frame Rate: %f, Format: %s",
                pImpl->exposure_time_param_, pImpl->gain_param_, pImpl->frame_rate_param_, pImpl->pixel_format_param_.c_str());

    // 初始化发布与订阅
    rclcpp::QoS qos(10);
    // 内部原始图像发布器，发布到 /image_raw，供本节点订阅处理（对外即原图）
    pImpl->p_img_in_pub = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", qos);
    pImpl->p_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", qos, std::bind(&HikvisionDriver::image_callback, this, std::placeholders::_1));
    // 检测结果
    pImpl->p_armor_detections_pub = this->create_publisher<vision_msgs::msg::Detection2DArray>("/armor_detections", qos);
    // 预处理调试图
    pImpl->p_armor_preproc_pub = this->create_publisher<sensor_msgs::msg::Image>("/armor_preprocessed_cpp", qos);
    // 预处理后的 20x28 canvas
    pImpl->p_armor_crop_pub = this->create_publisher<sensor_msgs::msg::Image>("/armor_crop_canvas", qos);
    // 信息发布
    pImpl->p_info_pub = create_publisher<HikImageInfo>("info", qos);

    // 相机初始化与抓图回调注册
    MV_CC_DEVICE_INFO_LIST stDeviceList; memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    MV_CHECK(logger, MV_CC_EnumDevices, MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

    bool first_device_assigned = false;
    for (uint32_t nDeviceId = 0; nDeviceId < stDeviceList.nDeviceNum; nDeviceId++) {
        auto *pDeviceInfo = stDeviceList.pDeviceInfo[nDeviceId];
        const char *pUserDefinedName = nullptr;
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            pUserDefinedName = (const char *)pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
            if (pUserDefinedName) {
                int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
                RCLCPP_INFO(logger, "found device: [%s]: GIGE, %s, %d.%d.%d.%d", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stGigEInfo.chModelName, nIp1, nIp2, nIp3, nIp4);
            } else {
                RCLCPP_INFO(logger, "found device: [unnamed]: GIGE, %s", pDeviceInfo->SpecialInfo.stGigEInfo.chModelName);
            }
            if (!first_device_assigned && pUserDefinedName && pImpl->camera_name == "camera") {
                pImpl->camera_name = pUserDefinedName; first_device_assigned = true;
                RCLCPP_INFO(logger, "no camera_name given, auto-selecting device '%s'", pImpl->camera_name.c_str());
            }
            if (pUserDefinedName && pImpl->camera_name == pUserDefinedName) {
                RCLCPP_INFO(logger, "camera_name matches desired name '%s' -> will try to open this device", pUserDefinedName);
            }
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            pUserDefinedName = (const char *)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
            if (pUserDefinedName) {
                RCLCPP_INFO(logger, "found device: [%s]: USB, %s", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
            } else {
                RCLCPP_INFO(logger, "found device: [unnamed]: USB, %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
            }
            if (!first_device_assigned && pUserDefinedName && pImpl->camera_name == "camera") {
                pImpl->camera_name = pUserDefinedName; first_device_assigned = true;
                RCLCPP_INFO(logger, "no camera_name given, auto-selecting device '%s'", pImpl->camera_name.c_str());
            }
            if (pUserDefinedName && pImpl->camera_name == pUserDefinedName) {
                RCLCPP_INFO(logger, "camera_name matches desired name '%s' -> will try to open this device", pUserDefinedName);
            }
        } else {
            RCLCPP_WARN(logger, "type(%d) not support", pDeviceInfo->nTLayerType);
        }
        if (pUserDefinedName && pImpl->camera_name == pUserDefinedName) {
            MV_CHECK(logger, MV_CC_CreateHandle, &pImpl->handle, pDeviceInfo);
            MV_CHECK(logger, MV_CC_OpenDevice, pImpl->handle);
            MV_CHECK(logger, MV_CC_RegisterImageCallBackEx, pImpl->handle, &HikvisionDriver::Impl::image_callback_ex, this);
            MV_CHECK(logger, MV_CC_StartGrabbing, pImpl->handle);
            break;
        }
    }
    if (pImpl->handle == nullptr && stDeviceList.nDeviceNum > 0) {
        RCLCPP_WARN(logger, "camera '%s' not found by name; falling back to first detected device", pImpl->camera_name.c_str());
        auto *pFirstDev = stDeviceList.pDeviceInfo[0];
        const char *firstName = nullptr;
        if (pFirstDev->nTLayerType == MV_GIGE_DEVICE) firstName = (const char *)pFirstDev->SpecialInfo.stGigEInfo.chUserDefinedName;
        else if (pFirstDev->nTLayerType == MV_USB_DEVICE) firstName = (const char *)pFirstDev->SpecialInfo.stUsb3VInfo.chUserDefinedName;
        if (firstName) { pImpl->camera_name = firstName; RCLCPP_INFO(logger, "auto-selected first device name '%s'", pImpl->camera_name.c_str()); }
        else { pImpl->camera_name = "first_device"; RCLCPP_INFO(logger, "auto-selected first device (unnamed)"); }
        MV_CHECK(logger, MV_CC_CreateHandle, &pImpl->handle, pFirstDev);
        MV_CHECK(logger, MV_CC_OpenDevice, pImpl->handle);
        MV_CHECK(logger, MV_CC_RegisterImageCallBackEx, pImpl->handle, &HikvisionDriver::Impl::image_callback_ex, this);
        MV_CHECK(logger, MV_CC_StartGrabbing, pImpl->handle);
    }
    if (pImpl->handle == nullptr) {
        RCLCPP_ERROR(logger, "camera '%s' not found", pImpl->camera_name.c_str());
        if (pImpl->simulate_mode) {
            RCLCPP_INFO(logger, "simulate mode enabled: publishing synthetic messages");
            auto period = std::chrono::duration<double>(1.0 / std::max(1.0, pImpl->frame_rate_param_));
            pImpl->sim_timer = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                [this]() {
                    auto img = std::make_unique<sensor_msgs::msg::Image>();
                    img->header.stamp = this->now(); img->header.frame_id = pImpl->camera_name;
                    img->height = 240; img->width = 320; img->is_bigendian = false; img->step = img->width; img->encoding = sensor_msgs::image_encodings::MONO8;
                    img->data.resize(img->height * img->step);
                    for (size_t y = 0; y < (size_t)img->height; ++y) for (size_t x = 0; x < (size_t)img->width; ++x) img->data[y * img->step + x] = static_cast<unsigned char>((x + y) % 256);
                    pImpl->p_img_in_pub->publish(std::move(img));

                    auto info = std::make_unique<HikImageInfo>();
                    info->header.stamp = this->now(); info->header.frame_id = pImpl->camera_name; info->dev_stamp.nanosec = 0;
                    pImpl->p_info_pub->publish(std::move(info));
                });
        }
    }
}

HikvisionDriver::~HikvisionDriver() {
    if (!pImpl) {
        return;
    }
    if (pImpl->handle == nullptr) {
        return;
    }
    auto logger = get_logger();
    MV_CHECK(logger, MV_CC_StopGrabbing, pImpl->handle);
    MV_CHECK(logger, MV_CC_CloseDevice, pImpl->handle);
    MV_CHECK(logger, MV_CC_DestroyHandle, pImpl->handle);
    pImpl->handle = nullptr;
}

}  // namespace hikvision_ros2_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hikvision_ros2_driver::HikvisionDriver);

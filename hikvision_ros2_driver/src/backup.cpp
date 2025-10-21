// hikvision_ros2_driver.cpp
// ... (保留之前的 include 和 define)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h> // 添加 cv_bridge
#include <vision_msgs/msg/detection2_d_array.hpp> // 用于发布检测结果
#include <opencv2/opencv.hpp> // 添加 OpenCV
#include <cstring>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cstdint>
#include <algorithm>
#include "hikvision_interface/msg/hik_image_info.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// ... (保留之前的 MV_CHECK 定义)

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p_img_raw_pub;
    // 1. 添加装甲板检测结果发布器
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr p_armor_detections_pub;
    std::string camera_name;

    rclcpp::TimerBase::SharedPtr sim_timer;
    bool simulate_mode = false;

    double exposure_time_param_;
    double gain_param_;
    double frame_rate_param_;
    std::string pixel_format_param_;
    void* handle = nullptr;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle;

    // 2. 添加图像订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr p_img_sub;

    static void image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
};

// 3. 实现图像回调函数，包含装甲板识别逻辑
void HikvisionDriver::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received image for armor detection"); // 调试信息

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // 假设输入是 BGR
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Mat img = cv_ptr->image.clone(); // 获取 OpenCV Mat 格式图像
    Mat result = img.clone(); // 每帧都创建一个新的结果图

    // --- 你的 OpenCV 装甲板识别逻辑 ---
    Mat kernel3 = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel5 = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat kernel7 = getStructuringElement(MORPH_RECT, Size(7, 7));
    Mat kernel9 = getStructuringElement(MORPH_RECT, Size(9, 9));

    // 预处理：灰度化 + 高斯模糊
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // 提取高亮区域（固定阈值）
    Mat brightMask;
    threshold(gray, brightMask, 220, 255, THRESH_BINARY); // 灰度值 > 220 的为白色
    morphologyEx(brightMask, brightMask, MORPH_OPEN, kernel7);
    morphologyEx(brightMask, brightMask, MORPH_CLOSE, kernel7);
    morphologyEx(brightMask, brightMask, MORPH_DILATE, kernel7);
    morphologyEx(brightMask, brightMask, MORPH_DILATE, kernel7);

    // 提取蓝色区域（HSV）
    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);
    Scalar lowerBlue(160, 200, 200);
    Scalar upperBlue(180, 255, 255);
    Mat blueMask;
    inRange(hsv, lowerBlue, upperBlue, blueMask);

    // 组合掩码
    Mat combinedMask = brightMask; // 使用膨胀后的核心作为最终掩码
    // 最终掩码是combinedMask

    // 查找轮廓
    vector<vector<Point>> contours;
    findContours(combinedMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 筛选轮廓：多特征联合筛选
    vector<RotatedRect> valid_rects;
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area < 50 ) continue; // 面积范围

        RotatedRect min_rect = minAreaRect(contour);

        // 长宽比筛选（1.5 ~ 5）
        double w = min_rect.size.width;
        double h = min_rect.size.height;
        double aspect_ratio = max(w, h) / min(w, h);
        if (aspect_ratio < 1.5 || aspect_ratio > 5) continue;

        // 方向筛选（45 ~ 135 度）
        if (abs(min_rect.angle) < 45 || abs(min_rect.angle) > 135) continue;

        // 紧密度筛选（面积 / 外接矩形面积 > 0.5）
        double rect_area = w * h;
        double compactness = area / rect_area;
        if (compactness < 0.3) continue;

        valid_rects.push_back(min_rect);
    }

    // 寻找“最匹配”的一对灯柱
    Rect armorBox; // 用于存储最终装甲板框
    bool armor_found = false;
    if (valid_rects.size() >= 2) {
        double best_distance = 10000;
        int best_i = -1, best_j = -1;

        for (size_t i = 0; i < valid_rects.size(); ++i) {
            for (size_t j = i + 1; j < valid_rects.size(); ++j) {
                Point2f center1 = valid_rects[i].center;
                Point2f center2 = valid_rects[j].center;

                double distance = norm(center1 - center2);
                double y_diff = abs(center1.y - center2.y);

                // Y轴对齐筛选
                if (y_diff > 100) continue;

                // 选择距离最近的一对
                if (distance < best_distance) {
                    best_distance = distance;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        if (best_i != -1 && best_j != -1) {
            // 计算装甲板区域（包围两个灯柱的矩形）
            Point2f pts1[4], pts2[4];
            valid_rects[best_i].points(pts1); // 获取灯柱1的4个顶点
            valid_rects[best_j].points(pts2); // 获取灯柱2的4个顶点

            // 将两个灯柱的8个顶点合并
            vector<Point2f> all_points;
            for (int k = 0; k < 4; ++k) {
                all_points.push_back(pts1[k]);
                all_points.push_back(pts2[k]);
            }

            // 计算包围所有点的最小外接矩形（即装甲板区域）
            armorBox = boundingRect(all_points);
            armor_found = true;
        }
    }
    // --- 识别逻辑结束 ---

    // 4. 发布检测结果
    auto detections_msg = std::make_unique<vision_msgs::msg::Detection2DArray>();
    detections_msg->header = msg->header; // 使用原始图像的时间戳和 frame_id

    if (armor_found) {
        vision_msgs::msg::Detection2D detection;
        detection.header = msg->header;
        detection.results.resize(1); // 假设每次只检测一个装甲板
        // detection.results[0].id = "armor_plate"; // 可选，设置 ID
        // detection.results[0].score = 0.95; // 可选，设置置信度，需要算法提供

        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.id = "armor_plate"; // 设置类别 ID
        hypothesis.score = 1.0; // 简单示例，可以传入实际置信度
        detection.results[0] = hypothesis;

        vision_msgs::msg::BoundingBox2D bbox;
        bbox.center.position.x = armorBox.x + armorBox.width / 2.0;
        bbox.center.position.y = armorBox.y + armorBox.height / 2.0;
        bbox.size_x = armorBox.width;
        bbox.size_y = armorBox.height;
        detection.bbox = bbox;

        detections_msg->detections.push_back(detection);

        // 5. 在图像上绘制框（可选，用于调试或可视化）
        rectangle(result, armorBox, Scalar(0, 255, 0), 2); // 装甲板框
    }

    // 发布检测结果
    pImpl->p_armor_detections_pub->publish(std::move(detections_msg));

    // (可选) 发布带框的图像用于调试
    sensor_msgs::msg::Image::SharedPtr result_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, result).toImageMsg();
    pImpl->p_img_raw_pub->publish(result_msg); // 注意：这里覆盖了原始图像发布，如果需要原始图像，需要创建另一个发布器
}

void HikvisionDriver::Impl::image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
    auto node = reinterpret_cast<HikvisionDriver *>(pUser);

    // RCLCPP_INFO(node->get_logger(), "Image callback called! Width: %d, Height: %d, PixelType: %d", pFrameInfo->nWidth, pFrameInfo->nHeight, (int)pFrameInfo->enPixelType);

    uint64_t dev_stamp = (uint64_t)pFrameInfo->nDevTimeStampHigh << 32ull | (uint64_t)pFrameInfo->nDevTimeStampLow;
    uint64_t host_stamp = pFrameInfo->nHostTimeStamp;

    // --- 发布 sensor_msgs::msg::Image ---
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

    // 6. 发布图像消息，这将触发 image_callback
    node->pImpl->p_img_sub->handle_message(std::move(p_img_raw_msg), node->get_graph_interface()->get_publisher_handle(node->pImpl->p_img_raw_pub));

    // --- 发布 HikImageInfo (原有部分) ---
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
    p_info_msg->red = 0u;
    p_info_msg->green = 0u;
    p_info_msg->blue = 0u;
    node->pImpl->p_info_pub->publish(std::move(p_info_msg));
}


HikvisionDriver::HikvisionDriver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("hikvision_ros2_driver_node", options), pImpl(std::make_unique<Impl>()) {
    auto logger = get_logger();

    // ... (保留之前的参数声明和获取) ...
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

    // ... (保留之前的参数回调注册) ...
    pImpl->param_cb_handle = this->add_on_set_parameters_callback([
        this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            std::string failed_msg;
            for (const auto &p : params) {
                try {
                    if (p.get_name() == "exposure_time") {
                        double v = p.as_double();
                        pImpl->exposure_time_param_ = v;
                        if (pImpl->handle) {
                            int ret = MV_CC_SetFloatValue(pImpl->handle, "ExposureTime", (float)v);
                            if (MV_OK != ret) {
                                result.successful = false; failed_msg = "Failed to set ExposureTime"; break;
                            }
                        }
                    } else if (p.get_name() == "gain") {
                        double g = p.as_double();
                        pImpl->gain_param_ = g;
                        if (pImpl->handle) {
                            int ret = MV_CC_SetFloatValue(pImpl->handle, "Gain", (float)g);
                            if (MV_OK != ret) {
                                result.successful = false; failed_msg = "Failed to set Gain"; break;
                            }
                        }
                    } else if (p.get_name() == "frame_rate") {
                        double f = p.as_double();
                        pImpl->frame_rate_param_ = f;
                        if (pImpl->handle) {
                            int ret = MV_CC_SetFloatValue(pImpl->handle, "AcquisitionFrameRate", (float)f);
                            if (MV_OK != ret) {
                                result.successful = false; failed_msg = "Failed to set AcquisitionFrameRate"; break;
                            }
                        }
                    }
                } catch (...) {
                    result.successful = false;
                    failed_msg = "Exception while applying parameter";
                }
            }
        if (!result.successful) result.reason = failed_msg;
            return result;
    });
    (void)pImpl->param_cb_handle;

    RCLCPP_INFO(logger, "trying to open camera: '%s'", pImpl->camera_name.c_str());
    RCLCPP_INFO(logger, "Initial parameters - Exposure: %f, Gain: %f, Frame Rate: %f, Format: %s",
                pImpl->exposure_time_param_, pImpl->gain_param_, pImpl->frame_rate_param_, pImpl->pixel_format_param_.c_str());

    // 7. 初始化 sensor_msgs::msg::Image 发布器和订阅器
    rclcpp::QoS qos(10);
    pImpl->p_img_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("/image_raw_with_armor", qos); // 修改话题名，或创建新的发布器用于原始图像
    pImpl->p_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", // 订阅由 SDK 回调发布的图像
        qos,
        std::bind(&HikvisionDriver::image_callback, this, std::placeholders::_1) // 绑定到新的回调函数
    );

    // 8. 初始化装甲板检测结果发布器
    pImpl->p_armor_detections_pub = this->create_publisher<vision_msgs::msg::Detection2DArray>("/armor_detections", qos);

    // 9. 初始化 HikImageInfo 发布器
    pImpl->p_info_pub = create_publisher<HikImageInfo>("info", qos);

    // ... (保留 SDK 初始化代码，但需要修改回调) ...
    // (SDK 初始化代码保持不变，但 image_callback_ex 只负责将 SDK 数据转换为 ROS Image 消息并发布)
    // 详细 SDK 代码省略，但注意在 image_callback_ex 中调用 p_img_raw_pub->publish
    // ... (SDK 代码结束) ...
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
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
                pImpl->camera_name = pUserDefinedName;
                first_device_assigned = true;
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
                pImpl->camera_name = pUserDefinedName;
                first_device_assigned = true;
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

            // 应用初始参数 (示例)
            // MV_CHECK(logger, MV_CC_SetEnumValue, pImpl->handle, "ExposureMode", MV_EXPOSURE_MODE_TIMED);
            // MV_CHECK(logger, MV_CC_SetFloatValue, pImpl->handle, "ExposureTime", pImpl->exposure_time_param_);
            // MV_CHECK(logger, MV_CC_SetEnumValue, pImpl->handle, "GainAuto", MV_GAIN_MODE_OFF);
            // MV_CHECK(logger, MV_CC_SetFloatValue, pImpl->handle, "Gain", pImpl->gain_param_);
            // MV_CHECK(logger, MV_CC_SetEnumValue, pImpl->handle, "AcquisitionFrameRateMode", MV_ACQ_FRAMERATE_MODE_STANDARD);
            // MV_CHECK(logger, MV_CC_SetFloatValue, pImpl->handle, "AcquisitionFrameRate", pImpl->frame_rate_param_);
            // int nPixelType = PixelType_Gvsp_BayerRG8;
            // if (pImpl->pixel_format_param_ == "PixelFormat_BayerRG8") nPixelType = PixelType_Gvsp_BayerRG8;
            // else if (pImpl->pixel_format_param_ == "PixelFormat_Mono8") nPixelType = PixelType_Gvsp_Mono8;
            // else if (pImpl->pixel_format_param_ == "PixelFormat_RGB8Packed") nPixelType = PixelType_Gvsp_RGB8_Packed;
            // else if (pImpl->pixel_format_param_ == "PixelFormat_BGR8Packed") nPixelType = PixelType_Gvsp_BGR8_Packed;
            // MV_CHECK(logger, MV_CC_SetEnumValue, pImpl->handle, "PixelFormat", nPixelType);

            MV_CHECK(logger, MV_CC_RegisterImageCallBackEx, pImpl->handle, &HikvisionDriver::Impl::image_callback_ex,
                     this);
            MV_CHECK(logger, MV_CC_StartGrabbing, pImpl->handle);
            break;
        }
    }
    if (pImpl->handle == nullptr && stDeviceList.nDeviceNum > 0) {
        RCLCPP_WARN(logger, "camera '%s' not found by name; falling back to first detected device", pImpl->camera_name.c_str());
        auto *pFirstDev = stDeviceList.pDeviceInfo[0];
        const char *firstName = nullptr;
        if (pFirstDev->nTLayerType == MV_GIGE_DEVICE) {
            firstName = (const char *)pFirstDev->SpecialInfo.stGigEInfo.chUserDefinedName;
        } else if (pFirstDev->nTLayerType == MV_USB_DEVICE) {
            firstName = (const char *)pFirstDev->SpecialInfo.stUsb3VInfo.chUserDefinedName;
        }
        if (firstName) {
            pImpl->camera_name = firstName;
            RCLCPP_INFO(logger, "auto-selected first device name '%s'", pImpl->camera_name.c_str());
        } else {
            pImpl->camera_name = "first_device";
            RCLCPP_INFO(logger, "auto-selected first device (unnamed)");
        }
        MV_CHECK(logger, MV_CC_CreateHandle, &pImpl->handle, pFirstDev);
        MV_CHECK(logger, MV_CC_OpenDevice, pImpl->handle);
        MV_CHECK(logger, MV_CC_RegisterImageCallBackEx, pImpl->handle, &HikvisionDriver::Impl::image_callback_ex,
                 this);
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
                    img->header.stamp = this->now();
                    img->header.frame_id = pImpl->camera_name;
                    img->height = 240;
                    img->width = 320;
                    img->is_bigendian = false;
                    img->step = img->width;
                    img->encoding = sensor_msgs::image_encodings::MONO8;
                    img->data.resize(img->height * img->step);
                    for (size_t y = 0; y < (size_t)img->height; ++y) {
                        for (size_t x = 0; x < (size_t)img->width; ++x) {
                            img->data[y * img->step + x] = static_cast<unsigned char>((x + y) % 256);
                        }
                    }
                    // 在模拟模式下也需要发布图像以触发回调
                    pImpl->p_img_sub->handle_message(std::move(img), this->get_graph_interface()->get_publisher_handle(pImpl->p_img_raw_pub));

                    auto info = std::make_unique<HikImageInfo>();
                    info->header.stamp = this->now();
                    info->header.frame_id = pImpl->camera_name;
                    info->dev_stamp.nanosec = 0;
                    pImpl->p_info_pub->publish(std::move(info));
                });
        }
    }
}

HikvisionDriver::~HikvisionDriver() {
    if (!pImpl) return;
    if (pImpl->handle == nullptr) return;
    auto logger = get_logger();

    MV_CHECK(logger, MV_CC_StopGrabbing, pImpl->handle);
    MV_CHECK(logger, MV_CC_CloseDevice, pImpl->handle);
    MV_CHECK(logger, MV_CC_DestroyHandle, pImpl->handle);
    pImpl->handle = nullptr;
}

}  // namespace hikvision_ros2_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hikvision_ros2_driver::HikvisionDriver);
#include <chrono>
#include <memory>
#include <mutex>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "MvCameraControl.h"

using namespace std::chrono_literals;

class HikCameraNode : public rclcpp::Node {
public:
    HikCameraNode() : Node("hik_camera") {
        // 1. 参数声明
        this->declare_parameter("topic_name", "/image_raw");
        this->declare_parameter("exposure_time", 10000.0);
        this->declare_parameter("gain", 10.0);

        std::string topic = this->get_parameter("topic_name").as_string();
        // 这里改回你之前最稳、能编译通过的写法
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);

        // 2. 初始化 SDK
        int ret = MV_CC_Initialize();
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "MVS SDK 初始化失败!");
            return;
        }

        // 3. 枚举设备
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (ret != MV_OK || stDeviceList.nDeviceNum == 0) {
            RCLCPP_ERROR(this->get_logger(), "未找到海康相机，请检查连接!");
            return;
        }

        // 4. 创建句柄并打开设备
        ret = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);
        if (ret != MV_OK) { RCLCPP_ERROR(this->get_logger(), "创建相机句柄失败"); return; }
        
        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) { RCLCPP_ERROR(this->get_logger(), "打开相机失败"); return; }

        // 5. 设置基础参数
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        MV_CC_SetFloatValue(handle_, "ExposureTime", this->get_parameter("exposure_time").as_double());
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        MV_CC_SetFloatValue(handle_, "Gain", this->get_parameter("gain").as_double());

        // 6. 注册图像回调函数
        ret = MV_CC_RegisterImageCallBackEx(handle_, &HikCameraNode::imageCallback, this);
        if (ret != MV_OK) { RCLCPP_ERROR(this->get_logger(), "注册回调失败"); return; }

        // 7. 开始取流
        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) { RCLCPP_ERROR(this->get_logger(), "开始取流失败"); return; }

        // 8. 注册参数动态回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "海康相机连接成功，正在发布图像到: %s", topic.c_str());
    }

    ~HikCameraNode() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
        MV_CC_Finalize();
    }

private:
    // 参数回调函数
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        if (!handle_) {
            result.successful = false;
            result.reason = "相机未连接，无法设置参数";
            return result;
        }

        for (const auto &param : parameters) {
            if (param.get_name() == "exposure_time") {
                double val = param.as_double();
                MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
                int ret = MV_CC_SetFloatValue(handle_, "ExposureTime", val);
                if (ret != MV_OK) {
                    result.successful = false;
                    result.reason = "设置曝光失败，错误码: " + std::to_string(ret);
                    return result;
                }
                RCLCPP_INFO(this->get_logger(), "成功更新曝光时间为: %.2f", val);
            } 
            else if (param.get_name() == "gain") {
                double val = param.as_double();
                MV_CC_SetEnumValue(handle_, "GainAuto", 0);
                int ret = MV_CC_SetFloatValue(handle_, "Gain", val);
                if (ret != MV_OK) {
                    result.successful = false;
                    result.reason = "设置增益失败，错误码: " + std::to_string(ret);
                    return result;
                }
                RCLCPP_INFO(this->get_logger(), "成功更新增益为: %.2f", val);
            }
        }
        return result;
    }

      // 静态图像回调
    static void imageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
        HikCameraNode* node = static_cast<HikCameraNode*>(pUser);
        node->publishImage(pData, pFrameInfo);
    }

    void publishImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo) {
        std::lock_guard<std::mutex> lock(mutex_);

        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_link";
        msg.height = pFrameInfo->nHeight;
        msg.width = pFrameInfo->nWidth;
        msg.is_bigendian = false;

        if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerRG8) {
            msg.encoding = "bayer_rggb8"; 
            msg.step = msg.width;
        } else if (pFrameInfo->enPixelType == PixelType_Gvsp_Mono8) {
            msg.encoding = "mono8";
            msg.step = msg.width;
        } else {
            msg.encoding = "mono8";
            msg.step = msg.width;
        }

        int dataSize = pFrameInfo->nFrameLen;
        msg.data.resize(dataSize);
        memcpy(msg.data.data(), pData, dataSize);

        pub_->publish(msg);
    }

    // 所有的变量声明必须放在这里
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    void* handle_ = nullptr;
    std::mutex mutex_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}
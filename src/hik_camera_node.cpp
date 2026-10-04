#include <chrono>
#include <memory>
#include <mutex>
#include <cstring>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "MvCameraControl.h"

using namespace std::chrono_literals;

class HikCameraNode : public rclcpp::Node {
public:
    HikCameraNode() : Node("hik_camera") {
        this->declare_parameter("topic_name", "/image_raw");
        this->declare_parameter("exposure_time", 10000.0);
        this->declare_parameter("gain", 10.0);
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("pixel_format", "BayerRG8");
        this->declare_parameter("serial_number", ""); // refresh

        std::string topic = this->get_parameter("topic_name").as_string();
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);

        int ret = MV_CC_Initialize();
        if (ret != MV_OK) { 
            RCLCPP_ERROR(this->get_logger(), "MVS SDK 初始化失败!");
            return;
        }

        connectCamera();

        reconnect_timer_ = this->create_wall_timer(
            1s, std::bind(&HikCameraNode::checkConnectionCallback, this));

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "海康相机节点已启动，等待连接...");
    }

    ~HikCameraNode() {
        disconnectCamera();
        MV_CC_Finalize();
    }

private:
    void connectCamera() {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (ret != MV_OK || stDeviceList.nDeviceNum == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "未找到海康相机，等待重连...");
            is_connected_ = false;
            return;
        }

        std::string target_serial = this->get_parameter("serial_number").as_string();
        int target_index = 0; 

        if (!target_serial.empty()) {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
                std::string serial;
                if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
                    serial = (char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber;
                } else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
                    serial = (char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber;
                }
                if (serial == target_serial) {
                    target_index = i;
                    break;
                }
            }
        }
        ret = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[target_index]);
        if (ret != MV_OK) return;
        
        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "打开相机失败");
            disconnectCamera(); // <--- 统一调用，不要手写 DestroyHandle
            return;
        }   

        double exp = this->get_parameter("exposure_time").as_double();
        double gain = this->get_parameter("gain").as_double();
        double fps = this->get_parameter("frame_rate").as_double();
        std::string pixel_format = this->get_parameter("pixel_format").as_string();

        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        MV_CC_SetFloatValue(handle_, "ExposureTime", exp);
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        MV_CC_SetFloatValue(handle_, "Gain", gain);
        
        MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", fps);
        
        MV_CC_SetEnumValueByString(handle_, "PixelFormat", pixel_format.c_str());

        ret = MV_CC_RegisterImageCallBackEx(handle_, &HikCameraNode::imageCallback, this);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "注册回调失败");
            disconnectCamera(); // <--- 统一调用！
            return;
        }

        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "开始取流失败");
            disconnectCamera(); // <--- 统一调用！
            return;
        }

        is_connected_ = true;
        last_frame_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "海康相机连接成功，已恢复参数 (曝光:%.2f, 增益:%.2f, 帧率:%.2f, 格式:%s)", 
                    exp, gain, fps, pixel_format.c_str());
    }

    void disconnectCamera() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        is_connected_ = false;
    }

    void checkConnectionCallback() {
        if (!is_connected_) {
            connectCamera();
            return;
        }

        auto now = this->now();
        auto elapsed = (now - last_frame_time_).seconds();
        if (elapsed > 3.0) {
            RCLCPP_ERROR(this->get_logger(), "超过 3 秒未收到图像数据，判定相机掉线！正在尝试重连...");
            disconnectCamera();
            connectCamera();
        }
    }

    static void imageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
        HikCameraNode* node = static_cast<HikCameraNode*>(pUser);
        node->publishImage(pData, pFrameInfo);
    }

    void publishImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_connected_) return;

        last_frame_time_ = this->now();

        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_link";
        msg.height = pFrameInfo->nHeight;
        msg.width = pFrameInfo->nWidth;
        msg.is_bigendian = false;

        std::string current_format = this->get_parameter("pixel_format").as_string();
        if (current_format == "BayerRG8") {
            msg.encoding = "bayer_rggb8"; 
            msg.step = msg.width;
        } else if (current_format == "Mono8") {
            msg.encoding = "mono8";
            msg.step = msg.width;
        } else if (current_format == "RGB8") {
            msg.encoding = "rgb8";
            msg.step = msg.width * 3;
        } else {
            msg.encoding = "mono8";
            msg.step = msg.width;
        }

        int dataSize = pFrameInfo->nFrameLen;
        msg.data.resize(dataSize);
        memcpy(msg.data.data(), pData, dataSize);

        pub_->publish(msg);
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        std::lock_guard<std::mutex> lock(mutex_);
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
            else if (param.get_name() == "frame_rate") {
                double val = param.as_double();
                MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
                int ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", val);
                if (ret != MV_OK) {
                    result.successful = false;
                    result.reason = "设置帧率失败，错误码: " + std::to_string(ret);
                    return result;
                }
                RCLCPP_INFO(this->get_logger(), "成功更新帧率为: %.2f", val);
            }
            else if (param.get_name() == "pixel_format") {
                std::string val = param.as_string();
                // 切换像素格式需要先暂停取流
                MV_CC_StopGrabbing(handle_);
                int ret = MV_CC_SetEnumValueByString(handle_, "PixelFormat", val.c_str());
                MV_CC_StartGrabbing(handle_);
                
                if (ret != MV_OK) {
                    result.successful = false;
                    result.reason = "设置像素格式失败，错误码: " + std::to_string(ret);
                    return result;
                }
                RCLCPP_INFO(this->get_logger(), "成功更新像素格式为: %s", val.c_str());
            }
        }
        return result;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    void* handle_ = nullptr;
    std::mutex mutex_;
    bool is_connected_ = false;
    rclcpp::Time last_frame_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}
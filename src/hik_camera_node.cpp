#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
    class HikCameraNode : public rclcpp::Node
    {
    public:
        explicit HikCameraNode(const rclcpp::NodeOptions &options) : Node("hik_camera", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");
            // Load camera info
            camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
            camera_info_manager_ =
                std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
            auto camera_info_url =
                this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
            if (camera_info_manager_->validateURL(camera_info_url))
            {
                camera_info_manager_->loadCameraInfo(camera_info_url);
                camera_info_msg_ = camera_info_manager_->getCameraInfo();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
            }

            // set image_msg info
            this->image_msg_.set__encoding("rgb8");
            this->image_msg_.header.set__frame_id("camera_optical_frame");
            this->image_msg_.set__width(camera_info_msg_.width);
            this->image_msg_.set__height(camera_info_msg_.height);
            this->image_msg_.set__step(camera_info_msg_.width * 3);

            // declare parameters
            bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
            auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
            camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
            declareParameters();
            params_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

            // set 1hz watchdog
            using namespace std::chrono_literals;
            watchdog_timer_ = create_wall_timer(1s, std::bind(&HikCameraNode::watchdog_callback, this));
        }

        ~HikCameraNode() override
        {
            if (MV_CC_IsDeviceConnected(camera_handle_) == MV_OK)
            {
                MV_CC_StopGrabbing(camera_handle_);
                MV_CC_CloseDevice(camera_handle_);
                MV_CC_DestroyHandle(&camera_handle_);
            }
            RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
        }

    private:
        void declareParameters()
        {
            rcl_interfaces::msg::ParameterDescriptor param_desc;
            MVCC_FLOATVALUE f_value;
            param_desc.integer_range.resize(1);
            param_desc.integer_range[0].step = 1;
            // Exposure time
            param_desc.description = "Exposure time in microseconds";
            param_desc.integer_range[0].from_value = 100;
            param_desc.integer_range[0].to_value = 20000;
            uint32_t exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
            MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
            RCLCPP_INFO(this->get_logger(), "Exposure time: %d", exposure_time);

            // Gain
            param_desc.description = "Gain";
            param_desc.integer_range[0].from_value = 0;
            param_desc.integer_range[0].to_value = 50;
            double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
            MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
            RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
        }

        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : parameters)
            {
                if (param.get_name() == "exposure_time")
                {
                    int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
                    if (MV_OK != status)
                    {
                        result.successful = false;
                        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "gain")
                {
                    int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
                    if (MV_OK != status)
                    {
                        result.successful = false;
                        result.reason = "Failed to set gain, status = " + std::to_string(status);
                    }
                }
                else
                {
                    result.successful = false;
                    result.reason = "Unknown parameter: " + param.get_name();
                }
            }
            return result;
        }

        static void hik_image_callback(
            unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser)
        {
            HikCameraNode *node_ptr = (HikCameraNode *)pUser;
            std::vector<uint8_t> image(node_ptr->camera_info_msg_.width*node_ptr->camera_info_msg_.height*3);
            memcpy(image.data(),pData,image.size());
            node_ptr->image_msg_.set__data(image);
            node_ptr->image_msg_.header.stamp = node_ptr->now();
            node_ptr->camera_info_msg_.header = node_ptr->image_msg_.header;
            node_ptr->camera_pub_.publish(node_ptr->image_msg_, node_ptr->camera_info_msg_);
        }

        void watchdog_callback(void)
        {
            if (!MV_CC_IsDeviceConnected(camera_handle_))
            {
                RCLCPP_ERROR(this->get_logger(), "Camera not connected");
                // enum device
                MV_CC_DEVICE_INFO_LIST device_list;
                nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
                RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);
                if (device_list.nDeviceNum == 0 && rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "No camera found!");
                    return;
                }
                MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
                MV_CC_OpenDevice(camera_handle_);

                // init grabbing
                MV_CC_SetIntValue(camera_handle_, "Width", camera_info_msg_.width);
                MV_CC_SetIntValue(camera_handle_, "Height", camera_info_msg_.height);
                MV_CC_SetPixelFormat(camera_handle_, PixelType_Gvsp_RGB8_Packed);
                MV_CC_SetFrameRate(camera_handle_, 200);
                MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
                MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
                MV_CC_RegisterImageCallBackEx(camera_handle_, &HikCameraNode::hik_image_callback, this);
                RCLCPP_INFO(this->get_logger(), "Camera register callback");
                MV_CC_StartGrabbing(camera_handle_);
            }else{
                MVCC_FLOATVALUE fps;
                MV_CC_GetFrameRate(camera_handle_,&fps);
                RCLCPP_INFO(this->get_logger(),"FPS:%6.2f",fps.fCurValue);
            }
        }

        sensor_msgs::msg::Image image_msg_;

        image_transport::CameraPublisher camera_pub_;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;
        int nRet = MV_OK;
        void *camera_handle_;
        MV_IMAGE_BASIC_INFO img_info_;

        MV_CC_PIXEL_CONVERT_PARAM convert_param_;

        std::string camera_name_;
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;

        OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    };
} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)

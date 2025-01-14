// Copyright 2021 Evan Flynn
// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Evan Flynn nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef USB_CAM__USB_DOUBLE_CAM_NODE_HPP_
#define USB_CAM__USB_DOUBLE_CAM_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "usb_cam/usb_cam.hpp"


std::ostream & operator<<(std::ostream & ostr, const rclcpp::Time & tm)
{
  ostr << tm.nanoseconds();
  return ostr;
}


namespace usb_cam
{

class UsbCamDoubleNode : public rclcpp::Node
{
public:
  explicit UsbCamDoubleNode(const rclcpp::NodeOptions & node_options);
  ~UsbCamDoubleNode();

  void init();
  void get_params();
  void assign_params(const std::vector<rclcpp::Parameter> & parameters);
  void set_v4l2_params();
  void update();
  bool take_and_send_image();
  void rectify_image(const cv::Mat &input_image, cv::Mat &output_image, 
                                     const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, 
                                     const cv::Mat &rectification_matrix, const cv::Mat &projection_matrix);
  void rotate_image_180(std::vector<uint8_t> &data, int width, int height);
  std::pair<std::vector<uint8_t>, std::vector<uint8_t>> split_image(const std::vector<uint8_t> &data, int width, int height, int step);
  // bool take_and_send_image_mjpeg();

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  UsbCam * m_camera;

  sensor_msgs::msg::Image::UniquePtr m_left_image_msg;
  sensor_msgs::msg::Image::UniquePtr m_right_image_msg;

  sensor_msgs::msg::Image::UniquePtr m_left_rectified_image_msg;
  sensor_msgs::msg::Image::UniquePtr m_right_rectified_image_msg;

  sensor_msgs::msg::CompressedImage::UniquePtr m_left_compressed_img_msg;
  sensor_msgs::msg::CompressedImage::UniquePtr m_right_compressed_img_msg;

  std::shared_ptr<image_transport::CameraPublisher> m_left_image_publisher;
  std::shared_ptr<image_transport::CameraPublisher> m_right_image_publisher;

  std::shared_ptr<image_transport::CameraPublisher> m_left_rect_image_publisher;
  std::shared_ptr<image_transport::CameraPublisher> m_right_rect_image_publisher;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_left_compressed_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_right_compressed_image_publisher;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_left_compressed_cam_info_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_right_compressed_cam_info_publisher;

  parameters_t m_parameters;

  sensor_msgs::msg::CameraInfo::SharedPtr m_left_camera_info_msg;
  sensor_msgs::msg::CameraInfo::SharedPtr m_right_camera_info_msg;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_left_camera_info;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_right_camera_info;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_service_capture;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameters_callback_handle;
};
}  // namespace usb_cam
#endif  // USB_CAM__USB_CAM_NODE_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "usb_cam/usb_cam_double_node.hpp"
#include "usb_cam/utils.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const char BASE_TOPIC_NAME_LEFT[] = "left/image_raw";
const char BASE_TOPIC_NAME_RIGHT[] = "right/image_raw";

namespace usb_cam
{

  UsbCamDoubleNode::UsbCamDoubleNode(const rclcpp::NodeOptions &node_options)
      : Node("usb_cam", node_options),
        m_camera(new usb_cam::UsbCam()),

        m_left_image_msg(new sensor_msgs::msg::Image()),
        m_right_image_msg(new sensor_msgs::msg::Image()),

        m_left_rectified_image_msg(new sensor_msgs::msg::Image()),
        m_right_rectified_image_msg(new sensor_msgs::msg::Image()),

        m_left_compressed_img_msg(nullptr),
        m_right_compressed_img_msg(nullptr),

        m_left_image_publisher(std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(this, BASE_TOPIC_NAME_LEFT,
                                                     rclcpp::QoS{100}.get_rmw_qos_profile()))),
        m_right_image_publisher(std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(this, BASE_TOPIC_NAME_RIGHT,
                                                     rclcpp::QoS{100}.get_rmw_qos_profile()))),

        m_left_rect_image_publisher(std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(this, "left/image_rect",
                                                     rclcpp::QoS{100}.get_rmw_qos_profile()))),
        m_right_rect_image_publisher(std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(this, "right/image_rect",
                                                     rclcpp::QoS{100}.get_rmw_qos_profile()))),

        m_left_compressed_image_publisher(nullptr),
        m_right_compressed_image_publisher(nullptr),

        m_left_compressed_cam_info_publisher(nullptr),
        m_right_compressed_cam_info_publisher(nullptr),

        m_parameters(),

        m_left_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
        m_right_camera_info_msg(new sensor_msgs::msg::CameraInfo()),

        m_service_capture(
            this->create_service<std_srvs::srv::SetBool>(
                "set_capture",
                std::bind(
                    &UsbCamDoubleNode::service_capture,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3)))
  {
    // declare params
    this->declare_parameter("camera_name", "default_cam");
    this->declare_parameter("left_camera_info_url", "");
    this->declare_parameter("right_camera_info_url", "");
    this->declare_parameter("framerate", 30.0);
    this->declare_parameter("frame_id", "default_cam");
    this->declare_parameter("image_height", 480);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("io_method", "mmap");
    this->declare_parameter("pixel_format", "yuyv");
    this->declare_parameter("av_device_format", "YUV422P");
    this->declare_parameter("video_device", "/dev/video0");
    this->declare_parameter("brightness", 50); // 0-255, -1 "leave alone"
    this->declare_parameter("contrast", -1);   // 0-255, -1 "leave alone"
    this->declare_parameter("saturation", -1); // 0-255, -1 "leave alone"
    this->declare_parameter("sharpness", -1);  // 0-255, -1 "leave alone"
    this->declare_parameter("gain", -1);       // 0-100?, -1 "leave alone"
    this->declare_parameter("auto_white_balance", true);
    this->declare_parameter("white_balance", 4000);
    this->declare_parameter("autoexposure", true);
    this->declare_parameter("exposure", 100);
    this->declare_parameter("autofocus", false);
    this->declare_parameter("focus", -1); // 0-255, -1 "leave alone"

    get_params();
    init();
    m_parameters_callback_handle = add_on_set_parameters_callback(
        std::bind(
            &UsbCamDoubleNode::parameters_callback, this,
            std::placeholders::_1));
  }

  UsbCamDoubleNode::~UsbCamDoubleNode()
  {
    RCLCPP_WARN(this->get_logger(), "Shutting down");
    m_left_image_msg.reset();
    m_right_image_msg.reset();
    m_left_compressed_img_msg.reset();
    m_right_compressed_img_msg.reset();
    m_left_camera_info_msg.reset();
    m_right_camera_info_msg.reset();
    m_left_camera_info.reset();
    m_right_camera_info.reset();
    m_timer.reset();
    m_service_capture.reset();
    m_parameters_callback_handle.reset();

    delete (m_camera);
  }

  void UsbCamDoubleNode::service_capture(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    (void)request_header;
    if (request->data)
    {
      m_camera->start_capturing();
      response->message = "Start Capturing";
    }
    else
    {
      m_camera->stop_capturing();
      response->message = "Stop Capturing";
    }
  }

  std::string resolve_device_path(const std::string &path)
  {
    if (std::filesystem::is_symlink(path))
    {
      // For some reason read_symlink only returns videox
      return "/dev/" + std::string(std::filesystem::read_symlink(path));
    }
    return path;
  }

  void UsbCamDoubleNode::init()
  {
    while (m_parameters.frame_id == "")
    {
      RCLCPP_WARN_ONCE(
          this->get_logger(), "Required Parameters not set...waiting until they are set");
      get_params();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // load the left camera info
    m_left_camera_info.reset(
        new camera_info_manager::CameraInfoManager(
            this, "narrow_stereo/left", m_parameters.left_camera_info_url));
    // check for default left camera info
    if (!m_left_camera_info->isCalibrated())
    {
      m_left_camera_info->setCameraName(m_parameters.device_name);
      m_left_camera_info_msg->header.frame_id = m_parameters.frame_id;
      // Разделяем на 2 так как у нас в одном кадре 2 изображения
      m_left_camera_info_msg->width = m_parameters.image_width / 2;
      m_left_camera_info_msg->height = m_parameters.image_height;
      m_left_camera_info->setCameraInfo(*m_left_camera_info_msg);
    }

    // load the right camera info
    m_right_camera_info.reset(
        new camera_info_manager::CameraInfoManager(
            this, "narrow_stereo/right", m_parameters.right_camera_info_url));
    // check for default right camera info
    if (!m_right_camera_info->isCalibrated())
    {
      m_right_camera_info->setCameraName(m_parameters.device_name);
      m_right_camera_info_msg->header.frame_id = m_parameters.frame_id;
      // Разделяем на 2 так как у нас в одном кадре 2 изображения
      m_right_camera_info_msg->width = m_parameters.image_width / 2;
      m_right_camera_info_msg->height = m_parameters.image_height;
      m_right_camera_info->setCameraInfo(*m_right_camera_info_msg);
    }

    // Check if given device name is an available v4l2 device
    auto available_devices = usb_cam::utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end())
    {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Device specified is not available or is not a vaild V4L2 device: `" << m_parameters.device_name << "`");
      RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
      for (const auto &device : available_devices)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
        RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
      }
      rclcpp::shutdown();
      return;
    }

    // if pixel format is equal to 'mjpeg', i.e. raw mjpeg stream, initialize compressed image message
    // and publisher
    if (m_parameters.pixel_format_name == "mjpeg")
    {
      m_left_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
      m_left_compressed_img_msg->header.frame_id = m_parameters.frame_id;

      m_right_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
      m_right_compressed_img_msg->header.frame_id = m_parameters.frame_id;

      m_left_compressed_image_publisher =
          this->create_publisher<sensor_msgs::msg::CompressedImage>(
              std::string(BASE_TOPIC_NAME_LEFT) + "/compressed", rclcpp::QoS(100));
      m_left_compressed_cam_info_publisher =
          this->create_publisher<sensor_msgs::msg::CameraInfo>(
              "left/camera_info", rclcpp::QoS(100));

      m_right_compressed_image_publisher =
          this->create_publisher<sensor_msgs::msg::CompressedImage>(
              std::string(BASE_TOPIC_NAME_RIGHT) + "/compressed", rclcpp::QoS(100));
      m_right_compressed_cam_info_publisher =
          this->create_publisher<sensor_msgs::msg::CameraInfo>(
              "right/camera_info", rclcpp::QoS(100));
    }

    m_left_image_msg->header.frame_id = m_parameters.frame_id;
    m_right_image_msg->header.frame_id = m_parameters.frame_id;

    // TODO Добавить вывод инфы о том что тут две камеры
    RCLCPP_INFO(
        this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
        m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
        m_parameters.image_width, m_parameters.image_height, m_parameters.io_method_name.c_str(),
        m_parameters.pixel_format_name.c_str(), m_parameters.framerate);
    // set the IO method
    io_method_t io_method =
        usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
    if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN)
    {
      RCLCPP_ERROR_ONCE(
          this->get_logger(),
          "Unknown IO method '%s'", m_parameters.io_method_name.c_str());
      rclcpp::shutdown();
      return;
    }

    // configure the camera
    m_camera->configure(m_parameters, io_method);

    set_v4l2_params();

    // start the camera
    m_camera->start();

    // TODO(lucasw) should this check a little faster than expected frame rate?
    // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
    const int period_ms = 1000.0 / m_parameters.framerate;
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&UsbCamDoubleNode::update, this));
    RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
  }

  void UsbCamDoubleNode::get_params()
  {
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    auto parameters = parameters_client->get_parameters(
        {"camera_name", "left_camera_info_url", "right_camera_info_url", "frame_id", "framerate", "image_height", "image_width",
         "io_method", "pixel_format", "av_device_format", "video_device", "brightness", "contrast",
         "saturation", "sharpness", "gain", "auto_white_balance", "white_balance", "autoexposure",
         "exposure", "autofocus", "focus"});

    assign_params(parameters);
  }

  void UsbCamDoubleNode::assign_params(const std::vector<rclcpp::Parameter> &parameters)
  {
    for (auto &parameter : parameters)
    {
      if (parameter.get_name() == "camera_name")
      {
        RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
        m_parameters.camera_name = parameter.value_to_string();
      }
      else if (parameter.get_name() == "left_camera_info_url")
      {
        m_parameters.left_camera_info_url = parameter.value_to_string();
      }
      else if (parameter.get_name() == "right_camera_info_url")
      {
        m_parameters.right_camera_info_url = parameter.value_to_string();
      }
      else if (parameter.get_name() == "frame_id")
      {
        m_parameters.frame_id = parameter.value_to_string();
      }
      else if (parameter.get_name() == "framerate")
      {
        RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
        m_parameters.framerate = parameter.as_double();
      }
      else if (parameter.get_name() == "image_height")
      {
        m_parameters.image_height = parameter.as_int();
      }
      else if (parameter.get_name() == "image_width")
      {
        m_parameters.image_width = parameter.as_int();
      }
      else if (parameter.get_name() == "io_method")
      {
        m_parameters.io_method_name = parameter.value_to_string();
      }
      else if (parameter.get_name() == "pixel_format")
      {
        m_parameters.pixel_format_name = parameter.value_to_string();
      }
      else if (parameter.get_name() == "av_device_format")
      {
        m_parameters.av_device_format = parameter.value_to_string();
      }
      else if (parameter.get_name() == "video_device")
      {
        m_parameters.device_name = resolve_device_path(parameter.value_to_string());
      }
      else if (parameter.get_name() == "brightness")
      {
        m_parameters.brightness = parameter.as_int();
      }
      else if (parameter.get_name() == "contrast")
      {
        m_parameters.contrast = parameter.as_int();
      }
      else if (parameter.get_name() == "saturation")
      {
        m_parameters.saturation = parameter.as_int();
      }
      else if (parameter.get_name() == "sharpness")
      {
        m_parameters.sharpness = parameter.as_int();
      }
      else if (parameter.get_name() == "gain")
      {
        m_parameters.gain = parameter.as_int();
      }
      else if (parameter.get_name() == "auto_white_balance")
      {
        m_parameters.auto_white_balance = parameter.as_bool();
      }
      else if (parameter.get_name() == "white_balance")
      {
        m_parameters.white_balance = parameter.as_int();
      }
      else if (parameter.get_name() == "autoexposure")
      {
        m_parameters.autoexposure = parameter.as_bool();
      }
      else if (parameter.get_name() == "exposure")
      {
        m_parameters.exposure = parameter.as_int();
      }
      else if (parameter.get_name() == "autofocus")
      {
        m_parameters.autofocus = parameter.as_bool();
      }
      else if (parameter.get_name() == "focus")
      {
        m_parameters.focus = parameter.as_int();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
      }
    }
  }

  /// @brief Send current parameters to V4L2 device
  /// TODO(flynneva): should this actuaully be part of UsbCam class?
  void UsbCamDoubleNode::set_v4l2_params()
  {
    // set camera parameters
    if (m_parameters.brightness >= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'brightness' to %d", m_parameters.brightness);
      m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
    }

    if (m_parameters.contrast >= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'contrast' to %d", m_parameters.contrast);
      m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
    }

    if (m_parameters.saturation >= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'saturation' to %d", m_parameters.saturation);
      m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
    }

    if (m_parameters.sharpness >= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'sharpness' to %d", m_parameters.sharpness);
      m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
    }

    if (m_parameters.gain >= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'gain' to %d", m_parameters.gain);
      m_camera->set_v4l_parameter("gain", m_parameters.gain);
    }

    // check auto white balance
    if (m_parameters.auto_white_balance)
    {
      m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
      RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_temperature_auto' to %d", 1);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
      m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
      m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
    }

    // check auto exposure
    if (!m_parameters.autoexposure)
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 1);
      RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
      // turn down exposure control (from max of 3)
      m_camera->set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
      m_camera->set_v4l_parameter("exposure_auto", 3);
    }

    // check auto focus
    if (m_parameters.autofocus)
    {
      m_camera->set_auto_focus(1);
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
      m_camera->set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
      m_camera->set_v4l_parameter("focus_auto", 0);
      if (m_parameters.focus >= 0)
      {
        RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
        m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
      }
    }
  }

  bool UsbCamDoubleNode::take_and_send_image()
  {
    // Получите размеры изображения
    int width = m_camera->get_image_width();
    int height = m_camera->get_image_height();
    int step = m_camera->get_image_step();
    int image_size = m_camera->get_image_size_in_bytes();
    // // Only resize if required
    // if (sizeof(m_image_msg->data) != m_camera->get_image_size_in_bytes()) {
    //   m_image_msg->width = m_camera->get_image_width();
    //   m_image_msg->height = m_camera->get_image_height();
    //   m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    //   m_image_msg->step = m_camera->get_image_step();
    //   if (m_image_msg->step == 0) {
    //     // Some formats don't have a linesize specified by v4l2
    //     // Fall back to manually calculating it step = size / height
    //     m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
    //   }
    //   m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
    // }

    // Создайте вектор для хранения сдвоенного изображения
    std::vector<uint8_t> double_image(image_size);

    // grab the image, pass image msg buffer to fill
    m_camera->get_image(reinterpret_cast<char *>(&double_image[0]));

    // Rotate the image by 180 degrees
    // this->rotate_image_180(double_image, width, height, step);

    // Split the image into two halves
    auto [left_image, right_image] = this->split_image(double_image, width, height, step);

    // Process each half as needed

    auto stamp = m_camera->get_image_timestamp();

    // Создание объектов cv::Mat для ректификации
    // RCLCPP_INFO(this->get_logger(), "Rectifying image");
    // RCLCPP_INFO(this->get_logger(), m_camera->get_pixel_format()->name().c_str());
    cv::Mat left_img(height, width/2, CV_8UC3, left_image.data());
    cv::Mat right_img(height, width/2, CV_8UC3, right_image.data());

    cv::Mat left_rectified, right_rectified;

    // Получаем информацию о камере
    auto left_info = m_left_camera_info->getCameraInfo();
    auto right_info = m_right_camera_info->getCameraInfo();

    // Создаем матрицы для камеры и коэффициентов искажения
    cv::Mat left_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(left_info.k.data()));
    cv::Mat left_dist_coeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(left_info.d.data()));

    cv::Mat right_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(right_info.k.data()));
    cv::Mat right_dist_coeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(right_info.d.data()));

    // Выполнение ректификации
    this->rectify_image(left_img, left_rectified, left_camera_matrix, left_dist_coeffs);
    this->rectify_image(right_img, right_rectified, right_camera_matrix, right_dist_coeffs);

    // Преобразование обратно в std::vector<uint8_t> после ректификации
    std::vector<uint8_t> left_rectified_data(left_rectified.data, left_rectified.data + left_rectified.total() * left_rectified.elemSize());
    std::vector<uint8_t> right_rectified_data(right_rectified.data, right_rectified.data + right_rectified.total() * right_rectified.elemSize());

    m_left_image_msg->height = height;
    // m_right_image_msg->height = height;

    m_left_image_msg->width = width / 2;
    // m_right_image_msg->width = width / 2;

    m_left_image_msg->encoding = m_camera->get_pixel_format()->ros();
    // m_right_image_msg->encoding = m_camera->get_pixel_format()->ros();

    m_left_image_msg->step = step / 2;
    // m_right_image_msg->step = step / 2;

    m_left_image_msg->header.stamp.sec = stamp.tv_sec;
    m_left_image_msg->header.stamp.nanosec = stamp.tv_nsec;
    // m_right_image_msg->header.stamp.sec = stamp.tv_sec;
    // m_right_image_msg->header.stamp.nanosec = stamp.tv_nsec;

      // Создаем новые сообщения с помощью std::unique_ptr
    m_left_rectified_image_msg = std::make_unique<sensor_msgs::msg::Image>(*m_left_image_msg);
    m_right_rectified_image_msg = std::make_unique<sensor_msgs::msg::Image>(*m_left_image_msg);

    m_left_image_msg->data = left_image;
    // m_right_image_msg->data = right_image;

    // config rectify msgs adding data
    m_left_rectified_image_msg->data = left_rectified_data;
    m_right_rectified_image_msg->data = right_rectified_data;

    // config camera info msgs
    *m_left_camera_info_msg = m_left_camera_info->getCameraInfo();
    *m_right_camera_info_msg = m_right_camera_info->getCameraInfo();
    m_left_camera_info_msg->header = m_left_image_msg->header;
    // m_right_camera_info_msg->header = m_right_image_msg->header;

    // publish images
    m_left_image_publisher->publish(*m_left_image_msg, *m_left_camera_info_msg);
    // m_right_image_publisher->publish(*m_right_image_msg, *m_right_camera_info_msg);
    
    m_left_rect_image_publisher->publish(*m_left_rectified_image_msg, *m_left_camera_info_msg);
    m_right_rect_image_publisher->publish(*m_right_rectified_image_msg, *m_right_camera_info_msg);
    return true;
  }

  void UsbCamDoubleNode::rectify_image(const cv::Mat &input_image, cv::Mat &output_image, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) 
{
    cv::Mat map1, map2;
    cv::Size image_size = input_image.size();
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(),
                                 camera_matrix, image_size, CV_16SC2, map1, map2);
    cv::remap(input_image, output_image, map1, map2, cv::INTER_LINEAR);
}

  void UsbCamDoubleNode::rotate_image_180(std::vector<uint8_t> &data, int width, int height, int step)
  {
    int row_size = step;
    std::vector<uint8_t> temp_data(data.size());

    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        int src_index = (y * row_size) + (x * (step / width));
        int dst_index = ((height - y - 1) * row_size) + ((width - x - 1) * (step / width));
        std::copy(data.begin() + src_index, data.begin() + src_index + (step / width), temp_data.begin() + dst_index);
      }
    }

    data = std::move(temp_data);
  }

  std::pair<std::vector<uint8_t>, std::vector<uint8_t>> UsbCamDoubleNode::split_image(const std::vector<uint8_t> &data, int width, int height, int step)
{
    // Определяем ширину левой и правой части
    int half_width = width / 2;
    int bytes_per_pixel = step / width; // Количество байтов на пиксель

    // Размеры строк для левой и правой части
    int left_row_size = half_width * bytes_per_pixel;
    int right_row_size = left_row_size;

    // Создаем векторы для левой и правой части изображения
    std::vector<uint8_t> left_image(height * left_row_size);
    std::vector<uint8_t> right_image(height * right_row_size);

    // Проверка размеров
    if (data.size() != static_cast<size_t>(height * step)) {
        RCLCPP_ERROR(this->get_logger(), "Ошибка: Размер входного вектора данных (%zu) не соответствует ожидаемому (%d x %d x %d)", data.size(), width, height, step);
        return {left_image, right_image};
    }

    // Копируем данные
    for (int i = 0; i < height; ++i)
    {
        // Проверка начала и конца диапазонов для копирования
        auto left_start = data.begin() + i * step;
        auto left_end = left_start + left_row_size;
        auto right_start = data.begin() + i * step + half_width * bytes_per_pixel;
        auto right_end = data.begin() + i * step + width * bytes_per_pixel;

        if (left_start > data.end() || left_end > data.end() || right_start > data.end() || right_end > data.end()) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка: Пределы диапазонов для копирования выходят за границы данных");
            break;
        }

        // Копирование левой части
        std::copy(left_start, left_end, left_image.begin() + i * left_row_size);

        // Копирование правой части
        std::copy(right_start, right_end, right_image.begin() + i * right_row_size);
    }

    return {left_image, right_image};
}
  // bool UsbCamDoubleNode::take_and_send_image_mjpeg()
  // {
  //   // Only resize if required
  //   if (sizeof(m_compressed_img_msg->data) != m_camera->get_image_size_in_bytes()) {
  //     m_compressed_img_msg->format = "jpeg";
  //     m_compressed_img_msg->data.resize(m_camera->get_image_size_in_bytes());
  //   }

  //   // grab the image, pass image msg buffer to fill
  //   m_camera->get_image(reinterpret_cast<char *>(&m_compressed_img_msg->data[0]));

  //   auto stamp = m_camera->get_image_timestamp();
  //   m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  //   m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;

  //   *m_camera_info_msg = m_camera_info->getCameraInfo();
  //   m_camera_info_msg->header = m_compressed_img_msg->header;

  //   m_compressed_image_publisher->publish(*m_compressed_img_msg);
  //   m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
  //   return true;
  // }

  rcl_interfaces::msg::SetParametersResult UsbCamDoubleNode::parameters_callback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
    m_timer->reset();
    assign_params(parameters);
    set_v4l2_params();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  void UsbCamDoubleNode::update()
  {
    if (m_camera->is_capturing())
    {
      // If the camera exposure longer higher than the framerate period
      // then that caps the framerate.
      // auto t0 = now();

      // bool isSuccessful = (m_parameters.pixel_format_name == "mjpeg") ?
      //   take_and_send_image_mjpeg() :
      //   take_and_send_image();

      bool isSuccessful = take_and_send_image();
      if (!isSuccessful)
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
      }
    }
  }
} // namespace usb_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamDoubleNode)

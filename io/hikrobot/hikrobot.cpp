#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include <string_view>
#include <unordered_map>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(double exposure_ms, double gain, const std::string & vid_pid)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  // Log enumerated devices to help distinguish multiple cameras/configs.
  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    auto * dev = device_list.pDeviceInfo[i];
    if (!dev) continue;
    if (dev->nTLayerType == MV_USB_DEVICE) {
      const auto & usb = dev->SpecialInfo.stUsb3VInfo;
      auto view = [](const unsigned char * buf, size_t n) -> std::string_view {
        size_t len = 0;
        while (len < n && buf[len] != 0) ++len;
        return std::string_view{reinterpret_cast<const char *>(buf), len};
      };
      tools::logger()->info(
        "HikRobot device[{}]: vendor=\"{}\" model=\"{}\" serial=\"{}\" user=\"{}\" vid={:#x} pid={:#x}",
        i, view(usb.chVendorName, sizeof(usb.chVendorName)),
        view(usb.chModelName, sizeof(usb.chModelName)),
        view(usb.chSerialNumber, sizeof(usb.chSerialNumber)),
        view(usb.chUserDefinedName, sizeof(usb.chUserDefinedName)), usb.idVendor, usb.idProduct);
    } else {
      tools::logger()->info("HikRobot device[{}]: tlayer={:#x}", i, dev->nTLayerType);
    }
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  MV_CC_SetFrameRate(handle_, 150);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw;
    bool logged_frame_info = false;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      const auto & frame_info = raw.stFrameInfo;
      const auto width = static_cast<int>(frame_info.nWidth);
      const auto height = static_cast<int>(frame_info.nHeight);
      const auto pixel_type = frame_info.enPixelType;

      if (!logged_frame_info) {
        tools::logger()->info(
          "HikRobot stream: {}x{}, src_pixel_type={:#x}", width, height,
          static_cast<unsigned int>(pixel_type));
        logged_frame_info = true;
      }

      cv::Mat bgr_image(height, width, CV_8UC3);

      MV_CC_PIXEL_CONVERT_PARAM cvt_param{};
      cvt_param.nWidth = frame_info.nWidth;
      cvt_param.nHeight = frame_info.nHeight;
      cvt_param.pSrcData = raw.pBufAddr;
      cvt_param.nSrcDataLen = frame_info.nFrameLen;
      cvt_param.enSrcPixelType = pixel_type;
      cvt_param.pDstBuffer = bgr_image.data;
      cvt_param.nDstBufferSize =
        static_cast<unsigned int>(bgr_image.total() * bgr_image.elemSize());
      cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
      if (ret != MV_OK) {
        // Fallbacks for common pixel formats; avoid throwing on unknown formats.
        tools::logger()->warn(
          "MV_CC_ConvertPixelType failed: {:#x} (src_pixel_type={:#x}, {}x{})", ret,
          static_cast<unsigned int>(pixel_type), width, height);

        if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
          cv::Mat bgr_view(height, width, CV_8UC3, raw.pBufAddr);
          bgr_image = bgr_view.clone();
        } else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
          cv::Mat rgb_view(height, width, CV_8UC3, raw.pBufAddr);
          cv::cvtColor(rgb_view, bgr_image, cv::COLOR_RGB2BGR);
        } else if (pixel_type == PixelType_Gvsp_Mono8) {
          cv::Mat mono_view(height, width, CV_8UC1, raw.pBufAddr);
          cv::cvtColor(mono_view, bgr_image, cv::COLOR_GRAY2BGR);
        } else {
          // Bayer8 patterns (OpenCV expects BGR output for downstream pipeline).
          const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> bayer8_map =
            {
              {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
              {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
              {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
              {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR},
            };
          auto it = bayer8_map.find(pixel_type);
          if (it == bayer8_map.end()) {
            tools::logger()->warn(
              "Unsupported HikRobot pixel type: {:#x} ({}x{}), dropping frame",
              static_cast<unsigned int>(pixel_type), width, height);
            ret = MV_CC_FreeImageBuffer(handle_, &raw);
            if (ret != MV_OK) {
              tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
              break;
            }
            continue;
          }
          cv::Mat bayer_view(height, width, CV_8UC1, raw.pBufAddr);
          cv::cvtColor(bayer_view, bgr_image, it->second);
        }
      }

      queue_.push({bgr_image, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io

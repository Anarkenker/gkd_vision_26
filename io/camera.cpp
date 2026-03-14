#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  if (camera_name != "hikrobot") {
    throw std::runtime_error("Only hikrobot camera is supported, got: " + camera_name + "!");
  }

  auto gain = tools::read<double>(yaml, "gain");
  auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io

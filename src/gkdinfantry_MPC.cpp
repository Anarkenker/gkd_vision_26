#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gkdcontrol.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/gkdinfantry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;

  io::GKDControl gkdcontrol(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gkdcontrol.imu_at(t - 1ms);
    tracker.set_enemy_color(gkdcontrol.color_at(t - 1ms));

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);
    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto state = target.ekf_x();
      tools::logger()->info(
        "[Infantry MPC] Target state -> x {:.3f} m, y {:.3f} m, z {:.3f} m, yaw {:.3f} rad",
        state[0], state[2], state[4], state[6]);
    }

    std::optional<auto_aim::Target> target =
      targets.empty() ? std::nullopt : std::optional<auto_aim::Target>(targets.front());
    auto plan = planner.plan(target, gkdcontrol.bullet_speed);
    io::Command command{plan.control, plan.fire, plan.yaw, plan.pitch};
    gkdcontrol.send(command);
  }

  return 0;
}

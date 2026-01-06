#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

struct Opts {
  std::string server_ip = "10.1.14.2";
  std::string table = "Arm";
  std::string joint = "J2";
  double wait_s = 0.5;      // initial wait for first updates
  double watch_hz = 10.0;   // watch print rate (Hz)
  double duration_s = -1.0; // <0 means run forever
  bool watch = false;       // if false, print once then exit
};

int main(int argc, char** argv) {
  Opts o;
  // Usage:
  //   nt4_dump_arm [server_ip] [joint] [wait_s] [watch_hz] [duration_s]
  // If watch_hz is provided, enters watch mode (prints repeatedly).
  if (argc >= 2) o.server_ip = argv[1];
  if (argc >= 3) o.joint = argv[2];
  if (argc >= 4) o.wait_s = std::stod(argv[3]);
  if (argc >= 5) {
    o.watch_hz = std::stod(argv[4]);
    o.watch = true;
  }
  if (argc >= 6) {
    o.duration_s = std::stod(argv[5]);
    o.watch = true;
  }

  auto inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("jetson_nt4_dump_arm");
  inst.SetServer(o.server_ip.c_str());

  nt::PubSubOptions opts;
  opts.periodic = 0.02;
  opts.sendAll = true;
  opts.keepDuplicates = true;
  opts.pollStorage = 1000;

  auto root = inst.GetTable(o.table);
  auto root_monitor = root->GetBooleanTopic("MonitorOnly").Subscribe(false, opts);
  auto root_active = root->GetStringTopic("ActiveJoint").Subscribe("", opts);
  // Optional execution-side gating/debug (if RoboRIO publishes them)
  auto root_enabled = root->GetBooleanTopic("Enabled").Subscribe(false, opts);
  auto root_inhibit = root->GetStringTopic("InhibitReason").Subscribe("", opts);
  auto root_mode = root->GetStringTopic("ControlMode").Subscribe("", opts);

  auto st = root->GetSubTable(o.joint);
  auto ros_enable = st->GetBooleanTopic("RosEnable").Subscribe(false, opts);
  // Some implementations publish Enabled per-joint (Arm/J2/Enabled) rather than root-level (Arm/Enabled).
  auto joint_enabled = st->GetBooleanTopic("Enabled").Subscribe(false, opts);
  auto ros_timeout = st->GetDoubleTopic("RosTimeoutS").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto ros_pos = st->GetDoubleTopic("RosPosRad").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto ros_vel = st->GetDoubleTopic("RosVelRadPerS").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto ros_acc = st->GetDoubleTopic("RosAccRadPerS2").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto ros_seq = st->GetDoubleTopic("RosSeq").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto angle_deg = st->GetDoubleTopic("AngleDeg").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  // Optional execution-side fields (names based on docs; may not exist in your RoboRIO build)
  auto ros_valid = st->GetBooleanTopic("RosValid").Subscribe(false, opts);
  auto ros_age_s = st->GetDoubleTopic("RosAgeS").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto out_volts = st->GetDoubleTopic("OutVoltsCmd").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto inhibit = st->GetStringTopic("InhibitReason").Subscribe("", opts);
  auto mode = st->GetStringTopic("ControlMode").Subscribe("", opts);

  // Wait for connection + a few update cycles
  for (int i = 0; i < 200; ++i) {
    if (inst.IsConnected()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(std::max(0.0, o.wait_s) * 1000.0)));

  // Print topic types (server truth) once after initial sync
  try {
    // NT4 topic names are absolute (start with '/')
    {
      const std::string root_prefix = "/" + o.table + "/";
      auto infos = inst.GetTopicInfo(root_prefix);
      std::cout << "topic_types(prefix=\"" << root_prefix << "\")\n";
      for (const auto& ti : infos) {
        std::cout << "  " << ti.name << " type=" << ti.type_str << "\n";
      }
      std::cout << "----\n";
    }
    const std::string prefix = "/" + o.table + "/" + o.joint + "/";
    auto infos = inst.GetTopicInfo(prefix);
    std::cout << "topic_types(prefix=\"" << prefix << "\")\n";
    for (const auto& ti : infos) {
      std::cout << "  " << ti.name << " type=" << ti.type_str << "\n";
    }
    std::cout << "----\n";
  } catch (...) {
    // ignore
  }

  auto print_once = [&]() {
    std::cout << "connected=" << (inst.IsConnected() ? "True" : "False") << "\n";
    std::cout << "server=" << o.server_ip << "\n";
    std::cout << "table=" << o.table << " joint=" << o.joint << "\n";
    std::cout << o.table << "/Enabled=" << (root_enabled.Get() ? "true" : "false") << "\n";
    std::cout << "Arm/MonitorOnly=" << (root_monitor.Get() ? "true" : "false") << "\n";
    std::cout << "Arm/ActiveJoint=\"" << root_active.Get() << "\"\n";
    if (!root_mode.Get().empty()) std::cout << o.table << "/ControlMode=\"" << root_mode.Get() << "\"\n";
    if (!root_inhibit.Get().empty()) std::cout << o.table << "/InhibitReason=\"" << root_inhibit.Get() << "\"\n";
    std::cout << o.table << "/" << o.joint << "/Enabled=" << (joint_enabled.Get() ? "true" : "false") << "\n";
    std::cout << o.table << "/" << o.joint << "/RosEnable=" << (ros_enable.Get() ? "true" : "false") << "\n";
    std::cout << o.table << "/" << o.joint << "/RosTimeoutS=" << ros_timeout.Get() << "\n";
    std::cout << o.table << "/" << o.joint << "/RosPosRad=" << ros_pos.Get() << "\n";
    std::cout << o.table << "/" << o.joint << "/RosVelRadPerS=" << ros_vel.Get() << "\n";
    std::cout << o.table << "/" << o.joint << "/RosAccRadPerS2=" << ros_acc.Get() << "\n";
    std::cout << o.table << "/" << o.joint << "/RosSeq=" << ros_seq.Get() << "\n";
    std::cout << o.table << "/" << o.joint << "/AngleDeg=" << angle_deg.Get() << "\n";
    // Execution-side diagnostics (if published)
    std::cout << o.table << "/" << o.joint << "/RosValid=" << (ros_valid.Get() ? "true" : "false") << "\n";
    std::cout << o.table << "/" << o.joint << "/RosAgeS=" << ros_age_s.Get() << "\n";
    std::cout << o.table << "/" << o.joint << "/OutVoltsCmd=" << out_volts.Get() << "\n";
    if (!mode.Get().empty()) std::cout << o.table << "/" << o.joint << "/ControlMode=\"" << mode.Get() << "\"\n";
    if (!inhibit.Get().empty()) std::cout << o.table << "/" << o.joint << "/InhibitReason=\"" << inhibit.Get() << "\"\n";
    std::cout << "----\n";
    std::cout.flush();
  };

  if (!o.watch) {
    print_once();
    return 0;
  }

  const double hz = std::max(0.2, o.watch_hz);
  const auto period = std::chrono::duration<double>(1.0 / hz);
  const auto t0 = std::chrono::steady_clock::now();
  while (true) {
    if (o.duration_s >= 0.0) {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed >= o.duration_s) break;
    }
    print_once();
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(period));
  }

  return 0;
}



#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <thread>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

struct NumStat {
  int events = 0;   // how many NT updates observed (queue entries)
  int changes = 0;  // how many times the value changed vs previous
  double last = std::numeric_limits<double>::quiet_NaN();
};

struct IntStat {
  int events = 0;
  int changes = 0;
  int64_t last = std::numeric_limits<int64_t>::min();
  bool has_last = false;
};

template <typename SubT>
static void print_topic_info_line(const std::string& label, const SubT& sub) {
  // Note: Topic::Exists() is not reliable as a "remote published" indicator across WPILib versions.
  // We print the canonical topic name only.
  std::cout << "  " << label << ": name=\"" << sub.GetTopic().GetName() << "\"\n";
}

int main(int argc, char** argv) {
  std::string server_ip = "10.1.14.2";
  std::string table = "Arm";
  std::string joint = "J2";
  double duration_s = 5.0;

  if (argc >= 2) server_ip = argv[1];
  if (argc >= 3) joint = argv[2];
  if (argc >= 4) duration_s = std::stod(argv[3]);

  auto inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("jetson_nt4_rate_check");
  inst.SetServer(server_ip.c_str());

  nt::PubSubOptions opts;
  // Aim to observe up to 100Hz streams (10ms)
  opts.periodic = 0.01;
  opts.sendAll = true;
  opts.keepDuplicates = true;
  opts.pollStorage = 20000;

  auto root = inst.GetTable(table);
  auto st = root->GetSubTable(joint);

  // Timing topics (optional; may not exist in your RoboRIO build)
  // - Some builds publish NowSec at root (Arm/NowSec)
  // - Some may publish per-joint (Arm/J2/NowSec)
  // - For a "hard" 100Hz proof, consider adding Arm/J2/NowSec100 + AngleSeq100 written by a dedicated 100Hz Notifier.
  auto now_sec_root_sub =
      root->GetDoubleTopic("NowSec").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto now_sec_joint_sub =
      st->GetDoubleTopic("NowSec").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto now_sec_100_sub =
      st->GetDoubleTopic("NowSec100").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);

  // Joint-level
  auto angle_deg_sub = st->GetDoubleTopic("AngleDeg").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto angle_deg_100_sub =
      st->GetDoubleTopic("AngleDeg100").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto angle_seq_100_sub =
      st->GetIntegerTopic("AngleSeq100").Subscribe(std::numeric_limits<int64_t>::min(), opts);
  auto goal_deg_sub = st->GetDoubleTopic("GoalDeg").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto out_volts_sub = st->GetDoubleTopic("OutVoltsCmd").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);
  auto ros_seq_sub = st->GetDoubleTopic("RosSeq").Subscribe(std::numeric_limits<double>::quiet_NaN(), opts);

  // Wait briefly for connection
  for (int i = 0; i < 200; ++i) {
    if (inst.IsConnected()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::map<std::string, NumStat> stats;
  std::map<std::string, IntStat> istats;
  const auto t0 = std::chrono::steady_clock::now();

  auto ingest_queue = [&](const std::string& name, auto& sub) {
    auto q = sub.ReadQueue();
    for (const auto& tv : q) {
      const double v = tv.value;
      if (std::isnan(v)) continue;
      auto& s = stats[name];
      s.events++;
      if (!std::isnan(s.last) && v != s.last) {
        s.changes++;
      }
      s.last = v;
    }
  };

  auto ingest_queue_int = [&](const std::string& name, auto& sub) {
    auto q = sub.ReadQueue();
    for (const auto& tv : q) {
      const int64_t v = tv.value;
      auto& s = istats[name];
      s.events++;
      if (s.has_last && v != s.last) {
        s.changes++;
      }
      s.last = v;
      s.has_last = true;
    }
  };

  // Ensure all names appear in output (even if 0 events), to make "missing topic" obvious.
  stats["NowSec(root)"];
  stats["NowSec(joint)"];
  stats["NowSec100(joint)"];
  stats["AngleDeg"];
  stats["AngleDeg100(joint)"];
  stats["GoalDeg"];
  stats["OutVoltsCmd"];
  stats["RosSeq"];
  istats["AngleSeq100(joint)"];

  std::cout << "topics:\n";
  print_topic_info_line("NowSec(root)", now_sec_root_sub);
  print_topic_info_line("NowSec(joint)", now_sec_joint_sub);
  print_topic_info_line("NowSec100(joint)", now_sec_100_sub);
  print_topic_info_line("AngleDeg", angle_deg_sub);
  print_topic_info_line("AngleDeg100(joint)", angle_deg_100_sub);
  print_topic_info_line("AngleSeq100(joint)", angle_seq_100_sub);
  print_topic_info_line("GoalDeg", goal_deg_sub);
  print_topic_info_line("OutVoltsCmd", out_volts_sub);
  print_topic_info_line("RosSeq", ros_seq_sub);

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - t0).count();
    if (elapsed >= duration_s) break;

    ingest_queue("NowSec(root)", now_sec_root_sub);
    ingest_queue("NowSec(joint)", now_sec_joint_sub);
    ingest_queue("NowSec100(joint)", now_sec_100_sub);
    ingest_queue("AngleDeg", angle_deg_sub);
    ingest_queue("AngleDeg100(joint)", angle_deg_100_sub);
    ingest_queue_int("AngleSeq100(joint)", angle_seq_100_sub);
    ingest_queue("GoalDeg", goal_deg_sub);
    ingest_queue("OutVoltsCmd", out_volts_sub);
    ingest_queue("RosSeq", ros_seq_sub);

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  std::cout << "duration_s=" << duration_s << " connected=" << (inst.IsConnected() ? "True" : "False")
            << " server=" << server_ip << " table=" << table << " joint=" << joint << "\n";
  for (const auto& kv : stats) {
    const auto& name = kv.first;
    const auto& s = kv.second;
    const double events_hz = (duration_s > 0) ? (static_cast<double>(s.events) / duration_s) : 0.0;
    const double changes_hz = (duration_s > 0) ? (static_cast<double>(s.changes) / duration_s) : 0.0;
    std::cout << name << " events=" << s.events << " ~" << events_hz << " Hz"
              << " changes=" << s.changes << " ~" << changes_hz << " Hz"
              << " last=" << s.last << "\n";
  }
  for (const auto& kv : istats) {
    const auto& name = kv.first;
    const auto& s = kv.second;
    const double events_hz = (duration_s > 0) ? (static_cast<double>(s.events) / duration_s) : 0.0;
    const double changes_hz = (duration_s > 0) ? (static_cast<double>(s.changes) / duration_s) : 0.0;
    std::cout << name << " events=" << s.events << " ~" << events_hz << " Hz"
              << " changes=" << s.changes << " ~" << changes_hz << " Hz"
              << " last=" << (s.has_last ? std::to_string(s.last) : "n/a") << "\n";
  }

  return 0;
}



#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/Topic.h>

int main(int argc, char** argv) {
  std::string server_ip = "10.1.14.2";
  std::string table = "Arm";
  std::string joint = "J2";
  std::string key = "RosSeq";
  double duration_s = 5.0;

  if (argc >= 2) server_ip = argv[1];
  if (argc >= 3) joint = argv[2];
  if (argc >= 4) key = argv[3];
  if (argc >= 5) duration_s = std::stod(argv[4]);

  auto inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("jetson_nt4_seq_monitor");
  inst.SetServer(server_ip.c_str());

  auto root = inst.GetTable(table);
  auto st = root->GetSubTable(joint);

  nt::PubSubOptions sub_opts;
  sub_opts.sendAll = true;          // request all value changes (no coalescing)
  sub_opts.keepDuplicates = true;   // keep duplicates if they occur
  sub_opts.pollStorage = 20000;     // large queue
  sub_opts.periodic = 0.02;         // request ~50Hz network updates

  auto sub =
      st->GetDoubleTopic(key).Subscribe(std::numeric_limits<double>::quiet_NaN(), sub_opts);

  // Wait briefly for connection
  for (int i = 0; i < 100; ++i) {
    if (inst.IsConnected()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  auto topic = st->GetTopic(key);
  std::cout << "server_topic_type=" << topic.GetTypeString() << "\n";

  const auto t0 = std::chrono::steady_clock::now();
  int changes = 0;
  int max_jump = 0;
  double last = std::numeric_limits<double>::quiet_NaN();

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - t0).count();
    if (elapsed >= duration_s) break;

    auto q = sub.ReadQueue();
    for (const auto& tv : q) {
      const double v = tv.value;
      if (std::isnan(v)) continue;
      if (!std::isnan(last)) {
        const int jump = static_cast<int>(std::llround(v - last));
        if (jump > max_jump) max_jump = jump;
      }
      last = v;
      changes++;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  std::cout << "connected=" << (inst.IsConnected() ? "True" : "False") << "\n";
  std::cout << "topic=" << table << "/" << joint << "/" << key << "\n";
  std::cout << "changes_in_" << duration_s << "s=" << changes << " max_jump=" << max_jump
            << " last=" << (std::isnan(last) ? -1.0 : last) << "\n";

  return 0;
}



// bag_writer — supervises the child process that writes the rosbag2 (normally `ros2 bag record`).
// Contract: docs/contracts/dyx3_recorder.md sections 2 and 6. Pure POSIX/std, no ROS: tests drive
// it with a fake child.
//
// The child is placed in its own process group so the whole recorder tree gets the same signal.
// Stop = SIGINT (rosbag2 finalises its database on SIGINT), then SIGTERM, then SIGKILL after the
// configured grace periods.
#pragma once

#include <sys/types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace dyx3_recorder {

class BagWriter {
public:
  ~BagWriter() { stop(0.0, 0.0); }

  // argv[0] is looked up in PATH. `dir` is the bag output directory being watched for size. False
  // if fork/exec failed.
  bool start(const std::vector<std::string>& argv, const std::string& dir);
  bool running();  // reaps the child if it exited
  // Graceful stop; returns the number of escalation steps used (0 = exited on SIGINT, 1 = needed
  // SIGTERM, 2 = SIGKILL).
  int stop(double sigint_wait_s, double sigterm_wait_s);
  int last_exit_code() const { return exit_code_; }
  bool exited_abnormally() const { return exited_ && exit_code_ != 0; }
  uint64_t bytes() const;  // recursive size of `dir`
  const std::string& dir() const { return dir_; }

private:
  bool wait_exit(double seconds);
  pid_t pid_{-1};
  bool exited_{false};
  int exit_code_{0};
  std::string dir_;
};

}  // namespace dyx3_recorder

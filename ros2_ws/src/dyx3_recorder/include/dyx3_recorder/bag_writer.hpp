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
#include <mutex>
#include <string>
#include <vector>

namespace dyx3_recorder {

// Thread-safe (REC-013): the status timer polls running()/bytes() while the mission callback
// stops the child on another executor thread. Process state is guarded by a mutex, and while a
// stop() is in progress it is the only caller of waitpid (running() then reports the child as
// running without reaping it), so the child is reaped exactly once and its exit code is never lost.
class BagWriter {
public:
  ~BagWriter() { stop(0.0, 0.0); }

  // argv[0] is looked up in PATH. `dir` is the bag output directory being watched for size. False
  // if fork/exec failed.
  bool start(const std::vector<std::string>& argv, const std::string& dir);
  bool running();  // reaps the child if it exited (unless a stop() owns the child)
  // Graceful stop; returns the number of escalation steps used (0 = exited on SIGINT, 1 = needed
  // SIGTERM, 2 = SIGKILL). A concurrent second stop() waits for the first and returns 0.
  int stop(double sigint_wait_s, double sigterm_wait_s);
  int last_exit_code() const;
  bool exited_abnormally() const;
  uint64_t bytes() const;  // recursive size of `dir`
  std::string dir() const;

private:
  int stop_locked(double sigint_wait_s, double sigterm_wait_s);  // stop_mu_ held
  bool reap_locked();                                             // m_ held; true = still running
  bool wait_exit(double seconds);                                 // stop owner only

  std::mutex stop_mu_;    // one start()/stop() at a time
  mutable std::mutex m_;  // everything below
  pid_t pid_{-1};
  bool stopping_{false};
  bool exited_{false};
  int exit_code_{0};
  std::string dir_;
};

}  // namespace dyx3_recorder

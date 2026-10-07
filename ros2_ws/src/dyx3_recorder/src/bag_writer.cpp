#include "dyx3_recorder/bag_writer.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <thread>

namespace dyx3_recorder {

bool BagWriter::start(const std::vector<std::string>& argv, const std::string& dir) {
  stop(0.0, 0.0);
  if (argv.empty()) return false;
  dir_ = dir;
  exited_ = false;
  exit_code_ = 0;
  std::vector<char*> args;
  for (const auto& a : argv) args.push_back(const_cast<char*>(a.c_str()));
  args.push_back(nullptr);
  int errpipe[2];
  if (pipe(errpipe) != 0) return false;
  fcntl(errpipe[1], F_SETFD, FD_CLOEXEC);
  const pid_t pid = fork();
  if (pid < 0) {
    close(errpipe[0]);
    close(errpipe[1]);
    return false;
  }
  if (pid == 0) {
    setpgid(0, 0);
    execvp(args[0], args.data());
    const int e = errno;
    (void)!write(errpipe[1], &e, sizeof e);
    _exit(127);
  }
  setpgid(pid, pid);  // also from the parent: no race with a fast signal
  close(errpipe[1]);
  int child_errno = 0;
  const ssize_t n =
      read(errpipe[0], &child_errno, sizeof child_errno);  // returns 0 at once if exec succeeded
  close(errpipe[0]);
  if (n > 0) {
    int st = 0;
    waitpid(pid, &st, 0);
    return false;  // exec failed (e.g. ros2 not on PATH)
  }
  pid_ = pid;
  return true;
}

bool BagWriter::running() {
  if (pid_ <= 0) return false;
  int st = 0;
  const pid_t r = waitpid(pid_, &st, WNOHANG);
  if (r == 0) return true;
  exited_ = true;
  exit_code_ = WIFEXITED(st) ? WEXITSTATUS(st) : 128 + (WIFSIGNALED(st) ? WTERMSIG(st) : 0);
  pid_ = -1;
  return false;
}

bool BagWriter::wait_exit(double seconds) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (running()) {
    if (std::chrono::steady_clock::now() >= end) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

int BagWriter::stop(double sigint_wait_s, double sigterm_wait_s) {
  if (pid_ <= 0) return 0;
  const pid_t pg = pid_;
  kill(-pg, SIGINT);
  if (wait_exit(sigint_wait_s)) return 0;
  kill(-pg, SIGTERM);
  if (wait_exit(sigterm_wait_s)) return 1;
  kill(-pg, SIGKILL);
  wait_exit(2.0);
  return 2;
}

uint64_t BagWriter::bytes() const {
  namespace fs = std::filesystem;
  std::error_code ec;
  uint64_t total = 0;
  if (dir_.empty() || !fs::exists(dir_, ec)) return 0;
  for (auto it = fs::recursive_directory_iterator(
           dir_, fs::directory_options::skip_permission_denied, ec);
       it != fs::recursive_directory_iterator(); it.increment(ec)) {
    if (ec) break;
    std::error_code e2;
    if (it->is_regular_file(e2)) total += it->file_size(e2);
  }
  return total;
}

}  // namespace dyx3_recorder

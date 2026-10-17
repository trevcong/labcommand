// Starts the installed node the way the image does, with rhythm_publish false. Activity is
// still created on that path. The process writes ROOM_MEMORY_STARTUP_MARK only after the
// constructor returns, so a missing Activity type support fails this test instead of hanging.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

void set_local_ros() {
    // Keep DDS on this machine. A default domain could be discovered by another host.
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
    setenv("ROS_DOMAIN_ID", "77", 1);
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: test_node_startup <room_memory_node>\n";
        return 2;
    }
    set_local_ros();
    char state_template[] = "/tmp/room-memory-startup-XXXXXX";
    if (mkdtemp(state_template) == nullptr) {
        std::perror("mkdtemp");
        return 1;
    }
    const std::filesystem::path state_dir(state_template);
    const std::filesystem::path mark = state_dir / "ready";
    const std::filesystem::path log = state_dir / "log";
    const std::string state_arg = "state_dir:=" + state_dir.string();
    const pid_t pid = fork();
    if (pid < 0) {
        std::perror("fork");
        std::filesystem::remove_all(state_dir);
        return 1;
    }
    if (pid == 0) {
        setenv("ROOM_MEMORY_STARTUP_MARK", mark.c_str(), 1);
        const int fd = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            if (fd > STDERR_FILENO) close(fd);
        }
        execl(argv[1], argv[1], "--ros-args", "-p", state_arg.c_str(), "-p", "rhythm_enable:=false", "-p",
              "rhythm_publish:=false", static_cast<char*>(nullptr));
        std::perror("execl");
        _exit(127);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    bool ready = false;
    bool exited = false;
    int status = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        if (std::filesystem::is_regular_file(mark)) {
            ready = true;
            break;
        }
        const pid_t waited = waitpid(pid, &status, WNOHANG);
        if (waited == pid) {
            exited = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!exited) {
        kill(pid, SIGTERM);
        const auto kill_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < kill_deadline) {
            if (waitpid(pid, &status, WNOHANG) == pid) {
                exited = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!exited) {
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
        }
    }
    const std::string output = read_file(log);
    std::error_code error;
    std::filesystem::remove_all(state_dir, error);
    if (!ready) {
        std::cerr << "room_memory_node did not finish startup with rhythm_publish false.\n"
                  << "Activity is created unconditionally; a missing labcommand_msgs/msg/Activity "
                  << "type support throws before the mark file.\n";
        if (exited) {
            if (WIFEXITED(status)) std::cerr << "exit " << WEXITSTATUS(status) << "\n";
            else if (WIFSIGNALED(status)) std::cerr << "signal " << WTERMSIG(status) << "\n";
        }
        std::cerr << output;
        return 1;
    }
    return 0;
}

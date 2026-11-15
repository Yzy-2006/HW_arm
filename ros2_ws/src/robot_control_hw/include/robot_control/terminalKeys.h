#pragma once

#include <optional>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

class TerminalKeys {
public:
    explicit TerminalKeys(int fd = STDIN_FILENO) : fd_(fd) {
        if (!isatty(fd_) || tcgetattr(fd_, &original_) != 0) return;
        auto settings = original_;
        settings.c_lflag &= ~(ICANON | ECHO);
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        enabled_ = tcsetattr(fd_, TCSANOW, &settings) == 0;
    }

    ~TerminalKeys() {
        if (enabled_) tcsetattr(fd_, TCSANOW, &original_);
    }

    TerminalKeys(const TerminalKeys&) = delete;
    TerminalKeys& operator=(const TerminalKeys&) = delete;

    bool enabled() const { return enabled_; }

    std::optional<char> poll() {
        if (!enabled_) return std::nullopt;
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        timeval timeout{0, 0};
        if (select(fd_ + 1, &fds, nullptr, nullptr, &timeout) <= 0) return std::nullopt;
        char key;
        if (read(fd_, &key, 1) != 1) return std::nullopt;
        // Ignore terminal escape sequences, including Home (ESC [ H).
        if (key == '\033') {
            escape_ = 1;
            return std::nullopt;
        }
        if (escape_ == 1) {
            escape_ = (key == '[' || key == 'O') ? 2 : 0;
            return std::nullopt;
        }
        if (escape_ == 2) {
            if (key >= '@' && key <= '~') escape_ = 0;
            return std::nullopt;
        }
        return key;
    }

private:
    int fd_;
    bool enabled_ = false;
    int escape_ = 0;
    termios original_{};
};

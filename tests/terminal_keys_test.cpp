#include "robot_control/terminalKeys.h"
#include <cassert>
#include <pty.h>

int main() {
    int master, slave;
    assert(openpty(&master, &slave, nullptr, nullptr, nullptr) == 0);
    termios before{}, after{};
    assert(tcgetattr(slave, &before) == 0);
    {
        TerminalKeys keys(slave);
        assert(keys.enabled());
        assert(tcgetattr(slave, &after) == 0);
        assert(!(after.c_lflag & ICANON));
        assert(!(after.c_lflag & ECHO));
        assert((after.c_lflag & ISIG) == (before.c_lflag & ISIG));
        assert(!keys.poll());
        assert(write(master, "h", 1) == 1);
        // PTY delivery is asynchronous, but polling must never wait for Enter.
        std::optional<char> key;
        for (int i = 0; i < 100 && !key; ++i) { key = keys.poll(); usleep(1000); }
        assert(key && *key == 'h');
        constexpr char sequence[] = "\033[H\033OH\033[1;2HE";
        assert(write(master, sequence, sizeof(sequence) - 1) == sizeof(sequence) - 1);
        key.reset();
        for (int i = 0; i < 100 && !key; ++i) { key = keys.poll(); usleep(1000); }
        assert(key && *key == 'E');
    }
    assert(tcgetattr(slave, &after) == 0);
    assert(after.c_lflag == before.c_lflag);
    assert(after.c_cc[VMIN] == before.c_cc[VMIN]);
    assert(after.c_cc[VTIME] == before.c_cc[VTIME]);
    close(master);
    close(slave);
    int pipefd[2];
    assert(pipe(pipefd) == 0);
    {
        TerminalKeys keys(pipefd[0]);
        assert(!keys.enabled());
        assert(!keys.poll());
    }
    close(pipefd[0]);
    close(pipefd[1]);
}

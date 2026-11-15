#include <cassert>
#include "robot_control/policyHomeGuard.h"

int main() {
    PolicyHomeGuard guard;
    auto ready = [&](double now, bool moving = false) {
        return guard.ready(now, moving);
    };
    assert(!ready(0));
    guard.begin();
    assert(!ready(10, true));
    assert(!ready(20, true)); // No countdown until the trajectory sending period ends.
    assert(!ready(21));
    assert(!ready(23.999));
    assert(ready(24));
    assert(ready(39));
    assert(!ready(40, true));
    assert(!ready(41));
    guard.finish();
    assert(!ready(50));
    guard.begin();
    assert(!ready(60));
    assert(!ready(62.999));
    assert(ready(63));
    bool rejected = false;
    try { guard.load(YAML::Load("policyHome: {resume_delay_s: -1}")); }
    catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);
}

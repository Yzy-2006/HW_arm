#include <array>
#include <cassert>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <vector>

#include "servo_control/rosIoWorker.h"

using namespace std::chrono_literals;

void test_blocked_ros_io_does_not_block_control() {
    ServoRosPublisher publisher;
    publisher.start();
    LatestRosInput<std::shared_ptr<int>> input;
    LatestRosInput<std::shared_ptr<int>> stop;
    std::promise<void> entered, received, unblock, published;
    auto entered_future = entered.get_future();
    auto received_future = received.get_future();
    auto gate = unblock.get_future().share();
    auto published_future = published.get_future();
    std::vector<int> sent;
    assert(publisher.post(ServoPublishChannel::feedback, [&] {
        entered.set_value();
        gate.wait();
        sent.push_back(0);
    }));
    assert(entered_future.wait_for(1s) == std::future_status::ready);
    std::thread receiver([&] {
        received.set_value();
        gate.wait();  // Simulate a DDS receive lock held for two seconds.
        input.store(std::make_shared<int>(999));
    });
    assert(received_future.wait_for(1s) == std::future_status::ready);
    std::thread release([&] {
        std::this_thread::sleep_for(2s);
        unblock.set_value();
    });

    const auto started = std::chrono::steady_clock::now();
    int cycles = 0;
    for (int sample = 1; sample <= 2000; ++sample) {
        assert(!input.take());
        assert(publisher.post(ServoPublishChannel::feedback, [&, sample] {
            sent.push_back(sample);
            if (sample == 2000) published.set_value();
        }));
        ++cycles;  // The real control thread exchanges EtherCAT here.
    }
    assert(cycles == 2000);
    assert(std::chrono::steady_clock::now() - started < 500ms);
    assert(published_future.wait_for(0ms) == std::future_status::timeout);

    stop.store(std::make_shared<int>(1), true);
    stop.store(std::make_shared<int>(2), true);
    assert(*stop.take() == 1);  // Ordinary samples cannot overwrite a latched stop.
    assert(published_future.wait_for(3s) == std::future_status::ready);
    receiver.join();
    release.join();
    publisher.stop();
    assert((sent == std::vector<int>{0, 2000}));  // No replay of old queued samples.
    assert(*input.take() == 999);
}

void test_incoming_snapshots_are_consistent_and_latest() {
    struct Sample {
        int sequence;
        std::array<int, 14> joints;
    };
    LatestRosInput<std::shared_ptr<Sample>> input;
    std::atomic<bool> finished{false};
    std::thread receiver([&] {
        for (int i = 1; i <= 20000; ++i) {
            auto sample = std::make_shared<Sample>();
            sample->sequence = i;
            sample->joints.fill(i);
            input.store(std::move(sample));
        }
        finished = true;
    });
    int previous = 0;
    while (!finished) {
        if (auto sample = input.take()) {
            assert(sample->sequence > previous);
            previous = sample->sequence;
            for (int value : sample->joints) assert(value == sample->sequence);
        }
    }
    receiver.join();
    if (auto sample = input.take()) previous = sample->sequence;
    assert(previous == 20000);
}

void test_publication_owns_its_snapshot() {
    struct FakePublisher {
        int received = 0;
        std::promise<void> done;
        void publish(const std::vector<int>& value) {
            received = value.at(0);
            done.set_value();
        }
    };
    ServoRosPublisher publisher;
    auto output = std::make_shared<FakePublisher>();
    auto done = output->done.get_future();
    publisher.start();
    std::vector<int> snapshot{42};
    while (!publisher.publish(ServoPublishChannel::feedback, output, snapshot)) {}
    snapshot[0] = 123;
    assert(done.wait_for(1s) == std::future_status::ready);
    publisher.stop();
    assert(output->received == 42);
    assert(!publisher.post(ServoPublishChannel::feedback, [] {}));
}

void test_publication_failure_is_reported() {
    ServoRosPublisher publisher;
    publisher.start();
    while (!publisher.post(ServoPublishChannel::feedback, [] { throw std::runtime_error("DDS failure"); })) {}
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!publisher.failed() && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
    assert(publisher.failed());
    assert(!publisher.post(ServoPublishChannel::feedback, [] {}));
    publisher.stop();
}

int main() {
    test_blocked_ros_io_does_not_block_control();
    test_incoming_snapshots_are_consistent_and_latest();
    test_publication_owns_its_snapshot();
    test_publication_failure_is_reported();
}

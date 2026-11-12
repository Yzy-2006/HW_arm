#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

// The receiver may wait for this mutex; the control thread only tries once.
template<class T>
class LatestRosInput {
public:
    void store(T value, bool keep_first = false) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!keep_first || !value_) std::swap(value_, value);
    }

    T take() {
        T value{};
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (lock.owns_lock()) std::swap(value_, value);
        return value;
    }

private:
    std::mutex mutex_;
    T value_{};
};

enum class ServoPublishChannel : size_t {
    feedback, gripper, debug1, debug2, debug3, control_test, compensation_test, comparison, status, log, count
};

// One in-flight task and one pending task per channel. DDS runs outside the mutex.
class ServoRosPublisher {
public:
    ~ServoRosPublisher() { stop(); }

    void start() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (thread_.joinable()) return;
        stopping_ = false;
        thread_ = std::thread([this] { run(); });
    }

    bool post(ServoPublishChannel channel, std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
            if (!lock.owns_lock() || stopping_ || failed_) return false;
            tasks_[static_cast<size_t>(channel)].swap(task);
        }
        ready_.notify_one();
        return true;
    }

    template<class Publisher, class Message>
    bool publish(ServoPublishChannel channel, Publisher publisher, Message message) {
        return post(channel, [publisher, message = std::move(message)] {
            publisher->publish(message);
        });
    }

    bool failed() const { return failed_.load(); }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_one();
        if (thread_.joinable()) thread_.join();
    }

private:
    bool pending() const {
        for (const auto& task : tasks_) if (task) return true;
        return false;
    }

    void run() noexcept {
        try {
            size_t next = 0;
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    ready_.wait(lock, [this] { return stopping_ || pending(); });
                    if (stopping_) return;
                    for (size_t i = 0; i < tasks_.size(); ++i) {
                        const auto index = (next + i) % tasks_.size();
                        if (!tasks_[index]) continue;
                        task.swap(tasks_[index]);
                        next = (index + 1) % tasks_.size();
                        break;
                    }
                }
                task();
            }
        } catch (...) {
            failed_ = true;
        }
    }

    std::mutex mutex_;
    std::condition_variable ready_;
    std::array<std::function<void()>, static_cast<size_t>(ServoPublishChannel::count)> tasks_{};
    bool stopping_ = true;
    std::atomic<bool> failed_{false};
    std::thread thread_;
};

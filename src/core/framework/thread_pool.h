#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>

namespace Framework {
class ThreadPool {
public:
    static ThreadPool& Instance();
    void Enqueue(std::function<void()> task);
private:
    ThreadPool(size_t threads = 4);
    ~ThreadPool();
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    bool stop_;
};
}

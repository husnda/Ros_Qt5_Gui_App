# Optimize Message Bus Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resolve main thread congestion by allowing `MessageBus` callbacks to execute in a background thread pool instead of forcing everything onto the Qt UI thread.

**Architecture:** Introduce a `ThreadPool` in the `Framework` layer. Enhance `MessageBus` to support `ExecutionPolicy` (MainThread vs. Background). High-frequency/heavy-processing display components will be migrated to the background policy.

**Tech Stack:** C++11/17 (Threads, Mutex, Atomic), Qt5 (QCoreApplication, QTimer for UI dispatching).

---

### Task 1: Baseline & Staged Fixes

**Files:**
- Modify: `src/mainwindow/display/manager/display_manager.cpp` (already staged)

- [ ] **Step 1: Verify the staged change**

Ensure the null-pointer fix in `display_manager.cpp` is correct and ready for commit.

- [ ] **Step 2: Commit the staged change**

```bash
git commit -m "fix: resolve null-pointer issue in display_manager.cpp"
```

---

### Task 2: Implement Framework ThreadPool

**Files:**
- Create: `src/core/framework/thread_pool.h`
- Create: `src/core/framework/thread_pool.cpp`
- Modify: `src/core/framework/CMakeLists.txt`

- [ ] **Step 1: Create ThreadPool header**

Define a simple, fixed-size thread pool to handle background callbacks.

```cpp
// src/core/framework/thread_pool.h
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
```

- [ ] **Step 2: Implement ThreadPool**

```cpp
// src/core/framework/thread_pool.cpp
#include "thread_pool.h"

namespace Framework {
ThreadPool& ThreadPool::Instance() {
    static ThreadPool instance;
    return instance;
}
ThreadPool::ThreadPool(size_t threads) : stop_(false) {
    for(size_t i = 0; i < threads; ++i)
        workers_.emplace_back([this] {
            for(;;) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex_);
                    this->condition_.wait(lock, [this]{ return this->stop_ || !this->tasks_.empty(); });
                    if(this->stop_ && this->tasks_.empty()) return;
                    task = std::move(this->tasks_.front());
                    this->tasks_.pop();
                }
                task();
            }
        });
}
void ThreadPool::Enqueue(std::function<void()> task) {
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        tasks_.emplace(std::move(task));
    }
    condition_.notify_one();
}
ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stop_ = true;
    }
    condition_.notify_all();
    for(std::thread &worker: workers_) worker.join();
}
}
```

- [ ] **Step 3: Update CMakeLists.txt**

Add `thread_pool.cpp` to the `framework` library.

- [ ] **Step 4: Commit**

```bash
git add src/core/framework/thread_pool.* src/core/framework/CMakeLists.txt
git commit -m "feat: add Framework ThreadPool for background task execution"
```

---

### Task 3: Refactor MessageBus for Execution Policies

**Files:**
- Modify: `src/core/framework/message_bus.h`

- [ ] **Step 1: Define ExecutionPolicy**

Add an enum to `MessageBus` to differentiate between UI and Background tasks.

```cpp
// src/core/framework/message_bus.h
namespace Framework {
enum class ExecutionPolicy {
    kMainThread,    // Default: Run on Qt UI thread
    kBackground     // Run on ThreadPool
};
// ...
```

- [ ] **Step 2: Update Subscriber Storage**

Modify the internal subscriber map to store the policy alongside the callback.

```cpp
// Update this structure in MessageBus class
struct SubscriberEntry {
    std::unique_ptr<CallbackBase> callback;
    ExecutionPolicy policy;
};
std::map<std::string, std::map<CallbackId, SubscriberEntry>> subscribers_;
```

- [ ] **Step 3: Update Subscribe method**

Update `Subscribe` to accept `ExecutionPolicy`.

```cpp
template<typename T>
CallbackId Subscribe(const std::string& topic, 
                     std::function<void(const T&)> callback, 
                     ExecutionPolicy policy = ExecutionPolicy::kMainThread) {
    std::lock_guard<std::mutex> lock(mutex_);
    CallbackId id = next_callback_id_.fetch_add(1);
    subscribers_[topic][id] = {std::make_unique<TypedCallback<T>>(callback), policy};
    return id;
}
```

- [ ] **Step 4: Update Publish method**

Dispatch tasks based on the subscriber's policy.

```cpp
// Inside MessageBus::Publish loop
auto& entry = pair.second;
auto policy = entry.policy;
auto callback_ptr = entry.callback.get();
auto data_copy = std::make_shared<T>(data);
const std::type_info* type_ptr = &type_id(T);

auto task = [callback_ptr, data_copy, type_ptr]() {
    callback_ptr->call(static_cast<const void*>(data_copy.get()), *type_ptr);
};

if (policy == ExecutionPolicy::kBackground) {
    ThreadPool::Instance().Enqueue(task);
} else {
    detail::ThreadSafeCallbackExecutor::Execute(task);
}
```

- [ ] **Step 5: Commit**

```bash
git add src/core/framework/message_bus.h
git commit -m "feat: enhance MessageBus with ExecutionPolicy support"
```

---

### Task 4: Migrate Heavy Callbacks to Background

**Files:**
- Modify: `src/mainwindow/display/display_occ_map.cpp`
- Modify: `src/channel/ros2/rclcomm.cpp` (and other high-frequency subscribers)

- [ ] **Step 1: Update Occupancy Map Subscription**

Since `ParseOccupyMap` already uses `QtConcurrent::run` internally, we can either let it run on a background thread from the start or keep it as is. However, migrating the `laser_points` processing is more critical.

- [ ] **Step 2: Update Laser Scan processing**

Identify subscribers of `MSG_ID_LASER_SCAN` and set them to `kBackground` if they perform heavy processing. (Wait, the `laser_callback` in `rclcomm` is the *publisher*. The *subscriber* is usually in the display layer).

Let's update `DisplayOccMap` to use `kBackground` and remove the internal `QtConcurrent::run` to simplify.

- [ ] **Step 3: Commit changes**

```bash
git add src/mainwindow/display/display_occ_map.cpp
git commit -m "perf: move Occupancy Map parsing to background policy"
```

---

### Task 5: Verification

- [ ] **Step 1: Run Framework Tests**

Run `src/core/framework/framework_test.cpp` to ensure the message bus still works correctly.

- [ ] **Step 2: Manual Smoke Test**

Launch the app and verify that maps and laser scans still render correctly. Observe UI responsiveness during high-freq data stream.

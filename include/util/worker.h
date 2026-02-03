//
// Created by jiaxv on 2026/1/11.
//

#ifndef OPENLDACS_WORKER_H
#define OPENLDACS_WORKER_H
#include "OpenLdacs.h"

namespace openldacs::util {
    class Worker {
public:
    Worker() = default;
    ~Worker() {
        stop_and_join_noexcept();
    }

    Worker(const Worker&) = delete;
    Worker& operator=(const Worker&) = delete;

    // 启动：把任意可调用对象 f(...) 放到线程里跑
    template <class F, class... Args>
    void start(F&& f, Args&&... args) {
        stop_and_join_noexcept(); // 避免重复 start 泄漏线程

        stop_.store(false, std::memory_order_relaxed);
        eptr_ = nullptr;

        // 参数建议：默认按值捕获（move/copy），避免悬空引用
        t_ = std::thread([this,
                          func = std::forward<F>(f),
                          tup  = std::make_tuple(std::forward<Args>(args)...)
                         ]() mutable {
            try {
                // 把 stop token（this）传进去也行；这里示范直接调用 func(tup...)
                std::apply(std::move(func), std::move(tup));
            } catch (...) {
                eptr_ = std::current_exception();
            }
        });
    }

    // 请求停止：配合线程内部等待点退出
    void request_stop() {
        stop_.store(true, std::memory_order_relaxed);
        cv_.notify_all();
    }

    // join 并在这里统一抛出线程异常（如果有）
    void join_and_rethrow() {
        if (t_.joinable()) t_.join();
        if (eptr_) std::rethrow_exception(eptr_);
    }

    // 提供给线程函数：检查是否需要停止
    bool stop_requested() const {
        return stop_.load(std::memory_order_relaxed);
    }

    // 提供给线程函数：等待某条件或停止信号（避免忙等）
    template <class Pred>
    bool wait_or_stop(std::unique_lock<std::mutex>& lk, Pred pred) {
        cv_.wait(lk, [&] { return stop_requested() || pred(); });
        return !stop_requested();
    }

    std::mutex& mutex() { return m_; }
    std::condition_variable& cv() { return cv_; }

private:
    void stop_and_join_noexcept() noexcept {
        request_stop();
        if (t_.joinable()) {
            try { t_.join(); } catch (...) { /* join 不会抛，这里只是保险 */ }
        }
        // 析构不抛异常：eptr_ 留给显式 join_and_rethrow() 处理
    }

    std::thread t_;
    std::atomic<bool> stop_{false};

    std::mutex m_;
    std::condition_variable cv_;

    std::exception_ptr eptr_{nullptr};
};

}

#endif //OPENLDACS_WORKER_H
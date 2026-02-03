//
// Created by jiaxv on 2026/1/11.
//

#ifndef OPENLDACS_BOUNDED_PRIORITY_QUEUE_H
#define OPENLDACS_BOUNDED_PRIORITY_QUEUE_H
#include "openldacs.h"
#include <optional>

namespace openldacs::util {

    enum class Priority {
        HIGH = 0,
        NORMAL
    };

    template<typename T>
    class BoundedPriorityQueue {
    public:
        explicit BoundedPriorityQueue(const size_t cap_high, const size_t cap_norm) : cap_high_(cap_high),
            cap_norm_(cap_norm) {
        }

        // 非阻塞 push，成功返回 true，队列满返回 false
        bool try_push(const T& item, Priority pri) {
            std::unique_lock<std::mutex> lk(m_);
            auto &q = pri == Priority::HIGH ? high_q_ : norm_q_;
            auto &cap = pri == Priority::HIGH ? cap_high_ : cap_norm_;
            if (q.size() >= cap) {
                return false;
            }
            q.push_back(item);
            cv_not_empty_.notify_one();
            return true;
        }

        void push(const T& item, const Priority pri) {
            std::unique_lock<std::mutex> lk(m_);
            auto &q = pri == Priority::HIGH ? high_q_ : norm_q_;
            auto &cap = pri == Priority::HIGH ? cap_high_ : cap_norm_;
            cv_not_full_.wait(lk, [&]{return  closed_|| q.size() < cap;});
            q.push_back(item);
            cv_not_empty_.notify_one();
        }


        // 非阻塞 pop，成功返回 true 并通过 item 输出，队列空返回 false
        bool try_pop(T& item) {
            std::unique_lock<std::mutex> lk(m_);
            if (high_q_.empty() && norm_q_.empty()) {
                return false;
            }
            if (!high_q_.empty()) {
                item = high_q_.front();
                high_q_.pop_front();
            } else {
                item = norm_q_.front();
                norm_q_.pop_front();
            }
            cv_not_full_.notify_one();
            return true;
        }

        std::optional<T> pop() {
            std::unique_lock<std::mutex> lk(m_);
            cv_not_empty_.wait(lk, [&]{return closed_ || !high_q_.empty() || !norm_q_.empty();});
            T item;
            if (closed_) {
                return std::nullopt;
            }
            if (!high_q_.empty()) {item = high_q_.front(); high_q_.pop_front();}
            else {item = norm_q_.front(); norm_q_.pop_front();}

            cv_not_full_.notify_one();
            return item;
        }

        void close() {
            std::lock_guard<std::mutex> lk(m_);
            closed_ = true;
            cv_not_empty_.notify_all();
            cv_not_full_.notify_all();
        }

    private:
        std::mutex m_;
        std::condition_variable cv_not_empty_;
        std::condition_variable cv_not_full_;
        std::deque<T> high_q_, norm_q_;
        size_t cap_high_, cap_norm_;
        bool closed_ = false;
    };

    template <typename T>
    class BoundedQueue {
    public:
        bool push(const T &msg) {
            std::lock_guard<std::mutex> lk(mu_);
            if (closed_) {
                return false;
            }
            q_.push(msg);
            cv_.notify_one();
            return true;
        }

        std::optional<T> pop_blocking() {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [&]{ return closed_ || !q_.empty(); });
            if (q_.empty()) {
                return std::nullopt;
            }
            T out = q_.front();
            q_.pop();
            return out;
        }

        void close() {
            std::lock_guard<std::mutex> lk(mu_);
            closed_ = true;
            cv_.notify_all();
        }

    private:
        std::queue<T> q_;
        std::mutex mu_;
        std::condition_variable cv_;
        bool closed_ = false;
    };

}

#endif //OPENLDACS_BOUNDED_PRIORITY_QUEUE_H

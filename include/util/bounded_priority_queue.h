//
// Created by jiaxv on 2026/1/11.
//

#ifndef OPENLDACS_BOUNDED_PRIORITY_QUEUE_H
#define OPENLDACS_BOUNDED_PRIORITY_QUEUE_H
#include "openldacs.h"

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

        void push(const T& item, Priority pri) {
            std::unique_lock<std::mutex> lk(m_);
            auto &q = pri == Priority::HIGH ? high_q_ : norm_q_;
            auto &cap = pri == Priority::HIGH ? cap_high_ : cap_norm_;
            cv_not_full_.wait(lk, [&]{return q.size() < cap;});
            q.push_back(item);
            cv_not_empty_.notify_one();
        }

        T pop() {
            std::unique_lock<std::mutex> lk(m_);
            cv_not_empty_.wait(lk, [&]{return !high_q_.empty() || !norm_q_.empty();});
            T item;
            if (!high_q_.empty()) {item = high_q_.front(); high_q_.pop_front();}
            else {item = norm_q_.front(); norm_q_.pop_front();}

            cv_not_full_.notify_one();
            return item;
        }

    private:
        std::mutex m_;
        std::condition_variable cv_not_empty_;
        std::condition_variable cv_not_full_;
        std::deque<T> high_q_, norm_q_;
        size_t cap_high_, cap_norm_;
    };


}

#endif //OPENLDACS_BOUNDED_PRIORITY_QUEUE_H
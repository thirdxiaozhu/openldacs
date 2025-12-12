//
// Created by 邹嘉旭 on 2025/12/11.
//

#ifndef OPENLDACS_UTIL_H
#define OPENLDACS_UTIL_H
#include <iomanip>
#include <ios>
#include <vector>
#include <random>
#include <iostream>

namespace openldacs::util {

    inline std::vector<uint8_t> generateRandomBytes(size_t count) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<uint8_t> dis(0, 255);

        std::vector<uint8_t> bytes(count);
        for (size_t i = 0; i < count; ++i) {
            bytes[i] = dis(gen);
        }
        return bytes;
    }

    inline std::ostream& operator<<(std::ostream& os, const uint8_t& v) {
        return os << std::hex << std::uppercase
              << std::setw(2) << std::setfill('0')
              << static_cast<int>(v);
    }

    template <typename T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
        os << "[ ";
        for (const auto& x : v)
            os << x << " ";
        os << "]";
        return os;
    }
}

#endif //OPENLDACS_UTIL_H
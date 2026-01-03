//
// Created by 邹嘉旭 on 2026/1/3.
//

#ifndef OPENLDACS_REED_SOLOMON_H
#define OPENLDACS_REED_SOLOMON_H

#include <correct.h>
#include <stdexcept>
#include <vector>

class ReedSolomon {
public:
    ReedSolomon(const uint8_t n, const uint8_t k)
        : n_(n),
          k_(k),
          reed_solomon_(correct_reed_solomon_create(correct_rs_primitive_polynomial_8_4_3_2_0, 1, 1, n-k)) {
    }

    void rsEncode(const std::vector<uint8_t> &input, std::vector<uint8_t> &output) const {
        if (correct_reed_solomon_encode(reed_solomon_, input.data(), k_, output.data()) == -1) {
            throw std::runtime_error("reed solomon encode failed");
        }
    }

    void rsDecode(const std::vector<uint8_t> &input, std::vector<uint8_t> &output) const {

        if (correct_reed_solomon_decode(reed_solomon_, input.data(), n_, output.data()) == -1) {
            throw std::runtime_error("reed solomon decode failed");
        }
    }


private:
    uint8_t n_, k_;
    correct_reed_solomon* reed_solomon_;
};

#endif //OPENLDACS_REED_SOLOMON_H
//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_OPENLDACS_H
#define OPENLDACS_OPENLDACS_H
#pragma once
#include <cstddef>
#include <cstdint>
#include <array>
#include <iostream>
#include <spdlog/spdlog.h>
#include <iomanip>
#include <ios>
#include <vector>
#include <random>
#include <iostream>
#include <itpp/comm/punct_convcode.h>
#include <itpp/base/vec.h>
#include <itpp/base/mat.h>
#include <map>
#include <eigen3/Eigen/Dense>
#include <span>

namespace openldacs {
    enum class CMS : int {
        QPSK_R12 = 0,
        QPSK_R23,
        QPSK_R34,
        QAM16_R12,
        QAM16_R23,
        QAM64_R12,
        QAM64_R23,
        QAM64_R34,
    };
}


#endif //OPENLDACS_OPENLDACS_H
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
#include <fstream>
#include <deque>
#include <signal.h>
#include <queue>

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

    enum CHANNEL: int {
        BCCH1_3 = 0,
        BCCH2,
        CCCH_DCH,
        FL_DCH,
    };

    enum ROLE: uint8_t{
        AS = 0x01,
        GS = 0x02,
    };

    using MVecU8 = std::vector<std::vector<uint8_t>>;
    using VecU8 = std::vector<uint8_t>;
    using VecCD = std::vector<std::complex<double>>;
}


#endif //OPENLDACS_OPENLDACS_H
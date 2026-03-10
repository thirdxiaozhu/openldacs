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
#include <itpp/itcomm.h>
#include <map>
#include <span>
#include <fstream>
#include <deque>
#include <signal.h>
#include <queue>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <queue>
#include <thread>

namespace openldacs {
    enum class CMS : uint8_t {
        QPSK_R12 = 0x00,
        QPSK_R23 = 0x01,
        QPSK_R34 = 0x02,
        QAM16_R12 = 0x10,
        QAM16_R23 = 0x11,
        QAM64_R12 = 0x20,
        QAM64_R23 = 0x21,
        QAM64_R34 = 0x22,
    };

    enum ChannelSlot: int {
        BCCH1_3 = 0,
        BCCH2,
        CCCH_DCH,
        FL_DCH,
        RACH,
        DCCH,
        RL_DCH,
    };

    enum ROLE: uint8_t{
        AS = 0x01,
        GS = 0x02,
    };

    using MVecU8 = std::vector<std::vector<uint8_t>>;
    using VecU8 = std::vector<uint8_t>;
    using VecCD = std::vector<std::complex<double>>;
    using VecCF = std::vector<std::complex<float>>;

    class OpenLdacsConfig {
    public:
        static OpenLdacsConfig& getInstance() {
            static OpenLdacsConfig instance; // 线程安全的懒加载
            return instance;
        }

        // 删除拷贝构造函数和赋值操作符
        OpenLdacsConfig(const OpenLdacsConfig&) = delete;
        OpenLdacsConfig& operator=(const OpenLdacsConfig&) = delete;

        [[nodiscard]] ROLE getRole() const {
            return role_;
        }

        void setRole(ROLE role) {
            role_ = role;
        }

        std::string getVersion() {
            return version_;
        }

    private:
        // 私有构造函数，防止外部直接创建实例
        OpenLdacsConfig() = default;
        ROLE role_ = AS;
        std::string version_ = "0.1.0";
    };

}


#endif //OPENLDACS_OPENLDACS_H
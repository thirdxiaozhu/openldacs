//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_FL_H
#define OPENLDACS_FL_H
#pragma once


#include "config.h"
#include "openldacs.h"
#include "Eigen/Dense"

namespace openldacs::phy::fl {
    using namespace phy::config;
    inline constexpr std::size_t n_fl_bc13_ofdm_symb = 13;
    inline constexpr std::size_t n_fl_bc2_ofdm_symb = 24;
    inline constexpr std::size_t n_sync_symb = 2;
    inline constexpr std::int64_t pos_sync1 = 0;
    inline constexpr std::int64_t pos_sync2 = 1;
    inline constexpr std::int64_t guard_left = 7;
    inline constexpr std::int64_t guard_right = 6;
    inline constexpr std::array<int, 12> sync_ind1 = {
        -24 + n_fft/2,
        -20 + n_fft/2,
        -16 + n_fft/2,
        -12 + n_fft/2,
        -8 + n_fft/2,
        -4 + n_fft/2,
        4 + n_fft/2,
        8 + n_fft/2,
        12 + n_fft/2,
        16 + n_fft/2,
        20 + n_fft/2,
        24 + n_fft/2
    };
    inline constexpr std::array<int, 24> sync_ind2 = {
        -24 + n_fft/2,
        -22 + n_fft/2,
        -20 + n_fft/2,
        -18 + n_fft/2,
        -16 + n_fft/2,
        -14 + n_fft/2,
        -12 + n_fft/2,
        -10 + n_fft/2,
        -8 + n_fft/2,
        -6 + n_fft/2,
        -4 + n_fft/2,
        -2 + n_fft/2,
        2 + n_fft/2,
        4 + n_fft/2,
        6 + n_fft/2,
        8 + n_fft/2,
        10 + n_fft/2,
        12 + n_fft/2,
        14 + n_fft/2,
        16 + n_fft/2,
        18 + n_fft/2,
        20 + n_fft/2,
        22 + n_fft/2,
        24 + n_fft/2,
    };

    enum class FLType: int {
        BC1_3 = 1,
        BC2,
        FL_DATA
     };

    class FLChannelHandler {
    public:
        virtual ~FLChannelHandler() = default;
        virtual void handle(const std::vector<uint8_t>&input) const = 0;
    };

    class BC1_3Handler:public FLChannelHandler {
    public:
        void handle(const std::vector<uint8_t>&input) const override;
    };

    class BC2Handler:public FLChannelHandler {
    public:
        void handle(const std::vector<uint8_t>&input) const override;
    };

    class FLDataHandler:public FLChannelHandler {
    public:
        explicit FLDataHandler() = default;

        void handle(const std::vector<uint8_t>&input) const override;

    private:
        static constexpr std::size_t n_fl_ofdm_symb_ = 54;
        static Eigen::MatrixXi init_frame_pattern();
        inline static Eigen::MatrixXi frame_pattern_ = init_frame_pattern();
    };

    class PhyFl {
    public:
        struct FLConfig {
        };

        explicit PhyFl();

        void process_fl_pkt(FLType type, const std::vector<uint8_t> &input) const;

    private:
        FLConfig config_;

        static std::unique_ptr<FLChannelHandler> make_handler(const FLType type) {
            switch (type) {
                case FLType::BC1_3:   return std::make_unique<BC1_3Handler>();
                case FLType::BC2:     return std::make_unique<BC2Handler>();
                case FLType::FL_DATA: return std::make_unique<FLDataHandler>();
                default: throw std::runtime_error("Unknown FLType");
            }
        }
    };
}

#endif //OPENLDACS_FL_H
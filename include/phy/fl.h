//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_FL_H
#define OPENLDACS_FL_H
#pragma once


#include "config.h"
#include "openldacs.h"
#include "Eigen/Dense"
#include "link.h"

namespace openldacs::phy::link::fl {
    class FLChannelHandler;
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

    inline constexpr std::array<int, 4> pilot_set0 = {
        -25 + n_fft/2,
        -1 + n_fft/2,
        1 + n_fft/2,
        25 + n_fft/2,
    };

    inline constexpr std::array<int, 2> pilot_set1 = {
        -17 + n_fft/2,
        17 + n_fft/2,
    };

    inline constexpr std::array<int, 4> pilot_set2 = {
        -21 + n_fft/2,
        -13 + n_fft/2,
        13 + n_fft/2,
        21 + n_fft/2,
    };

    inline constexpr std::array<int, 4> pilot_set3 = {
        -25 + n_fft/2,
        -9 + n_fft/2,
        9 + n_fft/2,
        25 + n_fft/2,
    };

    inline constexpr std::array<int, 2> pilot_set4 = {
        -5 + n_fft/2,
        5 + n_fft/2,
    };

    inline constexpr std::array<int, 2> pilot_set5 = {
        -1 + n_fft/2,
        1 + n_fft/2,
    };

    inline constexpr std::array<int, 14> pilot_set6 = {
        -25 + n_fft/2,
        -21 + n_fft/2,
        -17 + n_fft/2,
        -13 + n_fft/2,
        -9 + n_fft/2,
        -5 + n_fft/2,
        -1 + n_fft/2,
        1 + n_fft/2,
        5 + n_fft/2,
        9 + n_fft/2,
        13 + n_fft/2,
        17 + n_fft/2,
        21 + n_fft/2,
        25 + n_fft/2,
    };

    class PhyFl: public LinkBase {
    public:
        struct FLConfig {
            std::vector<std::vector<int>> pilot_sets = {
                {pilot_set1.begin(), pilot_set1.end()},
                {pilot_set2.begin(), pilot_set2.end()},
                {pilot_set3.begin(), pilot_set3.end()},
                {pilot_set4.begin(), pilot_set4.end()},
                {pilot_set5.begin(), pilot_set5.end()},
            };
        };

        explicit PhyFl();
        void process_packet(ChannelType type, const std::vector<uint8_t> &input) const override;

    private:
        FLConfig config_;
        std::unique_ptr<FLChannelHandler> make_handler(ChannelType type) const;
    };

    class FLChannelHandler {
    public:
        explicit FLChannelHandler(const PhyFl::FLConfig& config) : config_(config) {}
        virtual ~FLChannelHandler() = default;
        virtual void handle(const std::vector<uint8_t>&input) const = 0;
    protected:
        const PhyFl::FLConfig& config_;
        std::vector<int8_t> data_ind_;
        std::vector<int8_t> pilot_ind_;
        Eigen::MatrixXi frame_pattern_;

        virtual void init_frame_pattern()  = 0;
        static void set_data_pilot_index();

    };

    class BC1_3Handler:public FLChannelHandler {
    public:
        explicit BC1_3Handler(const PhyFl::FLConfig& config) : FLChannelHandler(config) {}
        void handle(const std::vector<uint8_t>&input) const override;
    private:
        void init_frame_pattern()  override;
    };

    class BC2Handler:public FLChannelHandler {
    public:
        explicit BC2Handler(const PhyFl::FLConfig& config) : FLChannelHandler(config) {}
        void handle(const std::vector<uint8_t>&input) const override;
    private:
        void init_frame_pattern()  override;
    };

    class FLDataHandler:public FLChannelHandler {
    public:
        explicit FLDataHandler(const PhyFl::FLConfig& config) : FLChannelHandler(config) {
            FLDataHandler::init_frame_pattern();
        }
        void handle(const std::vector<uint8_t>&input) const override;

    private:
        static constexpr std::size_t n_fl_ofdm_symb_ = 54;
        void init_frame_pattern() override;
    };
}

#endif //OPENLDACS_FL_H
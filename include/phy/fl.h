//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_FL_H
#define OPENLDACS_FL_H
#pragma once


#include "config.h"
#include "fl.h"
#include "openldacs.h"
#include "link.h"

namespace openldacs::phy::link::fl {
    class FLChannelHandler;
    class BC1_3Handler;
    class BC2Handler;
    class FLDataHandler;
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
        -24 + n_fft*3/2,
        -22 + n_fft*3/2,
        -20 + n_fft*3/2,
        -18 + n_fft*3/2,
        -16 + n_fft*3/2,
        -14 + n_fft*3/2,
        -12 + n_fft*3/2,
        -10 + n_fft*3/2,
        -8 + n_fft*3/2,
        -6 + n_fft*3/2,
        -4 + n_fft*3/2,
        -2 + n_fft*3/2,
        2 + n_fft*3/2,
        4 + n_fft*3/2,
        6 + n_fft*3/2,
        8 + n_fft*3/2,
        10 + n_fft*3/2,
        12 + n_fft*3/2,
        14 + n_fft*3/2,
        16 + n_fft*3/2,
        18 + n_fft*3/2,
        20 + n_fft*3/2,
        22 + n_fft*3/2,
        24 + n_fft*3/2,
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
        5 + n_fft/2 ,
        9 + n_fft/2,
        13 + n_fft/2,
        17 + n_fft/2,
        21 + n_fft/2,
        25 + n_fft/2,
    };

    using cd = std::complex<double>;
    inline constexpr std::array<cd, 4> pilot_seed0 = {
        1, -1, -1 , -1
    };

    inline constexpr std::array<cd, 2> pilot_seed1 = {
        1, -1
    };

    inline constexpr std::array<cd, 4> pilot_seed2 = {
        1, 1, cd(0, 1), cd(0, -1)
    };

    inline constexpr std::array<cd, 4> pilot_seed3 = {
        1, -1, cd(0, -1), cd(0, -1)
    };

    inline constexpr std::array<cd, 2> pilot_seed4 = {
        1,  cd(0, -1)
    };

    inline constexpr std::array<cd, 2> pilot_seed5 = {
        1,  -1
    };

    inline constexpr std::array<cd, 14> pilot_seed6 = {
        1, cd(0, -1), cd(0, 1), 1, cd(0, 1), cd(0, 1),
                         -1, -1, cd(0, 1), cd(0, 1), 1, cd(0, 1), cd(0, -1), 1
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

            std::vector<std::vector<cd>> pilot_seeds = {
                {pilot_seed1.begin(), pilot_seed1.end()},
                {pilot_seed2.begin(), pilot_seed2.end()},
                {pilot_seed3.begin(), pilot_seed3.end()},
                {pilot_seed4.begin(), pilot_seed4.end()},
                {pilot_seed5.begin(), pilot_seed5.end()},
            };

        };

        explicit PhyFl()
            :
             bc13_(std::make_unique<BC1_3Handler>(config_)),
             bc2_(std::make_unique<BC2Handler>(config_)),
             data_(std::make_unique<FLDataHandler>(config_)) {
        }
        void process_packet(ChannelType type, const std::vector<uint8_t> &input) const override;

    private:
        FLConfig config_;
        std::unique_ptr<BC1_3Handler> bc13_;
        std::unique_ptr<BC2Handler> bc2_;
        std::unique_ptr<FLDataHandler> data_;

        FLChannelHandler &get_handler(ChannelType type) const;
    };

    class FLChannelHandler {
    public:

        enum class ModulationType : int {
            _QPSK = 0,
            _16QAM,
            _64QAM,
        };

        struct CodingParams {
            ModulationType modulation_type;
            double coding_rate;                     //0.5 / 0.67 / 0.75
            int L = 7;                              // constraint length
            int a = 0, b = 0;                       // a / b
            std::vector<int> puncpat;               // 0/1 pattern; empty or {0} means "no puncture"
            int term_bits = 6;                      // L-1
            itpp::Punctured_Convolutional_Code cc;
            double rate_rs = 0.9;
        };

        enum class SymbolValue : int {
            GUARD = 0,
            DATA = 1,
            PILOT = 2,
        };

        struct ParamStruct {
            struct FrameInfo {
                std::vector<int> data_ind;
                std::vector<int> pilot_ind;
                Eigen::MatrixXi frame_pattern;
                size_t n_data = 0;
                size_t n_pilot = 0;
                Eigen::MatrixXi data_ind_packet;
                Eigen::MatrixXi pilot_ind_packet;
                Eigen::MatrixXi sync_ind_packet;

                std::vector<cd> pilot_seeds;
                std::vector<std::vector<cd>> sync_symbols;
            };

            FrameInfo frame_info_;
        };


        virtual ~FLChannelHandler() = default;
        virtual void handle(const std::vector<uint8_t>&input) const = 0;
    protected:
        FLChannelHandler(const PhyFl::FLConfig& config, ParamStruct init)
            : config_(config),
              params_(std::move(init)),
              coding_table_([this]() {
                  std::map<std::tuple<ModulationType, double>, CodingParams> map;
                  // 调用初始化函数
                  initialize_coding_table(map);
                  return map;
              }()) {
        }

        const PhyFl::FLConfig& config_;
        ParamStruct params_;
        std::map<std::tuple<ModulationType, double>, CodingParams> coding_table_;

        virtual ParamStruct build_params(const PhyFl::FLConfig &config) = 0;

        virtual void initialize_coding_table(std::map<std::tuple<ModulationType, double>, CodingParams>& table) = 0;

        virtual CodingParams set_coding_params(ModulationType modulation_type, double coding_rate) = 0;

    };

    class BC1_3Handler:public FLChannelHandler {
    public:
        explicit BC1_3Handler(const PhyFl::FLConfig& config) : FLChannelHandler(config, BC1_3Handler::build_params(config)) {}
        void handle(const std::vector<uint8_t>&input) const override;
    private:
        ParamStruct build_params(const PhyFl::FLConfig &config) override {
            ParamStruct params;
            return params;
        }

        void initialize_coding_table(std::map<std::tuple<ModulationType, double>, CodingParams>& table) override {

        };

        CodingParams set_coding_params(ModulationType modulation_type, double coding_rate) override {
            CodingParams params;
            return params;
        }
    };

    class BC2Handler:public FLChannelHandler {
    public:
        explicit BC2Handler(const PhyFl::FLConfig& config) : FLChannelHandler(config, BC2Handler::build_params(config)) {}
        void handle(const std::vector<uint8_t>&input) const override;
    private:
        ParamStruct build_params(const PhyFl::FLConfig &config) override {
            ParamStruct params;
            return params;
        }

        void initialize_coding_table(std::map<std::tuple<ModulationType, double>, CodingParams>& table) override {

        }

        CodingParams set_coding_params(ModulationType modulation_type, double coding_rate) override {
            CodingParams params;
            return params;
        }
    };

    class FLDataHandler:public FLChannelHandler {
    public:
        explicit FLDataHandler(const PhyFl::FLConfig& config) : FLChannelHandler(config, FLDataHandler::build_params(config)) {
            SPDLOG_INFO("!!!!!!!!!!!!! {}", params_.frame_info_.n_data);
        }
        void handle(const std::vector<uint8_t>&input) const override;

    private:
        static constexpr std::size_t n_fl_ofdm_symb_ = 54;
        static constexpr std::size_t n_frames_ = 9;

        static void compose_frame(const PhyFl::FLConfig &config, ParamStruct::FrameInfo& frame_info);
        static void set_pilots_sync_symbol(const PhyFl::FLConfig &config, const ParamStruct::FrameInfo& frame_info);
        static void build_frame_info(const PhyFl::FLConfig &config, ParamStruct::FrameInfo& frame_info);
        ParamStruct build_params(const PhyFl::FLConfig &config) override;

        void initialize_coding_table(std::map<std::tuple<ModulationType, double>, CodingParams>& table) override;

        CodingParams set_coding_params(ModulationType modulation_type, double coding_rate) override;
    };
}

#endif //OPENLDACS_FL_H
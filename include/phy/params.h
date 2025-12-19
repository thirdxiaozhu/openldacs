//
// Created by 邹嘉旭 on 2025/12/20.
//

#ifndef OPENLDACS_PARAMS_H
#define OPENLDACS_PARAMS_H

#include "openldacs.h"
#include  "link.h"
#include  "util/util.h"

namespace openldacs::phy::params {
    using namespace phy::link;
    using cd = std::complex<double>;
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

    struct ParamStruct {
        FrameInfo frame_info_;
    };

    enum class ModulationType : int { QPSK, QAM16, QAM64, };
    enum class SymbolValue : int { GUARD = 0, DATA = 1, PILOT = 2, };
    enum class CodingRate : int { R12, R23, R34 };

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


    using CodingKey = std::tuple<ModulationType, CodingRate>;
    struct CodingTable {
        std::map<CodingKey, CodingParams> coding_table;

        void set(ModulationType type, CodingRate rate) {
            coding_table[{type, rate}] = get_coding_params(type, rate);
        }

        static CodingParams get_coding_params(ModulationType type, CodingRate rate);
    };

    template<ChannelType CH>
    struct CodingTableInitializer;

    template<>
    struct CodingTableInitializer<ChannelType::BC1_3> {
        static void initialize(const ParamStruct& params, CodingTable& t) {
        }
    };

    template<>
    struct CodingTableInitializer<ChannelType::BC2> {
        static void initialize(const ParamStruct& params, CodingTable& t) {
        }
    };

    template<>
    struct CodingTableInitializer<ChannelType::FL_DATA> {
        static void initialize(const ParamStruct& params, CodingTable& t) {
            t.set(ModulationType::QPSK, CodingRate::R12);
        }
    };
}

#endif //OPENLDACS_PARAMS_H
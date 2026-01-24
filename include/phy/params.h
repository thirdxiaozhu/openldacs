//
// Created by 邹嘉旭 on 2025/12/20.
//

#ifndef OPENLDACS_PARAMS_H
#define OPENLDACS_PARAMS_H

#include <utility>

#include "config.h"
#include "openldacs.h"
#include "util/reed_solomon.h"
#include  "util/util.h"

namespace openldacs::phy::params {
    using cd = std::complex<double>;

    inline constexpr std::array<uint8_t, 412> random_output = {
        0xBF, 0x03, 0x82, 0x09, 0x0C, 0x36, 0x28, 0xB4, 0xF3, 0xBA, 0x29, 0x9C, 0xF5, 0x4A, 0x3F, 0xBC, 0x81, 0x8B,
        0x05, 0x3A, 0x1E, 0x9C, 0x47, 0x49, 0x93, 0xB5, 0x69, 0xBF, 0x75, 0x83, 0x3D, 0x0A, 0x8E, 0x3F, 0x24, 0x82,
        0xDB, 0x0E, 0xDA, 0x26, 0xDC, 0xD6, 0xCA, 0xF6, 0xBE, 0x37, 0x84, 0xB1, 0x1B, 0xA6, 0x59, 0xD5, 0xD4, 0xFC,
        0xFA, 0x0A, 0x1C, 0x3C, 0x48, 0x89, 0xB3, 0x35, 0xAA, 0xBD, 0xFF, 0x8C, 0x01, 0x28, 0x06, 0xF0, 0x16, 0x20,
        0x74, 0xC1, 0x3A, 0x86, 0x9F, 0x17, 0x42, 0x73, 0x8D, 0x29, 0x2E, 0xF6, 0xE6, 0x36, 0x54, 0xB5, 0xFB, 0xBC,
        0x19, 0x88, 0x55, 0x31, 0xFE, 0xA4, 0x07, 0xD8, 0x10, 0xD0, 0x62, 0xE1, 0x4E, 0x47, 0xA5, 0x91, 0xDD, 0x64,
        0xCF, 0x5A, 0xA3, 0xDF, 0xC8, 0xC0, 0xB2, 0x83, 0xAF, 0x09, 0xE2, 0x34, 0x4C, 0xB9, 0xAB, 0x95, 0xF9, 0x7C,
        0x17, 0x08, 0x72, 0x31, 0x2C, 0xA6, 0xEB, 0xD6, 0x78, 0xF5, 0x12, 0x3E, 0x6C, 0x85, 0x6B, 0x1F, 0x7A, 0x43,
        0x1D, 0x8A, 0x4D, 0x3D, 0xAE, 0x8D, 0xE7, 0x2C, 0x52, 0xE9, 0xEE, 0x74, 0x65, 0x39, 0x5E, 0x97, 0xC7, 0x70,
        0x93, 0x23, 0x6A, 0xCB, 0x7E, 0xBB, 0x07, 0x9A, 0x11, 0x5C, 0x67, 0xC9, 0x50, 0xB7, 0xE3, 0xB0, 0x49, 0xA1,
        0xB5, 0xC5, 0xBC, 0x9D, 0x8B, 0x4D, 0x3B, 0xAE, 0x99, 0xE7, 0x54, 0x53, 0xF9, 0xE8, 0x14, 0x70, 0x79, 0x21,
        0x16, 0xC6, 0x76, 0x95, 0x37, 0x7E, 0xB3, 0x07, 0xAA, 0x11, 0xFC, 0x64, 0x09, 0x58, 0x37, 0xD0, 0xB0, 0xE3,
        0xA2, 0x49, 0xCD, 0xB4, 0xAD, 0xBB, 0xED, 0x98, 0x6D, 0x51, 0x6F, 0xE7, 0x60, 0x53, 0x41, 0xEB, 0x84, 0x79,
        0x19, 0x16, 0x56, 0x75, 0xF5, 0x3C, 0x3E, 0x88, 0x87, 0x33, 0x12, 0xAA, 0x6F, 0xFD, 0x60, 0x0F, 0x40, 0x23,
        0x80, 0xC9, 0x02, 0xB6, 0x0F, 0xB4, 0x21, 0xB8, 0xC5, 0x92, 0x9D, 0x6F, 0x4F, 0x63, 0xA3, 0x49, 0xCB, 0xB4,
        0xB9, 0xBB, 0x95, 0x99, 0x7D, 0x57, 0x0F, 0xF2, 0x20, 0x2C, 0xC0, 0xEA, 0x82, 0x7F, 0x0D, 0x02, 0x2E, 0x0C,
        0xE4, 0x2A, 0x58, 0xFD, 0xD2, 0x0C, 0xEC, 0x2A, 0x68, 0xFD, 0x72, 0x0F, 0x2C, 0x22, 0xE8, 0xCE, 0x72, 0xA5,
        0x2F, 0xDE, 0xE0, 0xC6, 0x42, 0x95, 0x8F, 0x7D, 0x23, 0x0E, 0xCA, 0x26, 0xBC, 0xD7, 0x8A, 0xF1, 0x3E, 0x26,
        0x84, 0xD7, 0x1A, 0xF2, 0x5E, 0x2D, 0xC4, 0xEC, 0x9A, 0x6B, 0x5D, 0x7B, 0xCF, 0x18, 0xA2, 0x53, 0xCD, 0xE8,
        0xAC, 0x73, 0xE9, 0x28, 0x76, 0xF1, 0x36, 0x26, 0xB4, 0xD7, 0xBA, 0xF1, 0x9E, 0x25, 0x44, 0xDF, 0x9A, 0xC1,
        0x5E, 0x87, 0xC7, 0x10, 0x92, 0x63, 0x6D, 0x4B, 0x6F, 0xBB, 0x61, 0x9B, 0x45, 0x5B, 0x9F, 0xD9, 0x40, 0xD7,
        0x82, 0xF1, 0x0E, 0x26, 0x24, 0xD4, 0xDA, 0xFA, 0xDE, 0x1E, 0xC4, 0x46, 0x99, 0x97, 0x55, 0x73, 0xFF, 0x28,
        0x02, 0xF0, 0x0E, 0x20, 0x24, 0xC0, 0xDA, 0x82, 0xDF, 0x0E, 0xC2, 0x26, 0x8C, 0xD7, 0x2A, 0xF2,
    };
    struct FrameInfo {
        std::vector<int> data_ind;
        std::vector<int> pilot_ind;
        std::vector<int> sync_ind;
        itpp::imat frame_pattern;
        size_t n_data = 0;
        size_t n_pilot = 0;
        size_t n_sync1 = 12;
        size_t n_sync2 = 24;

        itpp::cvec pilot_seeds;
        itpp::cvec sync_symbols1;
        itpp::cvec sync_symbols2;

        itpp::cmat frame;
    };

    struct SyncParam {
        int corr_len1 = config::n_fft / 2 + config::n_g + config::n_ws / 2;
        int corr_diff1 = config::n_fft / 2;

        int corr_len2 = 3 * config::n_fft / 4 + config::n_g + config::n_ws / 2;
        int corr_diff2 = config::n_fft / 4;

        int upsample_rate = 1;
        int t_upsample = config::t_sample / upsample_rate;
        double threshold_peek = 0.4;

        itpp::vec M1;
        itpp::vec M2;
        itpp::vec angle1;
        itpp::vec angle2;
        itpp::vec freq1;
        itpp::vec freq2;

        void coarse_sync(const itpp::cvec &input);
    private:
        void find_peaks(std::vector<int> &peak_indices, std::vector<double> &peak_values);
        void find_reliable_peak(std::vector<int> &peak_indices, std::vector<double> &peak_values);

        void get_peak(const itpp::vec &input, const int start, const int end, double &peak_value, int &peak_ind);
        void frame_sync(const itpp::cvec &input);
        void find_sync_instances();
        void sync_correlation(const itpp::cvec &input, int corr_len, int corr_diff, itpp::vec &M, itpp::vec &angle_metric);
    };



    enum class ModulationType : int { QPSK, QAM16, QAM64, };
    enum class SymbolValue : int { GUARD = 0, DATA = 1, PILOT = 2, };
    // enum class CodingRate : int { R12, R23, R34 };
    struct HelicalInterleaverParams {
        int a, b = 0;
        std::vector<int> pattern;
        HelicalInterleaverParams(const int a, const int b) : a(a), b(b) {
        }
    };

    struct RSCoderParams {
        int n, k;
        int bits_uncoded;
        int bits_after_rs;
        util::ReedSolomon rs;

        RSCoderParams(const int n_val, const int k_val) : n(n_val), k(k_val), rs(n_val, k_val) {
            bits_uncoded = k_val * 8;      // 假设每个符号8位
            bits_after_rs = n_val * 8;
        }
    };

    struct ConvCodingParams {
        int bits_before_cc;
        int bits_coded;
        int pad_bits_after_rs;
        int pad_bits_after_cc;
    };

    struct CodingParams {
        ModulationType mod_type = ModulationType::QPSK;
        double coding_rate = 0.5;                     //0.5 / 0.67 / 0.75
        int rs_per_pdu = 1;

        int pdu_per_frame = 3;
        int bits_per_symb = 2;
        int bits_per_pdu = 0;
        int bytes_per_pdu = 0;
        mutable itpp::Punctured_Convolutional_Code cc;
        std::vector<int> puncpat;               // 0/1 pattern; empty or {0} means "no puncture"
        int int_size = 1;
        double rate_cod = 0;
        VecU8 randomize_vec;

        // 固定参数
        int L = 7;                              // constraint length
        int term_bits = 6;                      // L-1
        double rate_rs = 0.9;

        // 初始参数
        int a = 0, b = 0;                       // a / b
        int joint_frame = 0;
        HelicalInterleaverParams h_inter_params;
        RSCoderParams rs_params;
        ConvCodingParams conv_params{};

        // 当前是否有意义？
        int cc_cod = 1;
        int interleaver = 1;

        CodingParams(HelicalInterleaverParams h_params, RSCoderParams rs_params): h_inter_params(std::move(h_params)), rs_params(std::move(rs_params)) {
        }
    };


    using CodingKey = std::tuple<CMS, int>;


    struct CodingTable {
        std::map<CodingKey, CodingParams> coding_table;
        FrameInfo& frame_info;

        CodingTable(FrameInfo &frame_info) : frame_info(frame_info) {}

        CodingParams setCodingParams(CodingKey key, CHANNEL ch) const;
        void initCodingTable(std::initializer_list<CodingKey> keys, CHANNEL ch);

        const CodingParams &getCodingParams(const CodingKey &key) const {
            return coding_table.at(key);
        }
    };

    static const std::array<std::pair<CodingKey, CodingParams>, 1> init_bc13_coding_params = {
        {
            {
                {CMS::QPSK_R12, 1},
                CodingParams{HelicalInterleaverParams(43, 28) , RSCoderParams(74, 66)}
            }
        }
    };

    static const std::array<std::pair<CodingKey, CodingParams>, 1> init_bc2_coding_params = {
        {
            {
                {CMS::QPSK_R12, 1},
                CodingParams{HelicalInterleaverParams(40, 56) , RSCoderParams(139, 125)}
            }
        }
    };

    static const std::array<std::pair<CodingKey, CodingParams>, 6> init_fl_coding_params = {
        {
            {
                {CMS::QPSK_R12, 2},
                CodingParams{HelicalInterleaverParams(132, 74) , RSCoderParams(101, 91)}
            },
            {
                {CMS::QPSK_R12, 3},
                CodingParams{HelicalInterleaverParams(111, 132) , RSCoderParams(101, 91)}
            },
            {
                {CMS::QPSK_R23, 2},
                CodingParams{HelicalInterleaverParams(132, 74) , RSCoderParams(134, 120)}
            },
            {
                {CMS::QPSK_R23, 3},
                CodingParams{HelicalInterleaverParams(111, 132) , RSCoderParams(134, 120)}
            },
            {
                {CMS::QPSK_R34, 2},
                CodingParams{HelicalInterleaverParams(132, 74) , RSCoderParams(151, 135)}
            },
            {
                {CMS::QPSK_R34,  3},
                CodingParams{HelicalInterleaverParams(111, 132) , RSCoderParams(151, 135)}
            }
        }
    };

    inline CodingParams get_initial_coding_param(const CodingKey &key, CHANNEL ch) {
        bool is_found = false;
        const CodingParams *params_ptr = nullptr;

        auto find_in_table = [&](const auto &table) {
            for (const auto &kv : table) {
                if (kv.first == key) {
                    params_ptr = &kv.second;
                    return true;
                }
            }
            return false;
        };


        switch (ch) {
            case BCCH1_3:
                is_found = find_in_table(init_bc13_coding_params);
                break;
            case BCCH2:
                is_found = find_in_table(init_bc2_coding_params);
                break;
            case CCCH_DCH:
            case FL_DCH:
                is_found = find_in_table(init_fl_coding_params);
                break;
            default:
                throw std::runtime_error("Unsupported channel for coding params");
        }

        if (!is_found) {
            throw std::runtime_error("No such coding params");
        }

        CodingParams params = *params_ptr; //拷贝构造

        auto [cms, joint_frame] = key;
        params.joint_frame = joint_frame;

        switch (cms) {
            case CMS::QPSK_R12:
                params.rs_per_pdu = 1;
                params.bits_per_symb = 2;
                params.mod_type = ModulationType::QPSK;
                params.a = 1;
                params.b = 2;
                params.coding_rate = 0.5;
                params.puncpat = {1,1};
                break;
            case CMS::QPSK_R23:
                params.rs_per_pdu = 1;
                params.bits_per_symb = 2;
                params.mod_type = ModulationType::QPSK;
                params.a = 2;
                params.b = 3;
                params.coding_rate = 0.67;
                params.puncpat = {1,1,0,1};
                break;
            case CMS::QPSK_R34:
                params.rs_per_pdu = 1;
                params.bits_per_symb = 2;
                params.mod_type = ModulationType::QPSK;
                params.a = 3;
                params.b = 4;
                params.coding_rate = 0.75;
                params.puncpat = {1,1,0,1,1,0};
                break;
            case CMS::QAM16_R12:
                params.rs_per_pdu = 1;
                params.bits_per_symb = 4;
                params.mod_type = ModulationType::QAM16;
                params.a = 1;
                params.b = 2;
                params.coding_rate = 0.5;
                params.puncpat = {1,1};
                break;
            case CMS::QAM16_R23:
                params.rs_per_pdu = 2;
                params.bits_per_symb = 4;
                params.mod_type = ModulationType::QAM16;
                params.a = 2;
                params.b = 3;
                params.coding_rate = 0.67;
                params.puncpat = {1,1,0,1};
                break;
            case CMS::QAM64_R12:
                params.rs_per_pdu = 2;
                params.bits_per_symb = 6;
                params.mod_type = ModulationType::QAM64;
                params.a = 1;
                params.b = 2;
                params.coding_rate = 0.5;
                params.puncpat = {1,1};
                break;
            case CMS::QAM64_R23:
                params.rs_per_pdu = 2;
                params.bits_per_symb = 6;
                params.mod_type = ModulationType::QAM64;
                params.a = 2;
                params.b = 3;
                params.coding_rate = 0.67;
                params.puncpat = {1,1,0,1};
                break;
            case CMS::QAM64_R34:
                params.rs_per_pdu = 2;
                params.bits_per_symb = 6;
                params.mod_type = ModulationType::QAM64;
                params.a = 3;
                params.b = 4;
                params.coding_rate = 0.75;
                params.puncpat = {1,1,0,1,1,0};
                break;
        }

        return params;
    }


}

#endif //OPENLDACS_PARAMS_H
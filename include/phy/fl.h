//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_FL_H
#define OPENLDACS_FL_H
#pragma once


#include "config.h"
#include "openldacs.h"
#include "link.h"
#include "params.h"

namespace openldacs::phy::link::fl {

    class FLChannelHandler;
    class BC1_3Handler;
    class BC2Handler;
    class FLDataHandler;
    using namespace phy::config;
    using namespace phy::params;

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

    inline const std::vector<std::vector<int>> pilot_sets = {
        {pilot_set1.begin(), pilot_set1.end()},
        {pilot_set2.begin(), pilot_set2.end()},
        {pilot_set3.begin(), pilot_set3.end()},
        {pilot_set4.begin(), pilot_set4.end()},
        {pilot_set5.begin(), pilot_set5.end()},
    };

    inline const std::vector<std::vector<cd>> pilot_seeds = {
        {pilot_seed1.begin(), pilot_seed1.end()},
        {pilot_seed2.begin(), pilot_seed2.end()},
        {pilot_seed3.begin(), pilot_seed3.end()},
        {pilot_seed4.begin(), pilot_seed4.end()},
        {pilot_seed5.begin(), pilot_seed5.end()},
    };

    // Result after RS encoding of one SDU (still bytes/bits, not modulated symbols)
    struct RsEncodedUnit {
        // store RS-coded bytes (systematic + parity), whatever your RS encoder outputs
        std::vector<uint8_t> rs_bytes;
        // Keep original metadata for ordering
        uint16_t sdu_index;
    };

    struct BlockKey {
        DirectionType direction;
        uint32_t sf_id;
        uint16_t mf_id;

        // FL specific
        uint8_t acm_id;
        uint8_t fl_block_id;

        // // RL specific
        // uint16_t as_i;
        // uint16_t rl_block_seq;   // 0,1,2... within MF if split by Nlim/10 limit

        bool operator==(const BlockKey & o) const {
            return direction == o.direction &&
               sf_id == o.sf_id &&
               mf_id == o.mf_id &&
               acm_id == o.acm_id &&
               fl_block_id == o.fl_block_id;
        }

        explicit BlockKey(const PhySdu &sdu): direction(sdu.direction),
                                              sf_id(sdu.sf_id),
                                              mf_id(sdu.mf_id),
                                              acm_id(sdu.acm_id)
                                              {
            if (direction == DirectionType::FL) {
                if (sdu.sdu_index >= 1 && sdu.sdu_index <= 6) fl_block_id = 0;
                else if (sdu.sdu_index >= 7 && sdu.sdu_index <= 12) fl_block_id = 1;
                else if (sdu.sdu_index >= 13 && sdu.sdu_index <= 21) fl_block_id = 2;
                else fl_block_id = 3;
            }else {
                // RL
            }
        }
    };

    // interleaver
    struct BlockBuffer {
        size_t interleaver_count;
        bool is_cc;
        std::vector<RsEncodedUnit> units;
        itpp::bvec coded_bits;
    };

    class PhyFl final : public LinkBase {
    public:
        struct FLConfig {
        };

        explicit PhyFl()
            :config_(),
             bc13_(std::make_unique<BC1_3Handler>(config_)),
             bc2_(std::make_unique<BC2Handler>(config_)),
             data_(std::make_unique<FLDataHandler>(config_)) {
        }

        void processPacket(const PhySdu &sdu) const override;

    private:
        FLConfig config_;
        std::unique_ptr<BC1_3Handler> bc13_;
        std::unique_ptr<BC2Handler> bc2_;
        std::unique_ptr<FLDataHandler> data_;

        FLChannelHandler &getHandler(CHANNEL type) const;
    };

    struct BlockKeyHash {
        size_t operator()(const BlockKey& k) const noexcept {
            // crude hash, refine in real code
            size_t h = 14695981039346656037ull;
            auto mix = [&](uint64_t v){
                h ^= v + 0x9e3779b97f4a7c15ull + (h<<6) + (h>>2);
            };
            mix(static_cast<uint64_t>(k.direction));
            mix(k.sf_id);
            mix(k.mf_id);
            mix(k.acm_id);
            mix(k.fl_block_id);
            return h;
        }
    };

    class FLChannelHandler {
    public:
        virtual ~FLChannelHandler() = default;
        virtual void submit(PhySdu sdu, CMS cms) = 0;  // user-specific
        virtual void submit(PhySdu sdu) = 0;  // cell-specific

        const PhyFl::FLConfig& config() const noexcept { return config_; }
        const FrameInfo& frame() const noexcept { return frame_info_; }

        CMS get_cms() const {
            return default_cms_;
        }
        void set_cms(const CMS cms) {
            default_cms_ = cms;
        }

    protected:
        explicit FLChannelHandler(const PhyFl::FLConfig& config)
            : config_(config), coding_table_(frame_info_) {
        }

        const PhyFl::FLConfig& config_;
        FrameInfo frame_info_;
        CodingTable coding_table_;
        std::unordered_map<BlockKey, BlockBuffer, BlockKeyHash> block_map_;
        CMS default_cms_ = CMS::QPSK_R12;
        std::mutex block_m_;

        static size_t getInterleaverCount(const PhySdu &sdu) {
            if (sdu.direction == DirectionType::FL) {
                if (sdu.sdu_index >= 13 && sdu.sdu_index <= 21) return 9;
                return 6;
            }else {
                // RL
                return 0;
            }
        }

        virtual void initCodingTable() = 0;

        // frames
        void buildFrame();
        virtual void getFrameIndices() = 0;
        virtual void calcSequences() = 0;
        virtual void composeFrame() = 0;

        // channel coding
        virtual void channelCoding(BlockBuffer &block, const CodingParams &coding_params) = 0;

        static void randomizer(VecU8 &to_process, const CodingParams &coding_params);
        static RsEncodedUnit rsEncoder(const VecU8 &to_process, uint8_t index, const CodingParams &coding_params);
        static VecU8 blockInterleaver(const std::vector<RsEncodedUnit> &units,
                                      const CodingParams &coding_params);
        itpp::bvec convCode(const VecU8 &input, const CodingParams &coding_params) const;
        static itpp::bvec helicalInterleaver(const itpp::bvec &input, const CodingParams &coding_params);

        // modulation
        static itpp::cvec modulate(BlockBuffer &block, ModulationType mod_type);
        virtual itpp::cmat subcarrier_allocation(const itpp::cvec &input, int joint_frame) = 0;
        static itpp::cmat matrix_ifft(const itpp::cmat &to_process);
        static itpp::cvec windowing(const itpp::cmat &to_process, int joint_frame);

        // demod
        static itpp::cmat matrix_fft(const itpp::cmat &to_process);

    };

    class BC1_3Handler final:public FLChannelHandler {
    public:
        explicit BC1_3Handler(const PhyFl::FLConfig& config) : FLChannelHandler(config) {}
        void submit(PhySdu sdu, CMS cms) override;
        void submit(PhySdu sdu) override;
    private:
        void initCodingTable() override{};
        void getFrameIndices() override {};
        void calcSequences() override{};
        void composeFrame() override{};
        void channelCoding(BlockBuffer &block, const CodingParams &coding_params) override{};
        itpp::cmat subcarrier_allocation(const itpp::cvec &input, const int joint_frame) override {
            return nullptr;
        }
    };

    class BC2Handler final:public FLChannelHandler {
    public:
        explicit BC2Handler(const PhyFl::FLConfig& config) : FLChannelHandler(config) {}
        void submit(PhySdu sdu, CMS cms) override;
        void submit(PhySdu sdu) override;
    private:
        void initCodingTable() override{};
        void getFrameIndices() override {};
        void calcSequences() override{};
        void composeFrame() override{};
        void channelCoding(BlockBuffer &block, const CodingParams &coding_params) override{};
        itpp::cmat subcarrier_allocation(const itpp::cvec &input, const int joint_frame) override {
            return nullptr;
        }
    };

    class FLDataHandler final:public FLChannelHandler {
    public:
        explicit FLDataHandler(const PhyFl::FLConfig& config) : FLChannelHandler(config) {
            buildFrame();
            initCodingTable();
        }
        void submit(PhySdu sdu, CMS cms) override;
        void submit(PhySdu sdu) override;

    private:
        static constexpr std::size_t n_fl_ofdm_symb_ = 54;
        static constexpr std::size_t n_frames_ = 9;

        void initCodingTable() override {
            coding_table_.initCodingTable({
                {CMS::QPSK_R12, 2},
                {CMS::QPSK_R12, 3},
                {CMS::QPSK_R23, 2},
                {CMS::QPSK_R23, 3},
                {CMS::QPSK_R34, 2},
                {CMS::QPSK_R34, 3},
            });
        }

        void getFrameIndices() override;
        void calcSequences() override;
        void composeFrame() override;

        void channelCoding(BlockBuffer &block, const CodingParams &coding_params) override;
        itpp::cmat subcarrier_allocation(const itpp::cvec &input, int joint_frame) override;
    };
}

#endif //OPENLDACS_FL_H
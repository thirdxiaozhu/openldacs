//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_FL_H
#define OPENLDACS_FL_H
#pragma once


#include <queue>

#include "config.h"
#include "OpenLdacs.h"
#include "link.h"
#include "params.h"

namespace openldacs::phy::link::fl {

    class FLChannelHandler;
    class BC1_3Handler;
    class BC2Handler;
    class FLDataHandler;
    using namespace phy::config;
    using namespace phy::params;

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

    struct BlockBuffer {
        size_t interleaver_count;
        bool is_cc;
        std::vector<RsEncodedUnit> units;
        itpp::bvec coded_bits;
        itpp::cvec mod_vec;
        itpp::cmat frames_freq;
        itpp::cmat frame_time;
    };

    class PhySource {
    public:
        explicit PhySource(device::DevPtr& dev): dev_(dev){
            source_worker_.start([&] {
                while (!source_worker_.stop_requested()) {

                    // std::cout << tx_vecs << std::endl;
                }
            });
        }

        void enqueue(const BlockBuffer &buffer, const CHANNEL ch) {
            switch (ch) {
                case BCCH1_3:
                    bc13_queue_.push(buffer);
                    break;
                case BCCH2:
                    bc2_queue_.push(buffer);
                    break;
                case CCCH_DCH:
                case FL_DCH:
                    fl_data_queue_.push(buffer);
                    break;
                default:
                    throw std::runtime_error("Invalid channel");
            }
        }

        ~PhySource() {
            bc13_queue_.close();
            bc2_queue_.close();
            fl_data_queue_.close();
            source_worker_.request_stop();
            source_worker_.join_and_rethrow();
        }
    private:
        device::DevPtr& dev_;
        util::BoundedQueue<BlockBuffer> bc13_queue_;
        util::BoundedQueue<BlockBuffer> bc2_queue_;
        util::BoundedQueue<BlockBuffer> fl_data_queue_;
        util::Worker source_worker_;
    };

    class PhySink {
    public:
        explicit PhySink(device::DevPtr& dev): dev_(dev){
            sink_worker_.start([&] {
                while (!sink_worker_.stop_requested()) {
                    const auto bf = fl_data_queue_.pop_blocking();
                    if (!bf) {
                        break;
                    }

                    const itpp::cvec tx_vecs = windowing(bf.value());
                    // std::cout << tx_vecs << std::endl;

                    dev->sendData(tx_vecs, util::Priority::HIGH);
                }
            });
        }

        void enqueue(const BlockBuffer &buffer, const CHANNEL ch) {
            switch (ch) {
                case BCCH1_3:
                    bc13_queue_.push(buffer);
                    break;
                case BCCH2:
                    bc2_queue_.push(buffer);
                    break;
                case CCCH_DCH:
                case FL_DCH:
                    fl_data_queue_.push(buffer);
                    break;
                default:
                    throw std::runtime_error("Invalid channel");
            }
        }

        ~PhySink() {
            bc13_queue_.close();
            bc2_queue_.close();
            fl_data_queue_.close();
            sink_worker_.request_stop();
            sink_worker_.join_and_rethrow();
        }

    private:
        device::DevPtr& dev_;
        util::BoundedQueue<BlockBuffer> bc13_queue_;
        util::BoundedQueue<BlockBuffer> bc2_queue_;
        util::BoundedQueue<BlockBuffer> fl_data_queue_;
        util::Worker sink_worker_;
        std::optional<itpp::cvec> prev_post_;

        itpp::cvec windowing(const BlockBuffer &block);
    };

    class PhyFl final : public LinkBase {
    public:
        struct FLConfig {
            explicit FLConfig(device::DevPtr& dev): sink_(dev), source_(dev) {
            }
            PhySink sink_;
            PhySource source_;
        };

        explicit PhyFl(device::DevPtr& dev)
            :config_(dev),
             bc13_(std::make_unique<BC1_3Handler>(config_, dev)),
             bc2_(std::make_unique<BC2Handler>(config_, dev)),
             data_(std::make_unique<FLDataHandler>(config_, dev)) {
        }

        void processPacket(const PhySdu &sdu) const override;

    private:
        FLConfig config_;
        std::unique_ptr<BC1_3Handler> bc13_;
        std::unique_ptr<BC2Handler> bc2_;
        std::unique_ptr<FLDataHandler> data_;

        [[nodiscard]] FLChannelHandler &getHandler(CHANNEL type) const;
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
        explicit FLChannelHandler(PhyFl::FLConfig& config, device::DevPtr& dev)
            : config_(config), coding_table_(frame_info_), device_(dev) {
        }

        PhyFl::FLConfig& config_;
        FrameInfo frame_info_;
        CodingTable coding_table_;
        std::unordered_map<BlockKey, BlockBuffer, BlockKeyHash> block_map_;
        CMS default_cms_ = CMS::QPSK_R12;
        std::mutex block_m_;
        device::DevPtr& device_;

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
        void buildFrame(int symbols);
        void getFrameIndices(int symbols);
        void calcSequences(int symbols);
        void composeFrame(int symbols);

        // channel coding
        virtual void channelCoding(BlockBuffer &block, const CodingParams &coding_params) = 0;

        static void randomizer(VecU8 &to_process, const CodingParams &coding_params);
        static RsEncodedUnit rsEncoder(const VecU8 &to_process, uint8_t index, const CodingParams &coding_params);
        static VecU8 blockInterleaver(const std::vector<RsEncodedUnit> &units,
                                      const CodingParams &coding_params);
        itpp::bvec convCode(const VecU8 &input, const CodingParams &coding_params) const;
        static itpp::bvec helicalInterleaver(const itpp::bvec &input, const CodingParams &coding_params);

        // modulation
        static void modulate(BlockBuffer &block, ModulationType mod_type);
        virtual void subcarrier_allocation(BlockBuffer &block, int joint_frame) = 0;
        static void matrix_ifft(BlockBuffer &block);
        static std::vector<itpp::cvec> windowing(const itpp::cmat &to_process, int joint_frame);

        // demod
        static itpp::cmat matrix_fft(const itpp::cmat &to_process);
    };

    class BC1_3Handler final:public FLChannelHandler {
    public:
        explicit BC1_3Handler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev) {
            buildFrame(n_bc13_ofdm_symb);
            initCodingTable();
        }
        void submit(PhySdu sdu, CMS cms) override;
        void submit(PhySdu sdu) override;
    private:
        void initCodingTable() override {
            coding_table_.initCodingTable({
                                              {CMS::QPSK_R12, 1},
                                          }, BCCH1_3);
        };
        void channelCoding(BlockBuffer &block, const CodingParams &coding_params) override{};

        void subcarrier_allocation(BlockBuffer &block, const int joint_frame) override {
        }
    };

    class BC2Handler final:public FLChannelHandler {
    public:
        explicit BC2Handler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev) {
            buildFrame(n_bc2_ofdm_symb);
            initCodingTable();
        }
        void submit(PhySdu sdu, CMS cms) override;
        void submit(PhySdu sdu) override;
    private:
        void initCodingTable() override {
            coding_table_.initCodingTable({
                                              {CMS::QPSK_R12, 1},
                                          }, BCCH2);
        }
        void channelCoding(BlockBuffer &block, const CodingParams &coding_params) override{};

        void subcarrier_allocation(BlockBuffer &block, const int joint_frame) override {
        }
    };

    class FLDataHandler final:public FLChannelHandler {
    public:
        explicit FLDataHandler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev) {
            buildFrame(n_fl_ofdm_symb);
            initCodingTable();
        }
        void submit(PhySdu sdu, CMS cms) override;
        void submit(PhySdu sdu) override;

    private:
        // static constexpr std::size_t n_frames_ = 9;

        void initCodingTable() override {
            coding_table_.initCodingTable({
                                              {CMS::QPSK_R12, 2},
                                              // {CMS::QPSK_R12, 3},
                                              // {CMS::QPSK_R23, 2},
                                              // {CMS::QPSK_R23, 3},
                                              // {CMS::QPSK_R34, 2},
                                              // {CMS::QPSK_R34, 3},
                                          }, FL_DCH);
        }

        void channelCoding(BlockBuffer &block, const CodingParams &coding_params) override;

        void subcarrier_allocation(BlockBuffer &block, int joint_frame) override;
    };
}

#endif //OPENLDACS_FL_H

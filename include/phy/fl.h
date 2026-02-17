//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_FL_H
#define OPENLDACS_FL_H
#pragma once


#include <queue>
#include <itpp/base/matfunc.h>

#include "config.h"
#include "openldacs.h"
#include "link.h"
#include "params.h"

namespace openldacs::phy::link::fl {

    class FLChannelHandler;
    class BC1_3Handler;
    class BC2Handler;
    class FLDataHandler;
    using namespace openldacs::util;
    using namespace phy::params;



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
        using ChRxCallbackType = std::function<void(const itpp::cvec &, std::vector<double> &, std::vector<double> &)>;

        explicit PhySource(device::DevPtr& dev): dev_(dev){

            // 向dev注册接收回调队列
            dev->registerRxCallback([this](const itpp::cvec &f) {
                std::vector<double> t_coarse;
                std::vector<double> f_coarse;

                c_sync_param_.coarseSync(f, t_coarse, f_coarse);
                if (t_coarse.empty() || f_coarse.empty()) {
                    return;
                }

                if (const auto cb = rx_handlers_.at(FL_DCH); cb.has_value()) {
                    ChRxCallbackType cb_value = cb.value();
                    cb_value(f, t_coarse, f_coarse);
                }else {
                    throw std::runtime_error("No callback registered");
                }

                // switch (sync_state_.get_state()) {
                //     case SyncState::ACQUIRE:
                //         // ACQUIRE用来做BC1-BC2-BC3的同步
                //         // 即： 做一次粗同步，找到BC1-2-3的同步符号，如果没找到就舍弃，一直找直到确定位置
                //         // 对BC帧进行精同步，完成后状态机转换为TRACK
                //         frameSync(input);
                //         break;
                //     case SyncState::TRACK:
                //         // 无需做粗同步，直接根据帧长找同步点，并进行精同步
                //         sync_state_.set_state(SyncState::ACQUIRE);
                //         break;
                // }

            });
        }

        // 各信道注册
        void registerRecvHandler(CHANNEL channel, ChRxCallbackType callback) {
            rx_handlers_.emplace(channel, callback);
        }

        ~PhySource() = default;
    private:
        device::DevPtr& dev_;
        CoarseSyncParam c_sync_param_;
        SyncStateMachine sync_state_;
        std::map<CHANNEL, std::optional<ChRxCallbackType>> rx_handlers_;
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
            sink_worker_.requestStop();
            sink_worker_.joinAndRethrow();
        }

    private:
        device::DevPtr& dev_;
        BoundedQueue<BlockBuffer> bc13_queue_;
        BoundedQueue<BlockBuffer> bc2_queue_;
        BoundedQueue<BlockBuffer> fl_data_queue_;
        Worker sink_worker_;
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
        explicit FLChannelHandler(PhyFl::FLConfig &config, device::DevPtr &dev, const int ofdm_symb)
            : device_(dev), config_(config), frame_info_(ofdm_symb), coding_table_(frame_info_),
              QPSK_modulator_(ModulationType::QPSK), QAM16_modulator_(ModulationType::QAM16),
              QAM64_modulator_(ModulationType::QAM64), channel_est_(frame_info_, ofdm_symb),
              equalizer_(frame_info_, dev, ofdm_symb), ofdm_symb_(ofdm_symb), f_sync(ofdm_symb) {
        }

        std::mutex block_m_;
        device::DevPtr& device_;
        PhyFl::FLConfig& config_;

        FrameInfo frame_info_;
        CodingTable coding_table_;
        LdacsModulator QPSK_modulator_;
        LdacsModulator QAM16_modulator_;
        LdacsModulator QAM64_modulator_;
        ChannelEstimate channel_est_;
        Equalizer equalizer_;
        std::unordered_map<BlockKey, BlockBuffer, BlockKeyHash> block_map_;
        CMS default_cms_ = CMS::QPSK_R12;
        int ofdm_symb_;
        FineSyncParam f_sync;


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


        // channel coding
        virtual void channelCoding(BlockBuffer &block, const CodingParams &coding_params) = 0;

        static void randomizer(VecU8 &to_process, const CodingParams &coding_params);
        static std::vector<VecU8> derandomizer(const std::vector<VecU8> &to_process, const CodingParams &coding_params);
        static RsEncodedUnit rsEncoder(const VecU8 &to_process, uint8_t index, const CodingParams &coding_params);
        static std::vector<VecU8> rsDecoder(const itpp::imat &input, const CodingParams &coding_params);
        static itpp::ivec blockInterleaver(const std::vector<RsEncodedUnit> &units,
                                           const CodingParams &coding_params);
        itpp::imat blockDeinterleaver(const itpp::ivec &input, const CodingParams &coding_params);
        itpp::bvec convCode(const itpp::ivec &input, const CodingParams &coding_params) const;
        static itpp::bvec helicalInterleaver(const itpp::bvec &input, const CodingParams &coding_params);
        static itpp::vec helicalDeinterleaver(const itpp::vec &in, const CodingParams &p);

        // modulation
        void modulate(BlockBuffer &block, ModulationType mod_type);

        itpp::mat demodulate(const itpp::cmat &data_equ, const itpp::mat &noise, ModulationType mod_type) const;
        virtual void subcarrierAllocation(BlockBuffer &block, int joint_frame) = 0;
        static void matrixIfft(BlockBuffer &block);
        static std::vector<itpp::cvec> windowing(const itpp::cmat &to_process, int joint_frame);

        // demod
        static itpp::cmat matrixFft(const itpp::cmat &to_process);
        static itpp::cmat downsamplingFreq(const itpp::cmat &signal, int downsample);

    };

    class BC1_3Handler final:public FLChannelHandler {
    public:
        explicit BC1_3Handler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev, n_bc13_ofdm_symb) {
            initCodingTable();

            config_.source_.registerRecvHandler(BCCH1_3, [this](const itpp::cvec& input, std::vector<double> &t_coarse, std::vector<double> &f_coarse){
                itpp::cmat data_time;
                f_sync.synchronisation(input, t_coarse, f_coarse, data_time);
            });
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

        void subcarrierAllocation(BlockBuffer &block, const int joint_frame) override {
        }
    };

    class BC2Handler final:public FLChannelHandler {
    public:
        explicit BC2Handler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev, n_bc2_ofdm_symb) {
            initCodingTable();

            config_.source_.registerRecvHandler(BCCH2, [this](const itpp::cvec& input, std::vector<double> &t_coarse, std::vector<double> &f_coarse){
                itpp::cmat data_time;
                f_sync.synchronisation(input, t_coarse, f_coarse, data_time);
            });
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

        void subcarrierAllocation(BlockBuffer &block, const int joint_frame) override {
        }
    };

    class FLDataHandler final:public FLChannelHandler {
    public:
        explicit FLDataHandler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev, n_fl_ofdm_symb) {
            initCodingTable();

            config_.source_.registerRecvHandler(FL_DCH, [this](const itpp::cvec& input, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse){
                itpp::cmat data_time;
                f_sync.synchronisation(input, t_coarse, f_coarse, data_time);
                const itpp::cmat data_freq_up = matrixFft(data_time);
                const itpp::cmat data_freq = downsamplingFreq(data_freq_up, f_sync.sync.upsample_rate);

                const itpp::cmat chan_coeff_mat = channel_est_.channelEst(data_freq);

                itpp::cmat data_equ;
                itpp::mat sigma2_sum;
                equalizer_.equalize(data_freq, chan_coeff_mat, data_equ, sigma2_sum);

                const itpp::mat demod = demodulate(data_equ, sigma2_sum, ModulationType::QPSK); // 临时参数

                const CodingParams& params = coding_table_.getCodingParams({default_cms_, 2}); // 临时参数

                itpp::vec LLR_int = itpp::cvectorize(demod);

                if (LLR_int.size() != params.h_inter_params.int_bits_size_) {
                    SPDLOG_ERROR("unmatched size for helical inteleaver");
                    return;
                }

                itpp::vec deint = helicalDeinterleaver(LLR_int, params);
                deint.set_size(deint.size() - params.conv_params.pad_bits_after_cc, true);

                itpp::bvec vit_dec = params.cc.decode_tail(deint);

                if (vit_dec.size() % params.rs_params.bits_after_rs ) {
                    throw std::runtime_error("unmatched size for rs decoder");
                }

                itpp::ivec block_int = bitsToBytesMSB(vit_dec);

                itpp::imat rs_coded = blockDeinterleaver(block_int, params);
                std::vector<VecU8> random = rsDecoder(rs_coded, params);
                std::vector<VecU8> data = derandomizer(random, params);

                std::cout << data << std::endl;


            });

        }

        void submit(PhySdu sdu, CMS cms) override;

        void submit(PhySdu sdu) override;
    private:
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

        void subcarrierAllocation(BlockBuffer &block, int joint_frame) override;
    };
}

#endif //OPENLDACS_FL_H

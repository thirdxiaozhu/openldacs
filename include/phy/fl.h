//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_FL_H
#define OPENLDACS_FL_H
#pragma once


#include <algorithm>
#include <chrono>
#include <queue>
#include <thread>
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

    enum class ChannelState {
        BCCH1,
        BCCH2,
        BCCH3,
        DATA,
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
        using ChRxCallbackType = std::function<void(const itpp::cvec &, const std::vector<double> &, const std::vector<double> &)>;

        explicit PhySource(device::DevPtr& dev): dev_(dev){

            ////////////////////////////////////////////////////
            VecCD test_padding;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<> dis(0.0, 1.0); // 均值为0，标准差为1的正态分布

            for (int i = 0; i < 3400; i++) {
                test_padding.push_back(std::complex<double>(dis(gen), dis(gen)) * 0.1) ; // 缩放因子可根据需要调整
            }
            if (!sample_buffer.try_push(test_padding)) {
                SPDLOG_WARN("SampleBuffer full, drop test padding: {} samples", test_padding.size());
            }
            ////////////////////////////////////////////////////

            // 向dev注册接收回调队列
            dev->registerRxCallback([this](const VecCD &f) {
                if (!sample_buffer.try_push(f)) {
                    SPDLOG_WARN("SampleBuffer full, drop rx chunk: {} samples", f.size());
                }
            });

            source_worker_.start([&] {

                while (!source_worker_.stop_requested()) {
                    // 同步阶段

                    itpp::cvec curr_buf;
                    std::vector<double> t_coarse;
                    std::vector<double> f_coarse;


                    if (sync_state_.get_state() == SyncState::ACQUIRE) {
                        while (!source_worker_.stop_requested() && sync_state_.get_state() == SyncState::ACQUIRE) {
                            curr_buf = getSamples(acquire_sample);

                            const auto t0 = std::chrono::high_resolution_clock::now();
                            c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);

                            SPDLOG_INFO("{} ", t_coarse.size());

                            switch (t_coarse.size()) {
                                case 0:
                                    continue;
                                case 1:
                                    popSamplesTo(t_coarse[0] - threshold);
                                    continue;
                                case 2: {
                                    if (double interval = t_coarse[1] - t_coarse[0]; !inRange(
                                        interval, bcch13_sample, threshold)) {
                                        popSamplesTo(t_coarse[1] - threshold);
                                    } else {
                                        popSamplesTo(t_coarse[0] - threshold);
                                    }
                                    continue;
                                }
                                case 3: {
                                    if (double interval = t_coarse[2] - t_coarse[1]; !inRange(
                                        interval, bcch2_sample, threshold)) {
                                        popSamplesTo(t_coarse[2] - threshold);
                                    }
                                    continue;
                                }
                                case 4: {
                                    if (double interval = t_coarse[3] - t_coarse[2]; !inRange(
                                        interval, bcch13_sample, threshold)) {
                                        popSamplesTo(t_coarse[3] - threshold);
                                        continue;
                                    }


                                    try {
                                        fineSync(curr_buf, std::vector<double>(t_coarse.begin(), t_coarse.begin() + 1), std::vector<double>(f_coarse.begin(), f_coarse.begin() + 1), BCCH1_3);
                                        fineSync(curr_buf, std::vector<double>(t_coarse.begin() + 1, t_coarse.begin() + 2), std::vector<double>(f_coarse.begin() + 1, f_coarse.begin() + 2), BCCH2);
                                        fineSync(curr_buf, std::vector<double>(t_coarse.begin() + 2, t_coarse.begin() + 3), std::vector<double>(f_coarse.begin() + 2, f_coarse.begin() + 3), BCCH1_3);
                                    }catch (std::exception &e) {
                                        SPDLOG_ERROR("fineSync failed: {}", e.what());
                                        continue; // 或者切回 ACQUIRE 状态
                                    }

                                    SPDLOG_INFO("Super frame sync has finished!");

                                    popSamplesTo(t_coarse[3] - threshold); // 获取下一帧的数个样本

                                    sync_state_.set_state(SyncState::TRACK);
                                    current_channel_ = ChannelState::DATA;
                                    const auto t1 = std::chrono::high_resolution_clock::now();
                                    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).
                                            count();
                                    SPDLOG_INFO("coarseSync {} us", us);
                                    break;
                                }
                                default: {
                                    // 对异常峰值数量回退到最新峰值附近，避免缓冲持续膨胀
                                    popSamplesTo(t_coarse.back() - threshold);
                                    continue;
                                }
                            }

                            if (sync_state_.get_state() == SyncState::TRACK) {
                                break;
                            }
                        }
                    }

                    switch (current_channel_) {
                        case ChannelState::BCCH1: {
                            curr_buf = getSamples(bcch13_sample + threshold);
                            popSamplesTo(bcch13_sample);
                            current_channel_ = ChannelState::BCCH2;
                            c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                            fineSync(curr_buf, t_coarse, f_coarse, BCCH1_3);
                            break;
                        }
                        case ChannelState::BCCH2: {
                            curr_buf = getSamples(bcch2_sample + threshold);
                            popSamplesTo(bcch2_sample);
                            current_channel_ = ChannelState::BCCH3;
                            c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                            fineSync(curr_buf, t_coarse, f_coarse, BCCH2);
                            break;
                        }
                        case ChannelState::BCCH3: {
                            curr_buf = getSamples(bcch2_sample + threshold);
                            popSamplesTo(bcch2_sample );
                            current_channel_ = ChannelState::DATA;
                            c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                            fineSync(curr_buf, t_coarse, f_coarse, BCCH1_3);
                            break;
                        }
                        case ChannelState::DATA: {
                            switch (fl_counter % DATA_PER_MF) {
                                case 0:
                                case 1:
                                case 3: {
                                    curr_buf = getSamples(data_sample2 + threshold);
                                    popSamplesTo(data_sample2);
                                    if (fl_counter % DATA_PER_MF == 3) {
                                        if (mf_counter++ % MF_PER_SF == 3) {
                                            current_channel_ = ChannelState::BCCH1;
                                        }
                                    }
                                    break;
                                }
                                case 2: {
                                    curr_buf = getSamples(data_sample3 + threshold);
                                    popSamplesTo(data_sample3);
                                    break;
                                }
                                default: {

                                }
                            }

                            c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                            fineSync(curr_buf, t_coarse, f_coarse, FL_DCH);

                            fl_counter++;
                            break;
                        }
                        default: {
                            throw std::runtime_error("Invalid channel state");
                        }
                    }
                }
            });
        }

        itpp::cvec getSamples(const size_t size) {


            std::optional<VecCD> buf = sample_buffer.wait_get_for(
                size, std::chrono::milliseconds(acquire_wait_timeout_ms));
            if (!buf.has_value()) {
                throw std::runtime_error("No samples available");
            }

            return cdVecToCvec(buf.value());
        }

        void popSamplesTo(const uint32_t pos) {
            sample_buffer.popFront(pos);
        }

        void fineSync(const itpp::cvec &in_f, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse, const ChannelSlot ch) const {
            if (const auto cb = rx_handlers_.at(ch); cb.has_value()) {
                const ChRxCallbackType cb_value = cb.value();
                cb_value(in_f, t_coarse, f_coarse);
            }else {
                throw std::runtime_error("No callback registered");
            }
        }

        size_t coarseSync(int sample_length, std::vector<double> &t_coarse, std::vector<double> &f_coarse) {
            std::optional<VecCD> buf = sample_buffer.try_pop(sample_length);
            if (!buf.has_value()) return 0;

            const VecCD& buf_val = buf.value();
            itpp::cvec in_f = cdVecToCvec(buf_val);

            c_sync_param_.coarseSync(in_f, t_coarse, f_coarse);
            if (t_coarse.empty() || f_coarse.empty()) {
                return 0;
            }

            std::cout << t_coarse << std::endl;
            return t_coarse.size();
        }

        // 各信道注册
        void registerRecvHandler(ChannelSlot channel, ChRxCallbackType callback) {
            rx_handlers_.emplace(channel, callback);
        }

        ~PhySource() {
            source_worker_.requestStop();
            source_worker_.joinAndRethrow();
        }
    private:
        device::DevPtr& dev_;
        CoarseSyncParam c_sync_param_;
        SyncStateMachine sync_state_;
        std::map<ChannelSlot, std::optional<ChRxCallbackType>> rx_handlers_;
        SampleBuffer sample_buffer;
        Worker source_worker_;
        ChannelState current_channel_ = ChannelState::DATA;
        int fl_counter = 0;
        int mf_counter = 0;
        constexpr static int acquire_sample = 5000;
        constexpr static double threshold = 10.0;
        constexpr static int acquire_buffer_limit = acquire_sample * 8;
        constexpr static int acquire_wait_timeout_ms = 60;
        constexpr static double bcch13_sample = 1125.0; // 75 * 15
        constexpr static double bcch2_sample = 1950.0; // 75 * 26
        constexpr static double data_sample2 = 8100.0; // 75 * 54 * 2
        constexpr static double data_sample3 = 12150.0; // 75 * 54 * 3
    };

    class PhySink {
    public:
        explicit PhySink(device::DevPtr& dev): dev_(dev){
            sink_worker_.start([&] {
                while (!sink_worker_.stop_requested()) {
                    std::optional<BlockBuffer> bf;
                    switch (current_channel_) {
                        case ChannelState::BCCH1:
                        case ChannelState::BCCH3: {
                            bf = bc13_queue_.pop_blocking();
                            switch (current_channel_) {
                                case ChannelState::BCCH1: {
                                    SPDLOG_INFO("BC1");
                                    current_channel_ = ChannelState::BCCH2;
                                    break;
                                }
                                case ChannelState::BCCH3: {
                                    SPDLOG_INFO("BC3");
                                    current_channel_ = ChannelState::DATA;
                                    break;
                                }
                                default: {
                                    SPDLOG_WARN("Invalid State");
                                    current_channel_ = ChannelState::BCCH1;
                                    continue;
                                }
                            }
                            break;
                        }
                        case ChannelState::BCCH2: {
                                    SPDLOG_INFO("BC2");
                            bf = bc2_queue_.pop_blocking();
                            current_channel_ = ChannelState::BCCH3;
                            break;
                        }
                        case ChannelState::DATA: {

                            if (fl_counter++ % DATA_PER_MF == CC_DATA_IDX) {
                                SPDLOG_INFO("CC FL DATA");
                                bf = cc_fl_data_queue_.pop_blocking();
                            } else {
                                SPDLOG_INFO("FL DATA");
                                bf = fl_data_queue_.pop_blocking();
                            }

                            if (fl_counter == DATA_PER_MF * MF_PER_SF) {
                                current_channel_ = ChannelState::BCCH1;
                                fl_counter = 0;
                            }
                            break;
                        }
                        default: {
                            SPDLOG_WARN("Invalid State");
                            current_channel_ = ChannelState::BCCH1;
                            break;
                        }
                    }

                    // const auto bf = fl_data_queue_.pop_blocking();
                    // const auto bf = bc13_queue_.pop_blocking();
                    // const auto bf = bc2_queue_.pop_blocking();
                    // const auto bf = cc_fl_data_queue_.pop_blocking();
                    if (!bf) {
                        SPDLOG_WARN("Buffer is null");
                        continue;
                    }
                    const itpp::cvec tx_vecs = windowing(bf.value());
                    dev->sendData(tx_vecs, util::Priority::HIGH);
                }
            });
        }

        void enqueue(const BlockBuffer &buffer, const ChannelSlot ch) {
            switch (ch) {
                case BCCH1_3:
                    bc13_queue_.push(buffer);
                    break;
                case BCCH2:
                    bc2_queue_.push(buffer);
                    break;
                case CCCH_DCH:
                    cc_fl_data_queue_.push(buffer);
                    break;
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
            cc_fl_data_queue_.close();
            sink_worker_.requestStop();
            sink_worker_.joinAndRethrow();
        }

    private:
        device::DevPtr& dev_;
        BoundedQueue<BlockBuffer> bc13_queue_;
        BoundedQueue<BlockBuffer> bc2_queue_;
        BoundedQueue<BlockBuffer> cc_fl_data_queue_;
        BoundedQueue<BlockBuffer> fl_data_queue_;
        Worker sink_worker_;
        std::optional<itpp::cvec> prev_post_;
        ChannelState current_channel_ = ChannelState::BCCH1;

        uint8_t fl_counter = 0;

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

        [[nodiscard]] FLChannelHandler &getHandler(ChannelSlot type) const;
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
        void submitData(const PhySdu& sdu, const CodingParams &coding_params);  // user-specific
        virtual void submit(PhySdu sdu) = 0;  // cell-specific

        const PhyFl::FLConfig& config() const noexcept { return config_; }
        const FrameInfo& frame() const noexcept { return frame_info_; }

        CMS getCms() const {
            return default_cms_;
        }
        void set_cms(const CMS cms) {
            default_cms_ = cms;
        }

    protected:
        explicit FLChannelHandler(PhyFl::FLConfig &config, device::DevPtr &dev, const int ofdm_symb)
            : device_(dev), config_(config), frame_info_(ofdm_symb), QPSK_modulator_(ModulationType::QPSK),
              QAM16_modulator_(ModulationType::QAM16),
              QAM64_modulator_(ModulationType::QAM64), channel_est_(frame_info_, ofdm_symb),
              equalizer_(frame_info_, dev, ofdm_symb), ofdm_symb_(ofdm_symb), f_sync(ofdm_symb) {
        }

        std::mutex block_m_;
        device::DevPtr& device_;
        PhyFl::FLConfig& config_;

        FrameInfo frame_info_;
        LdacsModulator QPSK_modulator_;
        LdacsModulator QAM16_modulator_;
        LdacsModulator QAM64_modulator_;
        ChannelEstimate channel_est_;
        Equalizer equalizer_;
        std::unordered_map<BlockKey, BlockBuffer, BlockKeyHash> block_map_;
        CMS default_cms_ = CMS::QPSK_R12;
        // CMS default_cms_ = CMS::QPSK_R34;
        int ofdm_symb_;
        FineSyncParam f_sync;

        virtual const CodingTable &getCodingTable() const = 0;
        virtual size_t getInterleaverCount(const PhySdu &sdu) const = 0;

        // channel coding
        void channelCoding(BlockBuffer &block, const CodingParams &coding_params) const;

        static void randomizer(VecU8 &to_process, const CodingParams &coding_params);
        static RsEncodedUnit rsEncoder(const VecU8 &to_process, uint8_t index, const CodingParams &coding_params);
        static itpp::ivec blockInterleaver(const std::vector<RsEncodedUnit> &units,
                                           const CodingParams &coding_params);
        itpp::bvec convCode(const itpp::ivec &input, const CodingParams &coding_params) const;
        static itpp::bvec helicalInterleaver(const itpp::bvec &input, const CodingParams &coding_params);

        // modulation
        void modulate(BlockBuffer &block, ModulationType mod_type);

        void subcarrierAllocation(BlockBuffer &block, const int joint_frame);
        static void matrixIfft(BlockBuffer &block);
        static std::vector<itpp::cvec> windowing(const itpp::cmat &to_process, int joint_frame);

        // demod
        static itpp::cmat matrixFft(const itpp::cmat &to_process);
        static itpp::cmat downsamplingFreq(const itpp::cmat &signal, int downsample);
        itpp::mat demodulate(const itpp::cmat &data_equ, const itpp::mat &noise, ModulationType mod_type) const;
        itpp::imat blockDeinterleaver(const itpp::ivec &input, const CodingParams &coding_params);
        static std::vector<VecU8> rsDecoder(const itpp::imat &input, const CodingParams &coding_params);
        static std::vector<VecU8> derandomizer(const std::vector<VecU8> &to_process, const CodingParams &coding_params);
        static itpp::vec helicalDeinterleaver(const itpp::vec &in, const CodingParams &p);
    };

    class BC1_3Handler final:public FLChannelHandler {
    public:
        explicit BC1_3Handler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev, n_bc13_ofdm_symb) {

            config_.source_.registerRecvHandler(BCCH1_3, [this](const itpp::cvec& input, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse){
                itpp::cmat data_time;
                f_sync.synchronisation(input, t_coarse, f_coarse, data_time);
                const itpp::cmat data_freq_up = matrixFft(data_time);
                const itpp::cmat data_freq = downsamplingFreq(data_freq_up, f_sync.sync.upsample_rate);

                const itpp::cmat chan_coeff_mat = channel_est_.channelEst(data_freq);

                itpp::cmat data_equ;
                itpp::mat sigma2_sum;
                equalizer_.equalize(data_freq, chan_coeff_mat, data_equ, sigma2_sum);

                const itpp::mat demod = demodulate(data_equ, sigma2_sum, ModulationType::QPSK); // 临时参数

                const CodingParams& params = coding_table_.getCodingParams({CMS::QPSK_R12, 1}); // 临时参数

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
        void submit(PhySdu sdu) override;
        const CodingTable& getCodingTable() const override{
            return coding_table_;
        }
        size_t getInterleaverCount(const PhySdu &sdu) const override {
            return 1;
        }
    private:
        CodingTable coding_table_{
            frame_info_, {
                    {CMS::QPSK_R12, 1},
                },
                BCCH1_3
            };
    };

    class BC2Handler final:public FLChannelHandler {
    public:
        explicit BC2Handler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev, n_bc2_ofdm_symb) {
            config_.source_.registerRecvHandler(BCCH2, [this](const itpp::cvec& input, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse){
                itpp::cmat data_time;
                f_sync.synchronisation(input, t_coarse, f_coarse, data_time);
                const itpp::cmat data_freq_up = matrixFft(data_time);
                const itpp::cmat data_freq = downsamplingFreq(data_freq_up, f_sync.sync.upsample_rate);

                const itpp::cmat chan_coeff_mat = channel_est_.channelEst(data_freq);

                itpp::cmat data_equ;
                itpp::mat sigma2_sum;
                equalizer_.equalize(data_freq, chan_coeff_mat, data_equ, sigma2_sum);

                const itpp::mat demod = demodulate(data_equ, sigma2_sum, ModulationType::QPSK); // 临时参数

                const CodingParams& params = coding_table_.getCodingParams({CMS::QPSK_R12, 1}); // 临时参数

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
        void submit(PhySdu sdu) override;
        const CodingTable& getCodingTable() const override{
            return coding_table_;
        }
        size_t getInterleaverCount(const PhySdu &sdu) const override {
            return 1;
        }
    private:
        CodingTable coding_table_{
            frame_info_, {
                {CMS::QPSK_R12, 1},
            },
            BCCH2
        };
    };

    class FLDataHandler final:public FLChannelHandler {
    public:
        explicit FLDataHandler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev, n_fl_ofdm_symb) {
            config_.source_.registerRecvHandler(FL_DCH, [this](const itpp::cvec& input, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse){

                if (t_coarse.size() != 2 && f_coarse.size() != 3) {
                    throw std::runtime_error("unmatched size for coarse sync");
                }

                itpp::cmat data_time;
                f_sync.synchronisation(input, t_coarse, f_coarse, data_time);
                const itpp::cmat data_freq_up = matrixFft(data_time);
                const itpp::cmat data_freq = downsamplingFreq(data_freq_up, f_sync.sync.upsample_rate);

                const itpp::cmat chan_coeff_mat = channel_est_.channelEst(data_freq);

                itpp::cmat data_equ;
                itpp::mat sigma2_sum;
                equalizer_.equalize(data_freq, chan_coeff_mat, data_equ, sigma2_sum);

                const itpp::mat demod = demodulate(data_equ, sigma2_sum, ModulationType::QPSK); // 临时参数

                const CodingParams& params = coding_table_.getCodingParams({default_cms_, t_coarse.size()}); // 临时参数

                itpp::vec LLR_int = itpp::cvectorize(demod);

                if (LLR_int.size() != params.h_inter_params.int_bits_size_) {
                    SPDLOG_ERROR("unmatched size for helical inteleaver {} {}", LLR_int.size(), params.h_inter_params.int_bits_size_);
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

        void submit(PhySdu sdu) override;

        const CodingTable& getCodingTable() const override{
            return coding_table_;
        }

        size_t getInterleaverCount(const PhySdu &sdu) const override {
            if (sdu.direction == FL) {
                if (sdu.sdu_index >= 13 && sdu.sdu_index <= 21) return 9;
                return 6;
            }else {
                // RL
                return 0;
            }
        }

    private:
        CodingTable coding_table_{
            frame_info_, {
                {CMS::QPSK_R12, 2},
                {CMS::QPSK_R12, 3},
                // {CMS::QPSK_R23, 2},
                // {CMS::QPSK_R23, 3},
                // {CMS::QPSK_R34, 2},
                // {CMS::QPSK_R34, 3},
            },
            FL_DCH
        };
    };
}

#endif //OPENLDACS_FL_H

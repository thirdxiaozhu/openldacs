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
#include <cmath>

namespace openldacs::phy::link::fl {

    class FLChannelHandler;
    class BC1_3Handler;
    class BC2Handler;
    class FLDataHandler;
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

        explicit PhySource(device::DevPtr& dev): dev_(dev), context(1), publisher(context, ZMQ_PUB) {

            publisher.bind("tcp://127.0.0.1:5555");

            // 向dev注册接收回调队列
            dev->registerRxCallback([&](const VecCF &f) {
                if (const VecCD vd(f.begin(), f.end()); !sample_buffer.try_push(vd)) {
                    SPDLOG_WARN("SampleBuffer full, drop rx chunk: {} samples", f.size());
                }
                SPDLOG_INFO("================ {}", sample_buffer.size());
                // zmq::message_t message(f.size() * sizeof(VecCF::value_type));
                // memcpy(message.data(), f.data(), message.size());
                // publisher.send(message, zmq::send_flags::none);
            });

            source_worker_.start([&] {

                int sf_count = 0;
                auto resetToAcquire = [&](const char *reason, const double slide_samples = 0.0) {
                    if (slide_samples > 0.0) {
                        popSamplesTo(slide_samples);
                    }
                    SPDLOG_WARN("Reset to ACQUIRE: {}", reason);
                    sync_state_.set_state(SyncState::ACQUIRE);
                    current_channel_ = ChannelState::BCCH1;
                    fl_counter = 0;
                    mf_counter = 0;
                    track_fail_streak_ = 0;
                };

                auto handleTrackFailure = [&](const char *stage) {
                    ++track_fail_streak_;
                    SPDLOG_WARN("Track sync failed at {}, streak={}", stage, track_fail_streak_);
                    if (track_fail_streak_ >= track_reacquire_after_failures) {
                        resetToAcquire("track failure limit reached", track_reacquire_slide_samples);
                    }
                };

                while (!source_worker_.stop_requested()) {
                    try {
                        // 同步阶段
                        itpp::cvec curr_buf;
                        std::vector<double> t_coarse;
                        std::vector<double> f_coarse;

                        if (sync_state_.get_state() == SyncState::ACQUIRE) {
                            while (!source_worker_.stop_requested() && sync_state_.get_state() == SyncState::ACQUIRE) {
                                curr_buf = getSamples(acquire_sample);

                                const auto t0 = std::chrono::high_resolution_clock::now();
                                c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);

                                switch (t_coarse.size()) {
                                    case 0: {
                                        popSamplesTo(acquire_slide_samples);
                                        continue;
                                    }
                                    case 1:
                                        std::cout << "1 peak    " << t_coarse << std::endl;

                                        popSamplesTo(
                                            t_coarse[0] - threshold > 0.0
                                                ? t_coarse[0] - threshold
                                                : static_cast<double>(acquire_slide_samples));
                                        continue;
                                    case 2: {
                                        std::cout << "2 peaks    " << t_coarse << std::endl;

                                        if (double interval = t_coarse[1] - t_coarse[0]; !inRange(
                                            interval, bcch13_sample, threshold)) {
                                            popSamplesTo(t_coarse[1] - threshold);
                                        } else {
                                            popSamplesTo(t_coarse[0] - threshold);
                                        }
                                        continue;
                                    }
                                    case 3: {
                                        std::cout << "3 peaks    " << t_coarse << " ";

                                        if (double interval = t_coarse[2] - t_coarse[1]; !inRange(
                                            interval, bcch2_sample, threshold)) {
                                            popSamplesTo(t_coarse[2] - threshold);
                                            continue;
                                        }
                                        popSamplesTo(t_coarse[2] - threshold);
                                        continue;
                                    }
                                    case 4: {
                                        std::cout << "4 peaks    " << t_coarse << std::endl;

                                        if (double interval = t_coarse[3] - t_coarse[2]; !inRange(
                                            interval, bcch13_sample, threshold)) {
                                            popSamplesTo(t_coarse[3] - threshold);
                                            continue;
                                        }

                                        if (!fineSyncSafe(curr_buf,
                                                          std::vector<double>(t_coarse.begin(), t_coarse.begin() + 1),
                                                          std::vector<double>(f_coarse.begin(), f_coarse.begin() + 1),
                                                          BCCH1_3, "acquire:bcch1")
                                            || !fineSyncSafe(curr_buf,
                                                             std::vector<double>(t_coarse.begin() + 1, t_coarse.begin() + 2),
                                                             std::vector<double>(f_coarse.begin() + 1, f_coarse.begin() + 2),
                                                             BCCH2, "acquire:bcch2")
                                            || !fineSyncSafe(curr_buf,
                                                             std::vector<double>(t_coarse.begin() + 2, t_coarse.begin() + 3),
                                                             std::vector<double>(f_coarse.begin() + 2, f_coarse.begin() + 3),
                                                             BCCH1_3, "acquire:bcch3")) {
                                            popSamplesTo(acquire_slide_samples);
                                            continue;
                                        }

                                        SPDLOG_INFO("Super frame sync has finished!");
                                        popSamplesTo(t_coarse[3] - threshold); // 获取下一帧的数个样本

                                        sync_state_.set_state(SyncState::TRACK);
                                        current_channel_ = ChannelState::DATA;
                                        fl_counter = 0;
                                        mf_counter = 0;
                                        track_fail_streak_ = 0;
                                        const auto t1 = std::chrono::high_resolution_clock::now();
                                        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                                                            .count();
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

                        if (sync_state_.get_state() != SyncState::TRACK) {
                            continue;
                        }

                        switch (current_channel_) {
                            case ChannelState::BCCH1: {
                                curr_buf = getSamples(bcch13_sample + threshold);
                                c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                                if (!fineSyncSafe(curr_buf, t_coarse, f_coarse, BCCH1_3, "track:bcch1")) {
                                    handleTrackFailure("track:bcch1");
                                    continue;
                                }
                                popSamplesTo(bcch13_sample);
                                track_fail_streak_ = 0;
                                current_channel_ = ChannelState::BCCH2;
                                break;
                            }
                            case ChannelState::BCCH2: {
                                curr_buf = getSamples(bcch2_sample + threshold);
                                c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                                if (!fineSyncSafe(curr_buf, t_coarse, f_coarse, BCCH2, "track:bcch2")) {
                                    handleTrackFailure("track:bcch2");
                                    continue;
                                }
                                popSamplesTo(bcch2_sample);
                                track_fail_streak_ = 0;
                                current_channel_ = ChannelState::BCCH3;
                                break;
                            }
                            case ChannelState::BCCH3: {
                                curr_buf = getSamples(bcch13_sample + threshold);
                                c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                                if (!fineSyncSafe(curr_buf, t_coarse, f_coarse, BCCH1_3, "track:bcch3")) {
                                    handleTrackFailure("track:bcch3");
                                    continue;
                                }
                                popSamplesTo(bcch13_sample);
                                track_fail_streak_ = 0;
                                current_channel_ = ChannelState::DATA;
                                break;
                            }
                            case ChannelState::DATA: {
                                const int data_slot = fl_counter % DATA_PER_MF;
                                double consume_samples = 0.0;
                                switch (data_slot) {
                                    case 0:
                                    case 1:
                                    case 3: {
                                        curr_buf = getSamples(data_sample2 + threshold);
                                        consume_samples = data_sample2;
                                        break;
                                    }
                                    case 2: {
                                        curr_buf = getSamples(data_sample3 + threshold);
                                        consume_samples = data_sample3;
                                        break;
                                    }
                                    default: {
                                        throw std::runtime_error("Invalid data slot");
                                    }
                                }

                                c_sync_param_.coarseSync(curr_buf, t_coarse, f_coarse);
                                if (!fineSyncSafe(curr_buf, t_coarse, f_coarse, FL_DCH, "track:data")) {
                                    handleTrackFailure("track:data");
                                    continue;
                                }

                                popSamplesTo(consume_samples);
                                track_fail_streak_ = 0;
                                if (data_slot == 3) {
                                    if (mf_counter % MF_PER_SF == MF_PER_SF - 1) {
                                        current_channel_ = ChannelState::BCCH1;
                                        SPDLOG_INFO("============== {} super frames", sf_count++);
                                    }
                                    mf_counter++;
                                }
                                fl_counter++;
                                break;
                            }
                            default: {
                                throw std::runtime_error("Invalid channel state");
                            }
                        }
                    } catch (const std::exception &e) {
                        SPDLOG_ERROR("PhySource worker iteration failed: {}", e.what());
                        resetToAcquire("worker iteration exception", track_reacquire_slide_samples);
                        continue;
                    } catch (...) {
                        SPDLOG_ERROR("PhySource worker iteration failed: unknown exception");
                        resetToAcquire("worker iteration unknown exception", track_reacquire_slide_samples);
                        continue;
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

        void popSamplesTo(const double pos) {
            if (!std::isfinite(pos) || pos <= 0.0) {
                return;
            }
            sample_buffer.popFront(static_cast<size_t>(std::llround(pos)));
        }

        void fineSync(const itpp::cvec &in_f, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse, const ChannelSlot ch) const {
            const auto it = rx_handlers_.find(ch);
            if (it == rx_handlers_.end() || !it->second.has_value()) {
                throw std::runtime_error("No callback registered");
            }
            const ChRxCallbackType cb_value = it->second.value();
            cb_value(in_f, t_coarse, f_coarse);
        }

        bool fineSyncSafe(const itpp::cvec &in_f, const std::vector<double> &t_coarse,
                          const std::vector<double> &f_coarse, const ChannelSlot ch,
                          const char* stage) const noexcept {
            try {
                fineSync(in_f, t_coarse, f_coarse, ch);
                return true;
            } catch (const std::exception& e) {
                SPDLOG_ERROR("fineSync failed at {}: {}", stage, e.what());
            } catch (...) {
                SPDLOG_ERROR("fineSync failed at {}: unknown exception", stage);
            }
            return false;
        }

        // 各信道注册
        void registerRecvHandler(ChannelSlot channel, ChRxCallbackType callback) {
            rx_handlers_.emplace(channel, callback);
        }

        ~PhySource() noexcept {
            source_worker_.requestStop();
            source_worker_.joinNoexcept("PhySource::source_worker");
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
        constexpr static double threshold = 128.0;
        constexpr static int acquire_buffer_limit = acquire_sample * 8;
        constexpr static int acquire_wait_timeout_ms = 60;
        constexpr static int acquire_slide_samples = acquire_sample / 4;
        constexpr static int track_reacquire_after_failures = 3;
        constexpr static int track_reacquire_slide_samples = acquire_sample / 8;
        constexpr static double bcch13_sample = 1125.0; // 75 * 15
        constexpr static double bcch2_sample = 1950.0; // 75 * 26
        constexpr static double data_sample2 = 8100.0; // 75 * 54 * 2
        constexpr static double data_sample3 = 12150.0; // 75 * 54 * 3
        int track_fail_streak_ = 0;

        zmq::context_t context;
        zmq::socket_t publisher;
    };

    class PhySink {
    public:
        explicit PhySink(device::DevPtr& dev): dev_(dev){
            sink_worker_.start([&] {
                while (!sink_worker_.stop_requested()) {
                    try {
                    std::optional<BlockBuffer> bf;
                    switch (current_channel_) {
                        case ChannelState::BCCH1:
                        case ChannelState::BCCH3: {
                            bf = bc13_queue_.pop_blocking();
                            switch (current_channel_) {
                                case ChannelState::BCCH1: {
                                    current_channel_ = ChannelState::BCCH2;
                                    break;
                                }
                                case ChannelState::BCCH3: {
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
                            bf = bc2_queue_.pop_blocking();
                            current_channel_ = ChannelState::BCCH3;
                            break;
                        }
                        case ChannelState::DATA: {

                            if (fl_counter++ % DATA_PER_MF == CC_DATA_IDX) {
                                bf = cc_fl_data_queue_.pop_blocking();
                            } else {
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

                    if (!bf) {
                        SPDLOG_WARN("Buffer is null");
                        continue;
                    }
                    const itpp::cvec tx_vecs = windowing(bf.value());
                    dev->sendData(tx_vecs, util::Priority::HIGH);
                    } catch (const std::exception& e) {
                        SPDLOG_ERROR("PhySink worker iteration failed: {}", e.what());
                        continue;
                    } catch (...) {
                        SPDLOG_ERROR("PhySink worker iteration failed: unknown exception");
                        continue;
                    }
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

        ~PhySink() noexcept {
            bc13_queue_.close();
            bc2_queue_.close();
            fl_data_queue_.close();
            cc_fl_data_queue_.close();
            sink_worker_.requestStop();
            sink_worker_.joinNoexcept("PhySink::sink_worker");
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

    class FLChannelHandler: public ChannelHandler{
    public:
        virtual ~FLChannelHandler() = default;
        void submitData(const PhySdu& sdu, const CodingParams &coding_params);  // user-specific
        virtual void submit(PhySdu sdu) = 0;  // cell-specific

        const PhyFl::FLConfig& config() const noexcept { return config_; }
        const FrameInfo& frame() const noexcept { return frame_info_; }

    protected:
        explicit FLChannelHandler(PhyFl::FLConfig &config, device::DevPtr &dev, const int ofdm_symb)
            : device_(dev), config_(config), frame_info_(ofdm_symb),  channel_est_(frame_info_, ofdm_symb),
              equalizer_(frame_info_, dev, ofdm_symb), ofdm_symb_(ofdm_symb), f_sync(ofdm_symb) {
        }

        std::mutex block_m_;
        device::DevPtr& device_;
        PhyFl::FLConfig& config_;

        FrameInfo frame_info_;
        ChannelEstimate channel_est_;
        Equalizer equalizer_;
        std::unordered_map<BlockKey, BlockBuffer, BlockKeyHash> block_map_;
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

        void subcarrierAllocation(BlockBuffer &block, int joint_frame);
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

        void recvHandler(const itpp::cvec& input, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse, ModulationType
                         mod_type, const CodingParams &params);
    };

    class BC1_3Handler final:public FLChannelHandler {
    public:
        explicit BC1_3Handler(PhyFl::FLConfig& config, device::DevPtr& dev) : FLChannelHandler(config, dev, n_bc13_ofdm_symb) {

            config_.source_.registerRecvHandler(BCCH1_3, [this](const itpp::cvec& input, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse){
                const CodingParams& params = coding_table_.getCodingParams({CMS::QPSK_R12, 1}); // 临时参数
                recvHandler(input, t_coarse, f_coarse, ModulationType::QPSK, params);
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
                const CodingParams& params = coding_table_.getCodingParams({CMS::QPSK_R12, 1}); // 临时参数
                recvHandler(input, t_coarse, f_coarse, ModulationType::QPSK, params);
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

                if (t_coarse.size() != 2 && t_coarse.size() != 3) {
                    throw std::runtime_error("unmatched size for coarse sync");
                }

                const CodingParams& params = coding_table_.getCodingParams({default_cms_, t_coarse.size()}); // 临时参数
                recvHandler(input, t_coarse, f_coarse, ModulationType::QPSK, params);

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

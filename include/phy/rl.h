//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_RL_H
#define OPENLDACS_RL_H

#include "link.h"

namespace openldacs::phy::link::rl {

    // class RLChannelHandler;
    // class RAHandler;
    // class RLDataHandler;
    //
    // class PhyRl final : public LinkBase {
    // public:
    //     struct RLConfig {
    //         explicit RLConfig(device::DevPtr& dev){
    //         }
    //     };
    //
    //     explicit PhyRl(device::DevPtr& dev)
    //         :config_(dev),
    //          ra_(std::make_unique<RAHandler>(config_, dev)),
    //          data_(std::make_unique<RLDataHandler>(config_, dev)) {
    //     }
    //
    //     void processPacket(const PhySdu &sdu) const override;
    //
    // private:
    //     RLConfig config_;
    //     std::unique_ptr<RAHandler> ra_;
    //     std::unique_ptr<RLDataHandler> data_;
    //
    //     [[nodiscard]] RLChannelHandler &getHandler(ChannelSlot type) const;
    // };
    //
    // class RLChannelHandler: public ChannelHandler {
    //
    // protected:
    //     explicit RLChannelHandler(PhyRl::RLConfig &config, device::DevPtr &dev, const int ofdm_symb)
    //         : device_(dev), config_(config){
    //     }
    //
    //     device::DevPtr& device_;
    //     PhyRl::RLConfig& config_;
    // };
    //
    // class RAHandler: public RLChannelHandler {
    // public:
    //     explicit RAHandler(PhyRl::RLConfig& config, device::DevPtr& dev) : RLChannelHandler(config, dev, n_bc2_ofdm_symb) {
    //     }
    // protected:
    //
    // private:
    // };
    //
    // class RLDataHandler: public RLChannelHandler {
    // public:
    //     explicit RLDataHandler(PhyRl::RLConfig& config, device::DevPtr& dev) : RLChannelHandler(config, dev, n_bc2_ofdm_symb) {
    //     }
    //
    // private:
    //     CodingTable coding_table_{
    //         frame_info_, {
    //                 {CMS::QPSK_R12, 1},
    //             },
    //             RL_DCH
    //         };
    // };


    //
    // class RLChannelHandler {
    // public:
    //     virtual ~RLChannelHandler() = default;
    //     void submitData(const PhySdu& sdu, const CodingParams &coding_params);  // user-specific
    //     virtual void submit(PhySdu sdu) = 0;  // cell-specific
    //
    //     const PhyFl::FLConfig& config() const noexcept { return config_; }
    //     const FrameInfo& frame() const noexcept { return frame_info_; }
    //
    //     CMS getCms() const {
    //         return default_cms_;
    //     }
    //     void set_cms(const CMS cms) {
    //         default_cms_ = cms;
    //     }
    //
    // protected:
    //     explicit RLChannelHandler(PhyFl::FLConfig &config, device::DevPtr &dev, const int ofdm_symb)
    //         : device_(dev), config_(config), frame_info_(ofdm_symb), QPSK_modulator_(ModulationType::QPSK),
    //           QAM16_modulator_(ModulationType::QAM16),
    //           QAM64_modulator_(ModulationType::QAM64), channel_est_(frame_info_, ofdm_symb),
    //           equalizer_(frame_info_, dev, ofdm_symb), ofdm_symb_(ofdm_symb), f_sync(ofdm_symb) {
    //     }
    //
    //     std::mutex block_m_;
    //     device::DevPtr& device_;
    //     PhyFl::FLConfig& config_;
    //
    //     FrameInfo frame_info_;
    //     LdacsModulator QPSK_modulator_;
    //     LdacsModulator QAM16_modulator_;
    //     LdacsModulator QAM64_modulator_;
    //     ChannelEstimate channel_est_;
    //     Equalizer equalizer_;
    //     std::unordered_map<BlockKey, BlockBuffer, BlockKeyHash> block_map_;
    //     CMS default_cms_ = CMS::QPSK_R12;
    //     // CMS default_cms_ = CMS::QPSK_R34;
    //     int ofdm_symb_;
    //     FineSyncParam f_sync;
    //
    //     virtual const CodingTable &getCodingTable() const = 0;
    //     virtual size_t getInterleaverCount(const PhySdu &sdu) const = 0;
    //
    //     // channel coding
    //     void channelCoding(BlockBuffer &block, const CodingParams &coding_params) const;
    //
    //     static void randomizer(VecU8 &to_process, const CodingParams &coding_params);
    //     static RsEncodedUnit rsEncoder(const VecU8 &to_process, uint8_t index, const CodingParams &coding_params);
    //     static itpp::ivec blockInterleaver(const std::vector<RsEncodedUnit> &units,
    //                                        const CodingParams &coding_params);
    //     itpp::bvec convCode(const itpp::ivec &input, const CodingParams &coding_params) const;
    //     static itpp::bvec helicalInterleaver(const itpp::bvec &input, const CodingParams &coding_params);
    //
    //     // modulation
    //     void modulate(BlockBuffer &block, ModulationType mod_type);
    //
    //     void subcarrierAllocation(BlockBuffer &block, int joint_frame);
    //     static void matrixIfft(BlockBuffer &block);
    //     static std::vector<itpp::cvec> windowing(const itpp::cmat &to_process, int joint_frame);
    //
    //     // demod
    //     static itpp::cmat matrixFft(const itpp::cmat &to_process);
    //     static itpp::cmat downsamplingFreq(const itpp::cmat &signal, int downsample);
    //     itpp::mat demodulate(const itpp::cmat &data_equ, const itpp::mat &noise, ModulationType mod_type) const;
    //     itpp::imat blockDeinterleaver(const itpp::ivec &input, const CodingParams &coding_params);
    //     static std::vector<VecU8> rsDecoder(const itpp::imat &input, const CodingParams &coding_params);
    //     static std::vector<VecU8> derandomizer(const std::vector<VecU8> &to_process, const CodingParams &coding_params);
    //     static itpp::vec helicalDeinterleaver(const itpp::vec &in, const CodingParams &p);
    // };
}


#endif //OPENLDACS_RL_H
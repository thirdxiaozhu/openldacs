//
// Created by 邹嘉旭 on 2025/12/15.
//

#ifndef OPENLDACS_LINK_H
#define OPENLDACS_LINK_H

#include "openldacs.h"
#include "params.h"

namespace openldacs::phy::link {


    constexpr static int CC_DATA_IDX = 3-1; // 0-base
    constexpr static int MF_PER_SF = 4;
    constexpr static int DATA_PER_MF = 4;

    class LinkBase {
    public:

        virtual ~LinkBase() = default;
        explicit LinkBase() = default;

        virtual void processPacket(const params::PhySdu &sdu) const = 0;
    protected:
        // LinkType type_;
    private:
    };

    class ChannelHandler {
    public:
        virtual ~ChannelHandler() = default;

    protected:
        std::mutex block_m_;
        std::unordered_map<params::BlockKey, params::BlockBuffer, params::BlockKeyHash> block_map_;

        [[nodiscard]] CMS getCms() const {
            return default_cms_;
        }

        void setCms(const CMS cms) {
            default_cms_ = cms;
        }

        explicit ChannelHandler() : QPSK_modulator_(params::ModulationType::QPSK),
                                    QAM16_modulator_(params::ModulationType::QAM16),
                                    QAM64_modulator_(params::ModulationType::QAM64) {
        };

        virtual const params::FrameInfo &getFrame() const = 0;

        std::optional<params::BlockBuffer> collectBlockBuff(const params::PhySdu &sdu,
                                                                    params::ProcessUnit &unit, size_t int_count);

        // modulation
        void modulate(params::BlockBuffer &block, params::ModulationType mod_type) const;

        [[nodiscard]] itpp::mat demodulate(const itpp::cmat &data_equ, const itpp::mat &noise,
                                           params::ModulationType mod_type) const;

    private:
        params::LdacsModulator QPSK_modulator_;
        params::LdacsModulator QAM16_modulator_;
        params::LdacsModulator QAM64_modulator_;
        CMS default_cms_ = CMS::QPSK_R23;
    };
}

#endif //OPENLDACS_LINK_H
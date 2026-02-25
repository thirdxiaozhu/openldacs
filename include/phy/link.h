//
// Created by 邹嘉旭 on 2025/12/15.
//

#ifndef OPENLDACS_LINK_H
#define OPENLDACS_LINK_H

#include "openldacs.h"
#include "phy.h"
#include "params.h"

namespace openldacs::phy::link {

    using namespace phy::params;

    constexpr static int CC_DATA_IDX = 3-1; // 0-base
    constexpr static int MF_PER_SF = 4;
    constexpr static int DATA_PER_MF = 4;

    class LinkBase {
    public:

        virtual ~LinkBase() = default;
        explicit LinkBase() = default;

        virtual void processPacket(const PhySdu &sdu) const = 0;
    protected:
        // LinkType type_;
    private:
    };

    class ChannelHandler {


        CMS getCms() const {
            return default_cms_;
        }
        void set_cms(const CMS cms) {
            default_cms_ = cms;
        }
    protected:
        explicit ChannelHandler() : QPSK_modulator_(ModulationType::QPSK),
              QAM16_modulator_(ModulationType::QAM16),
              QAM64_modulator_(ModulationType::QAM64){
        };

        LdacsModulator QPSK_modulator_;
        LdacsModulator QAM16_modulator_;
        LdacsModulator QAM64_modulator_;
        CMS default_cms_ = CMS::QPSK_R12;
    };
}

#endif //OPENLDACS_LINK_H
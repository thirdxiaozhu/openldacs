//
// Created by 邹嘉旭 on 2025/12/15.
//

#ifndef OPENLDACS_LINK_H
#define OPENLDACS_LINK_H

#include "openldacs.h"
#include "phy.h"

namespace openldacs::phy::link {

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
}

#endif //OPENLDACS_LINK_H
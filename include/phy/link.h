//
// Created by 邹嘉旭 on 2025/12/15.
//

#ifndef OPENLDACS_LINK_H
#define OPENLDACS_LINK_H

#include "openldacs.h"
#include "phy.h"

namespace openldacs::phy::link {

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
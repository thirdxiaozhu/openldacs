//
// Created by 邹嘉旭 on 2025/12/15.
//

#ifndef OPENLDACS_LINK_H
#define OPENLDACS_LINK_H

#include "openldacs.h"

namespace openldacs::phy::link {

    enum class LinkType {
        FL = 0,
        RL = 1,
    };

    enum class ChannelType {
        BC1_3 = 0,
        BC2 = 1,
        FL_DATA = 2,
    };

    class LinkBase {
    public:
        virtual ~LinkBase() = default;
        explicit LinkBase() = default;
        virtual void process_packet(ChannelType type, const std::vector<uint8_t> &input) const = 0;
    protected:
        LinkType type_;
    private:
    };
}

#endif //OPENLDACS_LINK_H
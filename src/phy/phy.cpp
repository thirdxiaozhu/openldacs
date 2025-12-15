//
// Created by 邹嘉旭 on 2025/12/11.
//

#include "phy/phy.h"

#include "util/util.h"

namespace openldacs::phy::phy_service {
    using namespace openldacs::util;
    using namespace openldacs::phy::link;
    PhyService::PhyService(): config_() {
    }

    void PhyService::send_fl_data(const ChannelType type, const std::vector<uint8_t>& input) const {
        config_.PhyFL.process_packet(type, input);
    }

}

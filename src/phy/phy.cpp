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

    void PhyService::sendFlData(const CHANNEL type, const std::vector<uint8_t>& input) const {
        config_.PhyFL.processPacket(type, input);
    }

}

//
// Created by 邹嘉旭 on 2025/12/11.
//

#include "phy/phy.h"
#include "util/util.h"
#include "phy/fl.h"

namespace openldacs::phy {
    using namespace openldacs::util;
    PhyService::PhyService(): config_() {
        config_.PhyFL = std::make_unique<link::fl::PhyFl>();
    }

    void PhyService::sendFlData(const PhySdu &sdu) const {
        config_.PhyFL->processPacket(sdu);
    }

}

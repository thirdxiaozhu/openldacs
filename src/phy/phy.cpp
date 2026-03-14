//
// Created by 邹嘉旭 on 2025/12/11.
//

#include "phy/phy.h"
#include "util/util.h"
#include "phy/fl.h"
#include "phy/rl.h"

namespace openldacs::phy {
    PhyService::PhyService(const device::DeviceType dev_type, uint8_t role): config_() {
        config_.Device = device::DeviceFactory::createDevice(dev_type,  role);
        config_.PhyFL = std::make_unique<link::fl::PhyFl>(config_.Device);
        // config_.PhyRL = std::make_unique<link::rl::PhyRl>(config_.Device);
    }

    void PhyService::sendFlData(const params::PhySdu &sdu) const {
        config_.PhyFL->processPacket(sdu);
    }
}

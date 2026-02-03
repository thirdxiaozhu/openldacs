//
// Created by 邹嘉旭 on 2025/12/11.
//

#include "phy/phy.h"
#include "util/util.h"
#include "phy/fl.h"

namespace openldacs::phy {
    PhyService::PhyService(const device::DeviceType dev_type): config_() {
        config_.Device = device::DeviceFactory::createDevice(dev_type, AS | GS);
        config_.PhyFL = std::make_unique<link::fl::PhyFl>(config_.Device);
    }

    void PhyService::sendFlData(const PhySdu &sdu) const {
        config_.PhyFL->processPacket(sdu);
    }

}

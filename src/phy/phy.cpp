//
// Created by 邹嘉旭 on 2025/12/11.
//

#include "phy/phy.h"

namespace openldacs::phy::phy_service {
    PhyService::PhyService(): config_() {
    }

    void PhyService::send_fl_data(const fl::FLType type, const std::vector<uint8_t>& input) const {
        config_.PhyFL.process_fl_pkt(type);
    }

}
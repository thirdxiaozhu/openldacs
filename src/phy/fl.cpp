//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"

namespace openldacs::phy::fl {
    PhyFl::PhyFl() {
    };

    void PhyFl::process_fl_pkt(FLType type) const {
        if (type == FLType::BC1_3) {
            std::cout << "!!!!!!!!!" << std::endl;
            return;
        }
    }

}
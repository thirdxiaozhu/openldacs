//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"

namespace openldacs::phy::fl {
    FLPhy::FLPhy() {

    }

    void FLPhy::test_fl_send(FLType type) {
        if (type == FLType::BC1_3) {
            std::cout << "!!!!!!!!!" << std::endl;
            return;
        }
        std::cout << "???????????" << std::endl;
    }


}
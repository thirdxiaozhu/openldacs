//
// Created by 邹嘉旭 on 2025/12/11.
//

#ifndef OPENLDACS_PHY_H
#define OPENLDACS_PHY_H
#include "fl.h"

namespace openldacs::phy::phy_service {
    using namespace openldacs::phy::link;
    class PhyService {
    public:
        struct PhyConfig {
            fl::PhyFl PhyFL;
        };
        explicit PhyService ();
        void sendFlData(CHANNEL type, const std::vector<uint8_t>& input) const;

    private:
        PhyConfig config_;
    };
}

#endif //OPENLDACS_PHY_H
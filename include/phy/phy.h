//
// Created by 邹嘉旭 on 2025/12/11.
//

#ifndef OPENLDACS_PHY_H
#define OPENLDACS_PHY_H
#include "openldacs.h"
#include "phy/device.h"
#include "phy/params.h"
#include "phy/fl.h"
#include "phy/rl.h"

namespace openldacs::phy {

    using namespace openldacs::phy::params;

    // namespace link::fl {
    //     class PhyFl;
    // }
    //
    // namespace link::rl {
    //     class PhyRl;
    // }

    // namespace device {
    //     class Device;
    //     class USRP;
    // }



    class PhyService {
    public:
        struct PhyConfig {
            std::unique_ptr<link::fl::PhyFl> PhyFL;
            // std::unique_ptr<link::rl::PhyRl> PhyRL;
            std::unique_ptr<device::Device> Device;

        };
        explicit PhyService (device::DeviceType dev_type, uint8_t role);

        ~PhyService() = default;
        void sendFlData(const PhySdu &sdu) const;
        std::unique_ptr<device::Device>& getDevice() {
            return config_.Device;
        }
    private:
        PhyConfig config_;
    };
}

#endif //OPENLDACS_PHY_H
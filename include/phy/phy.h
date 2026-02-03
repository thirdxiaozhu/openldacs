//
// Created by 邹嘉旭 on 2025/12/11.
//

#ifndef OPENLDACS_PHY_H
#define OPENLDACS_PHY_H
#include "openldacs.h"
#include "phy/device.h"

namespace openldacs::phy {

    namespace link::fl {
        class PhyFl;
    }

    namespace device {
        class Device;
        class USRP;
    }

    enum  DirectionType {
        FL = 0,
        RL = 1,
    };

    struct PhySdu {
        DirectionType direction;
        uint32_t sf_id;
        uint16_t mf_id;
        uint8_t sdu_index;              // FL: 1-27             从 1 开始！！！！！
        uint8_t acm_id;                 // 0 for cell-spec
        CHANNEL channel;
        std::vector<uint8_t> payload;
    };

    class PhyService {
    public:
        struct PhyConfig {
            std::unique_ptr<link::fl::PhyFl> PhyFL;
            std::unique_ptr<device::Device> Device;
        };
        explicit PhyService (device::DeviceType dev_type);
        void sendFlData(const PhySdu &sdu) const;
        std::unique_ptr<device::Device>& getDevice() {
            return config_.Device;
        }
    private:
        PhyConfig config_;
    };
}

#endif //OPENLDACS_PHY_H
//
// Created by jiaxv on 2026/3/14.
//


#include  "openldacs.h"
#include "phy/config.h"

#include "phy/phy.h"
#include "phy/rl.h"
#include "util/util.h"


using namespace openldacs;
using namespace openldacs::phy::config;
using namespace openldacs::phy::link::rl;

uint32_t sf_id = 0;
uint8_t mf_id = 0;

int main() {
    try {
        const auto& config = OpenLdacsConfig::getInstance();
        std::cout << config.getRole() << std::endl;

        // 接管信号
        sigset_t set;
        sigemptyset(&set);
        sigaddset(&set, SIGINT);
        sigaddset(&set, SIGTERM);

        pthread_sigmask(SIG_BLOCK, &set, nullptr);

        const phy::PhyService phy_ser(phy::device::DeviceType::USRP, AS);


        // 3) 主线程阻塞等待信号
        std::cout << "Press Ctrl+C to exit...\n";
        int sig = 0;
        if (sigwait(&set, &sig) != 0) {
            std::cerr << "sigwait failed\n";
            return 1;
        }


        SPDLOG_WARN("[signal] received: {}", sig);

        // 4) 收到信号后，执行清理并优雅退出

        return 0;
    } catch (const std::exception& e) {
        SPDLOG_ERROR("example/test fatal exception: {}", e.what());
        return 1;
    } catch (...) {
        SPDLOG_ERROR("example/test fatal unknown exception");
        return 1;
    }
}

//
// Created by 邹嘉旭 on 2025/12/10.
//


#include  "openldacs.h"
#include "phy/config.h"

#include "phy/fl.h"
#include "phy/phy.h"
#include "util/util.h"

using  namespace  openldacs::phy;

using namespace openldacs;
using namespace openldacs::phy::config;
using namespace openldacs::phy::link::fl;

void test_fl(const PhyService &ser) {
    for (uint8_t sdu_ind = 1; sdu_ind <= 6; sdu_ind++) {
        PhySdu sdu = {
            .direction = FL,
            .sf_id = 0,
            .mf_id = 0,
            .sdu_index = sdu_ind,
            .acm_id = 0,
            .channel = FL_DCH,
        };

        sdu.payload.resize(91);
        for (int j = 0; j < sdu.payload.size(); j++) {
            sdu.payload[j] = j % 256;
        }

        ser.sendFlData(sdu);
    }
}

void test_bc13(const PhyService &ser) {
    PhySdu sdu = {
        .direction = FL,
        .sf_id = 0,
        .mf_id = 0,
        .sdu_index = 0,
        .acm_id = 0,
        .channel = BCCH1_3,
    };

    sdu.payload.resize(66);
    for (int j = 0; j < sdu.payload.size(); j++) {
        sdu.payload[j] = j % 256;
    }

    ser.sendFlData(sdu);
}

void test_bc2(const PhyService &ser) {
    PhySdu sdu = {
        .direction = FL,
        .sf_id = 0,
        .mf_id = 0,
        .sdu_index = 0,
        .acm_id = 0,
        .channel = BCCH2,
    };

    sdu.payload.resize(125);
    for (int j = 0; j < sdu.payload.size(); j++) {
        sdu.payload[j] = j % 256;
    }

    ser.sendFlData(sdu);
}

void test_cc(const PhyService &ser) {
    for (uint8_t sdu_ind = 13; sdu_ind <= 21; sdu_ind++) {
        PhySdu sdu = {
            .direction = FL,
            .sf_id = 0,
            .mf_id = 0,
            .sdu_index = sdu_ind,
            .acm_id = 0,
            .channel = CCCH_DCH,
        };

        sdu.payload.resize(91);
        for (int j = 0; j < sdu.payload.size(); j++) {
            sdu.payload[j] = j % 256;
        }

        ser.sendFlData(sdu);
    }
}

int main() {

    const auto& config = OpenLdacsConfig::getInstance();
    std::cout << config.getRole() << std::endl;

    // 接管信号
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);

    pthread_sigmask(SIG_BLOCK, &set, nullptr);

    std::cout << "n_fft = " << n_fft << std::endl;
    std::cout << "n_cp = " << n_cp << std::endl;
    std::cout << "n_ws = " << n_ws << std::endl;
    std::cout << "n_g = " << n_g << std::endl;
    std::cout << "bw_fft = " << bw_fft << std::endl;
    std::cout << "t_sample = " << t_sample << std::endl;
    std::cout << "delta_f = " << delta_f << std::endl;
    std::cout << "t_cp = " << t_cp << std::endl;
    std::cout << "t_ws = " << t_ws << std::endl;
    std::cout << "t_u = " << t_u << std::endl;
    std::cout << "t_symb = " << t_symb << std::endl;

    const PhyService phy_ser(device::DeviceType::USRP);

    // test_fl(phy_ser);
    // test_bc13(phy_ser);
    // test_bc2(phy_ser);
    test_cc(phy_ser);

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
}

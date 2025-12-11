//
// Created by 邹嘉旭 on 2025/12/10.
//


#include "phy/config.h"
#include <iostream>

#include "phy/fl.h"

int main() {
    using namespace openldacs::phy::config;
    using namespace openldacs::phy::fl;

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

    FLPhy fl_phy;
    fl_phy.test_fl_send(FLType::FL_DATA);

    return 0;
}

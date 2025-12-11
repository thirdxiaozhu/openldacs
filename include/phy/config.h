//
// Created by 邹嘉旭 on 2025/12/10.
//

#ifndef OPENLDACS_CONFIG_H
#define OPENLDACS_CONFIG_H

#include "openldacs.h"
#pragma once
namespace openldacs::phy::config {
    inline constexpr std::size_t n_fft = 64;                 // FFT size
    inline constexpr std::int64_t n_cp = 11;                     // number of cyclic prefix samples
    inline constexpr std::int64_t n_ws = 8;                      // number of window samples
    inline constexpr std::int64_t n_g = n_cp - n_ws;             // number of guard samples
    inline constexpr std::int64_t bw_fft = 625e3;                // FFT bandwidth
    inline constexpr double t_sample = 1.0/bw_fft;         // sample duration
    inline constexpr double delta_f = static_cast<double>(bw_fft) / n_fft; // subcarrier-spacing
    inline constexpr double t_cp = n_cp * t_sample;         // cyclic prefix duration
    inline constexpr double t_ws = n_ws * t_sample;         // window duration
    inline constexpr double t_u = n_fft * t_sample;         // useful duration
    inline constexpr double t_symb = t_u + t_cp;         // OFDM symbol duration
}

#endif //OPENLDACS_CONFIG_H
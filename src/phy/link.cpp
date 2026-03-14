//
// Created by 邹嘉旭 on 2025/12/15.
//

#include "phy/link.h"

namespace openldacs::phy::link {

    using namespace phy::params;

    void ChannelHandler::modulate(BlockBuffer &block, const ModulationType mod_type) const {
        switch (mod_type) {
            case ModulationType::QPSK: {
                block.mod_vec = QPSK_modulator_.modulate(block.coded_bits);
                return;
            }
            case ModulationType::QAM16: {
                block.mod_vec = QAM16_modulator_.modulate(block.coded_bits);
                return;
            }
            case ModulationType::QAM64: {
                block.mod_vec = QAM64_modulator_.modulate(block.coded_bits);
                return;
            }
            default: {
                throw std::runtime_error("Unknown ModulationType");
            }
        }
    }

    itpp::mat ChannelHandler::demodulate(const itpp::cmat &data_equ, const itpp::mat &noise, const ModulationType mod_type) const {
        switch (mod_type) {
            case ModulationType::QPSK: {
                return QPSK_modulator_.demod_soft_matlab(data_equ, noise, getFrame().data_ind);
            }
            case ModulationType::QAM16: {
                return QAM16_modulator_.demod_soft_matlab(data_equ, noise, getFrame().data_ind);
            }
            case ModulationType::QAM64: {
                return QAM64_modulator_.demod_soft_matlab(data_equ, noise, getFrame().data_ind);
            }
            default: {
                throw std::runtime_error("Unknown ModulationType");
            }
        }
    }
}
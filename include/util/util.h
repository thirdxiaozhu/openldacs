//
// Created by 邹嘉旭 on 2025/12/11.
//

#ifndef OPENLDACS_UTIL_H
#define OPENLDACS_UTIL_H
#include "openldacs.h"

namespace openldacs::util {

    inline std::vector<uint8_t> generateRandomBytes(size_t count) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<uint8_t> dis(0, 255);

        std::vector<uint8_t> bytes(count);
        for (size_t i = 0; i < count; ++i) {
            bytes[i] = dis(gen);
        }
        return bytes;
    }

    inline void find_value_imat(std::vector<int>& v, const itpp::imat &m, const int t) {
        for (int i = 0; i < m.size(); i++) {
            if (m(i) == t) {
                v.push_back(i);
            }
        }
    }



    inline std::ostream& operator<<(std::ostream& os, const uint8_t& v) {
        return os << std::hex << std::uppercase
              << std::setw(2) << std::setfill('0')
              << static_cast<int>(v);
    }

    template <typename T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
        os << "[ ";
        for (const auto& x : v)
            os << x << " ";
        os << "]";
        return os;
    }

    static itpp::bmat puncpatToMatrix2output(const std::vector<int>& punc_pat_serial)
    {
        // 只处理 n=2 的母码（1/2卷积码）最常见情况
        // punc_pat_serial 是对序列 [X1_1, X2_1, X1_2, X2_2, ...] 的 0/1 模式
        if (punc_pat_serial.size() % 2 != 0) throw std::runtime_error("punc_pat length must be even for n=2");

        int Period = static_cast<int>(punc_pat_serial.size()) / 2;
        itpp::bmat P(2, Period);
        for (int t = 0; t < Period; ++t) {
            P(0,t) = (punc_pat_serial[2*t + 0] != 0); // X^(1)
            P(1,t) = (punc_pat_serial[2*t + 1] != 0); // X^(2)
        }
        return P;
    }

    // static std::vector<std::size_t> makeHelicalPerm(std::size_t a, std::size_t b) {
    //     if (a == 0 || b == 0) throw std::invalid_argument("a and b must be > 0");
    //     const std::size_t N = a * b;
    //
    //     std::vector<std::size_t> perm(N, 0);
    //
    //     for (std::size_t l = 0; l < a; ++l) {
    //         for (std::size_t n = 0; n < b; ++n) {
    //             // 交织前索引 k：通常就是按行展开
    //             const std::size_t k = l * b + n;
    //             const std::size_t mk = b * ((3 * n + l) % a) + n;
    //
    //             if (mk >= N) throw std::runtime_error("mk out of range; check formula / (a,b)");
    //             perm[k] = mk;
    //         }
    //     }
    //
    //     // 可选：检查是否为真置换（0..N-1 各一次）
    //     std::vector<std::size_t> chk = perm;
    //     std::sort(chk.begin(), chk.end());
    //     for (std::size_t i = 0; i < N; ++i) {
    //         if (chk[i] != i) throw std::runtime_error("perm is not a permutation; check formula");
    //     }
    //     return perm;
    // }
    //
    //
    // static std::vector<int> interleaveHelical(int int_size, std::size_t a, std::size_t b) {
    //
    //     std::vector<int> in(int_size);
    //     std::iota(in.begin(), in.end(), 1);  // 从1开始填充
    //
    //     const std::size_t N = a * b;
    //     if (in.size() != N) throw std::invalid_argument("input size must be a*b");
    //     const auto perm = makeHelicalPerm(a, b);
    //
    //     std::vector<int> out(N);
    //     for (std::size_t k = 0; k < N; ++k) {
    //         out[perm[k]] = in[k];
    //     }
    //     return out;
    // }
    //
    static itpp::bvec bytesToBitsMSB(const itpp::ivec &input) {
        // std::cout << input.left(100) << std::endl;

        itpp::bvec b(8 * input.size());
        int k = 0;
        for (int r = 0; r < input.size(); ++r) {
            for (int i = 7; i >= 0; --i) {      // MSB -> LSB
                b(k++) = (input(r) >> i) & 0x01;
            }
        }
        return b;
    }

    static itpp::ivec bitsToBytesMSB(const itpp::bvec &bits) {
        if (bits.size() % 8 != 0) {
            throw std::runtime_error("bits size must be a multiple of 8");
        }

        itpp::ivec out(bits.size() / 8);
        out.zeros();
        for (int byte_idx = 0; byte_idx < out.size(); ++byte_idx) {
            uint8_t x = 0;
            for (int i = 0; i < 8; ++i) {
                x = static_cast<uint8_t>((x << 1) | (bits(byte_idx * 8 + i) ? 1 : 0));
            }
            out(byte_idx) = x;
        }
        // std::cout << out.left(100) << std::endl;
        return out;
    }



    inline void dump_cvec_constellation(const itpp::cvec& v, const std::string& file_path) {
        std::ofstream ofs(file_path);
        if (!ofs.is_open()) {
            throw std::runtime_error("failed to open " + file_path);
        }
        ofs << std::setprecision(17);
        for (int i = 0; i < v.size(); ++i) {
            ofs << v(i).real() << "," << v(i).imag() << "\n";
        }
    }

    inline void dump_cmat_constellation(const itpp::cmat& m, const std::string& file_path) {
        std::ofstream ofs(file_path);
        if (!ofs.is_open()) {
            throw std::runtime_error("failed to open " + file_path);
        }
        ofs << std::setprecision(17);
        for (int c = 0; c < m.cols(); ++c) {
            for (int r = 0; r < m.rows(); ++r) {
                const auto z = m(r, c);
                ofs << c << "," << z.real() << "," << z.imag() << "\n";
            }
        }
    }

    static void dump_ofdm_mag_per_symbol(const itpp::cmat& frames_freq,
                                     const std::string& prefix)
    {
        int N = frames_freq.rows();   // FFT size
        int M = frames_freq.cols();   // OFDM symbols

        for (int s = 0; s < M; ++s) {
            std::string fn = prefix + "_sym" + itpp::to_str(s) + ".dat";
            std::ofstream ofs(fn.c_str());
            ofs << std::setprecision(17);

            for (int k = 0; k < N; ++k) {
                double mag = std::abs(frames_freq(k, s));
                ofs << k << " " << mag << "\n";
            }
        }
    }

    static itpp::vec fliplrRowvec(const itpp::vec &v_row) {
        itpp::vec out(v_row.size());
        for (int i = 0; i < v_row.size(); ++i) {
            out(i) = v_row(v_row.size() - 1 - i);
        }
        return out;
    }


    static void normalize_cvec(itpp::cvec &cvec) {
        double max_val = itpp::max(abs(cvec));
        if (max_val > 0) {
             cvec /= max_val;
        }
    }


    static VecCD cvecToCdVecNormalize(const itpp::cvec &cvec) {
        VecCD ret;
        itpp::cvec input = cvec;

        // 先归一化
        normalize_cvec(input);

        for (int i = 0; i < input.size(); ++i) {
            ret.push_back(input(i));
        }

        return ret;
    }

    static itpp::cvec cdVecToCvec(const VecCD &vec) {
        itpp::cvec in_f(vec.size());
        for (int i = 0; i < vec.size(); i++) {
            in_f(i) = vec[i];
        }
        return in_f;
    }


}

#endif //OPENLDACS_UTIL_H
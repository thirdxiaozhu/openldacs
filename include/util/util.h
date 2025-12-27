//
// Created by 邹嘉旭 on 2025/12/11.
//

#ifndef OPENLDACS_UTIL_H
#define OPENLDACS_UTIL_H
#include <openldacs.h>

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

    template <typename T>
    void find_Xi(std::vector<T>& v, const Eigen::MatrixXi m, const int t) {
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

    static itpp::bmat puncpat_to_matrix_2output(const std::vector<int>& punc_pat_serial)
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

    static std::vector<std::size_t> make_helical_perm(std::size_t a, std::size_t b) {
        if (a == 0 || b == 0) throw std::invalid_argument("a and b must be > 0");
        const std::size_t N = a * b;

        std::vector<std::size_t> perm(N, 0);

        for (std::size_t l = 0; l < a; ++l) {
            for (std::size_t n = 0; n < b; ++n) {
                // 交织前索引 k：通常就是按行展开
                const std::size_t k = l * b + n;

                const std::size_t inner_mod = (3 * n + l) % a;
                const std::size_t mk = b * inner_mod + n;

                if (mk >= N) throw std::runtime_error("mk out of range; check formula / (a,b)");
                perm[k] = mk;
            }
        }

        // 可选：检查是否为真置换（0..N-1 各一次）
        std::vector<std::size_t> chk = perm;
        std::sort(chk.begin(), chk.end());
        for (std::size_t i = 0; i < N; ++i) {
            if (chk[i] != i) throw std::runtime_error("perm is not a permutation; check formula");
        }
        return perm;
    }


    static std::vector<int> interleave_helical(int int_size, std::size_t a, std::size_t b) {

        std::vector<int> in(int_size);
        std::iota(in.begin(), in.end(), 1);  // 从1开始填充

        const std::size_t N = a * b;
        if (in.size() != N) throw std::invalid_argument("input size must be a*b");
        const auto perm = make_helical_perm(a, b);

        std::vector<int> out(N);
        for (std::size_t k = 0; k < N; ++k) {
            out[perm[k]] = in[k];
        }
        return out;
    }
}

#endif //OPENLDACS_UTIL_H
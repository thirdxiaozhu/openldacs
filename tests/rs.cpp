#include <iostream>

// extern "C" {
//     // 声明 C 语言的函数，防止名称修饰
//     // correct_reed_solomon *correct_reed_solomon_create(
//     //     uint16_t, uint8_t, uint8_t, size_t);
//
//     void correct_reed_solomon_encode(correct_reed_solomon *rs, uint8_t *msg_raw, size_t message_length, uint8_t *msg_encode);
//
//     void correct_reed_solomon_decode(correct_reed_solomon *rs, uint8_t *msg_encode, size_t message_length, uint8_t *msg_decode);
// }
#ifdef __cplusplus
extern "C" {
#endif

#include <correct.h>
    // correct_convolutional *correct_convolutional_create(size_t inv_rate, size_t order,
    //                                                     const correct_convolutional_polynomial_t *poly);
    // void correct_reed_solomon_encode(correct_reed_solomon *rs, uint8_t *msg_raw, size_t message_length, uint8_t *msg_encode);
    // void correct_reed_solomon_decode(correct_reed_solomon *rs, uint8_t *msg_encode, size_t message_length, uint8_t *msg_decode);

#ifdef __cplusplus
}
#endif

int main() {
    srand(time(NULL));

    size_t block_length = 101;
    size_t min_distance = 10;
    // size_t block_length = 255;
    // size_t min_distance = 32;
    size_t message_length = block_length - min_distance;

    correct_reed_solomon *rs = correct_reed_solomon_create(
        correct_rs_primitive_polynomial_8_4_3_2_0, 1, 1, min_distance);

    uint8_t msg_raw[256] = {0};
    uint8_t msg_encode[256] = {0};
    uint8_t msg_decode[256] = {0};

    for (int i = 0; i < 256; i++) {
        msg_raw[i] = i;
    }

    correct_reed_solomon_encode(rs, msg_raw, 91, msg_encode);

    for (int i = 0; i < 128; i++) {
        printf("%d ", msg_encode[i]);
    }
    printf("\n\n");

    // 模拟一些错误
    msg_encode[2] ^= 0x57;
    msg_encode[3] ^= 0x57;
    msg_encode[4] ^= 0x57;
    msg_encode[5] ^= 0x57;
    msg_encode[100] ^= 0x57;

    correct_reed_solomon_decode(rs, msg_encode, 101, msg_decode);

    for (int i = 0; i < 128; i++) {
        printf("%d ", msg_decode[i]);
    }
    printf("\n\n");

    return 0;
}

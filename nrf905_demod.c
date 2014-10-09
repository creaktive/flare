#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lib_crc.h"

#define IOBUF           (8192)
#define SAMPLE_RATE     (16)
#define PREAMBLE        ('10101010101001100110') /* Manchester-encoded 0x3F6 (0xAAA66) */
#define PREAMBLE_BITS   (10)
#define PREAMBLE_SIZE   (2 * SAMPLE_RATE * PREAMBLE_BITS)
#define MAX_MSG_SIZE    (32)
#define BUFFER_SIZE     (PREAMBLE_SIZE + 2 * SAMPLE_RATE * MAX_MSG_SIZE * CHAR_BIT)

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 3 -a 6.2500000000e-02 0.0000000000e+00 -l */
float lp_filter_100khz(float input) {
    static float xv[4], yv[4];
    xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3];
    xv[3] = input / 1.886646578e+02;
    yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3];
    yv[3] = (xv[0] + xv[3]) + 3 * (xv[1] + xv[2])
        + (  0.4535459334 * yv[0])
        + ( -1.7151178300 * yv[1])
        + (  2.2191686183 * yv[2]);
    return yv[3];
}

static uint8_t hi_map[PREAMBLE_BITS] = { 1, 3, 5, 7, 9, 11, 12, 15, 16, 19 };
static uint8_t lo_map[PREAMBLE_BITS] = { 0, 2, 4, 6, 8, 10, 13, 14, 17, 18 };

void process_sample(int16_t A) {
    static int16_t buffer[BUFFER_SIZE];
    static uint16_t buffer_end = 0;
    static uint16_t buffer_skip = BUFFER_SIZE;
    static int32_t sum = 0;

    int16_t threshold;
    uint16_t i, j, k;
    uint8_t msg[MAX_MSG_SIZE];
    uint8_t msg_len;
    uint8_t m1, m2;
    uint16_t crc16;

    buffer[buffer_end] = A;
    if (++buffer_end == BUFFER_SIZE)
        buffer_end = 0;

    sum -= buffer[(buffer_end + BUFFER_SIZE) % BUFFER_SIZE];
    sum += buffer[(buffer_end + BUFFER_SIZE + PREAMBLE_SIZE) % BUFFER_SIZE];

    if (buffer_skip) {
        buffer_skip--;
    } else {
        threshold = sum / PREAMBLE_SIZE;

        j = 0;
        for (i = 0; i < PREAMBLE_BITS; i++)
            if (buffer[(buffer_end + hi_map[i] * SAMPLE_RATE) % BUFFER_SIZE] > threshold)
                j++;
        for (i = 0; i < PREAMBLE_BITS; i++)
            if (buffer[(buffer_end + lo_map[i] * SAMPLE_RATE) % BUFFER_SIZE] < threshold)
                j++;

        if (j == 2 * PREAMBLE_BITS) {
            memset(msg, 0, MAX_MSG_SIZE);
            crc16 = 0xffff;
            for (i = PREAMBLE_SIZE, j = 0; i < BUFFER_SIZE; i += SAMPLE_RATE * 2, j++) {
                msg_len = (j >> 3) + 1;
                m1 = buffer[(buffer_end + i) % BUFFER_SIZE] < threshold;
                m2 = buffer[(buffer_end + i + SAMPLE_RATE) % BUFFER_SIZE] < threshold;
                if (m1 == m2)
                    break;
                else if (m1)
                    msg[msg_len - 1] |= 1 << (7 - (j & 7));

                if ((j & 7) == 7) {
                    crc16 = update_crc_ccitt(crc16, msg[msg_len - 1]);
                    if (crc16 == 0) {
                        for (k = 0; k < msg_len; k++)
                            printf("%02x", msg[k]);
                        printf("\n");

                        buffer_skip = PREAMBLE_SIZE + 2 * SAMPLE_RATE * msg_len * CHAR_BIT;
                        break;
                    }
                }
            }
        }
    }
}

int process_stream(FILE *stream) {
    int16_t readbuf[IOBUF];
    uint16_t c;
    size_t len;

    while (!feof(stream)) {
        len = fread(readbuf, sizeof(int16_t), IOBUF, stream);
        for (c = 0; c < len; c++)
            process_sample(lp_filter_100khz(readbuf[c]));
    }
    return 0;
}

int main(int argc, char **argv) {
    return process_stream(stdin);
}

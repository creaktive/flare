#include <complex.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "lib_crc.h"

#define forceinline __inline__ __attribute__((always_inline)) static

#define symbol_samples      (16)
#define symbol_rate         (100000)
#define buffer_size         (1 << 13)
#define max_packet_bytes    (29)
#define preamble_bits       (20)
#define packet_samples      (symbol_samples * 2 * (preamble_bits + max_packet_bytes * 8))
#define dft_points          (symbol_samples)
#define sample_rate         (symbol_rate * symbol_samples)

#if buffer_size < packet_samples
#error "Adjust buffer_size to fit at least one packet + preamble!"
#endif

#if buffer_size & (buffer_size - 1)
#error "buffer_size has to be a power of 2!"
#endif

static complex float coeffs[dft_points];

static complex float cb_buf_iq[buffer_size];
static uint16_t cb_idx_iq;

#define cb_mask(n) (sizeof(cb_buf_##n) / sizeof(cb_buf_##n[0]) - 1)
#define cb_write(n, v) (cb_buf_##n[(cb_idx_##n++) & cb_mask(n)] = (v))
#define cb_readn(n, i) (cb_buf_##n[(cb_idx_##n + (~i)) & cb_mask(n)])

#define magnitude(v) (creal(v) * creal(v) + cimag(v) * cimag(v))
#define bit_compare(v) ((v) > threshold ? 1 : 0)

const uint8_t preamble_pattern[preamble_bits] = { 1,0,1,0,1,0,1,0,1,0,1,0,0,1,1,0,0,1,1,0 };

uint16_t validate_output(const uint8_t *packet, const uint16_t length) {
    uint16_t i, j;
    char output[128], *p;
    uint16_t crc16 = 0xffff;
    struct timeval tv;
    double timestamp, rms;

    for (i = 0, p = output; i < length; i++, p += 2) {
        crc16 = update_crc_ccitt(crc16, packet[i]);
        snprintf(p, 3, "%02x", packet[i]);

        if (crc16 == 0) {
            gettimeofday(&tv, NULL);
            timestamp = tv.tv_sec + tv.tv_usec / 1e6;
            timestamp -= (buffer_size / sample_rate) * 2;

            for (j = 0, rms = 0; j < packet_samples; j++)
                rms += magnitude(cb_readn(iq, j));
            rms /= packet_samples;

            snprintf(output + (i + 1) * 2, sizeof(output) + (i + 1) * 2,
                    "\t%.06f\t%.01f",
                    timestamp,
                    20.0 * log10(sqrt(rms) / 181.019336)
            );

            puts(output);
            fflush(stdout);

            /* memset((void *) packet, 0, max_packet_bytes); */

            return packet_samples;
        }
    }

    return 0;
}

forceinline void bit_slicer(const uint8_t channel, const int32_t amplitude) {
    static int32_t cb_buf_pcm[2][buffer_size];
    static uint16_t cb_idx_pcm[2];
    static int32_t sliding_sum[2];
    static uint16_t skip_samples[2];
    static uint8_t packet[max_packet_bytes];
    uint16_t i, j, k, l;
    int32_t threshold;
    int32_t average;
    uint8_t manchester_bits[packet_samples / symbol_samples];
    uint16_t bit_position, byte_position;

    cb_write(pcm[channel], amplitude);

    sliding_sum[channel] -= cb_readn(pcm[channel], packet_samples);
    sliding_sum[channel] += amplitude;

    if (skip_samples[channel]) {
        skip_samples[channel]--;
        return;
    }

    threshold = sliding_sum[channel] / (packet_samples / symbol_samples);

    for (
        i = packet_samples, j = l = 0;
        j < preamble_bits;
        i -= symbol_samples, j++
    ) {
        average = 0.;
        for (k = 0; k < symbol_samples; k++)
            average += cb_readn(pcm[channel], i + k);
        if (preamble_pattern[j] == bit_compare(average))
            l++; else break;
    }

    if (l == preamble_bits) {
        for (
            i = packet_samples - preamble_bits * symbol_samples, j = 0;
            j < max_packet_bytes * symbol_samples;
            i -= symbol_samples, j++
        ) {
            average = 0.;
            for (k = 0; k < symbol_samples; k++)
                average += cb_readn(pcm[channel], i + k);
            manchester_bits[j] = bit_compare(average);

            if (j % 2) {
                if (manchester_bits[j] != manchester_bits[j - 1]) {
                    // valid Manchester
                    bit_position = j / 2;
                    byte_position = bit_position / 8;
                    if (manchester_bits[j]) {
                        // set to 0
                        packet[byte_position] &= ~(1 << (7 - (bit_position & 7)));
                    } else {
                        // set to 1
                        packet[byte_position] |=  (1 << (7 - (bit_position & 7)));
                    }
                }
            }
        }

        skip_samples[channel] = validate_output((const uint8_t *) packet, max_packet_bytes);
    }
}

forceinline void sliding_dft(const int8_t i_sample, const int8_t q_sample) {
    complex float sample, prev_sample;
    uint16_t i;
    static complex float dft[dft_points];

    __real__ sample = i_sample;
    __imag__ sample = q_sample;
    cb_write(iq, sample);

    prev_sample = cb_readn(iq, dft_points);
    for (i = 1; i <= 4; i++)
        dft[i] = (dft[i] - prev_sample + sample) * coeffs[i];

    bit_slicer(0, magnitude(dft[1]) - magnitude(dft[2]));
    bit_slicer(1, magnitude(dft[3]) - magnitude(dft[4]));
}

int main(int argc, char **argv) {
    uint16_t i;
    size_t len;
    uint8_t raw_buffer[buffer_size * 2];

    for (i = 0; i < dft_points; i++)
        coeffs[i] = cexp(I * 2. * M_PI * i / dft_points);

    while (!feof(stdin)) {
        len = fread(raw_buffer, sizeof(raw_buffer[0]), sizeof(raw_buffer), stdin);
        for (i = 0; i < len; i += 2)
            sliding_dft(raw_buffer[i] - 127, raw_buffer[i + 1] - 127);
    }

    return 0;
}

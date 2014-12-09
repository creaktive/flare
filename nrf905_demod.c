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
#define max_packet_bytes    (29)
#define preamble_bits       (20)
#define packet_samples      (symbol_samples * 2 * (preamble_bits + max_packet_bytes * 8))
#define dft_points          (symbol_samples)
#define sample_rate         (symbol_rate * symbol_samples)
#define buffer_size         (1 << 13)
#define smooth_buffer_size  (1 << 3)
#define average_n           (7)

#if buffer_size < packet_samples
#error "Adjust buffer_size to fit at least one packet + preamble!"
#endif

#if (buffer_size & (buffer_size - 1)) || (smooth_buffer_size & (smooth_buffer_size - 1))
#error "buffer sizes has to be a power of 2!"
#endif

static complex float coeffs[dft_points];

static complex float cb_buf_iq[buffer_size];
static uint16_t cb_idx_iq;

#define cb_mask(n) (sizeof(cb_buf_##n) / sizeof(cb_buf_##n[0]) - 1)
#define cb_write(n, v) (cb_buf_##n[(cb_idx_##n++) & cb_mask(n)] = (v))
#define cb_readn(n, i) (cb_buf_##n[(cb_idx_##n + (~i)) & cb_mask(n)])

#define magnitude(v) (creal(v) * creal(v) + cimag(v) * cimag(v))

const uint8_t preamble_pattern[preamble_bits] = { 1,0,1,0,1,0,1,0,1,0,1,0,0,1,1,0,0,1,1,0 };

void output(const uint8_t *packet, const uint16_t length, const uint8_t channel) {
    uint16_t i, j;
    char output[128], *p;
    struct timeval tv;
    double timestamp, rms;

    for (i = 0, p = output; i < length; i++, p += 2)
        snprintf(p, 3, "%02x", packet[i]);

    gettimeofday(&tv, NULL);
    timestamp = tv.tv_sec + tv.tv_usec / 1e6;
    timestamp -= (buffer_size / sample_rate) * 2;

    for (j = 0, rms = 0; j < packet_samples; j++)
        rms += magnitude(cb_readn(iq, j));
    rms /= packet_samples;

    snprintf(output + i * 2, sizeof(output) + i * 2,
            "\t%.06f\t%.01f\t%d",
            timestamp,
            20.0 * log10(sqrt(rms) / 181.019336),
            channel + 117 // freq = (422.4 + (CH_NO / 10)) * (1 + HFREQ_PLL) MHz
    );

    puts(output);
    fflush(stdout);
}

forceinline void bit_slicer(const uint8_t channel, const int32_t amplitude) {
    static int32_t cb_buf_pcm[2][smooth_buffer_size];
    static uint16_t cb_idx_pcm[2];
    static uint8_t cb_buf_bit[2][buffer_size];
    static uint16_t cb_idx_bit[2];
    static int32_t sliding_sum[2];
    static uint16_t skip_samples[2];
    static uint8_t packet[max_packet_bytes];
    uint16_t i, j, k;
    uint16_t bad_manchester;
    uint16_t crc16 = 0xffff;

    cb_write(pcm[channel], amplitude);

    sliding_sum[channel] -= cb_readn(pcm[channel], average_n);
    sliding_sum[channel] += amplitude;

    cb_write(bit[channel], sliding_sum[channel] > 0 ? 1 : 0);

    if (skip_samples[channel]) {
        skip_samples[channel]--;
        return;
    }

    for (
        i = packet_samples, j = 0;
        j < preamble_bits;
        i -= symbol_samples, j++
    ) {
        if (preamble_pattern[j] != cb_readn(bit[channel], i))
            return;
    }

    bad_manchester = 0;
    for (
        i = packet_samples - preamble_bits * symbol_samples, j = 0;
        i > symbol_samples;
        i -= symbol_samples * 2, j++
    ) {
        k = j / 8;
        if (cb_readn(bit[channel], i) != cb_readn(bit[channel], i + symbol_samples)) {
            // valid Manchester
            if (cb_readn(bit[channel], i)) {
                // set to 1
                packet[k] |=  (1 << (7 - (j & 7)));
            } else {
                // set to 0
                packet[k] &= ~(1 << (7 - (j & 7)));
            }
        } else {
            // heuristic, skip if too many bit encoding errors
            if (++bad_manchester > j / 2)
                return;
        }

        if ((j & 7) == 7) {
            crc16 = update_crc_ccitt(crc16, packet[k]);
            if (crc16 == 0) {
                k++;
                output((const uint8_t *) packet, k, channel);
                skip_samples[channel] = symbol_samples * 2 * (preamble_bits + k * 8);
                /* memset((void *) packet, 0, sizeof(packet)); */
                return;
            }
        }
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
    for (i = 0; i < 4; i++)
        dft[i] = (dft[i] - prev_sample + sample) * coeffs[i];

    bit_slicer(0, magnitude(dft[0]) - magnitude(dft[1]));
    bit_slicer(1, magnitude(dft[2]) - magnitude(dft[3]));
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

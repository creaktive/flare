/* nrf905_demod, demodulator for nRF905 Single chip 433/868/915MHz Transceiver
 * Copyright (C) 2014 Stanislaw Pusep
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <complex.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "lib_crc.h"

/* Some subs are subs for organizational purposes only. They're not intended
 * to be reused. GCC 4.8 seems smart enough to figure it out on it's own,
 * but just in case, let's manually assure these are inlined.
 */
#define forceinline __inline__ __attribute__((always_inline)) static

/* "Magic" constants... These come directly or indirectly from the nRF905
 * specification. Here is a simplified explanation about how this program works.
 * First of all, nRF905 sends two kinds of pulses: mark & space. Mark means "1"
 * symbol and space means "0" (or vice-versa; who cares, more on this later).
 * If you tune nRF905 to use 868.2MHz (AKA "channel 117"), space is sent at a
 * slightly lower frequency (868.15MHz) while mark is sent at a slightly higher
 * one (868.25MHz). There's nothing interesting at exactly 868.2MHz (at least,
 * not for this program). So, fact #1: there's 100KHz separation between mark
 * & space.
 * Then, enter the sample rate. If we read 1 million of samples every second
 * (1MHz sample rate) each symbol (and therefore, mark/space pulse) will be
 * spread across just 10 samples. This means that we only have a serie of
 * 10 values to figure out the frequency. By the way, this serie is in
 * "time domain". Fourier transform turns it into "frequency domain".
 * But for 10 input values, it will produce 10 output values.
 * Thus, 10 samples => 10 frequencies. Which frequencies? Intuitively enough,
 * our 1MHz sample rate is equally split in 10 "bins" spaced by 100KHz.
 * Fact #2: each symbol has just enough samples to make it possible to
 * discriminate mark/space. Which means that we can tune to 868.15MHz, and
 * "bin 0" will filter our spaces, and "bin 1" will filter our marks...
 * Except we can not use "bin 0", because it is somewhat special (DC).
 * Instead, we tune to 868.05MHz and get "bin 1" as space and "bin 2" as mark.
 * Then we get "bin 3" and "bin 4" as space & mark for the next channel
 * (868.4MHz, AKA "channel 118"). And perhaps the next channel... And so on
 * until "bin 6", which wraps and gets us a "negative frequency" (that is,
 * something below 868.05MHz that we're tuned to). Let's not talk about bins
 * 6-10 for now.
 * Finally, fact #3: computers are not impressed when we round up our
 * arithmetics to 10, they prefer 16. That's why the sampling rate is 1.6MHz
 * and we have 16 samples per symbol.
 */
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

/* Here we store the precomputed coefficients for Discrete Fourier Transform.
 * "But isn't it terribly slow?!" Glad you asked; in this specific case,
 * DFT is actually faster than FFT! That is because due to the nature of the
 * demodulator, we can reuse the results of the computation of the previous
 * samples. Besides that, we don't need all the 16 frequency bins for the 16
 * samples, just 2 (mark/space) per channel.
 */
static complex float coeffs[dft_points];

/* Our samples are stored in a circular buffer. It overwrites the oldest
 * samples with the newest ones. Each raw I/Q sample pair from the RTL-SDR
 * dongle is stored as one complex number. Because Fourier transforms work on
 * complex numbers...
 */
static complex float cb_buf_iq[buffer_size];
static uint16_t cb_idx_iq;

#define cb_mask(n) (sizeof(cb_buf_##n) / sizeof(cb_buf_##n[0]) - 1)
#define cb_write(n, v) (cb_buf_##n[(cb_idx_##n++) & cb_mask(n)] = (v))
#define cb_readn(n, i) (cb_buf_##n[(cb_idx_##n + (~i)) & cb_mask(n)])

/* ...But to make any sense of the output, complex number has to be "squashed"
 * into good old float.
 */
#define magnitude(v) (creal(v) * creal(v) + cimag(v) * cimag(v))

/* Each nRF905 transmission starts with a pattern called "preamble". It is
 * something that is clearly distinguishable from the noise. In this case,
 * alternation of "0" and "1" symbols. We just try to match this specific bit
 * pattern against the RF stream. Almost like regular expression.
 */
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

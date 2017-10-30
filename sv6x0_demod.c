/* sv6x0_demod, demodulator for NiceRF SV6X0 433/868/915MHz transceiver's family
 *
 * Copyright (C) 2014 Stanislaw Pusep
 * ALtered to work with SV650 (PilotAware Bridge) by Linar Yusupov in 2017.
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
#include <stdlib.h>
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

#define symbol_samples      (24)
#define symbol_rate         (38400)
#define max_packet_bytes    (37)
#define preamble_bits       (40)
#define packet_samples      (symbol_samples * 2 * (preamble_bits + max_packet_bytes * 8))
#define dft_points          (symbol_samples)
#define sample_rate         (symbol_rate * symbol_samples)
#define buffer_size         (1 << 14)
#define smooth_buffer_size  (1 << 3)
#define average_n           (7)
#define payload_offset      (7)

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

/* Intermediary values of the signal demodulation process are stored in
 * circular buffers. It works by overwriting the oldest values with the newest
 * ones. Each circular buffer allocates 2 variables: the buffer (prefixed with
 * "cb_buf_") and the current element index (prefixed with "cb_idx_").
 */
static complex float cb_buf_iq[buffer_size];
static uint16_t cb_idx_iq;

/* Circular buffer accessors. These are macros instead of subroutines mainly
 * because there are many different data types for buffers to handle. Raw I/Q
 * samples are complex, magnitudes are integer, decoded bits are characters.
 * cb_write(buffer_name, X) inserts X into the last position of the buffer.
 * cb_readn(buffer_name, N) reads from Nth position of the buffer, where 0 is
 * the last position, 1 is the previous position, and so on.
 */
#define cb_mask(n) (sizeof(cb_buf_##n) / sizeof(cb_buf_##n[0]) - 1)
#define cb_write(n, v) (cb_buf_##n[(cb_idx_##n++) & cb_mask(n)] = (v))
#define cb_readn(n, i) (cb_buf_##n[(cb_idx_##n + (~i)) & cb_mask(n)])

/* To make any sense of the output, complex number has to be "squashed" into
 * good old float. However, we do not use sqrt() because it is too expensive!
 */
#define magnitude(v) (creal(v) * creal(v) + cimag(v) * cimag(v))

/* Each nRF905/Si4432 transmission starts with a pattern called "preamble". It is
 * something that is clearly distinguishable from the noise. In this case,
 * alternation of "0" and "1" symbols. We just try to match this specific bit
 * pattern against the RF stream. Almost like a regular expression.
 */

const uint8_t preamble_pattern[preamble_bits] = { 
  1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0
};

const uint8_t dewhitening_pattern[] = { 0x05, 0xb4, 0x05, 0xae, 0x14, 0xda,
  0xbf, 0x83, 0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2, 0x01, 0xa3, 0x26,
  0xac, 0xbb, 0x63, 0xf1, 0x01, 0xca, 0x07, 0xbd, 0xaf, 0x60, 0xc8, 0x12, 0xed,
  0x04, 0xbc, 0xf6, 0x12, 0x2c, 0x01, 0xd9, 0x04, 0xb1, 0xd5, 0x03, 0xab, 0x06,
  0xcf, 0x08, 0xe6, 0xf2, 0x07, 0xd0, 0x12, 0xc2, 0x09, 0x34, 0x20 };   

/* Runtime configuration globals.
 */
static uint8_t packet_bytes = 0, use_crc = 1;

/* Subroutine: output()
 * Description: print the decoded packet, timestamp, RSSI and channel ID
 * Input:
 *  packet: buffer with packet bytes
 *  length: size of the packet
 *  channel: ordinal of the channel buffer
 * Output: none
 */
void output(const uint8_t *packet, const uint16_t length, const uint8_t channel) {
    uint16_t i, j;
    char output[128], *p;
    struct timeval tv;
    double timestamp, rms;

    for (i = 0, p = output; i < length; i++, p += 2) {
      if (i >= payload_offset && i < (packet[payload_offset-1] + payload_offset)  ) {
        snprintf(p, 3, "%02x", packet[i] ^ dewhitening_pattern[i-payload_offset]);      
      } else {
        snprintf(p, 3, "%02x", packet[i]);
      }
    }

    /* Since all the data was already in the buffer, compensate the timestamp
     * subtracting the "time on the wire".
     */
    gettimeofday(&tv, NULL);
    timestamp = tv.tv_sec + tv.tv_usec / 1e6;
    timestamp -= (buffer_size / sample_rate) * 2;

    /* "RMS" as in "Root Mean Square".
     * Estimate the power of the signal we've just decoded.
     */
    for (j = 0, rms = 0; j < packet_samples; j++)
        rms += magnitude(cb_readn(iq, j));
    rms /= packet_samples;

    snprintf(output + i * 2, sizeof(output) + i * 2,
            "\t%.06f\t%.01f\t%d",
            timestamp,
            20.0 * log10(sqrt(rms) / 181.019336), // almost certainly wrong
            channel + 117 // freq = (422.4 + (CH_NO / 10)) * (1 + HFREQ_PLL) MHz
    );

    puts(output);
    fflush(stdout);
}

/* Subroutine: bit_slicer()
 * Description: recover bits from the channel
 * Input:
 *  channel: up to 2 channels are supported for now
 *  amplitude: sample value
 * Output: none
 */
forceinline void bit_slicer(const uint8_t channel, const int32_t amplitude) {
    /* Why is everything so "static"? As mentioned in the "forceinline"
     * comment way above, these subroutines are not real subroutines. Thus we
     * need to use "static" in order to preserve the buffer contents. Or use
     * global variables.
     * The best part is why the 'packet' buffer is also "static": nRF905
     * resends the packets (sometimes on different channels). If we miss some
     * bits on the first try, perhaps we manage to get them on the second
     * attempt. Note that this is only possible because we differentiate
     * "0" from "1" from "missing" during the decoding step!
     */
    static int32_t cb_buf_pcm[2][smooth_buffer_size];
    static uint16_t cb_idx_pcm[2];
    static uint8_t cb_buf_bit[2][buffer_size];
    static uint16_t cb_idx_bit[2];
    static int32_t sliding_sum[2];
    static uint16_t skip_samples[2];
    static uint8_t packet[max_packet_bytes];
    uint16_t i, j, k;

    uint16_t crc16 = 0x0000;


    /* Simplest possible noise filter (at least, in software): sliding average.
     */
    cb_write(pcm[channel], amplitude);

    sliding_sum[channel] -= cb_readn(pcm[channel], average_n);
    sliding_sum[channel] += amplitude;

    /* Input for bit_slicer() is the magnitude at the space pulse frequency minus
     * the magnitude at the mark pulse frequency. If this value is positive,
     * the space signal is stronger than the mark signal. Thus, by convention,
     * we have the "0" symbol. Same thing happens for the "1" symbol.
     * However, these symbols are not bits yet: actual bits are encoded using
     * the Manchester coding.
     */
    cb_write(bit[channel], sliding_sum[channel] > 0 ? 1 : 0);

    /* Don't reprocess samples if we already decoded this as a valid message.
     * This saves a lot of processing time, specially when dealing with busy
     * channels.
     */
    if (skip_samples[channel]) {
        skip_samples[channel]--;
        return;
    }

    /* Attempt to match the preamble bit pattern. Bail out on the first
     * discrepancy to spare CPU cycles. This is the hottest code path
     * (most CPU-intensive).
     */
    for (
        i = packet_samples, j = 0;
        j < preamble_bits;
        i -= symbol_samples, j++
    ) {
        if (preamble_pattern[j] != cb_readn(bit[channel], i))
            return;
    }

    /* When the preamble looks like valid, attempt to decode the rest of the
     * packet.
     */

    for (
        i = packet_samples - preamble_bits * symbol_samples, j = 0;
        i > symbol_samples;
        i -= symbol_samples, j++
    ) {
        k = j / 8;

        if (!cb_readn(bit[channel], i)) {
            // set to 1
            packet[k] |=  (1 << (7 - (j & 7)));
        } else {
            // set to 0
            packet[k] &= ~(1 << (7 - (j & 7)));
        }

        /* At the end of every 8-bit chunk, update CRC checksum. When the
         * checksum is (looks like) correct, proceed and output the packet
         * bytes, regardless the packet size. It is quite possible that some
         * incompletely processed packet appears to have a correct CRC.
         * If the desired packet size is known/fixed and/or CRC is unavailable,
         * it is possible to use packet size as the "packet received" condition.
         */
        if ((j & 7) == 7) {
            if (k >= payload_offset) {
              crc16 = use_crc ? update_crc_ccitt(crc16, packet[k]) : 0;
            }
            k++;
            if (crc16 == 0 && k == (packet_bytes ? packet_bytes : k)) {
                output((const uint8_t *) packet, k, channel);
                skip_samples[channel] = symbol_samples * 2 * (preamble_bits + k * 8);
                /* memset((void *) packet, 0, sizeof(packet)); */
                return;
            }
        }
    }
}

/* Subroutine: sliding_dft()
 * Description: transform the signal from time domain to frequency domain
 * Input:
 *  i_sample: In-Phase component
 *  q_sample: Quadrature component
 * Output: none
 */
forceinline void sliding_dft(const int8_t i_sample, const int8_t q_sample) {
    complex float sample, prev_sample;
    uint16_t i;
    static complex float dft[dft_points];

    /* Each raw I/Q ("In-Phase/Quadrature") sample pair from the RTL-SDR dongle
     * is stored as one complex float. Samples are not normalized (meaning the
     * values are not within the range [0,1]) because division is expensive.
     */
    __real__ sample = i_sample;
    __imag__ sample = q_sample;
    cb_write(iq, sample);

    /* Compute the Discrete Fourier Transform for the last 'dft_points' samples.
     * This works more-or-less like the moving average; instead of recalculating
     * the entire thing for every 'dft_points' samples, we "add" the recent ones
     * and "subtract" the oldest ones. Also, we don't compute the frequency bins
     * for the frequencies we don't use, anyway.
     * What kind of sorcery is this?! \(o_O)/
     * Unfortunately, there's a downside: the frequency resolution is locked
     * to the amount of samples per symbol. With Fast Fourier Transform, it is
     * possible to apply some smart "window function" to overcome the resolution
     * limitations. With Sliding DFT, the only practical window function is the
     * rectangular one (AKA "none at all"). But, again, it is just enough to
     * get the 100KHz resolution.
     */
    prev_sample = cb_readn(iq, dft_points);
    for (i = 1; i <= 4; i++)
        dft[i] = (dft[i] - prev_sample + sample) * coeffs[i];

    /* TODO: implement threads.
     * Now that the channels are separated, each one can be handled by
     * a different CPU. If only we have more than one CPU.
     * How this works: for each channel, we subtract the power of signal at
     * the mark frequency from the power of signal at the space frequency.
     * This way, the noise floor (which is expected to be more-or-less the same
     * at both frequencies) cancels out. It is feasible to use either mark
     * or space frequencies alone, but that would require extra computation
     * to tell signal apart from the noise floor.
     */
    bit_slicer(0, magnitude(dft[1]) - magnitude(dft[2])); // power at bins 1 & 2
    bit_slicer(1, magnitude(dft[3]) - magnitude(dft[4])); // power at bins 3 & 4
}

/* Subroutine: main()
 * Description: get chunks of data from STDIN and forward to sliding_dft()
 * Input:
 *  argc: argument counter
 *  argv: array of arguments
 * Output: exit code
 */
int main(int argc, char **argv) {
    uint16_t i;
    size_t len;
    uint8_t raw_buffer[buffer_size * 2]; // each I/Q sample has two bytes!
    int8_t param;

    /* Accepts packet size as a parameter. When specified, the decoded packet
     * have to have this exact size and a valid CRC-16. When negative, only
     * the packet size is checked, and CRC-16 is not computed.
     */
    if (argc == 2) {
        param = atoi(argv[1]);
        packet_bytes = abs(param);
        use_crc = param == packet_bytes;
        if (packet_bytes > max_packet_bytes) {
            fprintf(stderr, "can't decode more than %d bytes\n", max_packet_bytes);
            return 2;
        }
    } else if (argc > 2) {
        fprintf(stderr, "usage: %s [PACKET_BYTES]\n", argv[0]);
        return 1;
    }

    /* Pre-compute the DFT coefficients. We will only use some of them in
     * sliding_dft().
     */
    for (i = 0; i < dft_points; i++)
        coeffs[i] = cexp(I * 2. * M_PI * i / dft_points);

    /* Read chunks of data piped from rtl_sdr utility and call sliding_dft()
     * for each sample. The data comes in I/Q pairs, like: IQIQIQIQIQ...
     * Individual values (either I or Q) range is (0, 255), and to convert to
     * signed we need to subtract 127. No idea why RTL-SDR dongle doesn't use
     * signed integer by default (looks like the hardware itself returns the
     * data in this way).
     */
    while (!feof(stdin)) {
        len = fread(raw_buffer, sizeof(raw_buffer[0]), sizeof(raw_buffer), stdin);
        for (i = 0; i < len; i += 2)
            sliding_dft(raw_buffer[i] - 127, raw_buffer[i + 1] - 127);
    }

    return 0;
}

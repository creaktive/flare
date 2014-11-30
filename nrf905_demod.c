#include <complex.h>
#include <fftw3.h>
#include <math.h>
#include <stdint.h>
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
#define window_length       (symbol_samples)
#define fft_points          (symbol_samples)
#define sample_rate         (symbol_rate * symbol_samples)

#if buffer_size < packet_samples
#error "Adjust buffer_size to fit at least one packet + preamble!"
#endif

#if buffer_size & (buffer_size - 1)
#error "buffer_size has to be a power of 2!"
#endif

#if symbol_samples & (symbol_samples - 1)
#warning "FFTW is inefficient for the selected sample rate!"
#endif

static fftw_complex *fft_inp, *fft_out;
static fftw_plan fft_plan;
static double window_function[window_length];

#define hann(i)     (.50 * (1. - cos(2. * M_PI * (i) / (window_length - 1))))
#define hamming(i)  (.54 - .46 * cos(2. * M_PI * (i) / (window_length - 1)))
#define use_window  hamming

static double iq_buffer[2][buffer_size];
static uint16_t iq_index[2];

#define cb_mask(n) (sizeof(n##_buffer[0]) / sizeof(n##_buffer[0][0]) - 1)
#define cb_write(n, c, v) (n##_buffer[c][(n##_index[c]++) & cb_mask(n)] = (v))
#define cb_readn(n, c, i) (n##_buffer[c][(n##_index[c] + (~i)) & cb_mask(n)])

#define fft_magnitude(bin) (                        \
    __real__ fft_out[bin] * __real__ fft_out[bin] + \
    __imag__ fft_out[bin] * __imag__ fft_out[bin]   \
)
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

            for (j = 0, rms = 0; j < packet_samples; j++) {
                rms +=
                    cb_readn(iq, 0, j) * cb_readn(iq, 0, j) +
                    cb_readn(iq, 1, j) * cb_readn(iq, 1, j);
            }
            rms /= packet_samples;

            snprintf(output + (i + 1) * 2, sizeof(output) + (i + 1) * 2,
                    "\t%.06f\t%.01f",
                    timestamp,
                    20.0 * log10(sqrt(rms) / 181.019336)
            );

            puts(output);
            fflush(stdout);

            return packet_samples;
        }
    }

    return 0;
}

forceinline void bit_slicer(const uint8_t channel, const int32_t amplitude) {
    static int32_t pcm_buffer[2][buffer_size];
    static uint16_t pcm_index[2];
    static int32_t sliding_sum[2];
    static uint16_t skip_samples[2];
    static uint8_t packet[max_packet_bytes];
    uint16_t i, j, k, l;
    int32_t threshold;
    int32_t average;
    uint8_t manchester_bits[packet_samples / symbol_samples];
    uint16_t bit_position, byte_position;

    cb_write(pcm, channel, amplitude);

    sliding_sum[channel] -= cb_readn(pcm, channel, packet_samples);
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
            average += cb_readn(pcm, channel, i + k);
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
                average += cb_readn(pcm, channel, i + k);
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

forceinline void sliding_fft(const double i_sample, const double q_sample) {
    uint16_t i, j;

    cb_write(iq, 0, i_sample);
    cb_write(iq, 1, q_sample);

#if fft_points != window_length
    memset(fft_inp, 0, sizeof(fftw_complex) * fft_points);
#endif
    for (i = 0, j = window_length - 1; i < window_length; i++, j--) {
        __real__ fft_inp[i] = cb_readn(iq, 0, j) * window_function[j];
        __imag__ fft_inp[i] = cb_readn(iq, 1, j) * window_function[j];
    }

    fftw_execute(fft_plan);

    bit_slicer(0, fft_magnitude(3) - fft_magnitude(4));
    bit_slicer(1, fft_magnitude(1) - fft_magnitude(2));
}

int main(int argc, char **argv) {
    uint16_t i;
    size_t len;
    uint8_t raw_buffer[buffer_size * 2];
    uint16_t c;

    fft_inp = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * fft_points);
    fft_out = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * fft_points);
    fft_plan = fftw_plan_dft_1d(fft_points, fft_inp, fft_out, FFTW_FORWARD, FFTW_MEASURE);

    for (i = 0; i < window_length; i++)
        window_function[i] = use_window(i);

    while (!feof(stdin)) {
        len = fread(raw_buffer, sizeof(uint16_t), buffer_size / 2, stdin);
        for (c = 0; c < len * 2; c += 2)
            sliding_fft(raw_buffer[c] - 127., raw_buffer[c + 1] - 127.);
    }

    fftw_free(fft_inp);
    fftw_free(fft_out);
    fftw_destroy_plan(fft_plan);

    return 0;
}

/* flarm_decode, decoder for FLARM radio protocol
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

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flarm_codec.h"

/* http://en.wikipedia.org/wiki/XXTEA */
void btea(uint32_t *v, int8_t n, const uint32_t key[4]) {
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

    #define DELTA 0x9e3779b9
    // #define ROUNDS (6 + 52 / n)
    #define ROUNDS 6
    #define MX (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))

    if (n > 1) {
        /* Coding Part */
        rounds = ROUNDS;
        sum = 0;
        z = v[n - 1];
        do {
            sum += DELTA;
            e = (sum >> 2) & 3;
            for (p = 0; p < n - 1; p++) {
                y = v[p + 1];
                z = v[p] += MX;
            }
            y = v[0];
            z = v[n - 1] += MX;
        } while (--rounds);
    } else if (n < -1) {
        /* Decoding Part */
        n = -n;
        rounds = ROUNDS;
        sum = rounds * DELTA;
        y = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = n - 1; p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= MX;
            }
            z = v[n - 1];
            y = v[0] -= MX;
            sum -= DELTA;
        } while (--rounds);
    }
}

/* https://metacpan.org/source/GRAY/Geo-Distance-XS-0.13/XS.xs */
const float DEG_RADS = M_PI / 180.;
const float KILOMETER_RHO = 6371.64;
float haversine(float lat1, float lon1, float lat2, float lon2) {
    lat1 *= DEG_RADS; lon1 *= DEG_RADS;
    lat2 *= DEG_RADS; lon2 *= DEG_RADS;
    float a = sin(0.5 * (lat2 - lat1));
    float b = sin(0.5 * (lon2 - lon1));
    float c = a * a + cos(lat1) * cos(lat2) * b * b;
    float d = 2. * atan2(sqrt(c), sqrt(fabs(1. - c)));
    return d;
}

/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed) {
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

void make_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
    static const uint32_t table[4] = FLARM_KEY1;
    int8_t i;
    for (i = 0; i < 4; i++)
        key[i] = obscure(table[i] ^ ((timestamp >> 6) ^ address), FLARM_KEY2) ^ FLARM_KEY3;
}

char *flarm_decode(const flarm_packet *pkt, float ref_lat, float ref_lon, int16_t ref_alt, double timestamp, float rssi, int16_t channel) {
    if (!(pkt->magic == 0x10 || pkt->magic == 0x20)) {
        fprintf(stderr, "bad packet signature: 0x%02x\n", pkt->magic);
        return NULL;
    }

    uint32_t key[4];
    make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    btea((uint32_t *) pkt + 1, -5, key);

    int32_t round_lat = (int32_t) (ref_lat * 1e7) >> 7;
    int32_t lat = (pkt->lat - round_lat) % (uint32_t) 0x080000;
    if (lat >= 0x040000) lat -= 0x080000;
    lat = ((lat + round_lat) << 7) + 0x40;

    int32_t round_lon = (int32_t) (ref_lon * 1e7) >> 7;
    int32_t lon = (pkt->lon - round_lon) % (uint32_t) 0x100000;
    if (lon >= 0x080000) lon -= 0x100000;
    lon = ((lon + round_lon) << 7) + 0x40;

    int32_t vs = pkt->vs * (2 << (pkt->vsmult - 1));

    char tmp[32];
    static char out[512];

    out[0] = '\0';

    #define json_concat(...)                        \
        snprintf(tmp, sizeof(tmp), ##__VA_ARGS__);  \
        strncat(out, tmp, sizeof(out) - sizeof(tmp) - 1);

    json_concat("{\"addr\":\"%X\",", pkt->addr);
    if (timestamp > 1.4e9) {
        json_concat("\"time\":%.06f,", timestamp);
    }
    if (fabs(rssi) > 0.01) {
        json_concat("\"rssi\":%.01f,", rssi);
    }
    if (channel > 0) {
        json_concat("\"channel\":%d,", channel);
    }
    json_concat("\"lat\":%.07f,", lat / 1e7);
    json_concat("\"lon\":%.07f,", lon / 1e7);
    json_concat("\"dist\":%.02f,", haversine(ref_lat, ref_lon, lat / 1e7, lon / 1e7) * KILOMETER_RHO * 1000);
    json_concat("\"alt\":%d,", pkt->alt - ref_alt);
    json_concat("\"vs\":%d,", vs);
    json_concat("\"type\":%d,", pkt->type);
    json_concat("\"ns\":[%d,%d,%d,%d],", pkt->ns[0], pkt->ns[1], pkt->ns[2], pkt->ns[3]);
    json_concat("\"ew\":[%d,%d,%d,%d]}", pkt->ew[1], pkt->ew[1], pkt->ew[2], pkt->ew[3]);

    return out;
}

int main(int argc, char **argv) {
    float ref_lat, ref_lon, rssi;
    double timestamp;
    int16_t channel = -1;
    int16_t ref_alt = 0;
    char *line = NULL;
    char *p, *q;
    uint8_t buf[29];
    size_t len = 0;
    uint16_t i;
    uint8_t offset;

    if (argc >= 3 && argc <= 4) {
        ref_lat = atof(argv[1]);
        ref_lon = atof(argv[2]);
        if (argc == 4)
            ref_alt = atoi(argv[3]);
    } else {
        fprintf(stderr, "usage: %s LAT LON [GEOID_ALT]\n", argv[0]);
        return 1;
    }

    while (getline(&line, &len, stdin) != -1) {
        i = 0;
        for (p = line, q = p + strlen(line) - 1; p < q; p++) {
            if (isxdigit(*p)
                && isxdigit(*(p + 1))
                && sscanf(p, "%2hhx", &buf[i]) == 1
            ) {
                p++;
                if (++i == sizeof(buf))
                    break;
            } else {
                i = 0;
            }
        }

        if (i == 24 || i == 29) {
            timestamp = rssi = channel = 0;
            if (p++ < q)
                sscanf(p, "%lf %f %hd", &timestamp, &rssi, &channel);

            offset = buf[0] == 0x31 && buf[1] == 0xfa && buf[2] == 0xb6
                ? 3
                : 0;

            q = flarm_decode(
                (flarm_packet *) (buf + offset),
                ref_lat, ref_lon, ref_alt,
                timestamp,
                rssi,
                channel
            );

            if (q) puts(q);
            fflush(stdout);
        } else
            fprintf(stderr, "only packets with either 24 or 29 bytes are accepted\n");
    }

    return 0;
}

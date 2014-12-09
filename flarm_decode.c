#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flarm_codec.h"
#include "lib_crc.h"

static uint32_t const key1[4] = FLARM_KEY1;
static uint32_t const key2[4] = FLARM_KEY2;

/* https://en.wikipedia.org/wiki/XTEA#Implementations */
void xtea_decrypt(unsigned int num_rounds, uint32_t v[2], uint32_t const key[4]) {
    unsigned int i;
    uint32_t v0 = v[0], v1 = v[1], delta = 0x9E3779B9, sum = delta * num_rounds;
    for (i = 0; i < num_rounds; i++) {
        v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum >> 11) & 3]);
        sum -= delta;
        v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
    }
    v[0] = v0; v[1] = v1;
}

/* https://metacpan.org/source/GRAY/Geo-Distance-XS-0.13/XS.xs */
const float DEG_RADS = M_PI / 180.;
const float KILOMETER_RHO = 6371.64;
float haversine (float lat1, float lon1, float lat2, float lon2) {
    lat1 *= DEG_RADS; lon1 *= DEG_RADS;
    lat2 *= DEG_RADS; lon2 *= DEG_RADS;
    float a = sin(0.5 * (lat2 - lat1));
    float b = sin(0.5 * (lon2 - lon1));
    float c = a * a + cos(lat1) * cos(lat2) * b * b;
    float d = 2. * atan2(sqrt(c), sqrt(fabs(1. - c)));
    return d;
}

char *flarm_decode(flarm_packet *pkt, float ref_lat, float ref_lon, int16_t ref_alt, float timestamp, float rssi, int16_t channel) {
    xtea_decrypt(6, (uint32_t *) pkt + 1, key1);
    xtea_decrypt(6, (uint32_t *) pkt + 3, key2);

    int32_t round_lat = (int32_t) (ref_lat * 1e7) >> 7;
    int32_t lat = (((int16_t) ((pkt->lat - round_lat) & 0xffff) + round_lat) << 7) + 0x40;

    uint8_t s = lat >= 4.5e8 ? 8 : 7;
    int32_t round_lon = (int32_t) (ref_lon * 1e7) >> s;
    int32_t lon = (((int16_t) ((pkt->lon - round_lon) & 0xffff) + round_lon) << s) + (1 << (s - 1));

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
    if (channel >= 0) {
        json_concat("\"channel\":%d,", channel);
    }
    json_concat("\"lat\":%.07f,", lat / 1e7);
    json_concat("\"lon\":%.07f,", lon / 1e7);
    json_concat("\"dist\":%.02f,", haversine(ref_lat, ref_lon, lat / 1e7, lon / 1e7) * KILOMETER_RHO * 1000);
    json_concat("\"alt\":%d,", pkt->alt - ref_alt);
    json_concat("\"vs\":%d,", vs);
    json_concat("\"stealth\":%d,", pkt->stealth);
    json_concat("\"type\":%d,", pkt->type);
    json_concat("\"ns\":[%d,%d,%d,%d],", pkt->ns[0], pkt->ns[1], pkt->ns[2], pkt->ns[3]);
    json_concat("\"ew\":[%d,%d,%d,%d]}", pkt->ew[1], pkt->ew[1], pkt->ew[2], pkt->ew[3]);

    return out;
}

int main(int argc, char **argv) {
    float ref_lat, ref_lon, timestamp, rssi;
    int16_t channel = -1;
    int16_t ref_alt = 0;
    char *line = NULL;
    char *p, *q;
    uint8_t buf[29];
    size_t len = 0;
    uint16_t i;
    uint16_t crc16;

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
        crc16 = 0xffff;
        for (p = line, q = p + strlen(line) - 1; p < q; p++) {
            if (isxdigit(*p)
                && isxdigit(*(p + 1))
                && sscanf(p, "%2hhx", &buf[i]) == 1
            ) {
                p++;
                crc16 = update_crc_ccitt(crc16, buf[i]);
                if (++i == sizeof(buf))
                    break;
            } else {
                i = 0;
                crc16 = 0xffff;
            }
        }

        if (crc16 == 0) {
            timestamp = rssi = channel = 0;
            if (p++ < q)
                sscanf(p, "%f %f %hd", &timestamp, &rssi, &channel);

            q = flarm_decode(
                (flarm_packet *) (buf + 3),
                ref_lat, ref_lon, ref_alt,
                timestamp,
                rssi,
                channel
            );

            puts(q);
            fflush(stdout);
        } else
            fprintf(stderr, "record with bad CRC\n");
    }

    return 0;
}

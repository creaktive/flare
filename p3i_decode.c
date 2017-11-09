/* p3i_decode, decoder for P3I (PilotAware) radio protocol
 *
 * Copyright (C) 2014 Stanislaw Pusep
 * Copyright (C) 2017 Linar Yusupov
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

#include "Protocol_P3I.h"

char *p3i_decode(const p3i_packet_t *pkt, double timestamp, float rssi, int16_t channel) {

    char tmp[32];
    static char out[512]; 

    out[0] = '\0';

    #define json_concat(...)                        \
        snprintf(tmp, sizeof(tmp), ##__VA_ARGS__);  \
        strncat(out, tmp, sizeof(out) - sizeof(tmp) - 1);

    json_concat("{\"addr\":\"%X\",", pkt->icao);
    if (timestamp > 1.4e9) {
        json_concat("\"time\":%.06f,", timestamp);
    }
    if (fabs(rssi) > 0.01) {
        json_concat("\"rssi\":%.01f,", rssi);
    }
    if (channel > 0) {
        json_concat("\"channel\":%d,", channel);
    }
    json_concat("\"lat\":%.07f,", pkt->latitude);
    json_concat("\"lon\":%.07f,", pkt->longitude);
    json_concat("\"alt\":%d,", pkt->altitude);
    json_concat("\"type\":%d,", pkt->aircraft);
    json_concat("\"track\":%d,", pkt->track);
    json_concat("\"knots\":%d}", pkt->knots);

    return out;
}

int main(int argc, char **argv) {
    float rssi;
    double timestamp;
    int16_t channel = -1;
    char *line = NULL;
    char *p, *q;
    uint8_t buf[33];
    size_t len = 0;
    uint16_t i;
    uint8_t offset;

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
                if (i == 29) break;
                i = 0;
            }
        }

        if (i == 29 || i == 33) {
            timestamp = rssi = channel = 0;
            if (p++ < q)
                sscanf(p, "%lf %f %hd", &timestamp, &rssi, &channel);

            offset = (buf[0] == 0x2d && buf[1] == 0xd4 ? 2 : 0);

            offset += P3I_PAYLOAD_OFFSET;

            q = p3i_decode(
                (p3i_packet_t *) (buf + offset),
                timestamp,
                rssi,
                channel
            );

            if (q) puts(q);
            fflush(stdout);
        } else
            fprintf(stderr, "only packets with either 29 or 33 bytes are accepted (got %d)\n", i);

    }

    return 0;
}

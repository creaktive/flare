/* ogntp_decode, decoder for OGNTP radio protocol
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

#include "OGN/ogn.h"

typedef struct {

  /* Dummy type definition. Actual Tx/Rx packet format is defined in ogn.h */ 

} ogntp_packet_t;

static OGN_RxPacket Rx;

char *ogntp_decode(const ogntp_packet_t *pkt, double timestamp, float rssi, int16_t channel) {

    char tmp[32];
    static char out[512]; 
 
    Rx.recvBytes((uint8_t *) pkt);
    Rx.Packet.Dewhiten();

    out[0] = '\0';

    #define json_concat(...)                        \
        snprintf(tmp, sizeof(tmp), ##__VA_ARGS__);  \
        strncat(out, tmp, sizeof(out) - sizeof(tmp) - 1);

    json_concat("{\"addr\":\"%X\",", Rx.Packet.Header.Address);
    if (timestamp > 1.4e9) {
        json_concat("\"time\":%.06f,", timestamp);
    }
    if (fabs(rssi) > 0.01) {
        json_concat("\"rssi\":%.01f,", rssi);
    }
    if (channel > 0) {
        json_concat("\"channel\":%d,", channel);
    }
    json_concat("{\"addr type\":\"%X\",", Rx.Packet.Header.AddrType);
//    json_concat("{\"other\":\"%X\",", Rx.Packet.Header.Other);
//    json_concat("{\"parity\":\"%X\",", Rx.Packet.Header.Parity);
//    json_concat("{\"relay count\":\"%X\",", Rx.Packet.Header.RelayCount);
//    json_concat("{\"encrypted\":\"%X\",", Rx.Packet.Header.Encrypted);
//    json_concat("{\"emergency\":\"%X\",", Rx.Packet.Header.Emergency);

    json_concat("\"lat\":%.07f,", 0.0001/60*Rx.Packet.DecodeLatitude());
    json_concat("\"lon\":%.07f,", 0.0001/60*Rx.Packet.DecodeLongitude());
    json_concat("\"alt\":%d,", Rx.Packet.DecodeAltitude());
    json_concat("\"type\":%d,", Rx.Packet.Position.AcftType);
    json_concat("\"stealth\":%d}", Rx.Packet.Position.Stealth);

    return out;
}

int main(int argc, char **argv) {
    float rssi;
    double timestamp;
    int16_t channel = -1;
    char *line = NULL;
    char *p, *q;
    uint8_t buf[29];
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
                if (i == 24) break;
                i = 0;
            }
        }

        if (i == 24 || i == 29) {
            timestamp = rssi = channel = 0;
            if (p++ < q)
                sscanf(p, "%lf %f %hd", &timestamp, &rssi, &channel);

            offset = buf[0] == 0xf3 && buf[1] == 0x65 && buf[2] == 0x6c
                ? 3
                : 0;

            q = ogntp_decode(
                (ogntp_packet_t *) (buf + offset),
                timestamp,
                rssi,
                channel
            );

            if (q) puts(q);
            fflush(stdout);
        } else
            fprintf(stderr, "only packets with either 24 or 29 bytes are accepted (got %d)\n", i);

    }

    return 0;
}

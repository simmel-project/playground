#include <stdio.h>
#include <string.h>

#include "afsk.h"

#include "afsk/demod.h"
#include "afsk/mac.h"
#include "afsk/murmur3.h"

#include "printf.h"

static FSK_demod_const demod_table;
static FSK_demod_state demod_state;
static struct mac_state mac_state;
static int packet_count = 0;
static int corrupt_count = 0;
static demod_pkt_t packet;

uint32_t debug_print_sync = 1;

static int validate_packet(demod_pkt_t *pkt, int should_print) {
    unsigned int i;
    demod_pkt_ctrl_t *cpkt = &pkt->ctrl_pkt;
    demod_pkt_data_t *dpkt = &pkt->data_pkt;
    uint32_t hash;

    if (should_print) {
        printf("Got packet:\n");
        printf("    Header:\n");
        printf("        Version: %d\n", pkt->header.version);
        printf("           Type: %d  ", pkt->header.type);
    }
    switch (pkt->header.type) {

    case PKTTYPE_CTRL:
        if (should_print) {
            printf("Ctrl Packet\n");
            printf("       Reserved: %d\n", cpkt->reserved);
            printf("         Length: %ld\n", cpkt->length);
            printf("      Full Hash: %08lx\n", cpkt->fullhash);
            printf("           GUID: ");
            for (i = 0; i < 16; i++) printf("%02x ", cpkt->guid[i]);
            printf("\n");
            printf("    Packet Hash: %08lx ", cpkt->hash);
        }
        MurmurHash3_x86_32((uint8_t *)cpkt, sizeof(*cpkt) - sizeof(cpkt->hash),
                           MURMUR_SEED_BLOCK, &hash);
        if (hash != cpkt->hash) {
            if (should_print) printf("!= %08lx\n", hash);
            return 0;
        }
        if (should_print) printf("Ok\n");
        break;

    case PKTTYPE_DATA:
        // unstripe the transition xor's used to keep baud sync. We
        // don't xor the header or the ending hash, but xor
        // everything else..
        for (i = sizeof(pkt->header); i < sizeof(*dpkt) - sizeof(dpkt->hash);
             i++) {
            if (pkt->header.version == PKT_VER_1) {
                // baud striping on alpha and before
                if ((i % 16) == 7)
                    ((uint8_t *)pkt)[i] ^= 0x55;
                else if ((i % 16) == 15)
                    ((uint8_t *)pkt)[i] ^= 0xAA;
            } else if (pkt->header.version == PKT_VER_2) {
                // more dense baud striping to be used on beta and
                // beyond
                if ((i % 3) == 0)
                    ((uint8_t *)pkt)[i] ^= 0x35;
                else if ((i % 3) == 1)
                    ((uint8_t *)pkt)[i] ^= 0xac;
                else if ((i % 3) == 2)
                    ((uint8_t *)pkt)[i] ^= 0x95;
            }
        }

        if (should_print) {
            printf("Data Packet\n");
            printf("   Block Number: %d\n", dpkt->block);
            printf("    Packet Hash: %08lx ", dpkt->hash);
        }
        /* Make sure the packet's hash is correct. */
        MurmurHash3_x86_32((uint8_t *)dpkt, sizeof(*dpkt) - sizeof(dpkt->hash),
                           MURMUR_SEED_BLOCK, &hash);
        if (hash != dpkt->hash) {
            if (should_print) printf("!= %08lx\n", hash);
            return 0;
        }
        if (should_print) printf("Ok\n");
        break;

    default:
        if (should_print) printf(" (unknown type)\n");
        return 0;
        break;
    }
    return 1;
}

void afsk_init(const struct demod_config *cfg) {

    memset(&mac_state, 0, sizeof(mac_state));
    fsk_demod_generate_table(&demod_table, cfg->baud_rate, cfg->sample_rate,
                             cfg->f_lo, cfg->f_hi, cfg->filter_width);
    fsk_demod_init(&demod_table, &demod_state);
}

void afsk_run(int16_t *samples, size_t nsamples) {
    size_t processed_samples;

    int bit = 0;
    while (nsamples > 0) {
        if (fsk_demod(&demod_table, &demod_state, &bit, samples, nsamples, &processed_samples)) {
            if (mac_put_bit(&mac_state, bit, &packet, sizeof(packet))) {
                if (validate_packet(&packet, 0)) {
                    packet_count++;
                } else {
                    corrupt_count++;
                }
            }
        }
        nsamples -= processed_samples;
        samples += processed_samples;
    }
}

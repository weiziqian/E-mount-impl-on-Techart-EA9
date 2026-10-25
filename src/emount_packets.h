/* emount_packets.h -- the reply templates in emount_packets.c, one per
 * message, and the table em_send_id looks them up in. */
#ifndef EMOUNT_PACKETS_H
#define EMOUNT_PACKETS_H

#include <stdint.h>

#define EA9_PACKET_COUNT 16

struct ea9_packet {
	uint8_t  id;
	uint8_t *frame;
	uint16_t len;
};

/* Live, writable: the status frames are rewritten every body frame. */
#define EA9_LEN_01 41u
extern uint8_t em_msg_01[EA9_LEN_01];
#define EA9_LEN_05 105u
extern uint8_t em_msg_05[EA9_LEN_05];
#define EA9_LEN_06 48u
extern uint8_t em_msg_06[];          /* EM06_LEN_MAX bytes: the tail grows */
#define EA9_LEN_07 43u
extern uint8_t em_msg_07[EA9_LEN_07];
#define EA9_LEN_08 210u
extern uint8_t em_msg_08[EA9_LEN_08];
#define EA9_LEN_09 20u
extern uint8_t em_msg_09[EA9_LEN_09];
#define EA9_LEN_0B 11u
extern uint8_t em_msg_0B[EA9_LEN_0B];
#define EA9_LEN_0C 10u
extern uint8_t em_msg_0C[EA9_LEN_0C];
#define EA9_LEN_0D 10u
extern uint8_t em_msg_0D[EA9_LEN_0D];
#define EA9_LEN_10 10u
extern uint8_t em_msg_10[EA9_LEN_10];
#define EA9_LEN_19 10u
extern uint8_t em_msg_19[EA9_LEN_19];
#define EA9_LEN_1B 19u
extern uint8_t em_msg_1B[EA9_LEN_1B];
#define EA9_LEN_28 44u
extern uint8_t em_msg_28[EA9_LEN_28];
#define EA9_LEN_34 32u
extern uint8_t em_msg_34[EA9_LEN_34];
#define EA9_LEN_3D 72u
extern uint8_t em_msg_3D[EA9_LEN_3D];
#define EA9_LEN_3F 74u
extern uint8_t em_msg_3F[EA9_LEN_3F];

extern const struct ea9_packet ea9_packets[EA9_PACKET_COUNT];

#endif /* EMOUNT_PACKETS_H */

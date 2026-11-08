/* SPDX-License-Identifier: MIT */
/**
 * @file  test_proto.c
 * @brief Unit tests for the xiao_proto serial frame codec.
 *
 * The wire format is a contract: firmware examples and the host application
 * must produce identical bytes. These tests pin the byte layout, the CRC8
 * parameters (poly 0x07, init 0x00, over LEN..PAYLOAD) and the bounds
 * behaviour, so any accidental change to the encoding fails here first.
 */

#include "proto/frame.h"

#include <stdio.h>
#include <string.h>

static int g_checks = 0;
static int g_fail = 0;

#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_fail++;                                                       \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, (msg));          \
        }                                                                   \
    } while (0)

/* Reference CRC8 (poly 0x07, init 0x00) computed bit by bit. */
static uint8_t ref_crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1));
    }
    return crc;
}

/* CRC8 over an empty input must be the init value. */
static void test_crc8_empty(void)
{
    CHECK(proto_crc8(NULL, 0) == 0x00, "crc8(empty) != init 0x00");
}

/* CRC8 must match the reference implementation on assorted byte patterns. */
static void test_crc8_matches_reference(void)
{
    const uint8_t a[] = {0x00};
    const uint8_t b[] = {0xFF};
    const uint8_t c[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    const uint8_t d[] = {0xA5, 0x00, 0x7F, 0xDE, 0xAD, 0xBE, 0xEF};
    const uint8_t e[32] = {0};

    CHECK(proto_crc8(a, sizeof a) == ref_crc8(a, sizeof a), "crc8 {0x00}");
    CHECK(proto_crc8(b, sizeof b) == ref_crc8(b, sizeof b), "crc8 {0xFF}");
    CHECK(proto_crc8(c, sizeof c) == ref_crc8(c, sizeof c), "crc8 ramp");
    CHECK(proto_crc8(d, sizeof d) == ref_crc8(d, sizeof d), "crc8 magic");
    CHECK(proto_crc8(e, sizeof e) == ref_crc8(e, sizeof e), "crc8 zeros");
}

/* A frame is SOF | LEN | TYPE | SEQ | PAYLOAD | CRC8, little endian payload. */
static void test_frame_layout(void)
{
    const uint8_t payload[] = {0x11, 0x22, 0x33};
    uint8_t out[PROTO_FRAME_MAX];
    memset(out, 0xEE, sizeof out);

    size_t n = proto_frame_encode(PROTO_TYPE_IMU_SAMPLE, 0x07,
                                  payload, sizeof payload, out, sizeof out);
    CHECK(n == 4 + sizeof payload + 1, "frame length = SOF+LEN+TYPE+SEQ+payload+CRC8");
    CHECK(out[0] == PROTO_SOF, "byte0 must be SOF 0xA5");
    CHECK(out[1] == sizeof payload, "byte1 must be payload length");
    CHECK(out[2] == PROTO_TYPE_IMU_SAMPLE, "byte2 must be frame type");
    CHECK(out[3] == 0x07, "byte3 must be sequence number");
    CHECK(memcmp(&out[4], payload, sizeof payload) == 0, "payload must be copied verbatim");
    /* CRC8 covers LEN..PAYLOAD, i.e. everything after SOF and before the CRC. */
    CHECK(out[n - 1] == ref_crc8(&out[1], n - 2), "CRC8 must cover LEN..PAYLOAD");
    CHECK(out[n] == 0xEE, "encoder must not write past the frame");
}

/* SOF must be unique enough that a trivial payload cannot fake a frame start. */
static void test_sof_not_duplicated_in_header(void)
{
    const uint8_t payload[] = {0x00};
    uint8_t out[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(PROTO_TYPE_TEXT, 0x00, payload, sizeof payload,
                                  out, sizeof out);
    CHECK(n == 6, "single byte payload -> 6 byte frame");
    CHECK(PROTO_SOF != PROTO_TYPE_TEXT, "SOF must differ from frame type ids");
    CHECK(PROTO_SOF != PROTO_TYPE_ACK, "SOF must differ from ACK type id");
}

/* Zero-length payload is legal (header-only frame plus CRC). */
static void test_empty_payload(void)
{
    uint8_t out[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(PROTO_TYPE_TEXT, 0x00, NULL, 0, out, sizeof out);
    CHECK(n == 5, "empty payload -> 5 byte frame");
    CHECK(out[0] == PROTO_SOF, "empty frame still starts with SOF");
    CHECK(out[4] == ref_crc8(&out[1], 3), "empty frame CRC8 over LEN+TYPE+SEQ");
}

/* Payload above PROTO_PAYLOAD_MAX is rejected (returns 0, no partial write). */
static void test_payload_too_large(void)
{
    uint8_t big[PROTO_PAYLOAD_MAX + 1];
    uint8_t out[PROTO_FRAME_MAX + 8];
    memset(big, 0x5A, sizeof big);
    memset(out, 0xEE, sizeof out);

    size_t n = proto_frame_encode(PROTO_TYPE_IMU_SAMPLE, 0x00, big,
                                  (uint8_t)sizeof big, out, sizeof out);
    CHECK(n == 0, "payload > PROTO_PAYLOAD_MAX must be rejected");
    CHECK(out[0] == 0xEE, "rejected encode must not touch the output buffer");
}

/* Output buffer smaller than the frame is rejected. */
static void test_output_too_small(void)
{
    const uint8_t payload[] = {0x01, 0x02};
    uint8_t out[8];
    memset(out, 0xEE, sizeof out);

    size_t n = proto_frame_encode(PROTO_TYPE_EULER, 0x00, payload,
                                  sizeof payload, out, 2);
    CHECK(n == 0, "out_cap too small must be rejected");
    CHECK(out[0] == 0xEE, "rejected encode must not write");
}

/* Exact-capacity buffer must succeed (boundary of the size check). */
static void test_exact_capacity(void)
{
    const uint8_t payload[] = {0x04, 0x05};
    uint8_t out[4 + sizeof payload + 1];
    size_t n = proto_frame_encode(PROTO_TYPE_EULER, 0x09, payload,
                                  sizeof payload, out, sizeof out);
    CHECK(n == sizeof out, "exact capacity must succeed");
    CHECK(out[2] == PROTO_TYPE_EULER, "type preserved at exact capacity");
}

/* The published type ids are part of the contract; pin them. */
static void test_type_ids(void)
{
    CHECK(PROTO_SOF == 0xA5, "SOF must stay 0xA5");
    CHECK(PROTO_TYPE_IMU_SAMPLE == 0x01, "IMU_SAMPLE must stay 0x01");
    CHECK(PROTO_TYPE_IMU_CAL == 0x02, "IMU_CAL must stay 0x02");
    CHECK(PROTO_TYPE_EULER == 0x03, "EULER must stay 0x03");
    CHECK(PROTO_TYPE_QUAT == 0x04, "QUAT must stay 0x04");
    CHECK(PROTO_TYPE_ACK == 0x7E, "ACK must stay 0x7E");
    CHECK(PROTO_TYPE_TEXT == 0x7F, "TEXT must stay 0x7F");
    CHECK(PROTO_CMD_HEAD_LEN == 2, "CMD head = MOD_ID + FUNC_ID");
    CHECK(PROTO_ACK_HEAD_LEN == 3, "ACK head = MOD_ID + FUNC_ID + status");
    CHECK(PROTO_PAYLOAD_MAX == 96, "payload cap must stay 96");
}

/* Sequence numbers are the sender's concern: the codec stores them verbatim. */
static void test_sequence_is_verbatim(void)
{
    uint8_t out[PROTO_FRAME_MAX];
    for (unsigned seq = 0; seq < 256; seq += 85) {
        size_t n = proto_frame_encode(PROTO_TYPE_QUAT, (uint8_t)seq, NULL, 0,
                                      out, sizeof out);
        CHECK(n == 5, "frame with no payload is 5 bytes");
        CHECK(out[3] == (uint8_t)seq, "sequence byte stored verbatim");
    }
}

/* Regression: a payload of exactly PROTO_PAYLOAD_MAX must fit in a buffer sized
 * PROTO_FRAME_MAX. PROTO_FRAME_MAX was once one byte short, so a full-size TEXT
 * line (truncated to 96 bytes before encoding) was rejected and silently lost. */
static void test_max_payload_fits_frame_max(void)
{
    uint8_t payload[PROTO_PAYLOAD_MAX];
    uint8_t out[PROTO_FRAME_MAX];
    memset(payload, 0x41, sizeof payload);
    memset(out, 0xEE, sizeof out);

    size_t n = proto_frame_encode(PROTO_TYPE_TEXT, 0x2A, payload,
                                  PROTO_PAYLOAD_MAX, out, sizeof out);
    CHECK(n == PROTO_PAYLOAD_MAX + 5, "full payload frame = 96 + 5 = 101 bytes");
    CHECK(n == sizeof out, "PROTO_FRAME_MAX must be exactly big enough");
    CHECK(out[2] == PROTO_TYPE_TEXT, "full payload keeps its type");
    CHECK(out[PROTO_FRAME_MAX - 1] == ref_crc8(&out[1], PROTO_FRAME_MAX - 2),
          "full payload CRC8 covers LEN..PAYLOAD");
}

int main(void)
{
    test_max_payload_fits_frame_max();
    test_crc8_empty();
    test_crc8_matches_reference();
    test_frame_layout();
    test_sof_not_duplicated_in_header();
    test_empty_payload();
    test_payload_too_large();
    test_output_too_small();
    test_exact_capacity();
    test_type_ids();
    test_sequence_is_verbatim();

    printf("test_proto: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

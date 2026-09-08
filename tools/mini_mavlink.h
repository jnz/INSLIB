/** @file mini_mavlink.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Minimal MAVLink v2 ENCODER for tools/insrcv.c: the handful of messages
 * INSLIB/telemetry.py's MavlinkSender publishes, plus GPS_RAW_INT for the
 * raw receiver fix only the live path has, packed into a
 * caller-owned buffer. Output only, no receive path, no dialect headers
 * and no generated code. The generated MAVLink C library would be a
 * submodule and a build-system dependency for eleven fixed messages, and
 * the point of insrcv.c is to be the reference for host-side glue that
 * gets ported onto an embedded target.
 *
 * Wire format (MAVLink 2, mavlink.io/en/guide/serialization.html):
 *
 *   0xFD, payload_len, incompat_flags, compat_flags, seq, sysid, compid,
 *   msgid (3 bytes LE), payload, CRC16
 *
 * The CRC is X.25 (CRC-16/MCRF4XX) over everything from payload_len
 * through the payload, followed by the message's CRC_EXTRA byte, which is
 * what makes a receiver with a different idea of a message's layout
 * reject the frame instead of misreading it.
 *
 * Field order on the wire is NOT the order in the message definition:
 * MAVLink reorders every message's fields by descending type size before
 * packing. Each builder below writes them in the order pymavlink reports
 * as ordered_fieldnames, so the two receivers put identical bytes on the
 * wire.
 *
 * Messages that carry v2 EXTENSION fields (ATTITUDE_QUATERNION's
 * repr_offset_q, GPS_RAW_INT's alt_ellipsoid/h_acc/v_acc/vel_acc/
 * hdg_acc/yaw, HIGHRES_IMU's id, EKF_STATUS_REPORT's airspeed_variance,
 * STATUSTEXT's id/chunk_seq) are emitted at their base length. An
 * extension a receiver does not get is read as zero, which is exactly
 * what they mean here.
 *
 * EKF_STATUS_REPORT is an ardupilotmega message, not a common one, so a
 * consumer needs that dialect to decode it. pymavlink's default dialect
 * has it, which is why the Python sender can emit it too.
 */
#ifndef MINI_MAVLINK_H
#define MINI_MAVLINK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Message ids and their CRC_EXTRA, straight from the dialect. */
#define MINI_MAV_MSG_HEARTBEAT           0u
#define MINI_MAV_CRC_HEARTBEAT           50u
#define MINI_MAV_MSG_ATTITUDE            30u
#define MINI_MAV_CRC_ATTITUDE            39u
#define MINI_MAV_MSG_ATTITUDE_QUAT       31u
#define MINI_MAV_CRC_ATTITUDE_QUAT       246u
#define MINI_MAV_MSG_LOCAL_POSITION_NED  32u
#define MINI_MAV_CRC_LOCAL_POSITION_NED  185u
#define MINI_MAV_MSG_GLOBAL_POSITION_INT 33u
#define MINI_MAV_CRC_GLOBAL_POSITION_INT 104u
#define MINI_MAV_MSG_GPS_RAW_INT         24u
#define MINI_MAV_CRC_GPS_RAW_INT         24u
#define MINI_MAV_MSG_HIGHRES_IMU         105u
#define MINI_MAV_CRC_HIGHRES_IMU         93u
#define MINI_MAV_MSG_ALTITUDE            141u
#define MINI_MAV_CRC_ALTITUDE            47u
#define MINI_MAV_MSG_EKF_STATUS_REPORT   193u
#define MINI_MAV_CRC_EKF_STATUS_REPORT   71u
#define MINI_MAV_MSG_NAMED_VALUE_FLOAT   251u
#define MINI_MAV_CRC_NAMED_VALUE_FLOAT   170u
#define MINI_MAV_MSG_STATUSTEXT          253u
#define MINI_MAV_CRC_STATUSTEXT          83u

/* MAV_TYPE_GENERIC / MAV_AUTOPILOT_GENERIC, the honest answer for a
   navigation filter that flies nothing. */
#define MINI_MAV_TYPE_GENERIC      0u
#define MINI_MAV_AUTOPILOT_GENERIC 0u
#define MINI_MAV_VERSION           3u

/* EKF_STATUS_FLAGS. */
#define MINI_MAV_EKF_ATTITUDE       0x0001u
#define MINI_MAV_EKF_VELOCITY_HORIZ 0x0002u
#define MINI_MAV_EKF_VELOCITY_VERT  0x0004u
#define MINI_MAV_EKF_POS_HORIZ_REL  0x0008u
#define MINI_MAV_EKF_POS_HORIZ_ABS  0x0010u
#define MINI_MAV_EKF_POS_VERT_ABS   0x0020u
#define MINI_MAV_EKF_POS_VERT_AGL   0x0040u

/* GPS_FIX_TYPE, the subset a u-blox NAV-PVT fixType can map onto.
   MAVLink has no "dead reckoning only" and no "time only", so those map
   onto NO_FIX (see mav_gps_fix_type() in insrcv.c). */
#define MINI_MAV_GPS_FIX_NO_FIX    1u
#define MINI_MAV_GPS_FIX_2D        2u
#define MINI_MAV_GPS_FIX_3D        3u
#define MINI_MAV_GPS_FIX_RTK_FLOAT 5u
#define MINI_MAV_GPS_FIX_RTK_FIXED 6u

/* GPS_RAW_INT's "not known": every uint16 field (eph, epv, vel, cog) has
   the same one, satellites_visible has its own. */
#define MINI_MAV_GPS_U16_UNKNOWN  65535u
#define MINI_MAV_GPS_SATS_UNKNOWN 255u

/* MAV_SEVERITY. */
#define MINI_MAV_SEVERITY_WARNING 4u
#define MINI_MAV_SEVERITY_INFO    6u

/* NAMED_VALUE_FLOAT's name and STATUSTEXT's text are fixed-size char
   arrays, and a longer string is truncated rather than rejected. */
#define MINI_MAV_NAME_LEN 10
#define MINI_MAV_TEXT_LEN 50

#define MINI_MAV_HDR_LEN     10
#define MINI_MAV_MAX_PAYLOAD 255
#define MINI_MAV_FRAME_MAX   (MINI_MAV_HDR_LEN + MINI_MAV_MAX_PAYLOAD + 2)

/** One message under construction, plus the sequence counter a MAVLink
 *  stream has to carry so a receiver can count what it lost. */
typedef struct
{
    uint8_t  sysid;
    uint8_t  compid;
    uint8_t  seq;
    uint8_t  buf[MINI_MAV_FRAME_MAX];
    size_t   n;         /**< payload bytes written so far */
    uint32_t msgid;
    uint8_t  crc_extra;
} mini_mav_t;

/* X.25 / CRC-16/MCRF4XX, one byte at a time. The table-free form: eight
   bytes of state instead of half a kilobyte of table, and the messages
   here are short enough that it costs nothing worth measuring. */
static void mini_mav_crc(uint16_t* crc, uint8_t data)
{
    uint8_t tmp = (uint8_t)(data ^ (uint8_t)(*crc & 0xFFu));
    tmp         = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
    *crc        = (uint16_t)((uint16_t)(*crc >> 8) ^ (uint16_t)((uint16_t)tmp << 8) ^
                      (uint16_t)((uint16_t)tmp << 3) ^ (uint16_t)(tmp >> 4));
}

static void mini_mav_begin(mini_mav_t* m, uint32_t msgid, uint8_t crc_extra)
{
    m->msgid     = msgid;
    m->crc_extra = crc_extra;
    m->n         = 0;
}

/* Every put goes through here, so one bound check covers the lot. A
   builder that overran would silently drop the tail rather than write
   past the buffer, and none of the builders below can: the longest
   payload is 62 bytes. */
static void mini_mav_u8(mini_mav_t* m, uint8_t v)
{
    if (m->n >= MINI_MAV_MAX_PAYLOAD) { return; }
    m->buf[MINI_MAV_HDR_LEN + m->n] = v;
    m->n++;
}

static void mini_mav_u16(mini_mav_t* m, uint16_t v)
{
    mini_mav_u8(m, (uint8_t)(v & 0xFFu));
    mini_mav_u8(m, (uint8_t)(v >> 8));
}

static void mini_mav_i16(mini_mav_t* m, int16_t v) { mini_mav_u16(m, (uint16_t)v); }

static void mini_mav_u32(mini_mav_t* m, uint32_t v)
{
    mini_mav_u16(m, (uint16_t)(v & 0xFFFFu));
    mini_mav_u16(m, (uint16_t)(v >> 16));
}

static void mini_mav_i32(mini_mav_t* m, int32_t v) { mini_mav_u32(m, (uint32_t)v); }

static void mini_mav_u64(mini_mav_t* m, uint64_t v)
{
    mini_mav_u32(m, (uint32_t)(v & 0xFFFFFFFFu));
    mini_mav_u32(m, (uint32_t)(v >> 32));
}

static void mini_mav_f32(mini_mav_t* m, float v)
{
    /* memcpy, not a pointer cast: type punning through a uint32_t* is
       undefined behaviour and the buffer is unaligned anyway. */
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    mini_mav_u32(m, bits);
}

/* Fixed-size char[n], zero padded, truncated if the string is longer. */
static void mini_mav_chars(mini_mav_t* m, const char* s, size_t n)
{
    size_t i;
    for (i = 0; i < n; ++i)
    {
        const char c = s[i];
        if (c == '\0') { break; } /* stop before reading past the terminator */
        mini_mav_u8(m, (uint8_t)c);
    }
    for (; i < n; ++i) { mini_mav_u8(m, 0u); }
}

/** Close the frame: header, payload truncation, CRC. Returns the total
 *  number of bytes to send, starting at m->buf.
 *
 *  MAVLink 2 drops trailing zero bytes of the payload (at least one byte
 *  always remains) and the receiver zero-fills them again, which is how
 *  a mostly-empty NAMED_VALUE_FLOAT name or a short STATUSTEXT costs
 *  what it says rather than its declared size. */
static size_t mini_mav_end(mini_mav_t* m)
{
    size_t   len = m->n;
    uint16_t crc = 0xFFFFu;
    size_t   i;

    while (len > 1 && m->buf[MINI_MAV_HDR_LEN + len - 1] == 0u) { len--; }

    m->buf[0] = 0xFDu;
    m->buf[1] = (uint8_t)len;
    m->buf[2] = 0u; /* incompat_flags: unsigned frame */
    m->buf[3] = 0u; /* compat_flags */
    m->buf[4] = m->seq;
    m->buf[5] = m->sysid;
    m->buf[6] = m->compid;
    m->buf[7] = (uint8_t)(m->msgid & 0xFFu);
    m->buf[8] = (uint8_t)((m->msgid >> 8) & 0xFFu);
    m->buf[9] = (uint8_t)((m->msgid >> 16) & 0xFFu);
    m->seq++;

    for (i = 1; i < MINI_MAV_HDR_LEN + len; ++i) { mini_mav_crc(&crc, m->buf[i]); }
    mini_mav_crc(&crc, m->crc_extra);
    m->buf[MINI_MAV_HDR_LEN + len]     = (uint8_t)(crc & 0xFFu);
    m->buf[MINI_MAV_HDR_LEN + len + 1] = (uint8_t)(crc >> 8);
    return MINI_MAV_HDR_LEN + len + 2;
}

/* ===========================================================================
 * Message builders. Each fills m->buf and returns the frame length.
 * ===========================================================================
 */

static size_t mini_mav_heartbeat(mini_mav_t* m)
{
    mini_mav_begin(m, MINI_MAV_MSG_HEARTBEAT, MINI_MAV_CRC_HEARTBEAT);
    mini_mav_u32(m, 0u); /* custom_mode */
    mini_mav_u8(m, MINI_MAV_TYPE_GENERIC);
    mini_mav_u8(m, MINI_MAV_AUTOPILOT_GENERIC);
    mini_mav_u8(m, 0u); /* base_mode */
    mini_mav_u8(m, 0u); /* system_status */
    mini_mav_u8(m, MINI_MAV_VERSION);
    return mini_mav_end(m);
}

static size_t mini_mav_attitude(mini_mav_t* m, uint32_t boot_ms, const float rpy_rad[3],
                                const float rate_rps[3])
{
    int i;
    mini_mav_begin(m, MINI_MAV_MSG_ATTITUDE, MINI_MAV_CRC_ATTITUDE);
    mini_mav_u32(m, boot_ms);
    for (i = 0; i < 3; ++i) { mini_mav_f32(m, rpy_rad[i]); }
    for (i = 0; i < 3; ++i) { mini_mav_f32(m, rate_rps[i]); }
    return mini_mav_end(m);
}

static size_t mini_mav_attitude_quaternion(mini_mav_t* m, uint32_t boot_ms, const float q[4],
                                           const float rate_rps[3])
{
    int i;
    mini_mav_begin(m, MINI_MAV_MSG_ATTITUDE_QUAT, MINI_MAV_CRC_ATTITUDE_QUAT);
    mini_mav_u32(m, boot_ms);
    for (i = 0; i < 4; ++i) { mini_mav_f32(m, q[i]); }
    for (i = 0; i < 3; ++i) { mini_mav_f32(m, rate_rps[i]); }
    return mini_mav_end(m);
}

static size_t mini_mav_local_position_ned(mini_mav_t* m, uint32_t boot_ms, const float pos_ned[3],
                                          const float vel_ned[3])
{
    int i;
    mini_mav_begin(m, MINI_MAV_MSG_LOCAL_POSITION_NED, MINI_MAV_CRC_LOCAL_POSITION_NED);
    mini_mav_u32(m, boot_ms);
    for (i = 0; i < 3; ++i) { mini_mav_f32(m, pos_ned[i]); }
    for (i = 0; i < 3; ++i) { mini_mav_f32(m, vel_ned[i]); }
    return mini_mav_end(m);
}

static size_t mini_mav_global_position_int(mini_mav_t* m, uint32_t boot_ms, int32_t lat_1e7,
                                           int32_t lon_1e7, int32_t alt_mm, int32_t rel_alt_mm,
                                           const int16_t vel_cmps[3], uint16_t hdg_cdeg)
{
    int i;
    mini_mav_begin(m, MINI_MAV_MSG_GLOBAL_POSITION_INT, MINI_MAV_CRC_GLOBAL_POSITION_INT);
    mini_mav_u32(m, boot_ms);
    mini_mav_i32(m, lat_1e7);
    mini_mav_i32(m, lon_1e7);
    mini_mav_i32(m, alt_mm);
    mini_mav_i32(m, rel_alt_mm);
    for (i = 0; i < 3; ++i) { mini_mav_i16(m, vel_cmps[i]); }
    mini_mav_u16(m, hdg_cdeg);
    return mini_mav_end(m);
}

/* The RECEIVER's own fix, not the filter's solution: the one message
   that carries a satellite count and a fix type at all, so a ground
   station can tell "no sky" from "the filter is not using the sky". */
static size_t mini_mav_gps_raw_int(mini_mav_t* m, uint64_t t_usec, int32_t lat_1e7, int32_t lon_1e7,
                                   int32_t alt_mm, uint16_t eph_cm, uint16_t epv_cm,
                                   uint16_t vel_cmps, uint16_t cog_cdeg, uint8_t fix_type,
                                   uint8_t sats)
{
    mini_mav_begin(m, MINI_MAV_MSG_GPS_RAW_INT, MINI_MAV_CRC_GPS_RAW_INT);
    mini_mav_u64(m, t_usec);
    mini_mav_i32(m, lat_1e7);
    mini_mav_i32(m, lon_1e7);
    mini_mav_i32(m, alt_mm);
    mini_mav_u16(m, eph_cm);
    mini_mav_u16(m, epv_cm);
    mini_mav_u16(m, vel_cmps);
    mini_mav_u16(m, cog_cdeg);
    mini_mav_u8(m, fix_type);
    mini_mav_u8(m, sats);
    return mini_mav_end(m);
}

/* acc/gyr only: the magnetometer, pressure and temperature fields stay at
   zero and out of fields_updated, which is how HIGHRES_IMU says "this
   sensor is not in this message". */
static size_t mini_mav_highres_imu(mini_mav_t* m, uint64_t t_usec, const float acc_mps2[3],
                                   const float gyr_rps[3], uint16_t fields_updated)
{
    int i;
    mini_mav_begin(m, MINI_MAV_MSG_HIGHRES_IMU, MINI_MAV_CRC_HIGHRES_IMU);
    mini_mav_u64(m, t_usec);
    for (i = 0; i < 3; ++i) { mini_mav_f32(m, acc_mps2[i]); }
    for (i = 0; i < 3; ++i) { mini_mav_f32(m, gyr_rps[i]); }
    for (i = 0; i < 7; ++i) { mini_mav_f32(m, 0.0f); } /* mag, pressures, temp */
    mini_mav_u16(m, fields_updated);
    return mini_mav_end(m);
}

static size_t mini_mav_altitude(mini_mav_t* m, uint64_t t_usec, float monotonic, float amsl,
                                float local, float relative, float terrain, float bottom_clearance)
{
    mini_mav_begin(m, MINI_MAV_MSG_ALTITUDE, MINI_MAV_CRC_ALTITUDE);
    mini_mav_u64(m, t_usec);
    mini_mav_f32(m, monotonic);
    mini_mav_f32(m, amsl);
    mini_mav_f32(m, local);
    mini_mav_f32(m, relative);
    mini_mav_f32(m, terrain);
    mini_mav_f32(m, bottom_clearance);
    return mini_mav_end(m);
}

static size_t mini_mav_ekf_status_report(mini_mav_t* m, uint16_t flags, float vel_var,
                                         float pos_horiz_var, float pos_vert_var,
                                         float compass_var, float terrain_alt_var)
{
    mini_mav_begin(m, MINI_MAV_MSG_EKF_STATUS_REPORT, MINI_MAV_CRC_EKF_STATUS_REPORT);
    mini_mav_f32(m, vel_var);
    mini_mav_f32(m, pos_horiz_var);
    mini_mav_f32(m, pos_vert_var);
    mini_mav_f32(m, compass_var);
    mini_mav_f32(m, terrain_alt_var);
    mini_mav_u16(m, flags);
    return mini_mav_end(m);
}

static size_t mini_mav_named_value_float(mini_mav_t* m, uint32_t boot_ms, const char* name,
                                         float value)
{
    mini_mav_begin(m, MINI_MAV_MSG_NAMED_VALUE_FLOAT, MINI_MAV_CRC_NAMED_VALUE_FLOAT);
    mini_mav_u32(m, boot_ms);
    mini_mav_f32(m, value);
    mini_mav_chars(m, name, MINI_MAV_NAME_LEN);
    return mini_mav_end(m);
}

static size_t mini_mav_statustext(mini_mav_t* m, uint8_t severity, const char* text)
{
    mini_mav_begin(m, MINI_MAV_MSG_STATUSTEXT, MINI_MAV_CRC_STATUSTEXT);
    mini_mav_u8(m, severity);
    mini_mav_chars(m, text, MINI_MAV_TEXT_LEN);
    return mini_mav_end(m);
}

#endif /* MINI_MAVLINK_H */

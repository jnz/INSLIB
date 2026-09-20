/** @file insrcv.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Live data: UBX (custom) UDP stream -> insrcv -> PlotJuggler
 *
 * Binds a UDP port, drives nav_suite and publishes to PlotJuggler
 * (UDP/JSON).
 *
 * Wire format is described in: inslib_protocol.md:
 *   UBX framing (0xB5 0x62, class, id, len16, payload, 8-bit Fletcher).
 *   Custom class 0x40 carries additional input data (IMU, Baro, etc.)
 *
 * A datagram carries whole frames only, but the framer resynchronises
 * and checks the checksum regardless: UDP may drop and reorder, and the
 * frame-boundary rule bounds that damage rather than removing it.
 *
 * The IMU is the epoch clock: each IMU sample begins an epoch,
 * the most recent GNSS/baro/magnetometer/odometry sample since the last
 * epoch is attached, then nav_suite_update() runs.
 *
 * The magnetometer is fused only when mag: enable says so, unlike every
 * other sensor here: it is the one whose raw output is useless until it
 * has been calibrated against the platform it is mounted in
 * (tools/inslib_calib_gui.py writes the mag: keys this reads).
 *
 * --config reads the config.yaml subset shared with python/replay.py
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <signal.h>
#include <time.h> /* gmtime, for the magnetic model's fallback epoch */

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
/* Winsock's recv/send buffer length is an int, POSIX's is a size_t. The
   buffer lengths here are all size_t, so without a type to cast to, one
   of the two platforms gets a conversion the compiler objects to. */
typedef int sock_len_t;
#define SOCK_INVALID INVALID_SOCKET
#else
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
typedef int    sock_t;
typedef size_t sock_len_t;
#define SOCK_INVALID (-1)
#endif

#include "nav_suite.h"
#include "geodetic_toolbox.h"
#include "log.h"
#include "mini_yaml.h"
#include "mini_mavlink.h"

/* print_stats() redraws its status line in place (\r, no \n); anything
   else logged while that line is open must call this first to move off
   it. Forward-declared here since it is called from many places ahead
   of its definition near print_stats(). */
static void stats_line_break(void);

/* ===========================================================================
 * Constants
 * ===========================================================================
 */

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

#define CLASS_CUSTOM  0x40
#define ID_IMU        0x01
#define ID_BARO       0x02
/* 0x03 was the timepulse capture and is retired: the firmware now pairs
   the pulse with its TIM-TP announcement itself and sends 0x40/0x05. */
#define ID_TIMESYNC   0x05
#define ID_MAG        0x06
#define ID_ODOMETRY   0x80

#define CLASS_NAV     0x01
#define ID_NAV_PVT    0x07
#define ID_NAV_COV    0x36

/* Payload sizes of the custom messages (see the file header). */
#define IMU_PAYLOAD_LEN       38 /* 8 + 7*4 + 2 */
#define BARO_PAYLOAD_LEN      16 /* 8 + 2*4 */
#define MAG_PAYLOAD_LEN       24 /* 8 + 4*4 */
#define TIMESYNC_PAYLOAD_LEN  28 /* 8 + 3*4 + 4 + 2*2 */
#define ODOMETRY_PAYLOAD_LEN  30 /* 8 + 8 + 2*4 + 3*2 */
#define NAV_PVT_PAYLOAD_LEN   92
#define NAV_COV_PAYLOAD_LEN   64

/* Time sync flag bits, inslib_protocol.md 0x40/0x05. */
#define TS_FLAG_GPS_VALID   0x0001u
#define TS_FLAG_UTC_BASE    0x0002u
#define TS_FLAG_UTC_AVAIL   0x0004u
#define TS_FLAG_QERR_VALID  0x0008u
#define TS_FLAG_TP_SUSPECT  0x0010u

#define GPS_SEC_PER_WEEK 604800.0

/* Odometry flag bits, inslib_protocol.md 0x40/0x80. */
#define ODO_FLAG_DIR_VALID     0x0001u
#define ODO_FLAG_REVERSE       0x0002u
#define ODO_FLAG_T_DEGRADED    0x0004u
#define ODO_FLAG_DELAY_SUSPECT 0x0008u
#define ODO_FLAG_T_US_VALID    0x0010u

#define UBX_MAX_PAYLOAD 4096
/* One max frame plus one read chunk, so push() never has to grow. */
#define RX_BUF_LEN (UBX_MAX_PAYLOAD + 8 + 65536)
/* A conforming sender keeps a datagram at or below 1400 bytes, but the
   receive buffer is sized for the largest datagram UDP can deliver: a
   short read would silently truncate a frame instead of failing. */
#define READ_CHUNK 65536

#define UDP_PORT_DEFAULT 29800
#define UDP_BIND_DEFAULT "0.0.0.0"
/* Blocking recvfrom would never return on a silent link, so the socket
   is polled with this timeout: long enough not to spin the CPU, short
   enough that the stats line still ticks and Ctrl-C is responsive. */
#define RX_POLL_MS 50

/* An odometry sample older than this at its epoch is dropped rather than
   fused: past that age the hub's host/MCU mapping, not the measurement,
   dominates the error. */
#define ODO_MAX_AGE_MS 2000

/* How far AHEAD of the current epoch a sample may be stamped and still be
   treated as contemporaneous.
   A small negative age is normal, not a fault. The odometry delay is
   measured against the host clock, while the IMU stream reaches the host
   with a transport latency of its own (~100 ms over USB CDC), so the
   effective age at the epoch is the stated delay MINUS that latency and
   lands near zero with jitter on either side. Rejecting everything below
   zero would throw away half of a perfectly good measurement stream.
   A sample stamped whole seconds ahead is a different thing entirely: the
   mapping is wrong, and that still has to be caught. */
#define ODO_MAX_LEAD_MS 500

/* NAV-PVT fixType, u-blox: 0 none, 1 DR only, 2 2D, 3 3D, 4 GNSS+DR,
   5 time only. Two consumers with genuinely different needs read it.

   ins gets a three-dimensional position or nothing: 3 and 4 (both are 3D
   solutions, 4 with dead reckoning fused in) qualify, everything else
   does not. A 2D fix has no height at all, and feeding one as a position
   measurement means inventing the vertical component of a 3D measurement.

   The magnetic model only needs to know WHERE ON EARTH it is, to look up
   declination and the reference field. Those vary on the scale of degrees
   of latitude, so a 2D fix - or one hundreds of metres out - is worth just
   as much as an RTK fix, and refusing it would leave the AHRS on magnetic
   north for no gain. Hence GNSS_MIN_FIX_TYPE_POSITION, deliberately lower.

   Neither is configurable: both decide whether an epoch carries the kind
   of information the consumer needs at all, which is not a tuning
   choice. */
#define GNSS_MIN_FIX_TYPE_3D       3u
#define GNSS_MIN_FIX_TYPE_MAX_3D   4u
#define GNSS_MIN_FIX_TYPE_POSITION 2u

/* How far the platform may travel before the magnetic model is looked up
   again. Declination changes by roughly a degree per 100 km, well below
   the heading accuracy a magnetometer AHRS claims, so a coarse refresh is
   enough and a per-epoch lookup would be waste. */
#define WMM_REFRESH_M 25000.0

#define US_PER_SEC 1000000LL

/* PlotJuggler datagram budget. A full nav tree is ~2.5 kB of JSON; this
   leaves headroom without risking IP fragmentation surprises. */
#define JSON_BUF_LEN 8192

/* MAVLink output (see mav_send_suite). Port 14550 is where every ground
   station looks for a vehicle by default. */
#define MAV_IP_DEFAULT   "127.0.0.1"
#define MAV_PORT_DEFAULT 14550
/* Rate of the NAMED_VALUE_FLOAT sub-filter breakdown. That block is a
   couple of dozen messages per round, so it gets a rate of its own well
   below the standard messages, exactly as the Python sender does. */
#define MAV_SUBFILTER_HZ_DEFAULT 5.0
/* GLOBAL_POSITION_INT's "heading unknown". */
#define MAV_HDG_UNKNOWN 65535u
/* How long after the last GNSS fix ins accepted the MAVLink "gnss_used"
   flag stays at 1. Wide enough that a 1 Hz receiver never blinks it off
   between epochs, short enough that a lost antenna shows up at once. */
#define MAV_GNSS_USED_TIMEOUT_SEC 3.0
/* Below this ground speed a course over ground is noise, so GPS_RAW_INT
   gets the protocol's "unknown" instead. */
#define MAV_GPS_COG_MIN_MPS 0.2f

/* ===========================================================================
 * Decoded samples
 * ===========================================================================
 */

typedef struct
{
    int64_t  t_us;
    float    acc_mps2[3];
    float    gyr_rps[3];
    float    temp_c;
    uint16_t seq;
} imu_sample_t;

typedef struct
{
    int64_t t_us;
    float   pressure_pa;
    float   temp_c;
} baro_sample_t;

typedef struct
{
    int64_t t_us;
    float   mag_ut[3]; /* body frame, the firmware already remapped the axes */
    float   temp_c;    /* diagnostic only, not synchronous with the field */
} mag_sample_t;

typedef struct
{
    int64_t  t_us;
    uint32_t count;
    double   gps_s;   /* continuous GPS seconds, only valid with GPS_VALID */
    uint16_t week;
    uint16_t flags;
} timesync_sample_t;

typedef struct
{
    int64_t  t_us; /* MCU timebase, only valid with T_US_VALID */
    float    speed_mps;
    float    stddev_mps;
    uint16_t delay_ms; /* producer's own age estimate, diagnostics only */
    uint16_t flags;
    uint16_t kind;
} odo_sample_t;

typedef struct
{
    double   lat_deg, lon_deg;
    double   alt_m; /* ellipsoid */
    uint8_t  fix_type;
    uint8_t  num_sv;
    uint8_t  carr_soln; /* NAV-PVT flags bits 6..7: 0 none, 1 RTK float,
                           2 RTK fixed. Diagnostics only -- what the filter
                           acts on is hacc_m/vacc_m, which already carry
                           the effect of an RTK solution. */
    bool     fix_ok; /* NAV-PVT flags bit 0 (gnssFixOK): the receiver's own
                        verdict that this solution is inside its limits */
    float    hacc_m, vacc_m, sacc_mps;
    float    vel_ned[3];
    uint32_t itow_ms;
    uint16_t utc_year;  /* NAV-PVT UTC date, only meaningful with date_ok */
    uint8_t  utc_month;
    bool     date_ok;   /* NAV-PVT valid bit 0 (validDate) */
    bool     has_cov;
    float    cov_pos[6]; /* nn, ne, nd, ee, ed, dd */
    float    cov_vel[6];
} gnss_sample_t;

/* ===========================================================================
 * Little-endian readers (the wire is LE, the host may not be)
 * ===========================================================================
 */

static uint16_t rd_u16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

static uint32_t rd_u32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t* p) { return (int32_t)rd_u32(p); }

static uint64_t rd_u64(const uint8_t* p)
{
    return (uint64_t)rd_u32(p) | ((uint64_t)rd_u32(p + 4) << 32);
}

/* t_us is u64 microseconds since power-up and stays far below 2^63 (2^64
   us is ~584000 years), so the timeline fits an int64 without a range
   check and the arithmetic downstream stays signed. */
static int64_t rd_t_us(const uint8_t* p) { return (int64_t)rd_u64(p); }

static float rd_f32(const uint8_t* p)
{
    /* memcpy, not a pointer cast: the payload is unaligned and type
       punning through a float* is undefined behaviour. */
    const uint32_t bits = rd_u32(p);
    float          out;
    memcpy(&out, &bits, sizeof(out));
    return out;
}

/* ===========================================================================
 * UBX framer
 * ===========================================================================
 */

typedef struct
{
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t bad_length;
    uint64_t resync_bytes;
} framer_stats_t;

typedef struct
{
    uint8_t        buf[RX_BUF_LEN];
    size_t         len;
    framer_stats_t stats;
} framer_t;

static void ubx_checksum(const uint8_t* data, size_t n, uint8_t* ck_a, uint8_t* ck_b)
{
    uint8_t a = 0, b = 0;
    size_t  i;
    for (i = 0; i < n; ++i)
    {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

static void framer_drop_front(framer_t* f, size_t n)
{
    if (n >= f->len)
    {
        f->len = 0;
        return;
    }
    memmove(f->buf, f->buf + n, f->len - n);
    f->len -= n;
}

/* Index of the next 0xB5 0x62 in buf[from..len), or -1. */
static long framer_find_sync(const framer_t* f, size_t from)
{
    size_t i;
    if (f->len < 2) { return -1; }
    for (i = from; i + 1 < f->len; ++i)
    {
        if (f->buf[i] == UBX_SYNC1 && f->buf[i + 1] == UBX_SYNC2) { return (long)i; }
    }
    return -1;
}

typedef enum
{
    FRAME_OK = 0,   /* a complete, checksum-valid frame was extracted */
    FRAME_NEED_MORE, /* not enough bytes buffered yet */
    FRAME_RESYNC     /* dropped bytes to resynchronise; call again */
} frame_result_t;

/* Pull the next frame out of the buffer. Re-synchronises on the next
   sync pair after a bad checksum or an implausible length, so garbage in
   the middle of the stream costs at most the bytes up to the next valid
   frame and never desynchronises the parser permanently. */
static frame_result_t framer_step(framer_t* f, uint8_t* msg_class, uint8_t* msg_id,
                                  const uint8_t** payload, size_t* payload_len)
{
    const long i = framer_find_sync(f, 0);
    if (i < 0)
    {
        /* No sync: drop everything but a trailing 0xB5 that could be the
           first half of a sync pair split across two reads. */
        const size_t keep = (f->len > 0 && f->buf[f->len - 1] == UBX_SYNC1) ? 1u : 0u;
        if (f->len > keep)
        {
            f->stats.resync_bytes += f->len - keep;
            framer_drop_front(f, f->len - keep);
        }
        return FRAME_NEED_MORE;
    }
    if (i > 0)
    {
        f->stats.resync_bytes += (uint64_t)i;
        framer_drop_front(f, (size_t)i);
    }
    if (f->len < 6) { return FRAME_NEED_MORE; }

    const size_t length = rd_u16(f->buf + 4);
    if (length > UBX_MAX_PAYLOAD)
    {
        f->stats.bad_length++;
        framer_drop_front(f, 2); /* skip this sync, hunt for the next */
        return FRAME_RESYNC;
    }
    const size_t total = 6 + length + 2;
    if (f->len < total) { return FRAME_NEED_MORE; }

    uint8_t ck_a, ck_b;
    ubx_checksum(f->buf + 2, total - 4, &ck_a, &ck_b);
    if (ck_a != f->buf[total - 2] || ck_b != f->buf[total - 1])
    {
        f->stats.crc_errors++;
        framer_drop_front(f, 2);
        return FRAME_RESYNC;
    }

    *msg_class   = f->buf[2];
    *msg_id      = f->buf[3];
    *payload     = f->buf + 6;
    *payload_len = length;
    f->stats.frames_ok++;
    /* The caller consumes the payload before the next framer_step(), so
       handing out a pointer into the buffer is safe -- it is only
       invalidated by the framer_drop_front() the caller triggers next. */
    return FRAME_OK;
}

/* ===========================================================================
 * Payload decoders
 * ===========================================================================
 */

static bool decode_imu(const uint8_t* p, size_t n, imu_sample_t* out)
{
    if (n != IMU_PAYLOAD_LEN) { return false; }
    out->t_us = rd_t_us(p);
    int i;
    /* Firmware units are g / dps / degC; convert to SI for the filter. */
    for (i = 0; i < 3; ++i) { out->acc_mps2[i] = rd_f32(p + 8 + 4 * i) * INS_GRAVITY_NOMINAL; }
    for (i = 0; i < 3; ++i) { out->gyr_rps[i] = DEG2RAD(rd_f32(p + 20 + 4 * i)); }
    out->temp_c = rd_f32(p + 32);
    out->seq    = rd_u16(p + 36);
    return true;
}

static bool decode_baro(const uint8_t* p, size_t n, baro_sample_t* out)
{
    if (n != BARO_PAYLOAD_LEN) { return false; }
    out->t_us        = rd_t_us(p);
    out->pressure_pa = rd_f32(p + 8);
    out->temp_c      = rd_f32(p + 12);
    return true;
}

/* Layout: t_us u64, mag xyz f32 [uT], temp f32 (inslib_protocol.md
   0x40/0x06). uT is what the library's magnetic model works in, so unlike
   the IMU message there is no unit conversion here. The axes arrive
   already rotated into the same body frame as 0x40/0x01 by the driver's
   mounting remap: what is left for the host is the soft/hard iron
   calibration in the mag: config section. */
static bool decode_mag(const uint8_t* p, size_t n, mag_sample_t* out)
{
    if (n != MAG_PAYLOAD_LEN) { return false; }
    out->t_us = rd_t_us(p);
    int i;
    for (i = 0; i < 3; ++i) { out->mag_ut[i] = rd_f32(p + 8 + 4 * i); }
    out->temp_c = rd_f32(p + 20);
    return true;
}

/* Layout: t_us u64, count u32, tow_ms u32, tow_sub_ms u32 (2^-32 ms),
   q_err_ps i32, week u16, flags u16 (inslib_protocol.md 0x40/0x05).

   Without GPS_VALID the frame is proof that the pulse is alive and
   nothing more: every time field in it is zero, so gps_s stays invalid
   rather than being computed from zeros and published as GPS week 0. */
static bool decode_timesync(const uint8_t* p, size_t n, timesync_sample_t* out)
{
    if (n != TIMESYNC_PAYLOAD_LEN) { return false; }
    out->t_us  = rd_t_us(p);
    out->count = rd_u32(p + 8);
    out->week  = rd_u16(p + 24);
    out->flags = rd_u16(p + 26);
    out->gps_s = 0.0;
    if (out->flags & TS_FLAG_GPS_VALID)
    {
        const uint32_t tow_ms     = rd_u32(p + 12);
        const uint32_t tow_sub_ms = rd_u32(p + 16);
        const int32_t  q_err_ps   = rd_i32(p + 20);
        out->gps_s = (double)out->week * GPS_SEC_PER_WEEK
                     + ((double)tow_ms + (double)tow_sub_ms / 4294967296.0) * 1e-3;
        if (out->flags & TS_FLAG_QERR_VALID)
        {
            /* The residual the receiver knows it was off by. Sub-
               nanosecond, so it changes nothing here; applied because the
               sign convention belongs in the decoder, not in each user. */
            out->gps_s += (double)q_err_ps * 1e-12;
        }
    }
    return true;
}

/* Layout: t_unix_us u64, t_us u64, speed f32, stddev f32, delay_ms u16,
   flags u16, kind u16 (inslib_protocol.md 0x40/0x80).

   t_unix_us is skipped deliberately. This receiver works purely on the MCU
   clock, and the host timestamp is carried for OFFLINE consumers that want
   to redo the mapping over a whole session. Decoding it here would only
   invite someone to mix two timebases in one epoch.

   t_us is only meaningful once the hub has filled it in, so a frame
   without T_US_VALID is reported as such rather than used. */
static bool decode_odometry(const uint8_t* p, size_t n, odo_sample_t* out)
{
    if (n != ODOMETRY_PAYLOAD_LEN) { return false; }
    out->flags      = rd_u16(p + 26);
    out->speed_mps  = rd_f32(p + 16);
    out->stddev_mps = rd_f32(p + 20);
    out->delay_ms   = rd_u16(p + 24);
    out->kind       = rd_u16(p + 28);
    out->t_us       = (out->flags & ODO_FLAG_T_US_VALID) ? rd_t_us(p + 8) : 0;
    return true;
}

/* NAV-COV's full NED covariance is buffered and attached to the fix of
   the same epoch (matched by iTOW); NAV-COV usually arrives just after
   its NAV-PVT, so the buffered value is at most one epoch old and the
   covariance varies slowly enough for that to be harmless. */
typedef struct
{
    bool     valid;
    uint32_t itow_ms;
    float    cov_pos[6];
    float    cov_vel[6];
} cov_cache_t;

static void decode_nav_cov(const uint8_t* p, size_t n, cov_cache_t* c)
{
    if (n != NAV_COV_PAYLOAD_LEN) { return; }
    if (p[4] == 0 || p[5] == 0 || p[6] == 0) { return; } /* version/pos/vel valid flags */
    int i;
    for (i = 0; i < 6; ++i) { c->cov_pos[i] = rd_f32(p + 16 + 4 * i); }
    for (i = 0; i < 6; ++i) { c->cov_vel[i] = rd_f32(p + 40 + 4 * i); }
    c->itow_ms = rd_u32(p);
    c->valid   = true;
}

static bool decode_nav_pvt(const uint8_t* p, size_t n, const cov_cache_t* c, gnss_sample_t* out)
{
    if (n != NAV_PVT_PAYLOAD_LEN) { return false; }
    memset(out, 0, sizeof(*out));
    /* Every raw field goes through a named variable rather than being
       cast straight out of the reader: -Wbad-function-cast (on in the
       project's warning set) rejects casting a function result to a
       non-matching type, and the intermediate documents the wire unit. */
    const int32_t  lon_1e7 = rd_i32(p + 24);
    const int32_t  lat_1e7 = rd_i32(p + 28);
    const int32_t  height_mm = rd_i32(p + 32);
    const uint32_t hacc_mm = rd_u32(p + 40);
    const uint32_t vacc_mm = rd_u32(p + 44);
    const int32_t  vel_n_mmps = rd_i32(p + 48);
    const int32_t  vel_e_mmps = rd_i32(p + 52);
    const int32_t  vel_d_mmps = rd_i32(p + 56);
    const uint32_t sacc_mmps = rd_u32(p + 68);
    const uint16_t utc_year_raw = rd_u16(p + 4);

    out->itow_ms  = rd_u32(p);
    /* UTC date of this epoch (year u2 at 4, month u1 at 6), gated on the
       valid bitfield at 11: bit 0 is validDate, which the receiver only
       sets once it has decoded the date from the navigation message. It
       is the magnetic model's epoch when nothing better is on offer. */
    out->utc_year  = utc_year_raw;
    out->utc_month = p[6];
    out->date_ok   = (p[11] & 0x01u) != 0u && utc_year_raw >= 2000u &&
                     p[6] >= 1u && p[6] <= 12u;
    out->fix_type  = p[20];
    out->fix_ok    = (p[21] & 0x01u) != 0u;
    out->carr_soln = (uint8_t)((p[21] >> 6) & 0x03u);
    out->num_sv    = p[23];
    out->lon_deg  = (double)lon_1e7 * 1e-7;
    out->lat_deg  = (double)lat_1e7 * 1e-7;
    out->alt_m    = (double)height_mm * 1e-3; /* mm -> m, ellipsoid */
    out->hacc_m   = (float)hacc_mm * 1e-3f;
    out->vacc_m   = (float)vacc_mm * 1e-3f;
    out->vel_ned[0] = (float)vel_n_mmps * 1e-3f;
    out->vel_ned[1] = (float)vel_e_mmps * 1e-3f;
    out->vel_ned[2] = (float)vel_d_mmps * 1e-3f;
    out->sacc_mps   = (float)sacc_mmps * 1e-3f;

    /* iTOW wraps once a week; the difference is compared as a signed
       value so a fix straddling the rollover is not silently dropped. */
    if (c->valid)
    {
        const int32_t dt_ms = (int32_t)(out->itow_ms - c->itow_ms);
        if (dt_ms > -1000 && dt_ms < 1000)
        {
            memcpy(out->cov_pos, c->cov_pos, sizeof(out->cov_pos));
            memcpy(out->cov_vel, c->cov_vel, sizeof(out->cov_vel));
            out->has_cov = true;
        }
    }
    return true;
}

/* (nn, ne, nd, ee, ed, dd) -> symmetric 3x3, column-major. */
static void cov6_to_mat3(const float c6[6], float out[9])
{
    out[0] = c6[0]; out[3] = c6[1]; out[6] = c6[2];
    out[1] = c6[1]; out[4] = c6[3]; out[7] = c6[4];
    out[2] = c6[2]; out[5] = c6[4]; out[8] = c6[5];
}

/* GNSS fix (lat/lon/h) -> the ins filter's own local NED frame, i.e. the
   same origin as ins_get_position_local()/pos_ned, so PlotJuggler can
   overlay the raw fix on top of the estimated position. Same lat/lon/h
   delta + ins_dlatlonh_to_dned approach ins_fuse_gnss() uses internally
   (ins.c) to turn a fix into a position relative to f->origin_llh. */
static bool gnss_llh_to_ins_ned(const ins_t* f, double lat_rad, double lon_rad, double alt_m,
                                float pos_ned[3])
{
    if (!f->is_initialized) return false;
    const double dllh[3] = {lat_rad - f->origin_llh[0], lon_rad - f->origin_llh[1],
                            alt_m - f->origin_llh[2]};
    ins_dlatlonh_to_dned(dllh, lat_rad, alt_m, pos_ned);
    return true;
}

/* ===========================================================================
 * Input source: a bound UDP socket
 *
 * The only input. tools/inslib_hub.py owns the serial device and sends
 * the stream here; tools/inslib_replay_udp.py sends a recorded .ubx file
 * to the same port at the original pace. A session is therefore
 * reproducible without hardware, and this program cannot tell the two
 * apart -- which is exactly what makes the replay worth anything.
 * ===========================================================================
 */

#ifdef _WIN32
static bool net_init(void)
{
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        fprintf(stderr, "[insrcv] WSAStartup failed\n");
        return false;
    }
    return true;
}

static void net_cleanup(void) { WSACleanup(); }

static void sock_close(sock_t s)
{
    if (s != SOCK_INVALID) { closesocket(s); }
}
#else
static bool net_init(void) { return true; }
static void net_cleanup(void) {}

static void sock_close(sock_t s)
{
    if (s != SOCK_INVALID) { close(s); }
}
#endif

typedef struct
{
    sock_t fd;
} source_t;

static bool source_open_udp(source_t* s, const char* bind_addr, int port)
{
    memset(s, 0, sizeof(*s));
    s->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s->fd == SOCK_INVALID)
    {
        fprintf(stderr, "[insrcv] cannot create UDP socket\n");
        return false;
    }

    /* Several receivers on one host (this one plus, say, a monitor) are a
       normal setup, and a socket left in TIME_WAIT must not block a
       restart between two runs. */
    int on = 1;
    (void)setsockopt(s->fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((unsigned short)port);
    if (inet_pton(AF_INET, bind_addr, &addr.sin_addr) != 1)
    {
        fprintf(stderr, "[insrcv] bad bind address '%s'\n", bind_addr);
        sock_close(s->fd);
        s->fd = SOCK_INVALID;
        return false;
    }
    if (bind(s->fd, (const struct sockaddr*)&addr, sizeof(addr)) != 0)
    {
        fprintf(stderr, "[insrcv] cannot bind %s:%d (already in use?)\n", bind_addr, port);
        sock_close(s->fd);
        s->fd = SOCK_INVALID;
        return false;
    }

    return true;
}

/* > 0: bytes of one datagram. 0: nothing within the poll window. < 0:
   the socket failed.

   Polled with select() rather than read blockingly: on a silent link the
   loop still has to print its stats line and notice Ctrl-C.

   One datagram per call, never a concatenation of several: a datagram
   holds whole frames (inslib_protocol.md), so handing them to the framer
   one datagram at a time keeps a lost datagram from merging the frames
   around it into one bogus length. */
static long source_read(source_t* s, uint8_t* buf, size_t n)
{
    fd_set         rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(s->fd, &rfds);
    tv.tv_sec  = RX_POLL_MS / 1000;
    tv.tv_usec = (RX_POLL_MS % 1000) * 1000;

    /* The first argument is ignored on Windows and must be highest fd + 1
       on POSIX, where sock_t is that fd. */
    const int ready = select((int)s->fd + 1, &rfds, NULL, NULL, &tv);
    if (ready < 0)
    {
#ifndef _WIN32
        if (errno == EINTR) { return 0; } /* Ctrl-C: let the loop re-check */
#endif
        return -1;
    }
    if (ready == 0) { return 0; }

    const long got = (long)recvfrom(s->fd, (char*)buf, (sock_len_t)n, 0, NULL, NULL);
    if (got < 0)
    {
#ifdef _WIN32
        /* An ICMP port-unreachable from an earlier send can surface here
           as a receive error on Windows. It says nothing about this
           socket's ability to receive, so it must not end the session. */
        if (WSAGetLastError() == WSAECONNRESET) { return 0; }
#else
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) { return 0; }
#endif
        return -1;
    }
    return got;
}

static void source_close(source_t* s)
{
    sock_close(s->fd);
    s->fd = SOCK_INVALID;
}

/* ===========================================================================
 * Monotonic host clock (for publish throttling and the stats line)
 * ===========================================================================
 */

static double host_now_sec(void)
{
#ifdef _WIN32
    static LARGE_INTEGER freq;
    LARGE_INTEGER        now;
    if (freq.QuadPart == 0) { QueryPerformanceFrequency(&freq); }
    QueryPerformanceCounter(&now);
    return (double)now.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

/* ===========================================================================
 * PlotJuggler sender (UDP/JSON) + a bounded JSON writer
 * ===========================================================================
 */

typedef struct
{
    char   buf[JSON_BUF_LEN];
    size_t len;
    bool   overflow;
    bool   need_comma;
} json_t;

static void j_putc(json_t* j, char c)
{
    if (j->len + 1 >= sizeof(j->buf))
    {
        j->overflow = true;
        return;
    }
    j->buf[j->len++] = c;
}

static void j_puts(json_t* j, const char* s)
{
    while (*s) { j_putc(j, *s++); }
}

static void j_begin(json_t* j)
{
    j->len        = 0;
    j->overflow   = false;
    j->need_comma = false;
    j_putc(j, '{');
}

static void j_key(json_t* j, const char* k)
{
    if (j->need_comma) { j_putc(j, ','); }
    j_putc(j, '"');
    j_puts(j, k);
    j_puts(j, "\":");
    j->need_comma = true;
}

static void j_obj_begin(json_t* j, const char* k)
{
    j_key(j, k);
    j_putc(j, '{');
    j->need_comma = false;
}

static void j_obj_end(json_t* j)
{
    j_putc(j, '}');
    j->need_comma = true;
}

/* PlotJuggler rejects a datagram containing NaN/Inf, so a non-finite
   value is omitted entirely -- exactly what the Python sender's
   sanitize_for_json() does. A dropped key simply leaves a gap in the
   series instead of killing the whole message. */
static void j_num(json_t* j, const char* k, double v)
{
    /* Range comparison instead of isfinite(): MinGW's isfinite macro
       narrows its argument, which trips -Wfloat-conversion. NaN fails
       both comparisons and infinities fail one, so this rejects exactly
       the non-finite values. */
    if (!(v >= -DBL_MAX && v <= DBL_MAX)) { return; }
    char tmp[40];
    snprintf(tmp, sizeof(tmp), "%.9g", v);
    j_key(j, k);
    j_puts(j, tmp);
}

static void j_vec3(json_t* j, const char* key, const char* n0, const char* n1, const char* n2,
                   double v0, double v1, double v2)
{
    j_obj_begin(j, key);
    j_num(j, n0, v0);
    j_num(j, n1, v1);
    j_num(j, n2, v2);
    j_obj_end(j);
}

static void j_ned(json_t* j, const char* key, const float v[3])
{
    j_vec3(j, key, "n", "e", "d", v[0], v[1], v[2]);
}

static void j_xyz(json_t* j, const char* key, const float v[3])
{
    j_vec3(j, key, "x", "y", "z", v[0], v[1], v[2]);
}

static void j_rpy_deg(json_t* j, const char* key, const float rpy_rad[3])
{
    j_vec3(j, key, "roll", "pitch", "yaw", RAD2DEG(rpy_rad[0]),
           RAD2DEG(rpy_rad[1]), RAD2DEG(rpy_rad[2]));
}

typedef struct
{
    sock_t             fd;
    struct sockaddr_in dst;
    bool               enabled;
    double             t0;
} pj_sender_t;

/* net_init() has already run in main(): the receive socket needs Winsock
   too, so starting it here would leave --plotjuggler-off without it. */
static bool pj_open(pj_sender_t* p, const char* ip, int port)
{
    memset(p, 0, sizeof(*p));
    p->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (p->fd == SOCK_INVALID)
    {
        fprintf(stderr, "[insrcv] cannot create UDP socket\n");
        return false;
    }
    p->dst.sin_family = AF_INET;
    p->dst.sin_port   = htons((unsigned short)port);
    if (inet_pton(AF_INET, ip, &p->dst.sin_addr) != 1)
    {
        fprintf(stderr, "[insrcv] bad PlotJuggler address '%s'\n", ip);
        return false;
    }
    p->enabled = true;
    p->t0      = host_now_sec();
    return true;
}

/* Close the tree with the two fields PlotJuggler uses as its time axis
   (same names and meaning as the Python sender) and send it. */
static void pj_send(pj_sender_t* p, json_t* j)
{
    if (!p->enabled) { return; }
    j_num(j, "t_sec", host_now_sec() - p->t0);
    j_putc(j, '}');
    if (j->overflow)
    {
        /* Truncated JSON is not JSON: drop it rather than send a
           datagram PlotJuggler will reject anyway. */
        fprintf(stderr, "[insrcv] JSON buffer overflow, datagram dropped\n");
        return;
    }
    (void)sendto(p->fd, j->buf, (sock_len_t)j->len, 0, (const struct sockaddr*)&p->dst,
                 sizeof(p->dst));
}

static void pj_close(pj_sender_t* p)
{
    sock_close(p->fd);
    p->fd      = SOCK_INVALID;
    p->enabled = false;
}


/* ===========================================================================
 * Filter-state readers shared with the Python binding
 * ===========================================================================
 */

/* Marginal variance of every state from its UDU factors (U unit upper
 * triangular, column-major, leading dimension n): diag(P)_i =
 * sum_{k=i}^{n-1} U[i,k]^2 * d[k]. Same helper (and same leading-dimension
 * convention) as python/csrc/ins_capi.c's diag_covariance_of, so the C and
 * Python receivers report identical 1-sigma values. */
static void udu_diag(const float* U, const float* d, int n, float* diag_out)
{
    int i, k;
    for (i = 0; i < n; ++i)
    {
        float s = 0.0f;
        for (k = i; k < n; ++k) { s += U[i + k * n] * U[i + k * n] * d[k]; }
        diag_out[i] = s;
    }
}

/* 1-sigma of ins's error state, grouped like the Python stddev() dict:
   pos_ned(3) vel_ned(3) rpy(3) acc_bias(3) gyr_bias(3). */
static bool ins_stddev_of(const ins_t* f, float sd[15])
{
    if (!f->is_initialized) { return false; }
    float diag[INS_UNKNOWNS_MAX];
    udu_diag(f->U, f->d, f->n, diag);
    int i;
    for (i = 0; i < 15; ++i) { sd[i] = sqrtf(diag[i] > 0.0f ? diag[i] : 0.0f); }
    return true;
}

/* 1-sigma gyro bias uncertainty (the last 3 states of the 5/6-state
   error vector, see ahrs.h). */
static bool ahrs_gyr_bias_stddev_of(const ahrs_t* a, float o[3])
{
    if (!a->is_initialized) { return false; }
    float diag[AHRS_UNKNOWNS_MAX];
    udu_diag(a->U, a->d, a->n, diag);
    const int off = a->n - 3;
    int       i;
    for (i = 0; i < 3; ++i) { o[i] = sqrtf(diag[off + i] > 0.0f ? diag[off + i] : 0.0f); }
    return true;
}

/* 1-sigma [h_m, v_mps, acc_bias_mps2] of baro_alt's own 3-state error
   covariance. */
static bool baro_stddev_of(const baro_alt_t* b, float o[3])
{
    if (!b->is_initialized) { return false; }
    float diag[BARO_ALT_STATES];
    udu_diag(b->U, b->d, BARO_ALT_STATES, diag);
    int i;
    for (i = 0; i < 3; ++i) { o[i] = sqrtf(diag[i] > 0.0f ? diag[i] : 0.0f); }
    return true;
}

/* ===========================================================================
 * Status / "why is the 3D filter not running?"
 *
 * Mirrors INSLIB/telemetry.py's suite_status(): same codes, same texts,
 * same derivation from ins's own diagnostic counters. A fix that never
 * arrived, one rejected by the accuracy gate and one that arrived before
 * the WGS84 anchor existed are three different failures with three
 * different remedies, so they get three different codes.
 * ===========================================================================
 */

typedef enum
{
    BLOCKED_OK = 0,          /* ins ready, aiding fresh (mode FULL) */
    BLOCKED_COASTING,        /* ins ready but dead-reckoning */
    BLOCKED_WARMUP,          /* aiding accepted, not converged yet */
    BLOCKED_NO_AIDING,       /* no absolute-position measurement ever arrived */
    BLOCKED_RECEIVER_NO_FIX, /* NAV-PVT arrives, carrying no solution */
    BLOCKED_AIDING_REJECTED, /* every fix failed the accuracy gate */
    BLOCKED_NO_ANCHOR        /* fixes arrived with no WGS84 anchor for them */
} blocked_t;

static const char* blocked_text(blocked_t b)
{
    switch (b)
    {
        case BLOCKED_OK: return "3D filter running (FULL)";
        case BLOCKED_COASTING: return "3D filter coasting: no fresh position aiding";
        case BLOCKED_WARMUP: return "3D filter converging: aiding accepted, not ready yet";
        case BLOCKED_NO_AIDING: return "3D filter off: no GNSS/position measurements at all";
        case BLOCKED_RECEIVER_NO_FIX:
            return "3D filter off: the receiver has no 3D fix (NAV-PVT arrives without one)";
        case BLOCKED_AIDING_REJECTED:
            return "3D filter off: all GNSS fixes rejected (accuracy gate)";
        case BLOCKED_NO_ANCHOR: return "3D filter off: GNSS seen but no WGS84 anchor";
        default: return "unknown";
    }
}

/* nofix: NAV-PVT epochs the receiver itself marked as carrying no
   solution, so they never reached the filter. Told apart from "nothing
   arrived at all" because the two need opposite reactions - check the
   antenna, versus go outside. */
static blocked_t suite_blocked(const nav_suite_t* s, uint64_t nofix)
{
    const ins_diag_t* d = ins_get_diag(&s->ins);
    if (ins_is_ready(&s->ins))
    {
        return (nav_suite_get_mode(s) == NAV_SUITE_MODE_FULL) ? BLOCKED_OK : BLOCKED_COASTING;
    }
    if (d->n_gnss_used > 0) { return BLOCKED_WARMUP; }
    if (d->n_gnss_no_anchor > 0) { return BLOCKED_NO_ANCHOR; }
    if (d->n_gnss_rejected_noise > 0) { return BLOCKED_AIDING_REJECTED; }
    if (d->n_gnss_seen == 0)
    {
        return (nofix > 0) ? BLOCKED_RECEIVER_NO_FIX : BLOCKED_NO_AIDING;
    }
    /* Seen, neither used nor rejected: still queued this epoch. */
    return BLOCKED_WARMUP;
}

/* ===========================================================================
 * MAVLink sender (UDP, MAVLink 2)
 *
 * The C twin of INSLIB/telemetry.py's MavlinkSender: same messages, same
 * per-message rates, same NAMED_VALUE_FLOAT names, so a ground station or
 * a log analyser cannot tell a live session from a replay of one.
 * The framing lives in mini_mavlink.h.
 *
 * udpout, not a bound port: nothing here listens, and a filter that
 * accepts no commands should not pretend to. A ground station that wants
 * to see it points itself at this port.
 * ===========================================================================
 */

/* One deadline per message group. Values mirror MavlinkSender.intervals.
   Every group is capped independently of the others because their costs
   differ by an order of magnitude: ATTITUDE is one small datagram,
   the sub-filter block is two dozen. */
typedef enum
{
    MAV_RATE_HEARTBEAT = 0,
    MAV_RATE_LOCAL_POS,
    MAV_RATE_GLOBAL_POS,
    MAV_RATE_GPS_RAW,
    MAV_RATE_ALTITUDE,
    MAV_RATE_ATTITUDE,
    MAV_RATE_IMU,
    MAV_RATE_EKF_STATUS,
    MAV_RATE_SUBFILTER,
    MAV_RATE_COUNT
} mav_rate_t;

typedef struct
{
    sock_t             fd;
    struct sockaddr_in dst;
    bool               enabled;
    double             t0;
    mini_mav_t         m;
    double             interval[MAV_RATE_COUNT];
    double             next[MAV_RATE_COUNT];
    blocked_t          last_blocked;
    bool               have_last_blocked;
    /* "is the filter fusing GNSS right now?", derived from ins's own
       accepted-fix counter: the count itself says a fix was accepted at
       some point, the timestamp says how long ago. */
    uint32_t           gnss_used_count;
    double             gnss_used_time;
    bool               have_gnss_used;
} mav_sender_t;

/* net_init() has already run in main(), same as for pj_open(). */
static bool mav_open(mav_sender_t* p, const char* ip, int port, double subfilter_hz)
{
    int i;
    memset(p, 0, sizeof(*p));
    p->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (p->fd == SOCK_INVALID)
    {
        fprintf(stderr, "[insrcv] cannot create UDP socket\n");
        return false;
    }
    p->dst.sin_family = AF_INET;
    p->dst.sin_port   = htons((unsigned short)port);
    if (inet_pton(AF_INET, ip, &p->dst.sin_addr) != 1)
    {
        fprintf(stderr, "[insrcv] bad MAVLink address '%s'\n", ip);
        return false;
    }
    /* System 1, component 1: one vehicle, its autopilot. A GCS keys its
       whole display off this pair, so it has to be a plausible one. */
    p->m.sysid  = 1;
    p->m.compid = 1;
    p->enabled  = true;
    p->t0       = host_now_sec();

    if (!(subfilter_hz > 0.0)) { subfilter_hz = MAV_SUBFILTER_HZ_DEFAULT; }
    p->interval[MAV_RATE_HEARTBEAT]  = 1.0;
    p->interval[MAV_RATE_LOCAL_POS]  = 1.0 / 50.0;
    p->interval[MAV_RATE_GLOBAL_POS] = 1.0 / 20.0;
    p->interval[MAV_RATE_GPS_RAW]    = 1.0 / 5.0;
    p->interval[MAV_RATE_ALTITUDE]   = 1.0 / 20.0;
    p->interval[MAV_RATE_ATTITUDE]   = 1.0 / 50.0;
    p->interval[MAV_RATE_IMU]        = 1.0 / 50.0;
    p->interval[MAV_RATE_EKF_STATUS] = 1.0 / 10.0;
    p->interval[MAV_RATE_SUBFILTER]  = 1.0 / subfilter_hz;
    for (i = 0; i < MAV_RATE_COUNT; ++i) { p->next[i] = 0.0; }
    return true;
}

static bool mav_due(mav_sender_t* p, mav_rate_t k, double now)
{
    if (now < p->next[k]) { return false; }
    p->next[k] = now + p->interval[k];
    return true;
}

static void mav_emit(mav_sender_t* p, size_t n)
{
    (void)sendto(p->fd, (const char*)p->m.buf, (sock_len_t)n, 0, (const struct sockaddr*)&p->dst,
                 sizeof(p->dst));
}

static void mav_close(mav_sender_t* p)
{
    sock_close(p->fd);
    p->fd      = SOCK_INVALID;
    p->enabled = false;
}

/* MAVLink has no "unknown" in a rate or a velocity field, so a
   non-finite one goes out as 0 (telemetry.py's _f()). Fields where NaN
   IS the protocol's own "unknown", ALTITUDE's terrain and bottom
   clearance, keep it. */
static float mav_f(float v) { return isfinite(v) ? v : 0.0f; }

/* [m/s] -> [cm/s] as int16, the unit GLOBAL_POSITION_INT wants. Out of
   range or non-finite reads as 0: the alternative is an implementation
   defined conversion, and a wrapped velocity is worse than no velocity. */
static int16_t mav_cmps(float v)
{
    const float c = v * 100.0f;
    if (!(c > -32768.0f && c < 32767.0f)) { return 0; }
    return (int16_t)c;
}

/* [m] -> [mm] as int32, same reasoning as mav_cmps. */
static int32_t mav_mm(double v)
{
    const double mm = v * 1000.0;
    if (!(mm > -2147483000.0 && mm < 2147483000.0)) { return 0; }
    return (int32_t)mm;
}

/* [m] -> [cm] as uint16 for GPS_RAW_INT's eph/epv. Saturates to the
   protocol's own "unknown" rather than wrapping, which is also what the
   receiver's 4294967 m hAcc ("no idea") turns into. */
static uint16_t mav_acc_cm(float m)
{
    const float cm = m * 100.0f;
    if (!(cm >= 0.0f) || cm >= 65535.0f) { return MINI_MAV_GPS_U16_UNKNOWN; }
    return (uint16_t)cm;
}

/* Horizontal ground speed [cm/s], same saturation. */
static uint16_t mav_gps_speed_cmps(const float vel_ned[3])
{
    const float cmps = sqrtf(vel_ned[0] * vel_ned[0] + vel_ned[1] * vel_ned[1]) * 100.0f;
    if (!(cmps >= 0.0f) || cmps >= 65535.0f) { return MINI_MAV_GPS_U16_UNKNOWN; }
    return (uint16_t)cmps;
}

/* Course over ground [cdeg], 0..35999. */
static uint16_t mav_gps_cog_cdeg(const float vel_ned[3])
{
    if (!isfinite(vel_ned[0]) || !isfinite(vel_ned[1])) { return MINI_MAV_GPS_U16_UNKNOWN; }
    const float speed2 = vel_ned[0] * vel_ned[0] + vel_ned[1] * vel_ned[1];
    if (speed2 < MAV_GPS_COG_MIN_MPS * MAV_GPS_COG_MIN_MPS)
    {
        return MINI_MAV_GPS_U16_UNKNOWN;
    }
    float deg = (float)RAD2DEG(atan2f(vel_ned[1], vel_ned[0]));
    if (deg < 0.0f) { deg += 360.0f; }
    const uint16_t cdeg = (uint16_t)(deg * 100.0f);
    return (cdeg > 35999u) ? 0u : cdeg;
}

/* u-blox NAV-PVT fixType, gnssFixOK and carrSoln -> MAV_GPS_FIX_TYPE.
   An epoch the receiver itself marks as outside its limits goes out as
   NO_FIX: it carries a position, but not one anything should act on,
   and it is the same verdict the filter's own gate reaches. fixType 1
   (dead reckoning only) and 5 (time only) have no MAVLink spelling. */
static uint8_t mav_gps_fix_type(const gnss_sample_t* s)
{
    if (!s->fix_ok) { return MINI_MAV_GPS_FIX_NO_FIX; }
    switch (s->fix_type)
    {
        case 2u: return MINI_MAV_GPS_FIX_2D;
        case 3u: /* 3D */
        case 4u: /* GNSS + dead reckoning combined */
            if (s->carr_soln == 1u) { return MINI_MAV_GPS_FIX_RTK_FLOAT; }
            if (s->carr_soln == 2u) { return MINI_MAV_GPS_FIX_RTK_FIXED; }
            return MINI_MAV_GPS_FIX_3D;
        default: return MINI_MAV_GPS_FIX_NO_FIX;
    }
}

static void mav_nv(mav_sender_t* p, uint32_t boot_ms, const char* name, float value)
{
    if (!isfinite(value)) { return; }
    mav_emit(p, mini_mav_named_value_float(&p->m, boot_ms, name, value));
}

static void mav_statustext(mav_sender_t* p, uint8_t severity, const char* text)
{
    if (!p->enabled) { return; }
    mav_emit(p, mini_mav_statustext(&p->m, severity, text));
}

/* ===========================================================================
 * Receiver state
 * ===========================================================================
 */

typedef struct
{
    /* options */
    const char* udp_bind;
    int         udp_port;
    const char* pj_ip;
    int         pj_port;
    bool        pj_on;
    const char* mav_ip;
    int         mav_port;
    bool        mav_on;
    double      mav_subfilter_hz;
    bool        nav_on;
    bool        auto_zaru;
    bool        raw_overlay;
    double      pub_hz;
    double      raw_imu_hz;
    double      stats_sec;
    int         gnss_delay_ms;

    /* filter tuning */
    float acc_psd, gyr_psd;
    float acc_bias_rw, gyr_bias_rw;
    /* Known initial attitude (config section init_hint, same keys as
       tools/replay.c): a static "I just know it" assertion for ins's
       auto-init bootstrap. The yaw half is the one that earns its keep on
       a board with no magnetometer: nothing else can supply a heading at
       standstill, so without it the bootstrap starts at "unknown"
       (180 deg) and the platform has to move before GNSS makes yaw
       observable. Both halves are independent, each disarmed by a
       non-positive stddev. */
    float init_hint_roll_deg;
    float init_hint_pitch_deg;
    float init_hint_rpy_stddev_deg;
    float init_hint_yaw_deg;
    float init_hint_yaw_stddev_deg;
    /* Initial state uncertainty (config section init_stddev, same keys and
       same units as tools/replay.c - the "rad_deg" names are historical,
       both are degrees). Each 0 -> the value this receiver would have used
       anyway. */
    float init_pos_stddev_m;
    float init_vel_stddev_mps;
    float init_rpy_stddev_deg;
    float init_yaw_stddev_deg;
    float init_acc_bias_stddev_mps2;
    float init_gyr_bias_stddev_deg;
    /* Extra process-noise densities [X/sqrt(s)], 0 -> ins's own default. */
    float pos_pred_stddev_m_sqrts;
    float vel_pred_stddev_mps_sqrts;
    float rpy_pred_stddev_rad_sqrts;
    float baro_stddev_m;
    float baro_acc_bias_init_stddev_mps2; /* 0 -> baro_alt's own default */
    float leverarm_frd[3];
    float pos_fallback_m, vel_fallback_mps;
    float gate_hpos_m, gate_vpos_m, gate_hvel_mps, gate_vvel_mps;
    /* Solution-mode gates (REQ-NAV-051/052), each 0 -> ins's own default. */
    float start_hpos_m, start_vpos_m, start_hvel_mps, start_vvel_mps;
    float stop_hpos_m, stop_vpos_m, stop_hvel_mps, stop_vvel_mps;
    /* GNSS covariance conditioning (same keys as tools/replay.c): scale
       what the receiver reports, and floor it. A receiver's covariance is
       its own optimism, and both directions of correcting it belong to
       the dataset rather than to the library. Each 0 -> no change. */
    float pos_cov_scale, pos_cov_scale_height, vel_cov_scale;
    float pos_stddev_floor_hor_m, pos_stddev_floor_ver_m;
    float vel_stddev_floor_hor_mps, vel_stddev_floor_ver_mps;
    float init_dwell_sec, stop_dwell_sec;
    bool  init_dwell_disable, stop_disable;
    /* GNSS position decimation (REQ-NAV-063): fuse the position on every
       Nth epoch that offers a usable position AND velocity, the velocity
       alone on the other N-1. <= 1 -> off, 0 -> ins's own default. */
    int   pos_decimation;
    /* false -> the receiver is still decoded, counted and published, but
       nothing derived from it reaches the filter. For a run that is
       deliberately GNSS-free (an indoor inertial test, a repeatability
       check) this is one key instead of gates set to reject everything. */
    bool  gnss_enable;
    /* Same idea for the barometer: the samples keep being decoded, counted
       and published, they just stop reaching the filter -- one key instead
       of unplugging the sensor, so a run stays comparable against the same
       recording processed WITH the vertical channel.
       Default 1 here, unlike the replay harnesses, where the key gates
       READING baro.csv and therefore defaults to 0. A live receiver has no
       such file to open: the samples arrive over the wire whether or not
       anyone asked for them, and every session that ever ran fused them.
       Defaulting to 0 would silently take the vertical channel away from
       every existing setup whose config predates this key. */
    bool  baro_enable;
    /* Magnetometer (0x40/0x06). Same reasoning as baro_enable, and the
       same default: a sensor that is wired up is fused unless the config
       says otherwise, and an absent one costs nothing (no samples, no
       fusion). Off, the samples are still decoded, counted and published
       like every other sensor the filter is not using.

       The risk this default accepts is an UNCALIBRATED magnetometer:
       hard iron is a bias, so no amount of measurement noise removes the
       heading error it causes, only a calibration does
       (tools/inslib_calib_gui.py, mag.fixed_bias / mag.misalignment).
       What makes it acceptable is that ins does not fail silently there
       -- with a position known, the field-strength gate compares |B|
       against the WMM total field, downweights samples outside the
       tolerance band and warns about a persistent deviation, which is
       what an uncorrected hard iron of any relevant size looks like.
       Note the offline harness (tools/replay.c) keeps mag.enable at 0,
       because there the key does something else entirely: it opens
       mag.csv, and a dataset without one must not fail to replay. */
    bool  mag_enable;
    float mag_stddev_ut;       /* 0 -> a default */
    int   mag_min_delay_ms;    /* fusion rate limit, 0 -> ins default,
                                  negative -> fuse every sample */
    bool  mag_estimate_bias;   /* 18-state hard-iron estimation */
    float mag_misalignment[9]; /* soft iron + alignment, col-major, 0 -> I */
    float mag_fixed_bias[3];   /* hard iron [uT] */
    /* Automotive mode (REQ-NAV-034): the GNSS course over ground doubles as
       a yaw measurement, which only holds on a platform that cannot move
       sideways. Same three top-level keys as tools/replay.c and
       python/replay.py; each tuning value 0 -> ins's own default. */
    bool  automotive_mode;
    float automotive_min_speed_mps;
    float automotive_min_yaw_stddev_deg;
    bool  automotive_lateral_constraint;
    float automotive_lateral_stddev_mps;
    float automotive_lateral_max_yaw_rate_deg;
    float automotive_lateral_after_sec;
    float max_dr_sec;
    bool  allow_unlimited_dr;
    /* Free-inertial start (config section free_inertial_start), see
       fi_start_position() for what it does and what it costs. */
    bool     fi_enable;
    bool     fi_have_lat, fi_have_lon;
    double   fi_lat_deg, fi_lon_deg, fi_height_m;
    float    fi_stddev_m;
    bool     fi_armed;      /* still offering the declared position */
    int64_t  fi_next_t_us;  /* MCU time of the next offer */
    uint64_t n_gnss_nofix;  /* NAV-PVT epochs without a 3D solution */
    uint64_t n_gnss_off;    /* fixes dropped because gnss.enable is 0 */
    uint64_t n_baro_off;    /* samples dropped because baro.enable is 0 */
    uint64_t n_mag_off;     /* samples dropped because mag.enable is 0 */
    /* World Magnetic Model anchor (wmm_update). */
    float    wmm_year;      /* mag.wmm_year; 0 -> derive from the stream */
    double   wmm_gps_s;     /* newest GPS time from 0x40/0x05, 0 -> none */
    float    wmm_pvt_year;  /* decimal year from NAV-PVT's UTC date, 0 -> none */
    bool     wmm_set;
    int      wmm_rank;      /* epoch_rank_t the applied model was built with */
    double   wmm_lat_deg, wmm_lon_deg;
    uint64_t n_fi;          /* declarations offered */
    uint64_t n_fi_worse;    /* fixes ignored as wider than the declaration */
    bool  chi2_disable; /* REQ-SYS-015: never chi2-downweight any fusion (diagnostics only) */
    /* Auto-ZUPT/ZARU pseudo-measurement stddev (ins_init_t's
       zero_vel_stddev_mps/zero_rot_stddev_rps) plus the one stillness
       parameter set the whole suite shares (ins_options_t's auto_zupt_*,
       propagated to the ARS/AHRS and baro_alt by nav_suite_init,
       REQ-SUITE-020). All 0 -> the library's own default. */
    float zero_vel_stddev_mps;
    float zero_rot_stddev_deg;
    float auto_zupt_static_gyr_deg;
    float auto_zupt_static_acc_mps2;
    float auto_zupt_max_vel_mps;
    float auto_zupt_max_vel_stddev_mps;
    float auto_zupt_static_gyr_stddev_deg;
    float auto_zupt_static_acc_stddev_mps2;
    float auto_zupt_dwell_sec;
    float auto_zupt_min_interval_sec;
    /* Fixed IMU calibration (REQ-NAV-037), from --config: corrected =
       M * (raw - fixed_bias), M column-major 3x3, all-zero -> identity. */
    float acc_misalignment[9];
    float gyr_misalignment[9];
    float acc_fixed_bias[3]; /* [m/s^2] */
    float gyr_fixed_bias[3]; /* [rad/s] */

    /* runtime */
    nav_suite_t   suite;
    bool          suite_ready;
    framer_t      framer;
    cov_cache_t   cov;
    pj_sender_t   pj;
    mav_sender_t  mav;
    json_t        json;

    int64_t       last_imu_t_us;
    bool          have_last_imu;
    gnss_sample_t pending_gnss;
    bool          have_pending_gnss;
    baro_sample_t pending_baro;
    bool          have_pending_baro;
    mag_sample_t  pending_mag;
    bool          have_pending_mag;
    odo_sample_t  pending_odo;
    bool          have_pending_odo;

    double next_pub_sec;
    double next_raw_sec;
    blocked_t last_blocked;
    bool      have_last_blocked;

    /* stats */
    int           n_config_ignored; /* config.yaml keys insrcv does not use */
    uint64_t      n_imu, n_baro, n_mag, n_pps, n_pps_gps, n_gnss_ubx, n_gnss_pvt, n_gnss_cov;
    uint64_t      n_odo, n_odo_stale, n_odo_lead, n_odo_reverse, n_odo_unstamped;
    uint64_t      n_datagrams;
    uint64_t      n_bad_len, n_custom_unknown;
    odo_sample_t  last_odo;
    bool          have_last_odo;
    uint64_t      imu_drops;
    uint16_t      last_seq;
    bool          have_last_seq;
    imu_sample_t  last_imu;
    baro_sample_t last_baro;
    mag_sample_t  last_mag;
    bool          have_last_mag;
    gnss_sample_t last_gnss;
    bool          have_last_gnss;
    int64_t       imu_win_t0;
    uint64_t      imu_win_n;
    double        imu_hz;

    /* The counters as of the previous status line, so the line can show
       what is FLOWING right now. A total that only grows answers "did
       anything ever arrive", which is not the question being asked once a
       second; the totals are printed once at the end instead. */
    struct
    {
        double   t_sec;
        uint64_t n_imu, n_baro, n_gnss_pvt, n_mag, n_odo;
    } stats_prev;
} insrcv_t;

/* ===========================================================================
 * --config: the config.yaml subset shared with python/replay.py
 * --config and python/replay.py
 *
 * Same minimal YAML-subset reader as tools/replay.c (shared, mini_yaml.h):
 * two levels ("section:" + 2-space-indented "key: value"), scalars,
 * inline float lists "[a, b, c]" and '#' comments -- exactly what the
 * project's config writers emit (including python/insrcv_imu_calib.py).
 * Only the keys this receiver actually uses are consumed; unknown keys
 * are ignored (the same file also drives replay.py). Values are written
 * straight into insrcv_t, so a config key and its command-line twin
 * compose by position: whichever comes later on the command line wins.
 * ===========================================================================
 */

/* Keys of the shared schema that describe the REPLAY HARNESS itself
   rather than the filter: which files to read, what the dataset is
   called, how to score it against a reference. A live receiver has no
   counterpart to any of them, and every generated config.yaml carries
   the whole set, so warning about them would bury the warnings that
   matter under a screenful that never does.
   Deliberately NOT in here: keys that configure the FILTER but that this
   receiver does not implement yet (baro.acc_noise_mps2_sqrthz,
   baro_height_disable, ...). Those are exactly the ones a user expects to
   take effect, so they must be named. */
static bool config_key_is_replay_only(const char* sec, const char* key)
{
    if (sec[0] != '\0')
    {
        return !strcmp(sec, "score") || !strcmp(sec, "inputs");
    }
    return !strcmp(key, "name") || !strcmp(key, "aiding") || !strcmp(key, "init");
}

/* Apply one "section.key: value" pair. Returns -2 when a key this tool
   owns carries a value it cannot read, which config_load turns into a
   hard error (REQ-VER-028), and 0 otherwise. An UNKNOWN key is not an
   error here (see the forward-compatibility branch at the end), a
   malformed value for a known key is: the destination would keep a
   default the operator never wrote down. */
static int config_set(insrcv_t* r, const char* sec, const char* key, const char* val)
{
    char full[128];
    snprintf(full, sizeof(full), "%s%s%s", sec, sec[0] ? "." : "", key);
    const double d = strtod(val, (char**)0);

    /* Noise model / tuning scalars: a zero or missing value keeps the
       built-in default, mirroring NavHandler.from_spec()'s "x or dflt". */
    if (!strcmp(full, "imu.gyr_psd")) { if (d > 0.0) r->gyr_psd = (float)d; }
    else if (!strcmp(full, "imu.acc_psd")) { if (d > 0.0) r->acc_psd = (float)d; }
    else if (!strcmp(full, "imu.gyr_bias_rw")) { if (d > 0.0) r->gyr_bias_rw = (float)d; }
    else if (!strcmp(full, "imu.acc_bias_rw")) { if (d > 0.0) r->acc_bias_rw = (float)d; }
    else if (!strcmp(full, "imu.acc_misalignment")) { if (mini_yaml_list(val, r->acc_misalignment, 9) != 0) return -2; }
    else if (!strcmp(full, "imu.gyr_misalignment")) { if (mini_yaml_list(val, r->gyr_misalignment, 9) != 0) return -2; }
    else if (!strcmp(full, "imu.acc_fixed_bias")) { if (mini_yaml_list(val, r->acc_fixed_bias, 3) != 0) return -2; }
    else if (!strcmp(full, "imu.gyr_fixed_bias")) { if (mini_yaml_list(val, r->gyr_fixed_bias, 3) != 0) return -2; }
    /* Extra process noise on top of what the IMU noise model already
       propagates through the strapdown (ins.h: these are densities per
       sqrt(s), not rates). Same keys as tools/replay.c. */
    else if (!strcmp(full, "imu.pos_pred_stddev_m_sqrts"))
    {
        if (d > 0.0) { r->pos_pred_stddev_m_sqrts = (float)d; }
    }
    else if (!strcmp(full, "imu.vel_pred_stddev_mps_sqrts"))
    {
        if (d > 0.0) { r->vel_pred_stddev_mps_sqrts = (float)d; }
    }
    else if (!strcmp(full, "imu.rpy_pred_stddev_rad_sqrts"))
    {
        if (d > 0.0) { r->rpy_pred_stddev_rad_sqrts = (float)d; }
    }
    else if (!strcmp(full, "init_hint.roll_deg")) { r->init_hint_roll_deg = (float)d; }
    else if (!strcmp(full, "init_hint.pitch_deg")) { r->init_hint_pitch_deg = (float)d; }
    else if (!strcmp(full, "init_hint.rpy_stddev_deg"))
    {
        r->init_hint_rpy_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "init_hint.yaw_deg")) { r->init_hint_yaw_deg = (float)d; }
    else if (!strcmp(full, "init_hint.yaw_stddev_deg"))
    {
        r->init_hint_yaw_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "init_stddev.pos_init_stddev_m"))
    {
        if (d > 0.0) { r->init_pos_stddev_m = (float)d; }
    }
    else if (!strcmp(full, "init_stddev.vel_init_stddev_mps"))
    {
        if (d > 0.0) { r->init_vel_stddev_mps = (float)d; }
    }
    else if (!strcmp(full, "init_stddev.rpy_init_stddev_rad_deg"))
    {
        if (d > 0.0) { r->init_rpy_stddev_deg = (float)d; }
    }
    else if (!strcmp(full, "init_stddev.yaw_init_stddev_rad_deg"))
    {
        if (d > 0.0) { r->init_yaw_stddev_deg = (float)d; }
    }
    else if (!strcmp(full, "init_stddev.acc_bias_init_stddev_mps2"))
    {
        if (d > 0.0) { r->init_acc_bias_stddev_mps2 = (float)d; }
    }
    else if (!strcmp(full, "init_stddev.gyr_bias_init_stddev_rps_deg"))
    {
        if (d > 0.0) { r->init_gyr_bias_stddev_deg = (float)d; }
    }
    else if (!strcmp(full, "imu.zero_vel_stddev_mps"))
    {
        if (d > 0.0) { r->zero_vel_stddev_mps = (float)d; }
    }
    else if (!strcmp(full, "imu.zero_rot_stddev_deg"))
    {
        if (d > 0.0) { r->zero_rot_stddev_deg = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_static_gyr_deg"))
    {
        if (d > 0.0) { r->auto_zupt_static_gyr_deg = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_static_acc_mps2"))
    {
        if (d > 0.0) { r->auto_zupt_static_acc_mps2 = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_max_vel_mps"))
    {
        if (d > 0.0) { r->auto_zupt_max_vel_mps = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_max_vel_stddev_mps"))
    {
        if (d > 0.0) { r->auto_zupt_max_vel_stddev_mps = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_static_gyr_stddev_deg"))
    {
        if (d > 0.0) { r->auto_zupt_static_gyr_stddev_deg = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_static_acc_stddev_mps2"))
    {
        if (d > 0.0) { r->auto_zupt_static_acc_stddev_mps2 = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_dwell_sec"))
    {
        if (d > 0.0) { r->auto_zupt_dwell_sec = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_min_interval_sec"))
    {
        if (d > 0.0) { r->auto_zupt_min_interval_sec = (float)d; }
    }
    else if (!strcmp(full, "imu.auto_zupt_velocity_blind_disable"))
    {
        r->auto_zaru = ((int)d == 0);
    }
    else if (!strcmp(full, "baro.enable")) { r->baro_enable = ((int)d != 0); }
    else if (!strcmp(full, "baro.stddev_m")) { if (d > 0.0) r->baro_stddev_m = (float)d; }
    /* Spelled as in tools/replay.c, python/replay.py and every dataset
       config under datasets. The value IS a stddev, but one config file
       has to mean the same thing in the live receiver and in a replay,
       and the shorter spelling is the one already written everywhere. */
    else if (!strcmp(full, "baro.acc_bias_init_mps2"))
    {
        if (d > 0.0) { r->baro_acc_bias_init_stddev_mps2 = (float)d; }
    }
    else if (!strcmp(full, "gnss.leverarm_frd")) { if (mini_yaml_list(val, r->leverarm_frd, 3) != 0) return -2; }
    else if (!strcmp(full, "gnss.pos_stddev_fallback_m"))
    {
        float fb[2] = {0.0f, 0.0f};
        if (mini_yaml_list(val, fb, 2) != 0) return -2;
        if (fb[0] > 0.0f) { r->pos_fallback_m = fb[0]; }
    }
    else if (!strcmp(full, "gnss.vel_stddev_fallback_mps"))
    {
        if (d > 0.0) { r->vel_fallback_mps = (float)d; }
    }
    else if (!strcmp(full, "gnss.max_horizontal_pos_stddev_m"))
    {
        if (d > 0.0) { r->gate_hpos_m = (float)d; }
    }
    else if (!strcmp(full, "gnss.max_vertical_pos_stddev_m"))
    {
        if (d > 0.0) { r->gate_vpos_m = (float)d; }
    }
    else if (!strcmp(full, "gnss.max_horizontal_vel_stddev_mps"))
    {
        if (d > 0.0) { r->gate_hvel_mps = (float)d; }
    }
    else if (!strcmp(full, "gnss.max_vertical_vel_stddev_mps"))
    {
        if (d > 0.0) { r->gate_vvel_mps = (float)d; }
    }
    else if (!strcmp(full, "gnss.start_max_horizontal_pos_stddev_m"))
    {
        if (d > 0.0) { r->start_hpos_m = (float)d; }
    }
    else if (!strcmp(full, "gnss.start_max_vertical_pos_stddev_m"))
    {
        if (d > 0.0) { r->start_vpos_m = (float)d; }
    }
    else if (!strcmp(full, "gnss.start_max_horizontal_vel_stddev_mps"))
    {
        if (d > 0.0) { r->start_hvel_mps = (float)d; }
    }
    else if (!strcmp(full, "gnss.start_max_vertical_vel_stddev_mps"))
    {
        if (d > 0.0) { r->start_vvel_mps = (float)d; }
    }
    else if (!strcmp(full, "gnss.stop_max_horizontal_pos_stddev_m"))
    {
        if (d > 0.0) { r->stop_hpos_m = (float)d; }
    }
    else if (!strcmp(full, "gnss.stop_max_vertical_pos_stddev_m"))
    {
        if (d > 0.0) { r->stop_vpos_m = (float)d; }
    }
    else if (!strcmp(full, "gnss.stop_max_horizontal_vel_stddev_mps"))
    {
        if (d > 0.0) { r->stop_hvel_mps = (float)d; }
    }
    else if (!strcmp(full, "gnss.stop_max_vertical_vel_stddev_mps"))
    {
        if (d > 0.0) { r->stop_vvel_mps = (float)d; }
    }
    else if (!strcmp(full, "gnss.init_dwell_sec"))
    {
        if (d > 0.0) { r->init_dwell_sec = (float)d; }
    }
    else if (!strcmp(full, "gnss.init_dwell_disable")) { r->init_dwell_disable = ((int)d != 0); }
    else if (!strcmp(full, "gnss.stop_dwell_sec"))
    {
        if (d > 0.0) { r->stop_dwell_sec = (float)d; }
    }
    else if (!strcmp(full, "gnss.stop_disable")) { r->stop_disable = ((int)d != 0); }
    else if (!strcmp(full, "gnss.pos_decimation")) { r->pos_decimation = (int)d; }
    else if (!strcmp(full, "gnss.delay_ms")) { r->gnss_delay_ms = (int)d; }
    else if (!strcmp(full, "gnss.enable")) { r->gnss_enable = ((int)d != 0); }
    else if (!strcmp(full, "gnss.pos_cov_scale")) { r->pos_cov_scale = (float)d; }
    else if (!strcmp(full, "gnss.pos_cov_scale_height"))
    {
        r->pos_cov_scale_height = (float)d;
    }
    else if (!strcmp(full, "gnss.vel_cov_scale")) { r->vel_cov_scale = (float)d; }
    else if (!strcmp(full, "gnss.pos_stddev_floor_hor_m"))
    {
        r->pos_stddev_floor_hor_m = (float)d;
    }
    else if (!strcmp(full, "gnss.pos_stddev_floor_ver_m"))
    {
        r->pos_stddev_floor_ver_m = (float)d;
    }
    else if (!strcmp(full, "gnss.vel_stddev_floor_hor_mps"))
    {
        r->vel_stddev_floor_hor_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.vel_stddev_floor_ver_mps"))
    {
        r->vel_stddev_floor_ver_mps = (float)d;
    }
    else if (!strcmp(full, "mag.wmm_year")) { if (d > 0.0) { r->wmm_year = (float)d; } }
    else if (!strcmp(full, "mag.enable")) { r->mag_enable = ((int)d != 0); }
    else if (!strcmp(full, "mag.stddev_ut")) { if (d > 0.0) r->mag_stddev_ut = (float)d; }
    /* Passed through unclamped: ins reads 0 as "use my default" and a
       negative value as "no rate limit", and both have to survive the
       config. */
    else if (!strcmp(full, "mag.min_delay_ms")) { r->mag_min_delay_ms = (int)d; }
    else if (!strcmp(full, "mag.estimate_bias")) { r->mag_estimate_bias = ((int)d != 0); }
    else if (!strcmp(full, "mag.misalignment")) { if (mini_yaml_list(val, r->mag_misalignment, 9) != 0) return -2; }
    else if (!strcmp(full, "mag.fixed_bias")) { if (mini_yaml_list(val, r->mag_fixed_bias, 3) != 0) return -2; }
    else if (!strcmp(full, "free_inertial_start.enable"))
    {
        r->fi_enable = ((int)d != 0);
    }
    else if (!strcmp(full, "free_inertial_start.lat_deg"))
    {
        r->fi_lat_deg = d;
        r->fi_have_lat = true;
    }
    else if (!strcmp(full, "free_inertial_start.lon_deg"))
    {
        r->fi_lon_deg = d;
        r->fi_have_lon = true;
    }
    else if (!strcmp(full, "free_inertial_start.height_m")) { r->fi_height_m = d; }
    else if (!strcmp(full, "free_inertial_start.stddev_m"))
    {
        if (d > 0.0) { r->fi_stddev_m = (float)d; }
    }
    else if (!strcmp(full, "automotive_mode")) { r->automotive_mode = ((int)d != 0); }
    else if (!strcmp(full, "automotive_min_speed_mps"))
    {
        if (d > 0.0) { r->automotive_min_speed_mps = (float)d; }
    }
    else if (!strcmp(full, "automotive_lateral_constraint"))
    {
        r->automotive_lateral_constraint = ((int)d != 0);
    }
    else if (!strcmp(full, "automotive_lateral_stddev_mps"))
    {
        if (d > 0.0) { r->automotive_lateral_stddev_mps = (float)d; }
    }
    else if (!strcmp(full, "automotive_lateral_max_yaw_rate_deg"))
    {
        if (d > 0.0) { r->automotive_lateral_max_yaw_rate_deg = (float)d; }
    }
    else if (!strcmp(full, "automotive_lateral_after_sec"))
    {
        r->automotive_lateral_after_sec = (float)d;
    }
    else if (!strcmp(full, "automotive_min_yaw_stddev_deg"))
    {
        if (d > 0.0) { r->automotive_min_yaw_stddev_deg = (float)d; }
    }
    else if (!strcmp(full, "allow_unlimited_deadreckoning"))
    {
        r->allow_unlimited_dr = ((int)d != 0);
    }
    else if (!strcmp(full, "max_deadreckoning_sec")) { if (d > 0.0) r->max_dr_sec = (float)d; }
    else if (!strcmp(full, "chi2_disable")) { r->chi2_disable = ((int)d != 0); }
    else if (!config_key_is_replay_only(sec, key))
    {
        /* Everything else is named rather than swallowed. The two cases
           behind it - a typo, and a key that only the replay harness
           implements - are indistinguishable from here, and both end the
           same way: the setting the user wrote down has no effect. That
           is worth one line either way. */
        r->n_config_ignored++;
        fprintf(stderr, "[insrcv] config: \"%s\" not used by insrcv"
                        " (typo, or a replay-only key)\n", full);
    }
    return 0;
}

/* mini_yaml_parse() callback: unknown keys are ignored (forward
 * compatibility, see config_set's own comment), a value config_set cannot
 * read aborts the parse (REQ-VER-028). */
static int config_set_cb(void* ctx, const char* sec, const char* key, const char* val)
{
    if (config_set((insrcv_t*)ctx, sec, key, val) != 0)
    {
        fprintf(stderr, "[insrcv] config: \"%s%s%s\" is not a list of numbers\n", sec,
                sec[0] ? "." : "", key);
        return -1;
    }
    return 0;
}

static int config_load(insrcv_t* r, const char* path)
{
    if (mini_yaml_parse(path, r, config_set_cb) != 0)
    {
        fprintf(stderr, "[insrcv] cannot open config %s\n", path);
        return -1;
    }

    bool calib = false;
    int  i;
    for (i = 0; i < 9; ++i)
    {
        if (fabsf(r->acc_misalignment[i]) > 0.0f || fabsf(r->gyr_misalignment[i]) > 0.0f)
        {
            calib = true;
        }
    }
    for (i = 0; i < 3; ++i)
    {
        if (fabsf(r->acc_fixed_bias[i]) > 0.0f || fabsf(r->gyr_fixed_bias[i]) > 0.0f)
        {
            calib = true;
        }
    }
    bool mag_calib = false;
    for (i = 0; i < 9; ++i)
    {
        if (fabsf(r->mag_misalignment[i]) > 0.0f) { mag_calib = true; }
    }
    for (i = 0; i < 3; ++i)
    {
        if (fabsf(r->mag_fixed_bias[i]) > 0.0f) { mag_calib = true; }
    }
    fprintf(stderr, "[insrcv] config %s: gyr_psd=%g acc_psd=%g imu-calib=%s", path,
            (double)r->gyr_psd, (double)r->acc_psd, calib ? "on" : "off");
    if (r->mag_enable)
    {
        /* Only a configured value is echoed: the fallback belongs to ins,
           which logs it itself, and printing a second copy of it here is
           how the two drift apart. */
        fprintf(stderr, " mag=on mag-calib=%s", mag_calib ? "on" : "off");
        if (r->mag_stddev_ut > 0.0f)
        {
            fprintf(stderr, " mag-stddev=%.1fuT", (double)r->mag_stddev_ut);
        }
    }
    else if (mag_calib)
    {
        /* A calibration in the file and the sensor switched off is worth
           saying out loud: it is the one combination where somebody did
           the work and gets nothing for it. */
        fprintf(stderr, " mag=off (but a mag calibration is configured)");
    }
    if (r->n_config_ignored > 0)
    {
        fprintf(stderr, ", %d key(s) ignored (see above)", r->n_config_ignored);
    }
    fprintf(stderr, "\n");
    if (r->mag_enable && !mag_calib)
    {
        /* The one combination the on-by-default fusion makes easy to
           reach by accident. ins's field-strength gate catches a hard
           iron large enough to move |B|, but the gate needs a position
           before it can arm, so on the way to the first fix nothing is
           watching. Said once, here, rather than left for a puzzled look
           at the heading later. */
        fprintf(stderr, "[insrcv] mag: fused WITHOUT a calibration (no mag.fixed_bias / "
                        "mag.misalignment) - hard iron is a bias, it shows up directly in "
                        "yaw, set mag.enable: 0 if this sensor is not calibrated\n");
    }
    return 0;
}

/* ===========================================================================
 * Raw-sensor overlay (the sensor/ subtree), identical keys to
 * ===========================================================================
 */

static void publish_imu_raw(insrcv_t* r, const imu_sample_t* s)
{
    json_t* j = &r->json;
    j_begin(j);
    j_obj_begin(j, "sensor");
    j_obj_begin(j, "imu");
    j_xyz(j, "acc_mps2", s->acc_mps2);
    j_vec3(j, "gyr_dps", "x", "y", "z", RAD2DEG(s->gyr_rps[0]),
           RAD2DEG(s->gyr_rps[1]), RAD2DEG(s->gyr_rps[2]));
    j_num(j, "temp_c", s->temp_c);
    j_num(j, "t_us", (double)s->t_us);
    j_obj_end(j);
    j_obj_end(j);
    pj_send(&r->pj, j);
}

static void publish_baro_raw(insrcv_t* r, const baro_sample_t* s)
{
    json_t* j = &r->json;
    j_begin(j);
    j_obj_begin(j, "sensor");
    j_obj_begin(j, "baro");
    j_num(j, "pressure_pa", s->pressure_pa);
    j_num(j, "temp_c", s->temp_c);
    j_num(j, "alt_m", baro_alt_pressure_to_altitude(s->pressure_pa));
    j_num(j, "t_us", (double)s->t_us);
    j_obj_end(j);
    j_obj_end(j);
    pj_send(&r->pj, j);
}

/* corrected = M * (raw - bias), M column-major, an all-zero M meaning
   identity: the same reading of the imu: / mag: keys that ins.c applies
   before fusing (REQ-NAV-037, REQ-NAV-039). Repeated here because ins does
   it internally on its way into the filter, and the overlay and status line
   below have to show what the filter is working with rather than what came
   off the wire. Used for all three triads, the correction being the same
   affine form for each. */
static void sensor_apply_calib(const float M[9], const float bias[3], const float in[3],
                               float out[3])
{
    const float c[3] = {in[0] - bias[0], in[1] - bias[1], in[2] - bias[2]};
    float       msum = 0.0f;
    int         i;
    for (i = 0; i < 9; ++i) { msum += fabsf(M[i]); }
    if (msum > 0.0f)
    {
        for (i = 0; i < 3; ++i) { out[i] = M[i] * c[0] + M[i + 3] * c[1] + M[i + 6] * c[2]; }
    }
    else
    {
        for (i = 0; i < 3; ++i) { out[i] = c[i]; }
    }
}

/* The magnetometer's own check, and the reason this is published even
   when nothing fuses it: at rest or in motion, level or upside down, a
   calibrated magnetometer reads ONE number, the local field strength. So
   the overlay carries the measured magnitude next to the magnitude the
   World Magnetic Model expects at this position, which turns "is the
   calibration any good" into a line on a plot. Exactly the |a| against
   9.81 check the accelerometer gets in tools/inslib_calib_gui.py.

   norm_ut is after the mag: calibration, since that is what the filter
   sees; norm_raw_ut is before it, so the two together also say how much
   the calibration did.

   heading_deg/heading_raw_deg are the same before/after pairing, but for
   direction rather than magnitude: ahrs_leveling_from_acc() on the latest
   IMU sample supplies roll/pitch (so this works without ins/ahrs ever
   having initialised) and ahrs_mag_heading() de-tilts the field with
   them. Both use the CALIBRATED accelerometer, so that the before/after
   pair isolates the mag: calibration instead of also carrying an
   accelerometer bias through the horizon it is de-tilted against. Relative to magnetic north, NOT true north -- no declination is
   applied, since the point is to see the sensor's own output, not the
   filter's. Skipped without an IMU sample yet to level with. */
static void publish_mag_raw(insrcv_t* r, const mag_sample_t* s)
{
    json_t* j = &r->json;
    float   m[3];
    int     i;
    for (i = 0; i < 3; ++i) { m[i] = s->mag_ut[i]; }
    sensor_apply_calib(r->mag_misalignment, r->mag_fixed_bias, s->mag_ut, m);

    j_begin(j);
    j_obj_begin(j, "sensor");
    j_obj_begin(j, "mag");
    j_xyz(j, "mag_ut", m);
    j_num(j, "norm_ut", sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]));
    j_num(j, "norm_raw_ut",
          sqrtf(s->mag_ut[0] * s->mag_ut[0] + s->mag_ut[1] * s->mag_ut[1]
                + s->mag_ut[2] * s->mag_ut[2]));
    if (r->have_last_seq)
    {
        float roll_rad, pitch_rad, a_cal[3];
        sensor_apply_calib(r->acc_misalignment, r->acc_fixed_bias, r->last_imu.acc_mps2, a_cal);
        ahrs_leveling_from_acc(a_cal, &roll_rad, &pitch_rad);
        j_num(j, "heading_deg", (double)RAD2DEG(ahrs_mag_heading(m, roll_rad, pitch_rad)));
        j_num(j, "heading_raw_deg",
              (double)RAD2DEG(ahrs_mag_heading(s->mag_ut, roll_rad, pitch_rad)));
    }
    if (r->wmm_set)
    {
        const float* b = r->suite.ins.magnetic_n;
        j_num(j, "expected_ut", sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]));
    }
    j_num(j, "temp_c", s->temp_c);
    j_num(j, "t_us", (double)s->t_us);
    j_obj_end(j);
    j_obj_end(j);
    pj_send(&r->pj, j);
}

static void publish_gnss_raw(insrcv_t* r, const gnss_sample_t* s)
{
    json_t* j = &r->json;
    j_begin(j);
    j_obj_begin(j, "sensor");
    j_obj_begin(j, "gnss");
    j_num(j, "lat_deg", s->lat_deg);
    j_num(j, "lon_deg", s->lon_deg);
    j_num(j, "alt_m", s->alt_m);
    j_num(j, "fix_type", (double)s->fix_type);
    j_num(j, "num_sv", (double)s->num_sv);

    /* Same NED frame as INSLIB/pos_ned, so the raw fix overlays the
       estimated position directly in PlotJuggler. */
    float pos_ned[3];
    if (gnss_llh_to_ins_ned(&r->suite.ins, s->lat_deg * (M_PI / 180.0),
                            s->lon_deg * (M_PI / 180.0), s->alt_m, pos_ned))
    {
        j_ned(j, "pos_ned", pos_ned);
    }
    j_ned(j, "vel_ned", s->vel_ned);

    /* 1-sigma, not the raw hacc_m/vacc_m/sacc_mps: NAV-COV's per-axis
       diagonal when the receiver reports it, otherwise the NAV-PVT
       scalar accuracies (already stddevs, isotropic in N/E resp. all
       three velocity axes). */
    float pos_sigma[3], vel_sigma[3];
    if (s->has_cov)
    {
        pos_sigma[0] = sqrtf(fmaxf(s->cov_pos[0], 0.0f)); /* nn */
        pos_sigma[1] = sqrtf(fmaxf(s->cov_pos[3], 0.0f)); /* ee */
        pos_sigma[2] = sqrtf(fmaxf(s->cov_pos[5], 0.0f)); /* dd */
        vel_sigma[0] = sqrtf(fmaxf(s->cov_vel[0], 0.0f));
        vel_sigma[1] = sqrtf(fmaxf(s->cov_vel[3], 0.0f));
        vel_sigma[2] = sqrtf(fmaxf(s->cov_vel[5], 0.0f));
    }
    else
    {
        pos_sigma[0] = s->hacc_m;
        pos_sigma[1] = s->hacc_m;
        pos_sigma[2] = s->vacc_m;
        vel_sigma[0] = s->sacc_mps;
        vel_sigma[1] = s->sacc_mps;
        vel_sigma[2] = s->sacc_mps;
    }
    j_obj_begin(j, "sigma");
    j_ned(j, "pos_ned_m", pos_sigma);
    j_ned(j, "vel_ned_mps", vel_sigma);
    j_obj_end(j);

    j_obj_end(j);
    j_obj_end(j);
    pj_send(&r->pj, j);
}

/* The reverse bit is published but never acted on: h(x) = ||v_n|| has no
   sign, and the assumption it WOULD matter for (course over ground equals
   heading) lives in ins_options_t.automotive_mode, which is configured
   once rather than per epoch. See inslib_protocol.md 0x40/0x80. */
static void publish_odo_raw(insrcv_t* r, const odo_sample_t* s)
{
    json_t* j = &r->json;
    j_begin(j);
    j_obj_begin(j, "sensor");
    j_obj_begin(j, "odo");
    j_num(j, "speed_mps", (double)s->speed_mps);
    j_num(j, "stddev_mps", (double)s->stddev_mps);
    j_num(j, "kind", (double)s->kind);
    j_num(j, "dir_valid", (s->flags & ODO_FLAG_DIR_VALID) ? 1.0 : 0.0);
    j_num(j, "reverse", (s->flags & ODO_FLAG_REVERSE) ? 1.0 : 0.0);
    j_num(j, "t_degraded", (s->flags & ODO_FLAG_T_DEGRADED) ? 1.0 : 0.0);
    j_num(j, "delay_suspect", (s->flags & ODO_FLAG_DELAY_SUSPECT) ? 1.0 : 0.0);
    j_num(j, "t_us_valid", (s->flags & ODO_FLAG_T_US_VALID) ? 1.0 : 0.0);
    j_num(j, "producer_delay_ms", (double)s->delay_ms);
    j_num(j, "t_us", (double)s->t_us);
    j_obj_end(j);
    j_obj_end(j);
    pj_send(&r->pj, j);
}

static void publish_timesync_raw(insrcv_t* r, const timesync_sample_t* s)
{
    json_t* j = &r->json;
    j_begin(j);
    j_obj_begin(j, "sensor");
    j_obj_begin(j, "timesync");
    j_num(j, "t_us", (double)s->t_us);
    j_num(j, "count", (double)s->count);
    j_num(j, "gps_valid", (s->flags & TS_FLAG_GPS_VALID) ? 1.0 : 0.0);
    j_num(j, "suspect", (s->flags & TS_FLAG_TP_SUSPECT) ? 1.0 : 0.0);
    j_num(j, "utc_base", (s->flags & TS_FLAG_UTC_BASE) ? 1.0 : 0.0);
    j_num(j, "gps_week", (double)s->week);
    /* Seconds of week rather than the continuous count: a plot of GPS
       seconds since 1980 is a straight line with no resolution left for
       what is actually interesting, the offset against the MCU clock. */
    j_num(j, "gps_tow_s", s->gps_s - (double)s->week * GPS_SEC_PER_WEEK);
    j_obj_end(j);
    j_obj_end(j);
    pj_send(&r->pj, j);
}

/* ===========================================================================
 * Solution + sub-filter breakdown (the INSLIB/ subtree)
 *
 * Mirrors state_to_pj_tree() + subfilter_overlay_tree() from
 * INSLIB/telemetry.py, key for key, so the same PlotJuggler layout works
 * for both receivers.
 * ===========================================================================
 */

/* MAVLink twin of publish_suite(), mirroring MavlinkSender.send() and
   send_subfilters() from INSLIB/telemetry.py: same messages, same gating,
   same NAMED_VALUE_FLOAT names -- plus GPS_RAW_INT and the two gnss_*
   values, which need the raw receiver fix that only the live path has
   (the CSV replay carries no satellite count). `blocked` is handed in
   rather than recomputed so the two outputs of one publish round can
   never disagree about why the 3D filter is or is not running. */
static void mav_send_suite(insrcv_t* r, blocked_t blocked)
{
    mav_sender_t* p = &r->mav;
    if (!p->enabled) { return; }
    nav_suite_t* s = &r->suite;

    const double   now     = host_now_sec();
    const double   up_sec  = now - p->t0;
    const uint32_t boot_ms = (uint32_t)(up_sec * 1000.0);
    const uint64_t boot_us = (uint64_t)(up_sec * 1e6);

    /* A GCS discards a system it has not heard from, so the heartbeat
       goes out whatever else the filter is or is not producing. */
    if (mav_due(p, MAV_RATE_HEARTBEAT, now)) { mav_emit(p, mini_mav_heartbeat(&p->m)); }

    /* ins's counter of ACCEPTED fixes, watched for movement: the count
       alone cannot tell a filter that stopped being aided a minute ago
       from one that is aided every epoch. Both fields start at zero, so
       a run that never sees a fix never arms the flag. */
    const ins_diag_t* diag = ins_get_diag(&s->ins);
    if (diag->n_gnss_used != p->gnss_used_count)
    {
        p->gnss_used_count = diag->n_gnss_used;
        p->gnss_used_time  = now;
        p->have_gnss_used  = true;
    }
    const bool gnss_active =
        p->have_gnss_used && (now - p->gnss_used_time) < MAV_GNSS_USED_TIMEOUT_SEC;

    /* Everything is pre-zeroed: an accessor that says "no" leaves its
       output untouched, and a field MAVLink has no "unknown" for is
       better sent as 0 than as whatever was on the stack. */
    float pos_ned[3] = {0.0f, 0.0f, 0.0f};
    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    float rpy[3]     = {0.0f, 0.0f, 0.0f};
    float rate[3]    = {0.0f, 0.0f, 0.0f};
    float acc_n[3]   = {0.0f, 0.0f, 0.0f};
    float q[4]       = {1.0f, 0.0f, 0.0f, 0.0f};
    float  dummy[3];
    float  sd[15];
    float  baro_h = 0.0f, baro_v = 0.0f, h_m = 0.0f, h_ell = 0.0f;
    double llh[3];
    int    i;

    const bool have_pos = ins_get_position_local(&s->ins, pos_ned);
    /* The filter's own anchor, not ins_get_position_ecef() converted back:
       that one is BUILT from these three numbers, so the round trip would
       cost two conversions for a value already in hand (REQ-NAV-078). */
    const bool have_llh  = ins_get_latlonh(&s->ins, llh);
    const bool have_rpy  = nav_suite_get_rpy(s, &rpy[0], &rpy[1], &rpy[2]);
    const bool have_rate = ins_get_omega_b_nb(&s->ins, rate);
    const bool have_acc  = ins_get_acc_n(&s->ins, acc_n);
    const bool have_h     = nav_suite_get_height(s, &h_m);
    const bool have_h_ell = nav_suite_get_height_ellipsoid(s, &h_ell);
    const bool ars_on     = ahrs_get_rpy(&s->ars, &dummy[0], &dummy[1], &dummy[2]);
    const bool ahrs_on    = ahrs_get_rpy(&s->ahrs, &dummy[0], &dummy[1], &dummy[2]);
    const bool baro_on    = nav_suite_get_baro_alt(s, &baro_h, &baro_v);
    const bool ready      = ins_is_ready(&s->ins);
    const bool have_sd    = ins_stddev_of(&s->ins, sd);
    (void)ins_get_velocity_ned(&s->ins, vel_ned);

    bool have_q = ins_get_quaternion(&s->ins, q);
    if (!have_q && have_rpy)
    {
        /* ATTITUDE_ONLY: ins has no quaternion, but an AHRS has an
           attitude. Synthesize one rather than publish none, exactly as
           Navigator.state() does. */
        ins_quat_from_rpy(rpy[0], rpy[1], rpy[2], q);
        have_q = true;
    }

    if (have_pos && mav_due(p, MAV_RATE_LOCAL_POS, now))
    {
        /* Down comes from the suite's own arbitration (REQ-SUITE-007),
           i.e. the barometer whenever ins has no fresh aiding, so the
           height on the wire is the best one available and not just
           ins's. North and east have no such second source. */
        const float xyz[3] = {pos_ned[0], pos_ned[1], have_h ? -h_m : pos_ned[2]};
        mav_emit(p, mini_mav_local_position_ned(&p->m, boot_ms, xyz, vel_ned));
    }

    if (have_llh && mav_due(p, MAV_RATE_GLOBAL_POS, now))
    {
        const double  alt_ell_m    = llh[2];
        const double  lat_deg      = RAD2DEG(llh[0]);
        const double  lon_deg      = RAD2DEG(llh[1]);
        const int16_t vel_cmps[3]  = {mav_cmps(vel_ned[0]), mav_cmps(vel_ned[1]),
                                      mav_cmps(vel_ned[2])};
        /* alt is declared as MSL. There is no geoid model in this
           library, so it carries the ellipsoidal height instead and the
           difference is the geoid undulation, tens of metres in places. */
        mav_emit(p, mini_mav_global_position_int(&p->m, boot_ms, (int32_t)(lat_deg * 1e7),
                                                 (int32_t)(lon_deg * 1e7), mav_mm(alt_ell_m),
                                                 mav_mm(have_h ? (double)h_m : 0.0), vel_cmps,
                                                 MAV_HDG_UNKNOWN));
    }

    /* The RECEIVER's own fix, next to the filter's solution. Satellite
       count, fix type and RTK state live in no other message of the set,
       and "the filter has no position" and "the antenna sees nothing"
       are different failures with different remedies. It goes out
       whatever the filter is doing with the fix, including with GNSS
       aiding switched off (gnss.enable: 0): what it reports is the
       receiver, not the filter.
       alt is declared MSL and carries the ellipsoidal height, the same
       caveat as GLOBAL_POSITION_INT above. */
    if (r->have_last_gnss && mav_due(p, MAV_RATE_GPS_RAW, now))
    {
        const gnss_sample_t* g = &r->last_gnss;
        mav_emit(p, mini_mav_gps_raw_int(&p->m, boot_us, (int32_t)(g->lat_deg * 1e7),
                                         (int32_t)(g->lon_deg * 1e7), mav_mm(g->alt_m),
                                         mav_acc_cm(g->hacc_m), mav_acc_cm(g->vacc_m),
                                         mav_gps_speed_cmps(g->vel_ned),
                                         mav_gps_cog_cdeg(g->vel_ned), mav_gps_fix_type(g),
                                         g->num_sv));
    }

    /* ALTITUDE carries the vertical channel on its own. Without it a
       baro-only solution would never reach MAVLink at all: both messages
       above need a horizontal position, and in ATTITUDE_ONLY mode there
       is none even while baro_alt produces a perfectly good height.
       NAN here is the protocol's own "unknown", so it stays. */
    if ((have_h || have_h_ell) && mav_due(p, MAV_RATE_ALTITUDE, now))
    {
        const float local_up = have_h ? h_m : NAN;
        const float amsl     = have_h_ell ? h_ell : NAN;
        mav_emit(p, mini_mav_altitude(&p->m, boot_us, local_up, amsl, local_up, local_up, NAN,
                                      NAN));
    }

    if (mav_due(p, MAV_RATE_ATTITUDE, now))
    {
        /* Both spellings of the same attitude, so a consumer can take
           whichever it prefers. */
        const float w[3] = {mav_f(rate[0]), mav_f(rate[1]), mav_f(rate[2])};
        if (have_q) { mav_emit(p, mini_mav_attitude_quaternion(&p->m, boot_ms, q, w)); }
        if (have_rpy) { mav_emit(p, mini_mav_attitude(&p->m, boot_ms, rpy, w)); }
    }

    if (mav_due(p, MAV_RATE_IMU, now))
    {
        /* The NAVIGATION-frame acceleration and the body rates the filter
           reports, not the raw IMU sample. Same choice as the Python
           sender: HIGHRES_IMU is the only standard message that carries
           an acceleration at all, and what is interesting here is what
           the filter made of the input. fields_updated names exactly the
           axes that are really in the message. */
        unsigned fields = 0u;
        float    a[3]   = {0.0f, 0.0f, 0.0f};
        float    g[3]   = {0.0f, 0.0f, 0.0f};
        for (i = 0; i < 3; ++i)
        {
            if (have_acc && isfinite(acc_n[i]))
            {
                a[i] = acc_n[i];
                fields |= 1u << i;
            }
            if (have_rate && isfinite(rate[i]))
            {
                g[i] = rate[i];
                fields |= 1u << (i + 3);
            }
        }
        if (fields != 0u)
        {
            mav_emit(p, mini_mav_highres_imu(&p->m, boot_us, a, g, (uint16_t)fields));
        }
    }

    if (mav_due(p, MAV_RATE_EKF_STATUS, now))
    {
        /* Real flags and real variances: a consumer can tell "attitude
           only, no position" from "everything nominal" without parsing
           anything else. */
        unsigned flags = 0u;
        if (ready || ars_on || ahrs_on) { flags |= MINI_MAV_EKF_ATTITUDE; }
        if (ready)
        {
            flags |= MINI_MAV_EKF_VELOCITY_HORIZ | MINI_MAV_EKF_VELOCITY_VERT |
                     MINI_MAV_EKF_POS_HORIZ_REL;
            if (blocked == BLOCKED_OK) { flags |= MINI_MAV_EKF_POS_HORIZ_ABS; }
        }
        if (baro_on || ready) { flags |= MINI_MAV_EKF_POS_VERT_ABS | MINI_MAV_EKF_POS_VERT_AGL; }

        /* 0.01 m resp. 0.01 m/s squared while the filter has no
           covariance to report, the same placeholder the Python sender
           uses. A zero would read as "perfectly known". */
        float vel_var = 1e-4f, pos_h_var = 1e-4f, pos_v_var = 1e-4f, compass_var = 0.0f;
        if (have_sd)
        {
            vel_var   = sd[3] * sd[3];
            if (sd[4] * sd[4] > vel_var) { vel_var = sd[4] * sd[4]; }
            if (sd[5] * sd[5] > vel_var) { vel_var = sd[5] * sd[5]; }
            pos_h_var = (sd[0] > sd[1]) ? sd[0] * sd[0] : sd[1] * sd[1];
            pos_v_var = sd[2] * sd[2];
            compass_var = sd[8] * sd[8]; /* yaw variance [rad^2] */
        }
        mav_emit(p, mini_mav_ekf_status_report(&p->m, (uint16_t)flags, vel_var, pos_h_var,
                                               pos_v_var, compass_var, 0.0f));
    }

    if (!mav_due(p, MAV_RATE_SUBFILTER, now)) { return; }

    /* NAMED_VALUE_FLOAT is the one message every ground station and log
       analyser reads without a dialect extension, so the per-module
       outputs and their 1-sigma stay visible next to the standard
       messages. Names are capped at MAVLink's 10 characters and match
       telemetry.py's, so one analysis script reads both receivers. */
    const nav_suite_mode_t mode  = nav_suite_get_mode(s);
    const int              dr_ms = ins_deadreckoning_ms(&s->ins);
    mav_nv(p, boot_ms, "mode", (float)mode);
    mav_nv(p, boot_ms, "blocked", (float)blocked);
    mav_nv(p, boot_ms, "dr_ms", (float)dr_ms);
    mav_nv(p, boot_ms, "vert_zupt", nav_suite_get_vertical_zupt_active(s) ? 1.0f : 0.0f);

    /* Whether the 3D filter is FUSING GNSS right now, which is a
       different question from whether the receiver has a fix
       (GPS_RAW_INT.fix_type) and from whether the filter is up at all
       (mode/blocked): a fix that the accuracy gate rejects, or one that
       arrives with GNSS aiding switched off, leaves this at 0 while the
       receiver reports a perfectly good 3D solution. The satellite count
       rides along so it is plottable without the GPS_RAW_INT decode. */
    mav_nv(p, boot_ms, "gnss_used", gnss_active ? 1.0f : 0.0f);
    if (r->have_last_gnss) { mav_nv(p, boot_ms, "gnss_nsat", (float)r->last_gnss.num_sv); }

    if (baro_on)
    {
        float v3[3], acc_bias, offset, offset_sd;
        mav_nv(p, boot_ms, "ba_h", baro_h);
        mav_nv(p, boot_ms, "ba_vz", baro_v);
        if (baro_stddev_of(&s->baro_alt, v3))
        {
            mav_nv(p, boot_ms, "ba_sig_h", v3[0]);
            mav_nv(p, boot_ms, "ba_sig_v", v3[1]);
            mav_nv(p, boot_ms, "ba_sig_ab", v3[2]);
        }
        if (baro_alt_get_acc_bias(&s->baro_alt, &acc_bias))
        {
            mav_nv(p, boot_ms, "ba_accbia", acc_bias);
        }
        if (local_gnss_alt_get(&s->local_gnss, &offset, &offset_sd))
        {
            mav_nv(p, boot_ms, "ba_offset", offset);
            mav_nv(p, boot_ms, "ba_off_sig", offset_sd);
        }
    }

    {
        float v3[3], bias[3];
        if (nav_suite_get_rpy_ars(s, &v3[0], &v3[1], &v3[2]))
        {
            mav_nv(p, boot_ms, "ars_roll", RAD2DEG(v3[0]));
            mav_nv(p, boot_ms, "ars_pitch", RAD2DEG(v3[1]));
            mav_nv(p, boot_ms, "ars_yaw", RAD2DEG(v3[2]));
            if (ahrs_get_bias_gyr(&s->ars, bias)) { mav_nv(p, boot_ms, "ars_bz", RAD2DEG(bias[2])); }
        }
        if (nav_suite_get_rpy_ahrs(s, &v3[0], &v3[1], &v3[2]))
        {
            mav_nv(p, boot_ms, "ahr_roll", RAD2DEG(v3[0]));
            mav_nv(p, boot_ms, "ahr_pitch", RAD2DEG(v3[1]));
            mav_nv(p, boot_ms, "ahr_yaw", RAD2DEG(v3[2]));
            if (ahrs_get_bias_gyr(&s->ahrs, bias))
            {
                mav_nv(p, boot_ms, "ahr_bz", RAD2DEG(bias[2]));
            }
        }
    }

    if (have_sd)
    {
        float vel_sd = sd[3];
        if (sd[4] > vel_sd) { vel_sd = sd[4]; }
        if (sd[5] > vel_sd) { vel_sd = sd[5]; }
        mav_nv(p, boot_ms, "i3d_sig_h", (sd[0] > sd[1]) ? sd[0] : sd[1]);
        mav_nv(p, boot_ms, "i3d_sig_v", sd[2]);
        mav_nv(p, boot_ms, "i3d_sig_s", vel_sd);
        mav_nv(p, boot_ms, "i3d_sig_y", RAD2DEG(sd[8]));
    }

    /* The blocked reason as text, but only when it CHANGES: repeated at
       the publish rate it would flood a ground station's message log. */
    if (!p->have_last_blocked || blocked != p->last_blocked)
    {
        p->last_blocked      = blocked;
        p->have_last_blocked = true;
        mav_statustext(p,
                       (blocked <= BLOCKED_COASTING) ? MINI_MAV_SEVERITY_INFO
                                                     : MINI_MAV_SEVERITY_WARNING,
                       blocked_text(blocked));
    }
}

static void publish_suite(insrcv_t* r)
{
    nav_suite_t* s = &r->suite;
    json_t*      j = &r->json;
    float        v3[3], v3b[3];

    j_begin(j);
    j_obj_begin(j, "INSLIB");

    /* --- arbitrated best-available solution --------------------------- */
    const bool             ready = ins_is_ready(&s->ins);
    const nav_suite_mode_t mode  = nav_suite_get_mode(s);
    const int              dr_ms      = ins_deadreckoning_ms(&s->ins);
    const int              baro_dr_ms = baro_alt_deadreckoning_ms(&s->baro_alt);
    j_num(j, "ready", ready ? 1.0 : 0.0);
    j_num(j, "mode", (double)mode);
    j_num(j, "dr_ms", (double)dr_ms);

    if (ins_get_position_local(&s->ins, v3)) { j_ned(j, "pos_ned", v3); }
    else
    {
        /* No 3D position, but the vertical channel may still have a
           height: publish it in the same NED-down convention so
           pos_ned/d stays continuous across the mode change. This is the
           C twin of Navigator.state()'s baro fallback. */
        float h;
        if (nav_suite_get_height(s, &h))
        {
            j_obj_begin(j, "pos_ned");
            j_num(j, "d", -(double)h);
            j_obj_end(j);
        }
    }
    if (ins_get_velocity_ned(&s->ins, v3)) { j_ned(j, "vel_ned", v3); }

    j_obj_begin(j, "global");
    /* Straight from the filter's anchor (REQ-NAV-078), see above. */
    double llh[3];
    if (ins_get_latlonh(&s->ins, llh))
    {
        j_num(j, "lat_deg", RAD2DEG(llh[0]));
        j_num(j, "lon_deg", RAD2DEG(llh[1]));
        j_num(j, "alt_m", llh[2]);
    }
    else
    {
        float h_ell;
        if (nav_suite_get_height_ellipsoid(s, &h_ell)) { j_num(j, "alt_m", (double)h_ell); }
    }
    j_obj_end(j);

    if (nav_suite_get_rpy(s, &v3[0], &v3[1], &v3[2])) { j_rpy_deg(j, "att_deg", v3); }
    if (ins_get_omega_b_nb(&s->ins, v3))
    {
        j_vec3(j, "rate_dps", "roll", "pitch", "yaw", RAD2DEG(v3[0]),
               RAD2DEG(v3[1]), RAD2DEG(v3[2]));
    }
    if (ins_get_acc_n(&s->ins, v3)) { j_xyz(j, "acc_n", v3); }

    /* --- attitude-only sub-filters ------------------------------------ */
    if (nav_suite_get_rpy_ars(s, &v3[0], &v3[1], &v3[2]))
    {
        j_obj_begin(j, "ars");
        j_rpy_deg(j, "att_deg", v3);
        j_num(j, "n_downweighted", (double)s->ars.n_downweighted);
        if (ahrs_get_bias_gyr(&s->ars, v3b))
        {
            j_vec3(j, "gyr_bias_dps", "x", "y", "z", RAD2DEG(v3b[0]),
                   RAD2DEG(v3b[1]), RAD2DEG(v3b[2]));
        }
        if (ahrs_gyr_bias_stddev_of(&s->ars, v3b))
        {
            j_vec3(j, "gyr_bias_sigma_dps", "x", "y", "z", RAD2DEG(v3b[0]),
                   RAD2DEG(v3b[1]), RAD2DEG(v3b[2]));
        }
        j_obj_end(j);
    }
    if (nav_suite_get_rpy_ahrs(s, &v3[0], &v3[1], &v3[2]))
    {
        j_obj_begin(j, "ahrs");
        j_rpy_deg(j, "att_deg", v3);
        j_num(j, "n_downweighted", (double)s->ahrs.n_downweighted);
        if (ahrs_get_bias_gyr(&s->ahrs, v3b))
        {
            j_vec3(j, "gyr_bias_dps", "x", "y", "z", RAD2DEG(v3b[0]),
                   RAD2DEG(v3b[1]), RAD2DEG(v3b[2]));
        }
        if (ahrs_gyr_bias_stddev_of(&s->ahrs, v3b))
        {
            j_vec3(j, "gyr_bias_sigma_dps", "x", "y", "z", RAD2DEG(v3b[0]),
                   RAD2DEG(v3b[1]), RAD2DEG(v3b[2]));
        }
        j_obj_end(j);
    }

    /* --- the full 3D filter's own state, independent of arbitration --- */
    j_obj_begin(j, "full3d");
    if (nav_suite_get_rpy_ins(s, &v3[0], &v3[1], &v3[2])) { j_rpy_deg(j, "att_deg", v3); }
    if (ins_get_position_local(&s->ins, v3)) { j_ned(j, "pos_ned", v3); }
    if (ins_get_velocity_ned(&s->ins, v3)) { j_ned(j, "vel_ned", v3); }
    if (ins_get_bias_gyr(&s->ins, v3))
    {
        j_vec3(j, "gyr_bias_dps", "x", "y", "z", RAD2DEG(v3[0]),
               RAD2DEG(v3[1]), RAD2DEG(v3[2]));
    }
    if (ins_get_bias_acc(&s->ins, v3)) { j_xyz(j, "acc_bias_mps2", v3); }
    j_num(j, "dr_ms", (double)dr_ms);
    float sd[15];
    if (ins_stddev_of(&s->ins, sd))
    {
        j_obj_begin(j, "sigma");
        j_vec3(j, "gyr_bias_dps", "x", "y", "z", RAD2DEG(sd[12]),
               RAD2DEG(sd[13]), RAD2DEG(sd[14]));
        j_vec3(j, "acc_bias_mps2", "x", "y", "z", sd[9], sd[10], sd[11]);
        j_vec3(j, "pos_ned_m", "n", "e", "d", sd[0], sd[1], sd[2]);
        j_vec3(j, "vel_ned_mps", "n", "e", "d", sd[3], sd[4], sd[5]);
        j_vec3(j, "att_deg", "roll", "pitch", "yaw", RAD2DEG(sd[6]),
               RAD2DEG(sd[7]), RAD2DEG(sd[8]));
        j_obj_end(j);
    }
    j_num(j, "n_downweighted", (double)ins_get_diag(&s->ins)->n_downweighted);
    j_obj_end(j);

    /* --- the baro/accel vertical channel ------------------------------ */
    float h_m, v_mps;
    if (nav_suite_get_baro_alt(s, &h_m, &v_mps))
    {
        j_obj_begin(j, "baroalt");
        j_obj_begin(j, "pos_ned");
        j_num(j, "d", -(double)h_m);
        j_obj_end(j);
        j_obj_begin(j, "vel_ned");
        j_num(j, "d", -(double)v_mps);
        j_obj_end(j);
        j_num(j, "height_m", (double)h_m);
        j_num(j, "climb_mps", (double)v_mps);
        j_num(j, "zupt_applied", nav_suite_get_vertical_zupt_active(s) ? 1.0 : 0.0);
        j_num(j, "dr_ms", (double)baro_dr_ms);
        j_num(j, "n_downweighted", (double)s->baro_alt.n_downweighted);
        float acc_bias;
        if (baro_alt_get_acc_bias(&s->baro_alt, &acc_bias))
        {
            j_num(j, "acc_bias_mps2", (double)acc_bias);
        }
        float meas_a_z;
        if (baro_alt_get_measurement_a_z(&s->baro_alt, &meas_a_z))
        {
            j_num(j, "measurement_a_z", (double)meas_a_z);
        }
        float meas_h;
        if (baro_alt_get_measurement_h(&s->baro_alt, &meas_h))
        {
            j_num(j, "measurement_h_m", (double)meas_h);
        }
        if (baro_stddev_of(&s->baro_alt, v3))
        {
            j_obj_begin(j, "sigma");
            j_num(j, "height_m", v3[0]);
            j_num(j, "climb_mps", v3[1]);
            j_num(j, "acc_bias_mps2", v3[2]);
            j_obj_end(j);
        }
        float offset, offset_sd;
        if (local_gnss_alt_get(&s->local_gnss, &offset, &offset_sd))
        {
            j_num(j, "gnss_offset_m", (double)offset);
            j_num(j, "gnss_offset_sigma_m", (double)offset_sd);
            j_num(j, "gnss_offset_n_downweighted", (double)s->local_gnss.n_downweighted);
        }
        j_obj_end(j);
    }

    /* --- status bits and the blocked reason --------------------------- */
    const blocked_t   blocked = suite_blocked(s, r->n_gnss_nofix);
    const ins_diag_t* d       = ins_get_diag(&s->ins);
    const bool ars_on  = ahrs_get_rpy(&s->ars, &v3[0], &v3[1], &v3[2]);
    const bool ahrs_on = ahrs_get_rpy(&s->ahrs, &v3[0], &v3[1], &v3[2]);
    const bool baro_on = nav_suite_get_baro_alt(s, &h_m, &v_mps);
    float      dummy_off, dummy_sd;

    j_obj_begin(j, "status");
    j_num(j, "blocked", (double)blocked);
    j_num(j, "mode", (double)mode);
    /* Two independent dead-reckoning clocks: full3d_dr_ms is ins's time
       since the last accepted GNSS/local-position fix
       (ins_deadreckoning_ms), baro_dr_ms is the vertical channel's time
       since its last accepted barometer fusion
       (baro_alt_deadreckoning_ms, REQ-BARO-023). They diverge whenever
       only one of GNSS and the barometer is actually flowing, which is
       exactly the case a shared "dr_ms" used to hide. */
    j_num(j, "full3d_dr_ms", (double)dr_ms);
    j_num(j, "baro_dr_ms", (double)baro_dr_ms);
    j_num(j, "full3d_running", ready ? 1.0 : 0.0);
    j_num(j, "ars_running", ars_on ? 1.0 : 0.0);
    j_num(j, "ahrs_running", ahrs_on ? 1.0 : 0.0);
    j_num(j, "baro_running", baro_on ? 1.0 : 0.0);
    j_num(j, "baro_offset_locked",
          local_gnss_alt_get(&s->local_gnss, &dummy_off, &dummy_sd) ? 1.0 : 0.0);
    j_num(j, "gnss_seen", (double)d->n_gnss_seen);
    j_num(j, "gnss_used", (double)d->n_gnss_used);
    j_num(j, "gnss_rejected_noise", (double)d->n_gnss_rejected_noise);
    j_num(j, "gnss_no_anchor", (double)d->n_gnss_no_anchor);
    j_num(j, "fuse_fail", (double)d->n_fuse_fail);
    j_num(j, "invalid_input", (double)d->n_invalid_input);
    /* Timestamp health. Without these a restarted source looks exactly
       like a healthy filter that stopped moving: the tree keeps arriving
       at the publish rate while every epoch behind it is dropped. */
    j_num(j, "time_backward", (double)d->n_time_backward);
    j_num(j, "time_dropped", (double)d->n_time_dropped);
    /* A device restart is recovered inside ins itself (u64 timestamps need
       no unwrapping, see inslib_protocol.md "Timebase"), autonomously and
       for every consumer rather than just this one (REQ-NAV-070).
       time_restart_reset above is that same event, counted where it is
       acted on. */
    j_num(j, "time_restart_reset", (double)d->n_time_restart_reset);
    j_num(j, "zupt_ins", ins_auto_zupt_active(&s->ins) ? 1.0 : 0.0);
    j_num(j, "zaru_ars", ahrs_auto_zaru_active(&s->ars) ? 1.0 : 0.0);
    j_num(j, "zaru_ahrs", ahrs_auto_zaru_active(&s->ahrs) ? 1.0 : 0.0);
    j_num(j, "zaru_applied", nav_suite_get_zaru_active(s) ? 1.0 : 0.0);
    j_num(j, "vertical_zupt", nav_suite_get_vertical_zupt_active(s) ? 1.0 : 0.0);
    j_obj_begin(j, "overconfident");
    j_num(j, "full3d", d->overconfident ? 1.0 : 0.0);
    j_num(j, "ars", (s->ars.n_overconfident > 0) ? 1.0 : 0.0);
    j_num(j, "ahrs", (s->ahrs.n_overconfident > 0) ? 1.0 : 0.0);
    j_obj_end(j);
    j_obj_end(j); /* status */

    j_obj_begin(j, "zaru");
    j_num(j, "ins_active", ins_auto_zupt_active(&s->ins) ? 1.0 : 0.0);
    j_num(j, "ars_active", ahrs_auto_zaru_active(&s->ars) ? 1.0 : 0.0);
    j_num(j, "ahrs_active", ahrs_auto_zaru_active(&s->ahrs) ? 1.0 : 0.0);
    j_num(j, "applied", nav_suite_get_zaru_active(s) ? 1.0 : 0.0);
    j_num(j, "vertical_applied", nav_suite_get_vertical_zupt_active(s) ? 1.0 : 0.0);
    j_obj_end(j);

    j_obj_end(j); /* INSLIB */
    pj_send(&r->pj, j);

    mav_send_suite(r, blocked);

    /* Live, the interesting question is usually "why is the 3D filter
       still not up" -- print the reason once per change, so it is visible
       without a PlotJuggler session attached. */
    if (!r->have_last_blocked || blocked != r->last_blocked)
    {
        r->last_blocked      = blocked;
        r->have_last_blocked = true;
        stats_line_break();
        fprintf(stderr, "[insrcv] %s\n", blocked_text(blocked));
    }
}

/* ===========================================================================
 * Filter driving
 * ===========================================================================
 */

static void suite_bootstrap(insrcv_t* r, int64_t t_us)
{
    if (r->suite_ready) { return; }

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    init.time = t_us;
    /* auto_init: the first usable fix supplies the real position, so the
       prescribed one only has to be plausible. Geodetic, like everything
       else this tool hands the filter (REQ-NAV-081), and lat/lon/h all zero
       is a point on the equator rather than the centre of the Earth. */
    /* free_inertial_start: the origin IS the declared point, so the
       uncertainty of that statement is the initial position uncertainty by
       construction. Without this the bootstrap installs ins's own
       beginner-friendly 10 m prior (REQ-NAV-049) and an operator who
       states 1 cm gets 10 m - the number would only ever have decided
       whether the declaration passes the entry gate. */
    if (r->fi_enable) { init.pos_init_stddev_m = r->fi_stddev_m; }
    /* An explicit init_stddev.pos_init_stddev_m wins over the declaration's
       own uncertainty: it is the more specific thing to have written down
       (same precedence as tools/replay.c). */
    if (r->init_pos_stddev_m > 0.0f) { init.pos_init_stddev_m = r->init_pos_stddev_m; }
    init.pos_pred_stddev_m_sqrts   = r->pos_pred_stddev_m_sqrts;
    init.vel_pred_stddev_mps_sqrts = r->vel_pred_stddev_mps_sqrts;
    init.rpy_pred_stddev_rad_sqrts = r->rpy_pred_stddev_rad_sqrts;
    init.acc_bias_pred_stddev_mps2_sqrts = r->acc_bias_rw;
    init.gyr_bias_pred_stddev_rps_sqrts = r->gyr_bias_rw;
    /* Initial uncertainty for the states auto-init does not otherwise
       seed a variance for: ins.h leaves all of these at exactly 0
       (perfectly known) unless the caller sets them, which pins the
       corresponding Kalman gain at 0 forever -- no ZUPT/ZARU or GNSS
       fusion can ever move a state whose covariance starts at 0. Every
       other caller in this repo (tools/replay.c, the Python binding
       in python/csrc/ins_capi.c) already sets these; insrcv.c did not,
       which is what let a real ~1 deg/s IMU gyro bias free-integrate the
       attitude on tools/testbalkon2.ubx with no way for the filter to
       ever learn it. Values mirror replay.c's un-referenced-bias
       defaults. */
    {
        const float rp_deg = (r->init_rpy_stddev_deg > 0.0f) ? r->init_rpy_stddev_deg : 3.0f;
        init.rpy_init_stddev_rad[0] = DEG2RAD(rp_deg);
        init.rpy_init_stddev_rad[1] = DEG2RAD(rp_deg);
    }
    if (r->init_yaw_stddev_deg > 0.0f)
    {
        /* Left at 0 otherwise: ins reads that as "no yaw prior of its own"
           and falls back to its own default, which is what this receiver
           has always relied on. */
        init.rpy_init_stddev_rad[2] = DEG2RAD(r->init_yaw_stddev_deg);
    }
    init.acc_bias_init_stddev_mps2 =
        (r->init_acc_bias_stddev_mps2 > 0.0f) ? r->init_acc_bias_stddev_mps2 : 0.05f;
    init.gyr_bias_init_stddev_rps = (r->init_gyr_bias_stddev_deg > 0.0f)
                                        ? DEG2RAD(r->init_gyr_bias_stddev_deg)
                                        : DEG2RAD(0.5f);
    if (r->init_vel_stddev_mps > 0.0f) { init.vel_init_stddev_mps = r->init_vel_stddev_mps; }
    init.zero_vel_stddev_mps       = r->zero_vel_stddev_mps;
    init.zero_rot_stddev_rps       = DEG2RAD(r->zero_rot_stddev_deg);

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.auto_init                          = true;
    opt.gnss_max_horizontal_pos_stddev_m   = r->gate_hpos_m;
    opt.gnss_max_vertical_pos_stddev_m     = r->gate_vpos_m;
    opt.gnss_max_horizontal_vel_stddev_mps = r->gate_hvel_mps;
    opt.gnss_max_vertical_vel_stddev_mps   = r->gate_vvel_mps;
    /* Solution-mode gates (REQ-NAV-051/052), 0 -> ins's own defaults. */
    opt.gnss_start_max_horizontal_pos_stddev_m   = r->start_hpos_m;
    opt.gnss_start_max_vertical_pos_stddev_m     = r->start_vpos_m;
    opt.gnss_start_max_horizontal_vel_stddev_mps = r->start_hvel_mps;
    opt.gnss_start_max_vertical_vel_stddev_mps   = r->start_vvel_mps;
    opt.gnss_stop_max_horizontal_pos_stddev_m    = r->stop_hpos_m;
    opt.gnss_stop_max_vertical_pos_stddev_m      = r->stop_vpos_m;
    opt.gnss_stop_max_horizontal_vel_stddev_mps  = r->stop_hvel_mps;
    opt.gnss_stop_max_vertical_vel_stddev_mps    = r->stop_vvel_mps;
    opt.gnss_init_dwell_sec                      = r->init_dwell_sec;
    opt.gnss_init_dwell_disable                  = r->init_dwell_disable;
    opt.gnss_stop_dwell_sec                      = r->stop_dwell_sec;
    opt.gnss_stop_disable                        = r->stop_disable;
    opt.gnss_pos_decimation                      = r->pos_decimation;
    opt.gnss_pos_cov_scale                       = r->pos_cov_scale;
    opt.gnss_pos_cov_scale_height                = r->pos_cov_scale_height;
    opt.gnss_vel_cov_scale                       = r->vel_cov_scale;
    opt.gnss_pos_stddev_floor_hor_m              = r->pos_stddev_floor_hor_m;
    opt.gnss_pos_stddev_floor_ver_m              = r->pos_stddev_floor_ver_m;
    opt.gnss_vel_stddev_floor_hor_mps            = r->vel_stddev_floor_hor_mps;
    opt.gnss_vel_stddev_floor_ver_mps            = r->vel_stddev_floor_ver_mps;
    opt.allow_unlimited_deadreckoning      = r->allow_unlimited_dr;
    opt.max_deadreckoning_sec              = r->max_dr_sec;
    opt.chi2_disable                       = r->chi2_disable;
    /* Automotive mode (REQ-NAV-034): course over ground fused as yaw once
       the platform is moving fast enough. Both tuning values 0 -> ins's own
       defaults, as everywhere else in this receiver. */
    opt.automotive_mode                    = r->automotive_mode;
    opt.automotive_min_speed_mps           = r->automotive_min_speed_mps;
    opt.automotive_min_yaw_stddev          = (r->automotive_min_yaw_stddev_deg > 0.0f)
                                                 ? DEG2RAD(r->automotive_min_yaw_stddev_deg)
                                                 : 0.0f;
    opt.automotive_lateral_constraint      = r->automotive_lateral_constraint;
    opt.automotive_lateral_stddev_mps      = r->automotive_lateral_stddev_mps;
    opt.automotive_lateral_max_yaw_rate    = (r->automotive_lateral_max_yaw_rate_deg > 0.0f)
                                                 ? DEG2RAD(r->automotive_lateral_max_yaw_rate_deg)
                                                 : 0.0f;
    opt.automotive_lateral_after_sec       = r->automotive_lateral_after_sec;
    /* Stillness detection: one set for ins, both AHRS instances and
       baro_alt (REQ-SUITE-020). The velocity-blind ARS/AHRS fallback
       (REQ-AHRS-017) is what makes the live receiver work at all before
       the first GNSS fix -- ins stays attitude-only until then, so its
       own velocity-aware detector never runs and the z gyro bias would
       stay unobservable -- hence it is armed by default and only
       --no-auto-zaru (or the config key) opts out. */
    opt.auto_zupt_static_gyr_rps           = DEG2RAD(r->auto_zupt_static_gyr_deg);
    opt.auto_zupt_static_acc_mps2          = r->auto_zupt_static_acc_mps2;
    opt.auto_zupt_max_vel_mps              = r->auto_zupt_max_vel_mps;
    opt.auto_zupt_max_vel_stddev_mps       = r->auto_zupt_max_vel_stddev_mps;
    opt.auto_zupt_static_gyr_stddev_rps    = DEG2RAD(r->auto_zupt_static_gyr_stddev_deg);
    opt.auto_zupt_static_acc_stddev_mps2   = r->auto_zupt_static_acc_stddev_mps2;
    opt.auto_zupt_dwell_sec                = r->auto_zupt_dwell_sec;
    opt.auto_zupt_min_interval_sec         = r->auto_zupt_min_interval_sec;
    opt.auto_zupt_velocity_blind_disable   = !r->auto_zaru;
    /* Fixed IMU calibration (REQ-NAV-037), from --config; nav_suite
       forwards it to the parallel ARS/AHRS as well (REQ-SUITE-012). */
    memcpy(opt.imu_acc_misalignment, r->acc_misalignment, sizeof(opt.imu_acc_misalignment));
    memcpy(opt.imu_gyr_misalignment, r->gyr_misalignment, sizeof(opt.imu_gyr_misalignment));
    memcpy(opt.imu_acc_fixed_bias, r->acc_fixed_bias, sizeof(opt.imu_acc_fixed_bias));
    memcpy(opt.imu_gyr_fixed_bias, r->gyr_fixed_bias, sizeof(opt.imu_gyr_fixed_bias));
    /* Fixed magnetometer calibration (REQ-NAV-039): soft iron, hard iron
       and the alignment onto the IMU triad, all folded into the one
       matrix by tools/inslib_calib_gui.py. Applied whether or not the
       18-state hard-iron estimation runs: that one tracks what CHANGES
       after the calibration was taken, it does not replace it. */
    memcpy(opt.mag_misalignment, r->mag_misalignment, sizeof(opt.mag_misalignment));
    memcpy(opt.mag_fixed_bias, r->mag_fixed_bias, sizeof(opt.mag_fixed_bias));
    opt.estimate_mag_bias                  = r->mag_estimate_bias;
    opt.magnetometer_min_delay_ms          = r->mag_min_delay_ms;

    memset(&r->suite, 0, sizeof(r->suite));
    if (nav_suite_init(&r->suite, &init, &opt) != 0)
    {
        stats_line_break();
        fprintf(stderr, "[insrcv] nav_suite_init failed\n");
        exit(1);
    }
    /* Known initial attitude (config init_hint). Armed right after
       nav_suite_init and before the first update, as the contract asks:
       it is only consulted while neither AHRS has initialized and only
       until ins bootstraps, so a running filter never sees it. */
    if (r->init_hint_rpy_stddev_deg > 0.0f || r->init_hint_yaw_stddev_deg > 0.0f)
    {
        nav_suite_set_init_att_hint(&r->suite, DEG2RAD(r->init_hint_roll_deg),
                                    DEG2RAD(r->init_hint_pitch_deg),
                                    DEG2RAD(r->init_hint_rpy_stddev_deg),
                                    DEG2RAD(r->init_hint_yaw_deg),
                                    DEG2RAD(r->init_hint_yaw_stddev_deg));
        if (r->init_hint_yaw_stddev_deg > 0.0f)
        {
            stats_line_break();
            fprintf(stderr, "[insrcv] initial heading asserted: %.1f deg +-%.1f deg\n",
                    (double)r->init_hint_yaw_deg, (double)r->init_hint_yaw_stddev_deg);
        }
    }
    /* baro_alt acc-bias drift density and initial uncertainty; both must
       be set before the first baro sample latches the template
       (nav_suite.h contract). */
    if (r->acc_bias_rw > 0.0f) { r->suite.baro_cfg.acc_bias_drift_mps2_sqrthz = r->acc_bias_rw; }
    if (r->baro_acc_bias_init_stddev_mps2 > 0.0f)
    {
        r->suite.baro_cfg.acc_bias_init_stddev_mps2 = r->baro_acc_bias_init_stddev_mps2;
    }
    r->suite_ready = true;
}

/* ===========================================================================
 * Free-inertial start (config section free_inertial_start)
 * ===========================================================================
 *
 * The 3D solution normally waits for the first GNSS fix, because without one
 * there is no origin to navigate away from. This lets the operator supply
 * that origin instead -- "I am HERE, to within this much" -- so the filter
 * comes up on the IMU alone and coasts on the auto-ZUPTs and the barometer
 * until GNSS appears, or until the coasting budget runs out.
 *
 * It is a DECLARATION offered as a position measurement, and it is offered
 * only until ins has bootstrapped from it. After that the receiver goes
 * quiet and the filter is on its own: a source that kept repeating the same
 * position would pin the solution to that point, which is the opposite of
 * dead reckoning.
 *
 * While the declaration stands, a GNSS fix WIDER than it is not forwarded
 * to the filter (fi_fix_beats_declaration). Indoors the receiver keeps
 * producing fixes tens of metres wide, and such a fix does not merely
 * carry less than the declaration - it also resets the 3D entry dwell
 * (REQ-NAV-045) that the declaration is in the middle of earning, so
 * letting it through means the filter never starts at all. The first fix
 * that is at least as good hands the job over for good.
 *
 * What it cannot do, and why the run has to be bounded:
 *
 *   - Heading is not observable here. Nothing measures yaw without GNSS
 *     course or a magnetometer, and at standstill a MEMS gyro cannot find
 *     north either (earth rate sits far below its bias stability). The
 *     filter dead-reckons along whatever heading it started with, so the
 *     track is a shape without an absolute direction.
 *   - Horizontal position drifts with nothing on board to correct it. The
 *     barometer holds the vertical channel (REQ-NAV-054: barometric height
 *     is selected at this bootstrap, because a baro sample is fresh) and
 *     the ZUPTs hold velocity and the biases whenever the platform stands
 *     still. That is what makes the run useful at all, and it is also all
 *     of it.
 *
 * So max_deadreckoning_sec still applies, and combining this with
 * allow_unlimited_deadreckoning is refused rather than silently obeyed:
 * once the budget expires ins_is_ready() goes false, the suite falls back
 * to ATTITUDE_ONLY, and the first real fix re-acquires normally (auto-init
 * stays on throughout). The declaration is not re-offered after that.
 */
/* Point the World Magnetic Model at where the receiver says it is.
 *
 * Both attitude sources have to agree on the yaw datum: ins gets the full
 * NED reference field (and its field-strength gate), the magnetometer AHRS
 * gets the declination that turns magnetic north into true north. Until
 * this runs, an AHRS references magnetic north - which is a real heading
 * error of up to a few degrees in Europe and far more at high latitudes.
 *
 * The epoch comes from the receiver whenever the stream carries it, since
 * it knows the date better than a config file does and a stale hand-typed
 * year silently degrades declination by ~0.1 deg per year off: GPS time
 * from the 0x40/0x05 pulses first, else the UTC date NAV-PVT carries in
 * every epoch once its validDate bit is set. mag.wmm_year overrides both
 * for a session that gets neither, and the host clock is the last resort.
 * A year is a low bar for a clock to clear, and the alternative was no
 * magnetic model at all: without one the AHRS references magnetic north,
 * which is already a few degrees of heading error in Europe and much more
 * further north. The source is named in the log line so a wrong date is
 * traceable.
 *
 * A better source arriving later re-applies the model at once instead of
 * waiting for WMM_REFRESH_M of travel: a receiver that needs half a
 * minute to decode the date would otherwise leave the whole session on
 * whatever the host clock said, which is the source most likely to be
 * wrong on a host without a synchronised clock.
 */
typedef enum
{
    EPOCH_NONE = 0,
    EPOCH_HOST,   /* host clock: nothing better in the stream */
    EPOCH_STREAM, /* the receiver's own date: 0x40/0x05 or NAV-PVT */
    EPOCH_CONFIG  /* mag.wmm_year: the operator overrides both */
} epoch_rank_t;

static float host_decimal_year(void)
{
    const time_t     now = time(NULL);
    const struct tm* tm  = gmtime(&now);
    if (tm == NULL) { return 0.0f; }
    /* tm_yday is 0-based; 365.25 keeps leap years from mattering at the
       0.003 year level, which is far below what the model resolves. */
    return (float)(1900.0 + (double)tm->tm_year + (double)tm->tm_yday / 365.25);
}

static void wmm_update(insrcv_t* r, double lat_deg, double lon_deg, const char* source)
{
    if (!r->suite_ready) { return; }

    const char*  epoch_src  = "config";
    epoch_rank_t epoch_rank = EPOCH_CONFIG;
    float        year       = r->wmm_year;
    if (year <= 0.0f && r->wmm_gps_s > 0.0)
    {
        /* GPS seconds -> decimal year. The epoch is 1980-01-06, i.e. 5
           days into that year, and leap seconds are 18 orders of
           magnitude below what the model resolves. */
        year       = (float)(1980.0 + (5.0 + r->wmm_gps_s / 86400.0) / 365.25);
        epoch_src  = "GPS time";
        epoch_rank = EPOCH_STREAM;
    }
    if (year <= 0.0f && r->wmm_pvt_year > 0.0f)
    {
        year       = r->wmm_pvt_year;
        epoch_src  = "GNSS UTC date";
        epoch_rank = EPOCH_STREAM;
    }
    if (year <= 0.0f)
    {
        year       = host_decimal_year();
        epoch_src  = "host clock";
        epoch_rank = EPOCH_HOST;
    }
    if (year <= 0.0f) { return; } /* no epoch, no model */

    if (r->wmm_set && (int)epoch_rank <= r->wmm_rank)
    {
        /* Equirectangular metres: exact enough to decide "has it moved a
           quarter of the refresh distance", which is all this is for. */
        const double dn = (lat_deg - r->wmm_lat_deg) * 111320.0;
        const double de = (lon_deg - r->wmm_lon_deg) * 111320.0 *
                          cos(lat_deg * (M_PI / 180.0));
        if (dn * dn + de * de < WMM_REFRESH_M * WMM_REFRESH_M) { return; }
    }

    const double lat_rad = lat_deg * (M_PI / 180.0);
    const double lon_rad = lon_deg * (M_PI / 180.0);
    ins_set_magnetic_model_from_position(&r->suite.ins, lat_rad, lon_rad, year);
    ahrs_set_position(&r->suite.ahrs, (float)lat_rad, (float)lon_rad, year);
    /* Logged for the first model and whenever the epoch source improves.
       The position refreshes in between are routine and stay quiet. */
    if (!r->wmm_set || (int)epoch_rank > r->wmm_rank)
    {
        stats_line_break();
        fprintf(stderr,
                "[insrcv] magnetic model at %.3f %.3f (from %s), epoch %.2f (from %s)\n",
                lat_deg, lon_deg, source, (double)year, epoch_src);
    }
    r->wmm_set     = true;
    r->wmm_rank    = (int)epoch_rank;
    r->wmm_lat_deg = lat_deg;
    r->wmm_lon_deg = lon_deg;
}

/* Is this fix worth more than the operator's declaration? Compared on the
   horizontal 1-sigma the receiver itself reports (NAV-COV when present,
   else NAV-PVT's hAcc), against the declared one. A fix without a usable
   accuracy figure loses by default: "unknown" is not "good". */
static bool fi_fix_beats_declaration(const insrcv_t* r, const gnss_sample_t* g)
{
    float sd = g->hacc_m;
    if (g->has_cov && g->cov_pos[0] > 0.0f && g->cov_pos[3] > 0.0f)
    {
        const float sd_n = sqrtf(g->cov_pos[0]);
        const float sd_e = sqrtf(g->cov_pos[3]);
        sd               = (sd_n > sd_e) ? sd_n : sd_e;
    }
    if (!isfinite(sd) || sd <= 0.0f) { return false; }
    return sd <= r->fi_stddev_m;
}

static void fi_offer_start_position(insrcv_t* r, int64_t t_us, ins_measurements_t* m)
{
    /* ins_deadreckoning_ms() is negative exactly while ins has not
       initialized, which is the moment to stop talking -- not
       ins_is_ready(), which stays false for another second and a half of
       settling (INS_MIN_RUNTIME_UNTIL_READY_MS) during which the offers
       would already be fusing and holding the position down. */
    if (ins_deadreckoning_ms(&r->suite.ins) >= 0)
    {
        r->fi_armed = false;
        stats_line_break();
        fprintf(stderr, "[insrcv] free-inertial start: 3D solution up, declared position "
                        "withdrawn (%llu offer(s)); coasting from here\n",
                (unsigned long long)r->n_fi);
        return;
    }
    if (t_us < r->fi_next_t_us) { return; }
    /* Twice the 1 fix/s the 3D entry dwell requires (REQ-NAV-045), so the
       dwell is satisfied by margin rather than exactly on its boundary. */
    r->fi_next_t_us = t_us + US_PER_SEC / 2;

    m->gnss_pos.llh[0]       = r->fi_lat_deg * (M_PI / 180.0);
    m->gnss_pos.llh[1]       = r->fi_lon_deg * (M_PI / 180.0);
    m->gnss_pos.llh[2]       = r->fi_height_m;
    const float var          = r->fi_stddev_m * r->fi_stddev_m;
    m->gnss_pos.Qll_ned[0]   = var;
    m->gnss_pos.Qll_ned[4]   = var;
    m->gnss_pos.Qll_ned[8]   = var;
    m->gnss_pos.is_valid     = true;
    if (r->n_fi == 0)
    {
        stats_line_break();
        fprintf(stderr, "[insrcv] free-inertial start: offering the declared position"
                        " until the 3D solution comes up\n");
    }
    /* No velocity: a declared point says nothing about motion, and ins
       starts such a bootstrap at rest, which is the honest reading of an
       operator typing in where the vehicle is parked. */
    r->n_fi++;
}

static void on_imu(insrcv_t* r, const imu_sample_t* s)
{
    if (!r->nav_on) { return; }
    suite_bootstrap(r, s->t_us);

    float dt = 0.0f;
    if (r->have_last_imu) { dt = (float)((double)(s->t_us - r->last_imu_t_us) / 1e6); }
    r->last_imu_t_us = s->t_us;
    r->have_last_imu = true;
    /* A dropped-USB gap or a stale timestamp can make dt huge or
       negative, clamp to a sane forward step so the strapdown stays
       bounded (the covariance prediction then spans the gap instead). */
    if (!(dt >= 0.0f && dt <= 0.5f)) { dt = 0.0f; }

    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp        = s->t_us;
    m.strapdown_dt_sec = dt;
    m.acc.is_valid     = true;
    m.gyr.is_valid     = true;
    int i;
    for (i = 0; i < 3; ++i)
    {
        m.acc.data[i]     = s->acc_mps2[i];
        m.gyr.data[i]     = s->gyr_rps[i];
        m.acc.Qll_diag[i] = r->acc_psd;
        m.gyr.Qll_diag[i] = r->gyr_psd;
    }

    /* With GNSS off (or simply absent) the declared start position is the
       only thing that knows where on earth this is, so the magnetic model
       takes its anchor from there. Retried until it sticks: the epoch may
       only arrive later, with the first GPS time pulse. */
    if (r->fi_enable && !r->wmm_set)
    {
        wmm_update(r, r->fi_lat_deg, r->fi_lon_deg, "the declared start position");
    }

    if (r->fi_armed && !r->have_pending_gnss) { fi_offer_start_position(r, s->t_us, &m); }

    if (r->have_pending_gnss)
    {
        const gnss_sample_t* g = &r->pending_gnss;
        r->have_pending_gnss   = false;
        /* As NAV-PVT reports it. The fusion builds its residual from the
           geodetic difference, so nothing is converted on the way in
           (REQ-NAV-079). */
        m.gnss_pos.llh[0]       = g->lat_deg * (M_PI / 180.0);
        m.gnss_pos.llh[1]       = g->lon_deg * (M_PI / 180.0);
        m.gnss_pos.llh[2]       = g->alt_m;
        m.gnss_pos.is_valid     = true;
        if (g->has_cov) { cov6_to_mat3(g->cov_pos, m.gnss_pos.Qll_ned); }
        else
        {
            /* Diagonal from the NAV-PVT scalar accuracies, floored: a
               receiver reporting an implausibly small accuracy would
               otherwise dominate every other measurement. */
            const float hv = isfinite(g->hacc_m) && g->hacc_m > r->pos_fallback_m
                                 ? g->hacc_m * g->hacc_m
                                 : r->pos_fallback_m * r->pos_fallback_m;
            const float vv = isfinite(g->vacc_m) && g->vacc_m > r->pos_fallback_m
                                 ? g->vacc_m * g->vacc_m
                                 : r->pos_fallback_m * r->pos_fallback_m;
            m.gnss_pos.Qll_ned[0] = hv;
            m.gnss_pos.Qll_ned[4] = hv;
            m.gnss_pos.Qll_ned[8] = vv;
        }
        if (isfinite(g->vel_ned[0]) && isfinite(g->vel_ned[1]) && isfinite(g->vel_ned[2]))
        {
            m.gnss_vel.is_valid = true;
            for (i = 0; i < 3; ++i) { m.gnss_vel.vel_ned[i] = g->vel_ned[i]; }
            if (g->has_cov) { cov6_to_mat3(g->cov_vel, m.gnss_vel.Qll_ned); }
            else
            {
                const float sv = isfinite(g->sacc_mps) && g->sacc_mps > r->vel_fallback_mps
                                     ? g->sacc_mps * g->sacc_mps
                                     : r->vel_fallback_mps * r->vel_fallback_mps;
                m.gnss_vel.Qll_ned[0] = sv;
                m.gnss_vel.Qll_ned[4] = sv;
                m.gnss_vel.Qll_ned[8] = sv;
            }
        }
        for (i = 0; i < 3; ++i) { m.gnss_leverarm_b[i] = r->leverarm_frd[i]; }
        m.gnss_delay_ms = r->gnss_delay_ms;
    }

    if (r->have_pending_baro)
    {
        m.baro.is_valid    = true;
        m.baro.pressure_pa = r->pending_baro.pressure_pa;
        m.baro.stddev_m    = r->baro_stddev_m;
        r->have_pending_baro = false;
    }

    if (r->have_pending_mag)
    {
        /* Newest sample wins, like the barometer and the GNSS fix above:
           the magnetometer runs at a tenth of the IMU rate, so most epochs
           carry none and the ones that do carry the one that arrived since
           the last epoch. ins applies the mag: calibration itself and
           throttles the fusion rate (magnetometer_min_delay_ms), so
           nothing here has to. */
        /* 0 is passed straight through: ins reads an unset variance as
           "use the library's magnetometer default", so the number lives
           in exactly one place (src/sensor_defaults.h) instead of once
           per harness. */
        const float var = r->mag_stddev_ut * r->mag_stddev_ut;
        for (i = 0; i < 3; ++i)
        {
            m.mag.data[i]     = r->pending_mag.mag_ut[i];
            m.mag.Qll_diag[i] = var;
        }
        m.mag.is_valid      = true;
        r->have_pending_mag = false;
    }

    if (r->have_pending_odo)
    {
        const odo_sample_t* o = &r->pending_odo;
        r->have_pending_odo   = false;
        /* The hub stamps t_us at the MEASUREMENT epoch (inslib_protocol.md
           0x40/0x80), so the sample's age is simply how far this epoch has
           moved past it, and ins anchors the residual that far back in its
           state history rather than against the current state. */
        const int64_t age_us = s->t_us - o->t_us;
        const int     age_ms = (int)(age_us / 1000);
        if (age_ms >= -ODO_MAX_LEAD_MS && age_ms <= ODO_MAX_AGE_MS)
        {
            m.speed.speed_mps  = o->speed_mps;
            m.speed.stddev_mps = o->stddev_mps;
            m.speed.is_valid   = true;
            /* ins cannot anchor a residual in the future, so a sample that
               is slightly ahead fuses against the current state. */
            m.speed_delay_ms   = age_ms > 0 ? age_ms : 0;
            if (age_ms < 0) { r->n_odo_lead++; }
        }
        else
        {
            r->n_odo_stale++;
        }
    }

    nav_suite_update(&r->suite, &m);

    /* Host clock, unconditionally: the input arrives at the pace the
       sender chose, and a replay that wants the original pace produces it
       (inslib_replay_udp.py) instead of asking for special handling here.

       Consequence worth knowing: a sender that delivers in BURSTS caps
       this. Within one burst the host clock barely moves, so the whole
       burst yields a single publish and the effective rate is the burst
       rate, not --pub-hz (a 0.2 s loop in tools/inslib_hub.py once made
       this 5 Hz). The fix belongs on the sending side. */
    const double now = host_now_sec();
    if (now >= r->next_pub_sec)
    {
        r->next_pub_sec = now + 1.0 / r->pub_hz;
        publish_suite(r);
    }
    if (r->raw_overlay && now >= r->next_raw_sec)
    {
        r->next_raw_sec = now + 1.0 / r->raw_imu_hz;
        publish_imu_raw(r, s);
    }
}

/* ===========================================================================
 * Dispatch + stats
 * ===========================================================================
 */

static void stats_on_imu(insrcv_t* r, const imu_sample_t* s)
{
    r->n_imu++;
    if (r->have_last_seq)
    {
        r->imu_drops += (uint64_t)((uint16_t)(s->seq - r->last_seq - 1));
    }
    r->last_seq      = s->seq;
    r->have_last_seq = true;
    r->last_imu      = *s;

    /* Measured IMU rate over a 1 s window of the MCU clock. */
    if (r->imu_win_n == 0) { r->imu_win_t0 = s->t_us; }
    r->imu_win_n++;
    const int64_t span = s->t_us - r->imu_win_t0;
    if (span >= US_PER_SEC)
    {
        r->imu_hz     = (double)(r->imu_win_n - 1) * 1e6 / (double)span;
        r->imu_win_t0 = s->t_us;
        r->imu_win_n  = 1;
    }
}

static void dispatch(insrcv_t* r, uint8_t msg_class, uint8_t msg_id, const uint8_t* payload,
                     size_t len)
{
    if (msg_class == CLASS_CUSTOM)
    {
        if (msg_id == ID_IMU)
        {
            imu_sample_t s;
            if (decode_imu(payload, len, &s))
            {
                stats_on_imu(r, &s);
                on_imu(r, &s);
                if (!r->nav_on && r->raw_overlay)
                {
                    const double now = host_now_sec();
                    if (now >= r->next_raw_sec)
                    {
                        r->next_raw_sec = now + 1.0 / r->raw_imu_hz;
                        publish_imu_raw(r, &s);
                    }
                }
            }
            else { r->n_bad_len++; }
        }
        else if (msg_id == ID_BARO)
        {
            baro_sample_t s;
            if (decode_baro(payload, len, &s))
            {
                r->n_baro++;
                r->last_baro         = s;
                if (r->baro_enable)
                {
                    r->pending_baro      = s;
                    r->have_pending_baro = true;
                }
                else
                {
                    /* Still counted, still in the raw overlay and still the
                       altitude on the stats line: what stops is the fusion,
                       not the sensor. */
                    r->n_baro_off++;
                }
                if (r->raw_overlay) { publish_baro_raw(r, &s); }
            }
            else { r->n_bad_len++; }
        }
        else if (msg_id == ID_MAG)
        {
            mag_sample_t s;
            if (decode_mag(payload, len, &s))
            {
                r->n_mag++;
                r->last_mag      = s;
                r->have_last_mag = true;
                if (r->mag_enable)
                {
                    r->pending_mag      = s;
                    r->have_pending_mag = true;
                }
                else
                {
                    /* Decoded, counted and published, just not fused: an
                       uncalibrated magnetometer is a heading error, not a
                       heading, so this one is opt-in. */
                    r->n_mag_off++;
                }
                if (r->raw_overlay) { publish_mag_raw(r, &s); }
            }
            else { r->n_bad_len++; }
        }
        else if (msg_id == ID_ODOMETRY)
        {
            odo_sample_t s;
            if (decode_odometry(payload, len, &s))
            {
                r->n_odo++;
                if ((s.flags & (ODO_FLAG_DIR_VALID | ODO_FLAG_REVERSE))
                    == (ODO_FLAG_DIR_VALID | ODO_FLAG_REVERSE))
                {
                    r->n_odo_reverse++;
                }
                r->last_odo      = s;
                r->have_last_odo = true;
                if (s.flags & ODO_FLAG_T_US_VALID)
                {
                    r->pending_odo      = s;
                    r->have_pending_odo = true;
                }
                else
                {
                    /* The hub had no IMU stream to map this onto. It is in
                       the capture for an offline consumer to place later;
                       here it has no epoch to attach to. */
                    r->n_odo_unstamped++;
                }
                if (r->raw_overlay) { publish_odo_raw(r, &s); }
            }
            else { r->n_bad_len++; }
        }
        else if (msg_id == ID_TIMESYNC)
        {
            timesync_sample_t s;
            if (decode_timesync(payload, len, &s))
            {
                r->n_pps++;
                if (s.flags & TS_FLAG_GPS_VALID)
                {
                    r->n_pps_gps++;
                    /* The one absolute-time source in the stream; the
                       magnetic model's epoch comes from it. */
                    r->wmm_gps_s = s.gps_s;
                }
                if (r->raw_overlay) { publish_timesync_raw(r, &s); }
            }
            else { r->n_bad_len++; }
        }
        else { r->n_custom_unknown++; }
        return;
    }

    /* Standard u-blox frame (GNSS). */
    r->n_gnss_ubx++;
    if (msg_class == CLASS_NAV && msg_id == ID_NAV_COV)
    {
        r->n_gnss_cov++;
        /* Only ever read together with the NAV-PVT it belongs to, so with
           GNSS off there is nothing for it to be cached for. */
        if (r->gnss_enable) { decode_nav_cov(payload, len, &r->cov); }
        return;
    }
    if (msg_class == CLASS_NAV && msg_id == ID_NAV_PVT)
    {
        gnss_sample_t s;
        if (decode_nav_pvt(payload, len, &r->cov, &s))
        {
            r->n_gnss_pvt++;
            r->last_gnss      = s;
            r->have_last_gnss = true;
            if (r->raw_overlay) { publish_gnss_raw(r, &s); }
            /* The date, before anything that can drop this epoch: it is a
               clock reading, not a measurement, so it is worth having even
               from a fix the filter refuses (and with gnss.enable 0, the
               same way the 0x40/0x05 time pulses are still taken). Mid-
               month is the epoch of a month-long bin, and the model moves
               by ~0.1 deg of declination per YEAR, so the day is noise. */
            if (s.date_ok)
            {
                r->wmm_pvt_year =
                    (float)s.utc_year + ((float)s.utc_month - 0.5f) / 12.0f;
            }
            /* Anything short of a 3D solution is not a weak position
               measurement, it is none: indoors the receiver keeps emitting
               epochs with fixType 0, hAcc 0xFFFFFFFF (4294967 m) and
               sAcc 999 m/s to say exactly that, and a 2D fix has no height
               to give at all. Handing them to the filter as position fixes
               made it conclude, after gnss_stop_dwell_sec of them, that
               its aiding had DEGRADED and drop out of the 3D solution
               (REQ-NAV-052) - ending a dead-reckoning run that nothing had
               actually gone wrong with. The offline converter has always
               filtered on fixType (--min-fix-type); this is the live path
               catching up. */
            if (!r->gnss_enable)
            {
                /* Decoded, counted and in the raw overlay - but this run
                   was asked to navigate without GNSS, and that includes
                   not quietly taking the magnetic model's anchor from it
                   (free_inertial_start's declared position supplies that
                   instead). */
                r->n_gnss_off++;
                return;
            }
            /* Coarse position for the magnetic model: a 2D fix is plenty
               (see GNSS_MIN_FIX_TYPE_POSITION) and it is wanted long
               before the 3D solution is up. */
            if (s.fix_ok && s.fix_type >= GNSS_MIN_FIX_TYPE_POSITION)
            {
                char src[32];
                snprintf(src, sizeof(src), "GNSS fixType %u", (unsigned)s.fix_type);
                wmm_update(r, s.lat_deg, s.lon_deg, src);
            }
            if (!s.fix_ok || s.fix_type < GNSS_MIN_FIX_TYPE_3D ||
                s.fix_type > GNSS_MIN_FIX_TYPE_MAX_3D)
            {
                r->n_gnss_nofix++;
                return;
            }
            if (r->fi_armed && !fi_fix_beats_declaration(r, &s))
            {
                /* Indoors the receiver still produces the occasional fix,
                   tens of metres wide. Handing it to the filter would do
                   two kinds of damage: it is worse than what the operator
                   declared, and every fix below the 3D entry gate RESETS
                   the entry dwell (REQ-NAV-045) that the declaration is
                   busy earning - so the filter would never start at all.
                   It stays in the raw overlay, it just does not aid. */
                r->n_fi_worse++;
                if (r->n_fi_worse == 1)
                {
                    stats_line_break();
                    fprintf(stderr, "[insrcv] free-inertial start: ignoring GNSS wider than"
                                    " the declared +-%g m while starting up\n",
                            (double)r->fi_stddev_m);
                }
                return;
            }
            if (r->fi_armed)
            {
                /* A fix at least as good as the declaration carries more
                   than the operator's word: hand over for good. */
                r->fi_armed = false;
                stats_line_break();
                fprintf(stderr, "[insrcv] free-inertial start: GNSS within the declared"
                                " +-%g m arrived, declaration withdrawn (%llu offer(s),"
                                " %llu wider fix(es) ignored)\n",
                        (double)r->fi_stddev_m, (unsigned long long)r->n_fi,
                        (unsigned long long)r->n_fi_worse);
            }
            r->pending_gnss       = s;
            r->have_pending_gnss  = true;
        }
    }
}

/* print_stats() below redraws its line in place (\r, no \n) so a long
   run does not scroll the terminal once a second. Anything else that
   goes to stderr while that line is open -- a LOG_WARN from the
   library, a free-inertial-start message, ... -- calls
   stats_line_break() first, or it would land inside the open line
   instead of getting a line of its own. */
static bool g_stats_line_open = false;
static int  g_stats_line_len  = 0;

static void stats_line_break(void)
{
    if (g_stats_line_open)
    {
        fputc('\n', stderr);
        g_stats_line_open = false;
    }
}

/* NAV-PVT fixType and carrSoln as text, for the status line only. */
static const char* fix_type_text(uint8_t fix_type)
{
    switch (fix_type)
    {
        case 0u:  return "nofix";
        case 1u:  return "dr";
        case 2u:  return "2D";
        case 3u:  return "3D";
        case 4u:  return "3D+dr";
        case 5u:  return "time";
        default:  return "?";
    }
}

static const char* carr_soln_text(uint8_t carr_soln)
{
    switch (carr_soln)
    {
        case 1u:  return " float";
        case 2u:  return " FIX";
        default:  return "";
    }
}

/* The status line. It answers one question - is everything still
   arriving, and what is the solution right now - so it carries RATES and
   the current state, and only those exception counters that are not
   zero. Totals belong in the closing summary: they grow without ever
   saying anything about the last second, and a line long enough to wrap
   is a line that cannot be read at a glance at all. */
static void print_stats(insrcv_t* r, double elapsed)
{
    const double dt = elapsed - r->stats_prev.t_sec;
    /* No usable interval to divide by. Happens on the final call when it
       lands right behind a periodic one; redrawing then would replace a
       good line with a line of zeros, which reads exactly like every
       sensor having just died. Leaving the last line standing is both
       simpler and truer. */
    if (dt <= 1e-3) { return; }
    const double inv = 1.0 / dt;
    /* Arrival rates, not the device-clock rate in imu_hz: that one is
       measured over a window of IMU TIMESTAMPS and therefore freezes at
       its last value the moment the stream stops, which would show a dead
       link as a healthy 800 Hz. A count over the wall-clock interval
       reads 0 when nothing arrives, which is the whole point of the
       line. imu_hz still describes the sensor itself and is reported in
       the closing summary. */
    const double imu_hz  = (double)(r->n_imu - r->stats_prev.n_imu) * inv;
    const double baro_hz = (double)(r->n_baro - r->stats_prev.n_baro) * inv;
    const double gnss_hz = (double)(r->n_gnss_pvt - r->stats_prev.n_gnss_pvt) * inv;
    const double mag_hz  = (double)(r->n_mag - r->stats_prev.n_mag) * inv;
    const double odo_hz  = (double)(r->n_odo - r->stats_prev.n_odo) * inv;

    char line[512];
    int  n = snprintf(line, sizeof(line), "%5.0fs | imu %3.0f baro %4.1f gnss %3.1f",
                      elapsed, imu_hz, baro_hz, gnss_hz);
    if (r->n_mag > 0 && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " mag %4.1f%s", mag_hz,
                      r->mag_enable ? "" : "(off)");
    }
    if (r->n_odo > 0 && n > 0 && n < (int)sizeof(line))
    {
        /* Rate and last value together: the two answer different halves of
           the same question (is the producer alive, and does it agree with
           the vehicle). */
        n += snprintf(line + n, sizeof(line) - (size_t)n, " odo %4.1f@%.1fm/s%s", odo_hz,
                      (double)r->last_odo.speed_mps,
                      (r->last_odo.flags & ODO_FLAG_DIR_VALID)
                              && (r->last_odo.flags & ODO_FLAG_REVERSE)
                          ? "(rev)"
                          : "");
    }
    /* Exception counters: silent while they are zero, which is what makes
       them worth looking at when they are not. */
    if (r->imu_drops > 0 && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " drop%llu",
                      (unsigned long long)r->imu_drops);
    }
    if (r->n_gnss_off > 0 && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " gnss-off%llu",
                      (unsigned long long)r->n_gnss_off);
    }
    if (r->n_baro_off > 0 && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " baro-off%llu",
                      (unsigned long long)r->n_baro_off);
    }
    if (r->n_odo_stale > 0 && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " odo-stale%llu",
                      (unsigned long long)r->n_odo_stale);
    }
    if (r->n_fi > 0 && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " decl%llu%s",
                      (unsigned long long)r->n_fi, r->fi_armed ? "*" : "");
        if (r->n_fi_worse > 0 && n > 0 && n < (int)sizeof(line))
        {
            n += snprintf(line + n, sizeof(line) - (size_t)n, "(-%llu)",
                          (unsigned long long)r->n_fi_worse);
        }
    }
    if (r->n_pps > 0 && n > 0 && n < (int)sizeof(line))
    {
        /* T once a pulse carried a GPS time, p while the receiver is
           pulsing without a time solution. The counts themselves say
           nothing more than that and are in the closing summary. */
        n += snprintf(line + n, sizeof(line) - (size_t)n, " pps%s",
                      r->n_pps_gps > 0 ? "T" : "p");
    }
    if (r->have_last_seq && n > 0 && n < (int)sizeof(line))
    {
        /* Calibrated, like |m| below and for the same reason: this line is
           meant to show what the filter is working with. It also turns |a|
           into the accelerometer's own version of the |m| check, since a
           calibrated accelerometer reads one number at rest in every
           attitude, and that number is g. The raw triad stays available on
           the sensor.imu overlay, which is what the calibration tool reads. */
        float a[3], w[3];
        sensor_apply_calib(r->acc_misalignment, r->acc_fixed_bias, r->last_imu.acc_mps2, a);
        sensor_apply_calib(r->gyr_misalignment, r->gyr_fixed_bias, r->last_imu.gyr_rps, w);
        const float  amag   = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        const double gz_dps = (double)(RAD2DEG(w[2]));
        n += snprintf(line + n, sizeof(line) - (size_t)n, " | |a|%.2f", (double)amag);
        /* gz is a motion indicator, and at rest it says nothing that |a|
           does not: shown only once the unit is actually turning. */
        if (fabs(gz_dps) >= 1.0 && n > 0 && n < (int)sizeof(line))
        {
            n += snprintf(line + n, sizeof(line) - (size_t)n, " gz%+.0f", gz_dps);
        }
    }
    if (r->n_baro > 0 && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " alt%.1f",
                      (double)baro_alt_pressure_to_altitude(r->last_baro.pressure_pa));
    }
    if (r->have_last_mag && n > 0 && n < (int)sizeof(line))
    {
        /* Calibrated magnitude against the model's, the magnetometer's
           equivalent of |a| against 9.81 above: a calibrated sensor reads
           the same |m| in every attitude, so a number that moves as the
           unit is turned is a calibration that is not done. Without a
           magnetic model yet (no position) only the measurement is shown,
           since there is nothing to compare it against. */
        float m[3];
        sensor_apply_calib(r->mag_misalignment, r->mag_fixed_bias, r->last_mag.mag_ut, m);
        const float mag_ut = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        if (r->wmm_set)
        {
            const float* b = r->suite.ins.magnetic_n;
            n += snprintf(line + n, sizeof(line) - (size_t)n, " |m|%.0f/%.0f",
                          (double)mag_ut,
                          (double)sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]));
        }
        else
        {
            n += snprintf(line + n, sizeof(line) - (size_t)n, " |m|%.0f", (double)mag_ut);
        }
        /* Tilt-compensated heading off the magnetometer, magnetic north.
           Computed from the calibrated field m above, so it is the
           direction the mag: keys hand the filter rather than what came
           off the wire: a compass reading that can be held against an
           external one. Independent of ins/ahrs ever having initialised,
           levelled off the latest IMU sample, so it is only as good as
           that sample's leveling assumption (still against gravity, which
           is a rough proxy in a moving vehicle). The leveling reads the
           CALIBRATED accelerometer: an uncorrected bias tilts the horizon
           this heading is de-tilted with, which would show up as a heading
           error the magnetometer never made. */
        if (r->have_last_seq && n > 0 && n < (int)sizeof(line))
        {
            float roll_rad, pitch_rad, a_cal[3];
            sensor_apply_calib(r->acc_misalignment, r->acc_fixed_bias, r->last_imu.acc_mps2,
                               a_cal);
            ahrs_leveling_from_acc(a_cal, &roll_rad, &pitch_rad);
            const float hdg_deg = RAD2DEG(ahrs_mag_heading(m, roll_rad, pitch_rad));
            n += snprintf(line + n, sizeof(line) - (size_t)n, " hdgmag%.0f", (double)hdg_deg);
        }
    }
    if (r->have_last_gnss && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " | %s %usv%s",
                      fix_type_text(r->last_gnss.fix_type),
                      (unsigned)r->last_gnss.num_sv,
                      carr_soln_text(r->last_gnss.carr_soln));
        /* Horizontal/vertical position accuracy and speed accuracy as
           reported by the receiver (NAV-PVT hAcc/vAcc/sAcc) -- the
           numbers that gate GNSS position fusion/3D entry-exit and the
           velocity gate. Without a position fix they are the receiver's
           "invalid" sentinel (thousands of kilometres, 999 m/s) and
           printing that is worse than printing nothing. */
        if (r->last_gnss.fix_type >= GNSS_MIN_FIX_TYPE_POSITION
            && n > 0 && n < (int)sizeof(line))
        {
            n += snprintf(line + n, sizeof(line) - (size_t)n, " %.2f/%.2fm %.2fm/s",
                          (double)r->last_gnss.hacc_m, (double)r->last_gnss.vacc_m,
                          (double)r->last_gnss.sacc_mps);
        }
    }
    if (nav_suite_get_zaru_active(&r->suite) && n > 0 && n < (int)sizeof(line))
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " ZUPT");
    }
    /* Framing faults, only once there are any: on a healthy link this is
       "crc=0 resync=0B" forever, twenty characters that never change. */
    if (r->framer.stats.crc_errors > 0 || r->framer.stats.resync_bytes > 0)
    {
        if (n > 0 && n < (int)sizeof(line))
        {
            n += snprintf(line + n, sizeof(line) - (size_t)n, " | crc%u resync%lluB",
                          (unsigned)r->framer.stats.crc_errors,
                          (unsigned long long)r->framer.stats.resync_bytes);
        }
        if (r->framer.stats.bad_length > 0 && n > 0 && n < (int)sizeof(line))
        {
            snprintf(line + n, sizeof(line) - (size_t)n, " badlen%u",
                     (unsigned)r->framer.stats.bad_length);
        }
    }
    r->stats_prev.t_sec      = elapsed;
    r->stats_prev.n_imu      = r->n_imu;
    r->stats_prev.n_baro     = r->n_baro;
    r->stats_prev.n_gnss_pvt = r->n_gnss_pvt;
    r->stats_prev.n_mag      = r->n_mag;
    r->stats_prev.n_odo      = r->n_odo;

    const int len = (int)strlen(line);
    int       pad;
    fprintf(stderr, "\r%s", line);
    /* \r only rewinds the cursor, it does not clear what was there: pad
       with spaces out to the widest line printed so far so a shorter
       line does not leave a stale tail behind. */
    for (pad = len; pad < g_stats_line_len; ++pad) { fputc(' ', stderr); }
    g_stats_line_len  = len;
    g_stats_line_open = true;
    fflush(stderr);
}

/* ===========================================================================
 * main
 * ===========================================================================
 */

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* The library's default log sink writes to stdout, but insrcv reserves
   stdout for --capture - (the raw binary stream) and puts its own
   diagnostics on stderr: redirect the library there too, or a log line
   would land in the middle of the byte stream. */
static void stderr_log_sink(int level, const char* file, int line, const char* fmt, va_list args)
{
    const char* name;
    stats_line_break();
    switch (level)
    {
    case LOG_LEVEL_FATAL: name = "FATAL"; break;
    case LOG_LEVEL_ERROR: name = "ERROR"; break;
    case LOG_LEVEL_WARN:  name = "WARN";  break;
    default:              name = "INFO";  break;
    }
    fprintf(stderr, "[%-5s] %s:%d: ", name, file, line);
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
}

static void usage(const char* argv0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "\n"
            "  UBX stream (UDP) -> nav_suite -> PlotJuggler (UDP/JSON).\n"
            "\n"
            "  The stream comes from tools/inslib_hub.py, which owns the serial\n"
            "  device and redistributes it, or from tools/inslib_replay_udp.py,\n"
            "  which sends a recorded .ubx at its original pace. This program\n"
            "  never opens a serial port: that would make recording the same\n"
            "  session impossible, since a port has exactly one owner.\n"
            "\n"
            "input:\n"
            "  --udp-port <n>        UDP port to listen on (default %d)\n"
            "  --udp-bind <addr>     address to bind (default %s; use 127.0.0.1\n"
            "                        to accept local senders only)\n"
            "output:\n"
            "  --raw-only            publish raw sensors only, don't run the filter\n"
            "  --no-raw-overlay      don't mirror raw sensors alongside the estimate\n"
            "  --pj-ip <addr>        PlotJuggler address (default 127.0.0.1)\n"
            "  --pj-port <n>         PlotJuggler UDP port (default 9870)\n"
            "  --plotjuggler-off     disable PlotJuggler telemetry\n"
            "  --mavlink             also send MAVLink 2 (HEARTBEAT, ATTITUDE,\n"
            "                        ATTITUDE_QUATERNION, LOCAL_POSITION_NED,\n"
            "                        GLOBAL_POSITION_INT, GPS_RAW_INT, ALTITUDE,\n"
            "                        HIGHRES_IMU, EKF_STATUS_REPORT, plus the\n"
            "                        sub-filter breakdown as NAMED_VALUE_FLOAT and\n"
            "                        the blocked reason as STATUSTEXT). Same rates\n"
            "                        as python/replay.py --mavlink, which has no\n"
            "                        GPS_RAW_INT: that needs the raw receiver fix,\n"
            "                        which only the live path has\n"
            "  --mav-ip <addr>       MAVLink destination (default %s)\n"
            "  --mav-port <n>        MAVLink UDP port (default %d)\n"
            "  --mav-subfilter-hz <f>\n"
            "                        NAMED_VALUE_FLOAT block rate (default %g)\n"
            "  --pub-hz <f>          estimate publish rate (default 50)\n"
            "  --raw-imu-hz <f>      raw IMU publish rate cap (default 100)\n"
            "  --stats-sec <f>       stats line interval, 0 disables (default 1)\n"
            "  --no-auto-zaru        don't arm the ARS/AHRS velocity-blind auto-ZARU\n"
            "filter:\n"
            "  --config <yaml>       read the config.yaml subset shared with\n"
            "                        python/replay.py: imu noise + fixed IMU\n"
            "                        calibration, baro enable/stddev,\n"
            "                        gnss gates/lever arm/delay, auto-ZUPT/ZARU\n"
            "                        pseudo-measurement stddev + static gate,\n"
            "                        automotive_mode (yaw from the GNSS course\n"
            "                        over ground).\n"
            "                        Applied where it appears: later command-line\n"
            "                        options override it.\n"
            "  --acc-psd <f>         accelerometer PSD [(m/s^2)^2/Hz] (default 3.85e-6)\n"
            "  --gyr-psd <f>         gyro PSD [(rad/s)^2/Hz] (default (0.02 deg/s)^2)\n"
            "  --acc-bias-rw <f>     accel bias random walk (default 3e-4)\n"
            "  --gyr-bias-rw <f>     gyro bias random walk (default 2e-6)\n"
            "  --baro-stddev <f>     barometric altitude 1-sigma [m] (0 -> library default)\n"
            "  --baro-acc-bias-init-stddev <f>\n"
            "                        baro_alt's INITIAL vertical accel-bias 1-sigma\n"
            "                        [m/s^2] (0 -> library default, 0.02). The\n"
            "                        default is tight for a real sensor's post-\n"
            "                        leveling vertical bias (often ~0.03-0.05\n"
            "                        m/s^2): the tighter the prior, the slower\n"
            "                        baroalt/acc_bias_mps2 is pulled toward it via\n"
            "                        ZUPT/baro alone (minutes, not seconds) -- widen\n"
            "                        this to match the sensor if convergence looks\n"
            "                        implausibly slow.\n"
            "  --leverarm <f,f,f>    GNSS antenna lever arm, body FRD [m]\n"
            "  --gnss-delay-ms <n>   GNSS measurement age [ms] (default 0)\n"
            "  --max-dr-sec <f>      max IMU-only coasting [s] (0 -> library default)\n"
            "  --unlimited-dr        never expire the coasting window\n"
            "  --chi2-disable        never chi2-downweight any fusion (REQ-SYS-015,\n"
            "                        diagnostics only: fuses every measurement at its\n"
            "                        nominal variance, even gross outliers)\n",
            argv0, UDP_PORT_DEFAULT, UDP_BIND_DEFAULT, MAV_IP_DEFAULT, MAV_PORT_DEFAULT,
            MAV_SUBFILTER_HZ_DEFAULT);
}

static bool arg_f(int argc, char** argv, int* i, const char* name, float* out)
{
    if (strcmp(argv[*i], name) != 0) { return false; }
    if (*i + 1 >= argc) { fprintf(stderr, "[insrcv] %s needs a value\n", name); exit(2); }
    *out = (float)atof(argv[++(*i)]);
    return true;
}

static bool arg_d(int argc, char** argv, int* i, const char* name, double* out)
{
    if (strcmp(argv[*i], name) != 0) { return false; }
    if (*i + 1 >= argc) { fprintf(stderr, "[insrcv] %s needs a value\n", name); exit(2); }
    *out = atof(argv[++(*i)]);
    return true;
}

static bool arg_i(int argc, char** argv, int* i, const char* name, int* out)
{
    if (strcmp(argv[*i], name) != 0) { return false; }
    if (*i + 1 >= argc) { fprintf(stderr, "[insrcv] %s needs a value\n", name); exit(2); }
    *out = atoi(argv[++(*i)]);
    return true;
}

static bool arg_s(int argc, char** argv, int* i, const char* name, const char** out)
{
    if (strcmp(argv[*i], name) != 0) { return false; }
    if (*i + 1 >= argc) { fprintf(stderr, "[insrcv] %s needs a value\n", name); exit(2); }
    *out = argv[++(*i)];
    return true;
}

int main(int argc, char** argv)
{
    static insrcv_t r; /* static: ~200 kB of filter state, not stack-sized */
    memset(&r, 0, sizeof(r));
    /* Unbuffered diagnostics. Redirected to a file or a pipe, stderr would
       otherwise be block-buffered, and a receiver is normally stopped by
       killing it -- which then discards the last few kB, i.e. exactly the
       lines describing whatever made you stop it. */
    setvbuf(stderr, (char*)0, _IONBF, 0);
    setvbuf(stdout, (char*)0, _IOLBF, 0);
    log_set_sink(stderr_log_sink);

    r.udp_bind         = UDP_BIND_DEFAULT;
    r.udp_port         = UDP_PORT_DEFAULT;
    /* Not left at the memset zero: pj_close() runs even when
       --plotjuggler-off skipped pj_open(), and fd 0 is stdin. */
    r.pj.fd            = SOCK_INVALID;
    r.pj_ip            = "127.0.0.1";
    r.pj_port          = 9870;
    r.pj_on            = true;
    r.mav.fd           = SOCK_INVALID; /* same reason as r.pj.fd above */
    r.mav_ip           = MAV_IP_DEFAULT;
    r.mav_port         = MAV_PORT_DEFAULT;
    r.mav_on           = false; /* opt-in: nothing listens on 14550 by default */
    r.mav_subfilter_hz = MAV_SUBFILTER_HZ_DEFAULT;
    r.nav_on           = true;
    r.auto_zaru        = true;
    r.raw_overlay      = true;
    /* 50, as --help and the <= 0 fallback below have always said. The
       initializer said 25 and won, so the publish rate was half the
       documented one. */
    r.pub_hz           = 50.0;
    r.raw_imu_hz       = 100.0;
    r.stats_sec        = 1.0;
    r.acc_psd          = 0.0f;
    r.gyr_psd          = 0.0f;
    /* Declared start position, 1-sigma. Kept at the order of ins's own 3D
       entry gate (gnss_start_max_horizontal_pos_stddev_m): a declaration
       claiming to be less certain than that gate admits would be rejected
       by it, and the filter would never enter 3D at all. */
    r.gnss_enable      = true;
    r.baro_enable      = true; /* see the field: the harness default is the other way */
    r.mag_enable       = true; /* see the field, and the offline harness is again
                                  the other way (there the key opens a file) */
    r.fi_stddev_m      = 2.0f;
    r.acc_bias_rw      = 0.0f;
    r.gyr_bias_rw      = 0.0f;
    /* r.baro_stddev_m left at 0: baro_alt_update() then falls back to
       baro_alt's own library default (BARO_ALT_DEFAULT_BARO_STDDEV_M)
       rather than insrcv maintaining a second, parallel default. */
    r.pos_fallback_m   = 0.0f;
    r.vel_fallback_mps = 0.0f;
    r.gate_hpos_m      = 0.0f;
    r.gate_vpos_m      = 0.0f;
    r.gate_hvel_mps    = 0.0f;
    r.gate_vvel_mps    = 0.0f;
    r.start_hpos_m     = 0.0f;
    r.start_vpos_m     = 0.0f;
    r.start_hvel_mps   = 0.0f;
    r.start_vvel_mps   = 0.0f;
    r.stop_hpos_m      = 0.0f;
    r.stop_vpos_m      = 0.0f;
    r.stop_hvel_mps    = 0.0f;
    r.stop_vvel_mps    = 0.0f;
    r.init_dwell_sec   = 0.0f;
    r.stop_dwell_sec   = 0.0f;
    r.init_dwell_disable = false;
    r.stop_disable       = false;
    r.pos_decimation     = 0; /* 0 -> ins's own default (REQ-NAV-063) */
    r.gnss_delay_ms    = 0;

    int i;
    for (i = 1; i < argc; ++i)
    {
        const char* leverarm = NULL;
        const char* config   = NULL;
        if (arg_s(argc, argv, &i, "--config", &config))
        {
            if (config_load(&r, config) != 0) { return 2; }
            continue;
        }
        if (arg_i(argc, argv, &i, "--udp-port", &r.udp_port)) { continue; }
        if (arg_s(argc, argv, &i, "--udp-bind", &r.udp_bind)) { continue; }
        if (arg_s(argc, argv, &i, "--pj-ip", &r.pj_ip)) { continue; }
        if (arg_i(argc, argv, &i, "--pj-port", &r.pj_port)) { continue; }
        if (arg_s(argc, argv, &i, "--mav-ip", &r.mav_ip)) { continue; }
        if (arg_i(argc, argv, &i, "--mav-port", &r.mav_port)) { continue; }
        if (arg_d(argc, argv, &i, "--mav-subfilter-hz", &r.mav_subfilter_hz)) { continue; }
        if (arg_d(argc, argv, &i, "--pub-hz", &r.pub_hz)) { continue; }
        if (arg_d(argc, argv, &i, "--raw-imu-hz", &r.raw_imu_hz)) { continue; }
        if (arg_d(argc, argv, &i, "--stats-sec", &r.stats_sec)) { continue; }
        if (arg_f(argc, argv, &i, "--acc-psd", &r.acc_psd)) { continue; }
        if (arg_f(argc, argv, &i, "--gyr-psd", &r.gyr_psd)) { continue; }
        if (arg_f(argc, argv, &i, "--acc-bias-rw", &r.acc_bias_rw)) { continue; }
        if (arg_f(argc, argv, &i, "--gyr-bias-rw", &r.gyr_bias_rw)) { continue; }
        if (arg_f(argc, argv, &i, "--baro-stddev", &r.baro_stddev_m)) { continue; }
        if (arg_f(argc, argv, &i, "--baro-acc-bias-init-stddev",
                  &r.baro_acc_bias_init_stddev_mps2))
        {
            continue;
        }
        if (arg_f(argc, argv, &i, "--max-dr-sec", &r.max_dr_sec)) { continue; }
        if (arg_i(argc, argv, &i, "--gnss-delay-ms", &r.gnss_delay_ms)) { continue; }
        if (arg_s(argc, argv, &i, "--leverarm", &leverarm))
        {
            if (sscanf(leverarm, "%f,%f,%f", &r.leverarm_frd[0], &r.leverarm_frd[1],
                       &r.leverarm_frd[2]) != 3)
            {
                fprintf(stderr, "[insrcv] --leverarm wants 'x,y,z'\n");
                return 2;
            }
            continue;
        }
        if (strcmp(argv[i], "--raw-only") == 0) { r.nav_on = false; continue; }
        if (strcmp(argv[i], "--no-raw-overlay") == 0) { r.raw_overlay = false; continue; }
        if (strcmp(argv[i], "--plotjuggler-off") == 0) { r.pj_on = false; continue; }
        if (strcmp(argv[i], "--mavlink") == 0) { r.mav_on = true; continue; }
        if (strcmp(argv[i], "--no-auto-zaru") == 0) { r.auto_zaru = false; continue; }
        if (strcmp(argv[i], "--unlimited-dr") == 0) { r.allow_unlimited_dr = true; continue; }
        if (strcmp(argv[i], "--chi2-disable") == 0) { r.chi2_disable = true; continue; }
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            usage(argv[0]);
            return 0;
        }
        fprintf(stderr, "[insrcv] unknown option '%s'\n", argv[i]);
        usage(argv[0]);
        return 2;
    }

    if (r.udp_port <= 0 || r.udp_port > 65535)
    {
        fprintf(stderr, "[insrcv] --udp-port must be 1..65535\n");
        return 2;
    }
    if (r.pub_hz <= 0.0) { r.pub_hz = 50.0; }
    if (r.raw_imu_hz <= 0.0) { r.raw_imu_hz = 100.0; }

    if (r.fi_enable)
    {
        if (!r.fi_have_lat || !r.fi_have_lon)
        {
            fprintf(stderr, "[insrcv] free_inertial_start needs lat_deg and lon_deg:"
                            " the whole point is the origin you supply.\n");
            return 2;
        }
        r.fi_armed = true;
        /* Developer mode, and it behaves like one: once the 3D filter is
           up it stays up, and the two mechanisms that would otherwise end
           it are both taken out of the loop.
             - max_deadreckoning_sec: coasting without absolute aiding is
               not a fault here, it IS the mode. A budget would stop the
               filter mid-experiment for doing exactly what it was asked.
             - the 3D exit gate: a run of poor-but-valid fixes (20 m
               indoors, a canyon, a tunnel mouth) makes ins conclude after
               gnss.stop_dwell_sec that its aiding degraded, and re-arm
               (REQ-NAV-052).
           What is NOT switched off: the 3D entry gate, and every fix is
           still judged on its own accuracy before it may fuse. GNSS keeps
           its power to CORRECT the solution, it just loses its power to
           end it. Whoever asks for free inertial navigation is asking to
           watch the drift, and a receiver deciding when the experiment is
           over defeats the point. */
        r.allow_unlimited_dr = true;
        r.stop_disable       = true;
        fprintf(stderr,
                "[insrcv] free-inertial start: declared %.7f %.7f h=%.1f m +-%g m\n"
                "[insrcv]   developer mode: the 3D filter runs until you stop it"
                " (no coasting budget, GNSS quality cannot end it)\n"
                "[insrcv]   heading is not observable without GNSS or a magnetometer:"
                " the track gets a shape, not a direction\n",
                r.fi_lat_deg, r.fi_lon_deg, r.fi_height_m, (double)r.fi_stddev_m);
        if (r.max_dr_sec > 0.0f)
        {
            fprintf(stderr, "[insrcv]   max_deadreckoning_sec %.0f s is ignored in this"
                            " mode\n",
                    (double)r.max_dr_sec);
        }
    }
    if (!r.gnss_enable)
    {
        fprintf(stderr, "[insrcv] gnss.enable 0: NAV-PVT/NAV-COV are decoded and"
                        " published, but nothing derived from them reaches the"
                        " filter\n");
    }
    if (!r.baro_enable)
    {
        fprintf(stderr, "[insrcv] baro.enable 0: pressure samples are decoded and"
                        " published, but the baro/accel vertical channel gets"
                        " none of them\n");
    }
    if (r.automotive_mode)
    {
        /* Said out loud because it is an assumption about the VEHICLE, not a
           tuning value: on anything that can move sideways or hover (a
           multicopter in wind, a boat with drift, a handheld sensor) course
           over ground is not heading, and the filter would be fusing a wrong
           yaw with full confidence. */
        fprintf(stderr, "[insrcv] automotive_mode: the GNSS course over ground is"
                        " fused as yaw above the minimum ground speed\n");
        if (!r.gnss_enable)
        {
            fprintf(stderr, "[insrcv]   gnss.enable is 0, so no course reaches the"
                            " filter and the mode does nothing\n");
        }
    }

    if (r.mav_port <= 0 || r.mav_port > 65535)
    {
        fprintf(stderr, "[insrcv] --mav-port must be 1..65535\n");
        return 2;
    }
    if (r.mav_on && !r.nav_on)
    {
        /* Not an error, but the combination does nothing: every MAVLink
           message here describes the solution, and --raw-only computes
           none. Better said out loud than debugged at the receiving
           end. */
        fprintf(stderr, "[insrcv] --mavlink carries the solution, and --raw-only computes"
                        " none: no MAVLink will be sent\n");
    }

    if (!net_init()) { return 1; }
    if (r.pj_on && !pj_open(&r.pj, r.pj_ip, r.pj_port))
    {
        net_cleanup();
        return 1;
    }
    if (r.mav_on && !mav_open(&r.mav, r.mav_ip, r.mav_port, r.mav_subfilter_hz))
    {
        pj_close(&r.pj);
        net_cleanup();
        return 1;
    }

    source_t src;
    if (!source_open_udp(&src, r.udp_bind, r.udp_port))
    {
        mav_close(&r.mav);
        pj_close(&r.pj);
        net_cleanup();
        return 1;
    }

    signal(SIGINT, on_sigint);

    fprintf(stderr, "[insrcv] listening on %s:%d -> PlotJuggler:%s (%s:%d)  filter:%s\n",
            r.udp_bind, r.udp_port, r.pj_on ? "on" : "off", r.pj_ip, r.pj_port,
            r.nav_on ? "on" : "raw-only");
    if (r.mav_on)
    {
        fprintf(stderr, "[insrcv] MAVLink 2 -> %s:%d (sysid 1, compid 1)\n", r.mav_ip,
                r.mav_port);
    }
    if (r.stats_sec > 0.0)
    {
        /* The status line below carries no field labels of its own: every
           character spent naming a field is a character closer to the
           wrap that makes the line unreadable. Named once here instead. */
        fprintf(stderr,
                "[insrcv] status line: sensor rates in Hz, then the fix (type, satellites,"
                " RTK)\n"
                "[insrcv]   |a| m/s2, alt m, |m| measured/model uT, counters only once"
                " they are not zero\n");
    }

    const double t_start     = host_now_sec();
    double       t_next_stat = t_start + r.stats_sec;
    uint8_t      chunk[READ_CHUNK];
    unsigned long long epochs = 0;

    while (!g_stop)
    {
        const long got = source_read(&src, chunk, sizeof(chunk));
        if (got < 0)
        {
            stats_line_break();
            fprintf(stderr, "[insrcv] UDP receive error, stopping\n");
            break;
        }
        if (got > 0)
        {
            r.n_datagrams++;
            /* A datagram never splits a frame (inslib_protocol.md), so a
               partial frame left over from the previous one is not a
               continuation: it is the tail of a datagram that was lost or
               reordered. Dropping it here stops the framer from splicing
               two unrelated datagrams into one bogus frame. */
            if (r.framer.len > 0)
            {
                r.framer.stats.resync_bytes += r.framer.len;
                r.framer.len = 0;
            }
            if ((size_t)got > sizeof(r.framer.buf))
            {
                /* Cannot happen with READ_CHUNK <= the buffer's slack, but
                   a wedged parser must not corrupt memory. */
                r.framer.stats.resync_bytes += (uint64_t)got;
                continue;
            }
            memcpy(r.framer.buf + r.framer.len, chunk, (size_t)got);
            r.framer.len += (size_t)got;

            for (;;)
            {
                uint8_t        cls, id;
                const uint8_t* payload;
                size_t         plen;
                const frame_result_t fr = framer_step(&r.framer, &cls, &id, &payload, &plen);
                if (fr == FRAME_NEED_MORE) { break; }
                if (fr == FRAME_RESYNC) { continue; }
                dispatch(&r, cls, id, payload, plen);
                framer_drop_front(&r.framer, 6 + plen + 2);
                epochs++;
            }
        }

        const double now = host_now_sec();
        if (r.stats_sec > 0.0 && now >= t_next_stat)
        {
            print_stats(&r, now - t_start);
            t_next_stat = now + r.stats_sec;
        }
    }

    print_stats(&r, host_now_sec() - t_start);
    stats_line_break(); /* keep the final status line, summary starts fresh below it */
    if (r.n_custom_unknown > 0 || r.n_bad_len > 0)
    {
        fprintf(stderr, "[insrcv] unknown custom msgs=%llu, bad-length payloads=%llu\n",
                (unsigned long long)r.n_custom_unknown, (unsigned long long)r.n_bad_len);
    }
    fprintf(stderr, "[insrcv] processed epochs=%llu from %llu datagrams\n",
            (unsigned long long)epochs, (unsigned long long)r.n_datagrams);
    /* The totals the status line deliberately does not repaint every
       second. Here they are worth having: what a session received in
       full is exactly the question a summary answers. */
    fprintf(stderr,
            "[insrcv] samples: imu=%llu@%.1fHz (device clock, drops=%llu) baro=%llu"
            " mag=%llu gnss=%llu (pvt=%llu, nofix=%llu) pps=%llu/%llu with GPS time\n",
            (unsigned long long)r.n_imu, r.imu_hz, (unsigned long long)r.imu_drops,
            (unsigned long long)r.n_baro, (unsigned long long)r.n_mag,
            (unsigned long long)r.n_gnss_ubx, (unsigned long long)r.n_gnss_pvt,
            (unsigned long long)r.n_gnss_nofix, (unsigned long long)r.n_pps_gps,
            (unsigned long long)r.n_pps);
    if (r.framer.stats.crc_errors > 0 || r.framer.stats.resync_bytes > 0
        || r.framer.stats.bad_length > 0)
    {
        fprintf(stderr, "[insrcv] framing: crc=%u resync=%lluB badlen=%u\n",
                (unsigned)r.framer.stats.crc_errors,
                (unsigned long long)r.framer.stats.resync_bytes,
                (unsigned)r.framer.stats.bad_length);
    }
    if (r.n_odo > 0)
    {
        fprintf(stderr,
                "[insrcv] odometry: %llu samples, %llu reverse, %llu fused at delay 0"
                " (stamped ahead), %llu dropped as out of range, %llu unstamped\n",
                (unsigned long long)r.n_odo, (unsigned long long)r.n_odo_reverse,
                (unsigned long long)r.n_odo_lead, (unsigned long long)r.n_odo_stale,
                (unsigned long long)r.n_odo_unstamped);
    }
    source_close(&src);
    mav_close(&r.mav);
    pj_close(&r.pj);
    net_cleanup();
    return 0;
}

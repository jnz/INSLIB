/** @file state_dump_shim.c
 * @brief Exact per-epoch state dump for the regression check r1.sh
 * (REQ-VER-051).
 *
 * Linked into tools/replay.c with -Wl,--wrap=nav_suite_update, so the replay
 * of any INSLIB version can be dumped without changing its source. After every
 * nav_suite_update() one line goes to the file named by INSLIB_STATE_DUMP:
 * the ins nominal state, the diagonal of the ins covariance, suite mode and
 * readiness, suite attitude and height. Every float is written in C99 hex
 * (%a), so two runs compare bit for bit with cmp.
 */
#include <stdio.h>
#include <stdlib.h>

#include "nav_suite.h"

void __real_nav_suite_update(nav_suite_t* s, const ins_measurements_t* m);
void __wrap_nav_suite_update(nav_suite_t* s, const ins_measurements_t* m);

static FILE* g_dump;
static int   g_dump_opened;

static void dump_open(void)
{
    g_dump_opened    = 1;
    const char* path = getenv("INSLIB_STATE_DUMP");
    if (path == NULL || path[0] == '\0') { return; }
    g_dump = fopen(path, "w");
    if (g_dump == NULL)
    {
        perror(path);
        exit(1);
    }
    fprintf(g_dump, "# t_us,mode,ins_init,ins_ready,lat,lon,h,pos_local[3],vel_ned[3],qbn[4],"
                    "acc_bias[3],gyr_bias[3],mag_bias[3],P_diag[n],suite_rpy[3],suite_h\n");
}

void __wrap_nav_suite_update(nav_suite_t* s, const ins_measurements_t* m)
{
    __real_nav_suite_update(s, m);
    if (!g_dump_opened) { dump_open(); }
    if (g_dump == NULL) { return; }

    const ins_t*           ins  = &s->ins;
    const nav_suite_mode_t mode = nav_suite_get_mode(s);
    int                    i, k;
    fprintf(g_dump, "%lld,%d,%d,%d", (long long)m->timestamp, (int)mode,
            ins->is_initialized ? 1 : 0, ins_is_ready(ins) ? 1 : 0);
    for (i = 0; i < 3; ++i) { fprintf(g_dump, ",%a", ins->latlonh[i]); }
    for (i = 0; i < 3; ++i) { fprintf(g_dump, ",%a", (double)ins->state.pos_local[i]); }
    for (i = 0; i < 3; ++i) { fprintf(g_dump, ",%a", (double)ins->state.vel_ned[i]); }
    for (i = 0; i < 4; ++i) { fprintf(g_dump, ",%a", (double)ins->state.qbn[i]); }
    for (i = 0; i < 3; ++i) { fprintf(g_dump, ",%a", (double)ins->state.acc_bias[i]); }
    for (i = 0; i < 3; ++i) { fprintf(g_dump, ",%a", (double)ins->state.gyr_bias[i]); }
    for (i = 0; i < 3; ++i) { fprintf(g_dump, ",%a", (double)ins->state.mag_bias[i]); }
    /* P_ii = d_i + sum over k > i of U(i,k)^2 d_k, U unit upper triangular,
       column-major */
    for (i = 0; i < ins->n; ++i)
    {
        float p = ins->d[i];
        for (k = i + 1; k < ins->n; ++k)
        {
            const float u = ins->U[i + k * ins->n];
            p += u * u * ins->d[k];
        }
        fprintf(g_dump, ",%a", (double)p);
    }
    float r = 0.0f, p = 0.0f, y = 0.0f, h = 0.0f;
    (void)nav_suite_get_rpy(s, &r, &p, &y);
    (void)nav_suite_get_height(s, &h);
    fprintf(g_dump, ",%a,%a,%a,%a\n", (double)r, (double)p, (double)y, (double)h);
}

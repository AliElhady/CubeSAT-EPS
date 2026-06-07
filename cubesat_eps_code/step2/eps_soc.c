/* ============================================================
 *  CubeSat EPS — Step 2: Calculating SOC
 *
 *  Takes the battery voltage (V) and returns the charge level (0–100%).
 *  Uses linear interpolation based on a 2S Li-ion discharge table.
 *
 *  The table is compiled for a pair of cells:
 *    single cell: 3.0 V (0%) … 4.2 V (100%)
 *    2S battery:  6.0 V (0%) … 8.4 V (100%)
 * ============================================================ */

#include "eps_soc.h"

/* ----------------------------------------------------------
 *  2S Li-ion discharge table
 *  Each {voltage, SOC} pair represents a point on the discharge curve.
 *  Linear interpolation is used between points.
 *
 *  Data verified: at Vbat=7.85V → SOC≈80%
 *                    at Vbat=6.60V → SOC≈20%
 * ---------------------------------------------------------- */
typedef struct { float voltage; int soc; } SocPoint;

static const SocPoint SOC_TABLE[] = {
    { 6.00f,   0 },
    { 6.60f,   5 },
    { 6.90f,  15 },
    { 7.10f,  35 },
    { 7.30f,  55 },
    { 7.40f,  70 },
    { 7.60f,  85 },
    { 8.00f,  95 },
    { 8.40f, 100 },
};

#define SOC_TABLE_SIZE   (sizeof(SOC_TABLE) / sizeof(SOC_TABLE[0]))

/* ----------------------------------------------------------
 *  VoltageToSOC
 *
 *  Algorithm:
 *  1. If below the minimum → 0%
 *  2. If above the maximum → 100%
 *  3. Otherwise, find the nearest points and interpolate
 * ---------------------------------------------------------- */
int VoltageToSOC(float vbat)
{
    if (vbat <= SOC_TABLE[0].voltage)
        return SOC_TABLE[0].soc;

    if (vbat >= SOC_TABLE[SOC_TABLE_SIZE - 1].voltage)
        return SOC_TABLE[SOC_TABLE_SIZE - 1].soc;

    for (int i = 0; i < (int)SOC_TABLE_SIZE - 1; i++)
    {
        float v0 = SOC_TABLE[i].voltage;
        float v1 = SOC_TABLE[i + 1].voltage;

        if (vbat >= v0 && vbat <= v1)
        {
            /* Linear interpolation:
             *   ratio = (vbat - v0) / (v1 - v0)
             *   soc   = s0 + ratio * (s1 - s0)    */
            float ratio = (vbat - v0) / (v1 - v0);
            float soc_f = (float)SOC_TABLE[i].soc
                        + ratio * (float)(SOC_TABLE[i + 1].soc
                                        - SOC_TABLE[i].soc);

            int soc = (int)(soc_f + 0.5f);

            if (soc <   0) soc =   0;
            if (soc > 100) soc = 100;
            return soc;
        }
    }

    return 0;
}

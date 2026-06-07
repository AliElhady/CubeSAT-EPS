#include "eps_soc.h"
typedef struct { float voltage; int soc; } SocPoint;

static const SocPoint SOC_TABLE[] = 
{
    { 6.00f, 0 },
    { 6.60f, 5 },
    { 6.90f, 15 },
    { 7.10f, 35 },
    { 7.30f, 55 },
    { 7.40f, 70 },
    { 7.60f, 85 },
    { 8.00f, 95 },
    { 8.40f, 100 },
};

#define SOC_TABLE_SIZE   (sizeof(SOC_TABLE) / sizeof(SOC_TABLE[0]))

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
            float ratio = (vbat - v0) / (v1 - v0);
            float soc_f = (float)SOC_TABLE[i].soc
                        + ratio * (float)(SOC_TABLE[i + 1].soc
                                        - SOC_TABLE[i].soc);

            int soc = (int)(soc_f + 0.5f);

            if (soc < 0) soc = 0;
            if (soc > 100) soc = 100;
            return soc;
        }
    }

    return 0;
}

#include "point_cloud_plane_params.h"

float m_sqrt(float x)
{
    float half_x = 0.5 * x;
    int i = *((int *)&x);
    i = 0x5f3759df - (i >> 1);
    x = *((float *)&i);
    x = x * (1.5 - (half_x * x * x));
    return 1 / x;
}
float getVar(float x[], int len)
{
    int m = len;
    float sum = 0;
    for (int i = 0; i < m; i++)
    {
        sum += x[i];
    }
    float dAve = sum / m;
    float dVar = 0;
    for (int i = 0; i < m; i++)
    {
        dVar += (x[i] - dAve) * (x[i] - dAve);
    }
    return dVar / m;
}
#include "sun.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int sun_event_min(bool rise, int yday, double lat, double lon, int tz_offset_min)
{
    if (lat == 0.0 && lon == 0.0) return -1;

    double g = 2.0 * M_PI / 365.0 * yday;
    double eqt = 229.18 * (0.000075 + 0.001868 * cos(g) - 0.032077 * sin(g) -
                           0.014615 * cos(2 * g) - 0.040849 * sin(2 * g));
    double decl = 0.006918 - 0.399912 * cos(g) + 0.070257 * sin(g) -
                  0.006758 * cos(2 * g) + 0.000907 * sin(2 * g) -
                  0.002697 * cos(3 * g) + 0.00148 * sin(3 * g);
    double latr = lat * M_PI / 180.0;
    double cosha = cos(90.833 * M_PI / 180.0) / (cos(latr) * cos(decl)) - tan(latr) * tan(decl);
    if (cosha > 1.0 || cosha < -1.0) return -1; /* polar day / night */

    double ha = acos(cosha) * 180.0 / M_PI;
    double utc = rise ? 720.0 - 4.0 * (lon + ha) - eqt : 720.0 - 4.0 * (lon - ha) - eqt;
    int loc = (int)lround(utc) + tz_offset_min;
    return ((loc % 1440) + 1440) % 1440;
}

#pragma once
#include <stdbool.h>

/* Sunrise/sunset (NOAA approximation).
 * yday: 0-based day of year (struct tm tm_yday); lon east-positive.
 * Returns local minutes after midnight, or -1 if unavailable
 * (lat/lon both 0 = not configured, or polar day/night). */
int sun_event_min(bool rise, int yday, double lat, double lon, int tz_offset_min);

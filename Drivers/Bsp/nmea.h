#ifndef _NMEA_H
#define _NMEA_H

/*
 *  NMEA GPS Protocol Parser
 *  Simplified version - Protocol parsing only
 *  Supports GGA frames
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* NMEA parse error codes */
#define NMEA_OK                    0   /* Success */
#define NMEA_ERR_INVALID_INPUT    -1   /* NULL pointer or invalid buffer */
#define NMEA_ERR_SHORT_LENGTH     -2   /* Buffer length too short */
#define NMEA_ERR_FORMAT           -3   /* Invalid format - $GNGGA not found */
#define NMEA_ERR_NO_CHECKSUM      -4   /* No checksum delimiter (*) found */
#define NMEA_ERR_CHECKSUM_FAIL    -5   /* Checksum validation failed */
#define NMEA_ERR_NO_FIX           -6   /* No valid fix (fix_quality = 0) */

typedef struct
{
  uint8_t  satellite;
  uint8_t  time_h;
  uint8_t  time_m;
  uint8_t  time_s;
  uint8_t  date_y;
  uint8_t  date_m;
  uint8_t  date_d;
  float    latitude_deg;
  float    longitude_deg;
  float    precision_m;
  float    altitude_m;
  float    speed_knots;
  float    course_deg;

  /* Valid flags */
  uint8_t  fix_quality;      /* 0=invalid, 1=GPS fix, 2=DGPS fix, etc */
  uint8_t  valid_satellite;
  uint8_t  valid_time;
  uint8_t  valid_date;
  uint8_t  valid_latitude;
  uint8_t  valid_longitude;
  uint8_t  valid_precision;
  uint8_t  valid_altitude;
  uint8_t  valid_speed;
  uint8_t  valid_course;

} nmea_gnss_t;

/* Global GNSS data structure */
extern nmea_gnss_t g_nmea_gnss;

/* API Functions */
uint8_t nmea_checksum(const uint8_t *sentence, uint16_t len);
float   nmea_convert(float raw_degrees);

/* Parse functions - return error code (0=success, negative=error) */
int nmea_parse(const uint8_t *buf, uint16_t len);
int nmea_parse_gngga(const uint8_t *buf, uint16_t len);

#endif

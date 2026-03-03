#include "nmea.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Global GNSS data structure */
nmea_gnss_t g_nmea_gnss = {0};

/**
 * Calculate NMEA checksum
 * @param sentence: pointer to NMEA sentence (without $)
 * @param len: length of sentence
 * @return: calculated checksum byte
 */
uint8_t nmea_checksum(const uint8_t *sentence, uint16_t len)
{
  uint8_t chk = 0;
  
  if (sentence == NULL || len == 0)
    return 0;
  
  for (uint16_t i = 0; i < len; i++)
  {
    uint8_t c = sentence[i];
    if (c == '*' || c == '\r' || c == '\n')
      break;
    chk ^= c;
  }
  
  return chk;
}

/**
 * Convert raw NMEA degrees to decimal degrees
 * @param raw_degrees: raw value (e.g., 4807.038 for 48°07.038')
 * @return: decimal degrees (e.g., 48.117300)
 */
float nmea_convert(float raw_degrees)
{
  int firstdigits = ((int)raw_degrees) / 100;
  float nexttwodigits = raw_degrees - (float)(firstdigits * 100.0f);
  return (float)(firstdigits + nexttwodigits / 60.0f);
}

/**
 * Parse GNGGA sentence (GGA is the most important for positioning)
 * Format: $GNGGA,hhmmss.ss,ddmm.mmmm,a,dddmm.mmmm,a,x,xx,x.x,x.x,M,x.x,M,,*hh
 * 
 * @param buf: pointer to NMEA sentence buffer
 * @param len: length of buffer
 * @return: NMEA_OK (0) if success, negative error code on failure
 */
int nmea_parse_gngga(const uint8_t *buf, uint16_t len)
{
  if (buf == NULL || len < 20)
    return NMEA_ERR_INVALID_INPUT;

  char line[256];
  if (len > (uint16_t)(sizeof(line) - 1))
    return NMEA_ERR_SHORT_LENGTH;

  /* Copy to local buffer for processing */
  memcpy(line, buf, len);
  line[len] = '\0';

  /* Find $GNGGA */
  char *found = strstr(line, "GNGGA");
  if (found == NULL)
    return NMEA_ERR_FORMAT;

  /* Check for $ before GNGGA */
  found -= 1;
  if (found[0] != '$')
    return NMEA_ERR_FORMAT;

  /* Find checksum delimiter */
  char *checksum_pos = strchr(found, '*');
  if (checksum_pos == NULL)
    return NMEA_ERR_NO_CHECKSUM;

  /* Calculate and verify checksum */
  uint8_t c = nmea_checksum((const uint8_t*)(found + 1), (uint16_t)(checksum_pos - found - 1));
  uint8_t cc = 0;
  sscanf(checksum_pos + 1, "%02X", (unsigned int*)&cc);
  
  if (cc != c)
    return NMEA_ERR_CHECKSUM_FAIL;

  /* Parse tokens */
  char *saveptr = NULL;
  char temp_line[256];
  memcpy(temp_line, found, checksum_pos - found);
  temp_line[checksum_pos - found] = '\0';

  char *str = strtok_r(temp_line, ",*", &saveptr);
  uint8_t index = 0;
  
  memset(&g_nmea_gnss, 0, sizeof(nmea_gnss_t));

  while (str != NULL && index < 15)
  {
    switch (index)
    {
      case 0: /* $GNGGA */
        break;

      case 1: /* UTC Time (hhmmss.ss) */
        if (strlen(str) >= 6)
        {
          g_nmea_gnss.time_h = (uint8_t)((str[0] - '0') * 10 + (str[1] - '0'));
          g_nmea_gnss.time_m = (uint8_t)((str[2] - '0') * 10 + (str[3] - '0'));
          g_nmea_gnss.time_s = (uint8_t)((str[4] - '0') * 10 + (str[5] - '0'));
          g_nmea_gnss.valid_time = 1;
        }
        break;

      case 2: /* Latitude (ddmm.mmmm) */
        if (strlen(str) > 0)
        {
          float lat = atof(str);
          g_nmea_gnss.latitude_deg = nmea_convert(lat);
          g_nmea_gnss.valid_latitude = 1;
        }
        break;

      case 3: /* Latitude direction (N/S) */
        if (str[0] == 'S')
          g_nmea_gnss.latitude_deg = -g_nmea_gnss.latitude_deg;
        break;

      case 4: /* Longitude (dddmm.mmmm) */
        if (strlen(str) > 0)
        {
          float lon = atof(str);
          g_nmea_gnss.longitude_deg = nmea_convert(lon);
          g_nmea_gnss.valid_longitude = 1;
        }
        break;

      case 5: /* Longitude direction (E/W) */
        if (str[0] == 'W')
          g_nmea_gnss.longitude_deg = -g_nmea_gnss.longitude_deg;
        break;

      case 6: /* Fix quality */
        g_nmea_gnss.fix_quality = (uint8_t)atoi(str);
        break;

      case 7: /* Number of satellites */
        g_nmea_gnss.satellite = (uint8_t)atoi(str);
        g_nmea_gnss.valid_satellite = (g_nmea_gnss.satellite > 0) ? 1 : 0;
        break;

      case 8: /* Horizontal Dilution of Precision (HDOP) */
        g_nmea_gnss.precision_m = atof(str);
        g_nmea_gnss.valid_precision = 1;
        break;

      case 9: /* Altitude above mean sea level */
        g_nmea_gnss.altitude_m = atof(str);
        g_nmea_gnss.valid_altitude = 1;
        break;

      default:
        break;
    }

    str = strtok_r(NULL, ",*", &saveptr);
    index++;
  }

  /* Return error if no valid fix */
  if (g_nmea_gnss.fix_quality == 0)
    return NMEA_ERR_NO_FIX;

  return NMEA_OK;
}

/**
 * Generic NMEA sentence parser
 * Dispatches to specific sentence parsers based on type
 * 
 * @param buf: pointer to NMEA sentence buffer
 * @param len: length of buffer
 * @return: NMEA_OK (0) if success, negative error code on failure
 */
int nmea_parse(const uint8_t *buf, uint16_t len)
{
  if (buf == NULL || len < 20)
    return NMEA_ERR_INVALID_INPUT;

  /* Check for GGA sentence */
  if (NULL != (char*)memmem(buf, len, (const uint8_t*)"GGA", 3))
  {
    return nmea_parse_gngga(buf, len);
  }

  /* Add other sentence types here in the future */
  
  return NMEA_ERR_FORMAT;
}

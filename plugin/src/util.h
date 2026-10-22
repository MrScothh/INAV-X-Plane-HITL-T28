#pragma once

#include "config.h"

#define CONST_PI 3.14159265358979323846

extern void setView();

extern void buildAssetFilename(char pName[MAX_PATH], const char* pFileName);
extern void playSound(const char* pFileName);

extern void LOG(const char* fmt, ...);

extern float latDistanceM(double lat1, double lon1, double elev1, double lat2, double lon2, double elev2);

extern void disableBrakes();
extern void setView();

extern float clampf(float value, float minValue, float maxValue);
extern int16_t clampToInt16(float value);

extern void delayMS(uint32_t valueMS);

// GetTickCount() moves in 15.6 ms steps on Windows: too coarse to pace the frames sent to INAV
extern uint64_t microsNow();

extern void getClipboardText(char str[1024]);

extern bool saveIniFile();

extern int smallestPowerOfTwo(int value, int minValue);

extern std::vector<std::filesystem::path> getFontPaths(const char* subPath, bool directories);

extern std::string toLower(const std::string& str);

extern bool validateIpAddress(const std::string ipAddress);

#if LIN || APL
extern bool IsDebuggerPresent();
extern uint32_t GetTickCount();
#endif

#define SETTINGS_SECTION                    "settings"
#define SETTINGS_GPS_NUMSAT                 "gps_numSat"
#define SETTINGS_GPS_TIMEOUT                "gps_timeout"
#define SETTINGS_GPS_GLITCH                 "gps_glitch"
#define SETTINGS_MAG_FAILURE                "mag_failure"
#define SETTINGS_ATTITUDE_USE_SENSORS       "attitude_use_sensors"
#define SETTINGS_OSD_TYPE                   "osd_type"
#define SETTINGS_OSD_SMOOTHED               "osd_smoothed"
#define SETTINGS_BATTERY_EMULATION          "battery_emulation"
#define SETTINGS_MUTE_BEEPER                "mute_beeper"
#define SETTINGS_FIX_NOON                   "fix_noon"
#define SETTINGS_SENSOR_NOISE               "sensor_noise"
#define SETTINGS_MAG_CURRENT                "mag_current"
#define SETTINGS_BARO_DYNAMIC               "baro_dynamic"
#define SETTINGS_FAST_LINK                  "fast_link"
#define SETTINGS_ESC_MODEL                  "esc_model"
#define SETTINGS_GEAR_SERVO                 "gear_servo"
#define SETTINGS_FLAP_SERVO                 "flap_servo"
#define SETTINGS_FLAP_SERVO_UP              "flap_servo_up"
#define SETTINGS_FLAP_SERVO_FULL            "flap_servo_full"
#define SETTINGS_SIMULATE_PITOT             "simulate_pitot"
#define SETTINGS_SIMULATE_PITOT_FAILURE_HW  "simulate_pitot_failure"
#define SETTINGS_SIMULATE_PITOT_FAILURE_60  "simulate_pitot_failure_60"
#define SETTINGS_GRAPH_TYPE                 "graph_type"
#define SETTINGS_MAP_MARKING_TYPE           "map_markings_type"
#define SETTINGS_VIDEOLINK_SIMULATION       "videolink_simulation"
#define SETTINGS_ANALOG_OSD_FONT            "analog_osd_font"
#define SETTINGS_DIGITAL_OSD_FONT           "digital_osd_font"
#define SETTINGS_SITL_IP                    "sitl_ip"


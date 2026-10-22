#include "simData.h"
#include "stats.h"
#include "graph.h"
#include "util.h"
#include "osd.h"
#include "map.h"

#include <math.h>
#include <random>

TSimData g_simData;

// u-blox M10/F10 as the T-28 runs them
#define GPS_RATE_HZ   10

#define DEG2RAD         (.017453292519943f) /* PI / 180 */
#define DECIDEGREES_TO_RADIANS(angle) (((angle) / 10.0f) * DEG2RAD)

//==============================================================
//==============================================================
void TSimData::init()
{
  this->gps_lastUpdate = microsNow() - 1000000;
  this->GPSHasNewData = false;

  this->gps_fix = GPS_FIX_3D;
  this->gps_numSat = 12;
  this->gps_glitch = GPS_GLITCH_NONE;
  this->gps_timeout = false;

  this->simulate_mag_failure = false;

  this->df_lattitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
  this->df_longitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
  this->df_elevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
  this->df_zulu_time = XPLMFindDataRef("sim/time/zulu_time_sec");
  this->fixNoon = true;
  this->sensorNoise = false;
  this->magCurrent = false;
  this->baroDynamic = false;
  this->fastLink = false;
  this->escModel = true;
  this->escActive = false;
  this->escCurrent = 0.0f;
  this->isT28 = false;
  this->gearServo = -1;
  this->flapServo = -1;
  this->flapServoUp = 1000;
  this->flapServoFull = 2000;
  this->servoPollAt = 0;
  this->df_gear_handle = XPLMFindDataRef("sim/cockpit2/controls/gear_handle_down");
  this->df_flap_ratio = XPLMFindDataRef("sim/cockpit2/controls/flap_ratio");
  this->controlHead = 0;
  this->controlCount = 0;
  this->df_override_torque = XPLMFindDataRef("sim/operation/override/override_torque_motors");
  this->df_engn_trq = XPLMFindDataRef("sim/flightmodel/engine/ENGN_TRQ");
  this->df_prop_speed = XPLMFindDataRef("sim/flightmodel/engine/POINT_tacrad");
  this->df_throttle_handle = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio");
  this->df_tailnum = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");
  this->df_rho = XPLMFindDataRef("sim/weather/rho");
  this->df_faxil_plug = XPLMFindDataRef("sim/flightmodel/forces/faxil_plug_acf");
  this->df_jzz = XPLMFindDataRef("sim/aircraft/weight/acf_Jzz_unitmass");
  this->df_magvar = XPLMFindDataRef("sim/flightmodel/position/magnetic_variation");
  //this->df_agl = XPLMFindDataRef("sim/flightmodel/position/y_agl");

  this->df_local_vx = XPLMFindDataRef("sim/flightmodel/position/local_vx");
  this->df_local_vy = XPLMFindDataRef("sim/flightmodel/position/local_vy");
  this->df_local_vz = XPLMFindDataRef("sim/flightmodel/position/local_vz");

  this->df_speed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
  this->df_airspeed = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");

  this->df_roll = XPLMFindDataRef("sim/flightmodel/position/phi");
  this->df_pitch = XPLMFindDataRef("sim/flightmodel/position/theta");
  this->df_yaw = XPLMFindDataRef("sim/flightmodel/position/psi");
  this->df_hpath = XPLMFindDataRef("sim/flightmodel/position/hpath");

	// Accelerometer
  this->df_accel_x = XPLMFindDataRef("sim/flightmodel/forces/g_axil");
  this->df_accel_y = XPLMFindDataRef("sim/flightmodel/forces/g_side");
  this->df_accel_z = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");

	// Gyro
  this->df_gyro_x = XPLMFindDataRef("sim/flightmodel/position/P");
  this->df_gyro_y = XPLMFindDataRef("sim/flightmodel/position/Q");
  this->df_gyro_z = XPLMFindDataRef("sim/flightmodel/position/R");

	// Barometer
  this->df_baro = XPLMFindDataRef("sim/weather/barometer_current_inhg");

	// // Velocity
	// xPlane.requestDataRef('sim/flightmodel/forces/vx_acf_axis', 10, function (ref, value) {
	//     simData.velocity_x = value; isSimDataUpdated = true;
	// });
	// xPlane.requestDataRef('sim/flightmodel/forces/vy_acf_axis', 10, function (ref, value) {
	//     simData.velocity_y = value; isSimDataUpdated = true;
	// });
	// xPlane.requestDataRef('sim/flightmodel/forces/vz_acf_axis', 10, function (ref, value) {
	//     simData.velocity_z = value; isSimDataUpdated = true;
	// });
	// xPlane.setDataRef('sim/operation/override/override_control_surfaces', 1);

	//static XPLMDataRef wrt = XPLMFindDataRef("sim/graphics/view/world_render_type");

  this->estimated_attitude_roll = 0;
  this->estimated_attitude_pitch = 0;
  this->estimated_attitude_yaw = 0;

  this->simulatePitot = true;
  this->simulatePitotFailureHW = false;
  this->simulatePitotFailure60 = false;
  this->airspeed = 0;

	//---- output ----
  this->muteBeeper = true;
  this->attitude_use_sensors = false;

  this->lastUpdateMS = 0;

  this->df_override_joystick = XPLMFindDataRef("sim/operation/override/override_joystick");
  this->df_control_throttle = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");
  this->df_control_roll = XPLMFindDataRef("sim/joystick/yoke_roll_ratio");
  this->df_control_pitch = XPLMFindDataRef("sim/joystick/yoke_pitch_ratio");
  this->df_control_yaw = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

  this->isAirplane = false;
  this->isArmed = false;
  this->isOSDDisabled = false;
  this->isSupportedOSDNotFound = false;

  this->setBateryEmulation(BATTERY_INFINITE);
}

//==============================================================
//==============================================================
float TSimData::gauss(float sigma)
{
  static std::mt19937 rng(12345);
  std::normal_distribution<float> n(0.0f, sigma);
  return n(rng);
}

//==============================================================
//==============================================================
void TSimData::keepNoon()
{
  if (!this->fixNoon)
  {
    return;
  }

  // solar noon over the aircraft's longitude, in UTC seconds of the day
  const double lon = XPLMGetDatad(this->df_longitude);
  const float noon = (float)fmod(12.0 * 3600.0 - lon / 15.0 * 3600.0 + 86400.0, 86400.0);
  float diff = fabsf(XPLMGetDataf(this->df_zulu_time) - noon);
  diff = fminf(diff, 86400.0f - diff);
  // five minutes of sun travel are invisible; a jump back every five minutes is not worth a per-frame write
  if (diff > 300.0f)
  {
    XPLMSetDataf(this->df_zulu_time, noon);
  }
}

//==============================================================
//==============================================================
// Brushless motor and ESC of the T-28 (EFLM4115 850 Kv, Avian Lite 70 A; R and I0 fitted to its logs with the
// manual's 12x7 prop, t28/pick_esc.py): duty = throttle, motor current (duty Vb - w / Kv) / R, so the ESC brakes the
// prop down to the commanded speed, throttle 0 included. The 4S sags under load but never runs down
#define ESC_KV_RAD      (850.0f * 2.0f * (float)CONST_PI / 60.0f)
#define ESC_R_OHM       0.07f
#define ESC_I0_A        3.0f
#define ESC_VOC         16.6f
#define ESC_RBATT_OHM   0.023f
#define ESC_I_MAX_A     75.0f
#define ESC_I_BRAKE_A   60.0f
#define ESC_I_BASE_A    1.9f
#define T28_JZZ_UNITMASS 0.09f
// a braked prop slows the air more than X-Plane's blade model gives: the logs' glide at zero throttle wants more drag
// between 14 and 24 m/s, constant as a drag area (0.005 m2 gave 60 % of the gap), full once the ESC brakes by 0.05 N m
#define ESC_BRAKED_PROP_CDA_M2  0.0085f
#define ESC_BRAKE_FULL_NM       0.05f

void TSimData::updateMotor()
{
  char tail[41] = { 0 };
  if (this->df_tailnum != NULL)
  {
    XPLMGetDatab(this->df_tailnum, tail, 0, 40);
  }
  this->isT28 = (strncmp(tail, "T28RC", 5) == 0);
  if (this->isT28)
  {
    // X-Plane recomputes the moments of inertia from the drawn shape at every load, whatever the ACF says, and gets
    // the yaw one at a quarter of a near-planar airframe's (Jzz ~ Jxx + Jyy): 0.02 m2/kg against 0.09
    if (fabsf(XPLMGetDataf(this->df_jzz) - T28_JZZ_UNITMASS) > 0.001f)
    {
      XPLMSetDataf(this->df_jzz, T28_JZZ_UNITMASS);
    }
  }
  // the fit belongs to the T-28's motor: any other aircraft keeps X-Plane's
  const bool want = this->escModel && this->isT28;
  if (!want)
  {
    if (this->escActive)
    {
      XPLMSetDatai(this->df_override_torque, 0);
      this->escActive = false;
    }
    return;
  }
  if (!this->escActive)
  {
    XPLMSetDatai(this->df_override_torque, 1);
    this->escActive = true;
  }

  float duty = 0.0f;
  float w = 0.0f;
  XPLMGetDatavf(this->df_throttle_handle, &duty, 0, 1);
  XPLMGetDatavf(this->df_prop_speed, &w, 0, 1);
  duty = clampf(duty, 0.0f, 1.0f);
  const float vb = ESC_VOC - ESC_RBATT_OHM * this->escCurrent;
  const float im = clampf((duty * vb - w / ESC_KV_RAD) / ESC_R_OHM, -ESC_I_BRAKE_A, ESC_I_MAX_A);
  // bearing and iron losses, faded in over the first few rad/s so a stopped prop does not chatter
  float q = (im - ESC_I0_A * w / (fabsf(w) + 10.0f)) / ESC_KV_RAD;
  XPLMSetDatavf(this->df_engn_trq, &q, 0, 1);
  const float braking = clampf(-q / ESC_BRAKE_FULL_NM, 0.0f, 1.0f);
  if (braking > 0.0f)
  {
    const float v = XPLMGetDataf(this->df_airspeed);
    const float drag = ESC_BRAKED_PROP_CDA_M2 * 0.5f * XPLMGetDataf(this->df_rho) * v * v * braking;
    XPLMSetDataf(this->df_faxil_plug, XPLMGetDataf(this->df_faxil_plug) + drag);
  }
  // the logged current never goes below the electronics: the sensor does not read regeneration
  this->escCurrent = fmaxf(0.0f, duty * im) + ESC_I_BASE_A;
}

//==============================================================
//==============================================================
// every 200 ms: one extra MSP command, within the stock firmware's budget on the safe link rate
void TSimData::pollServos()
{
  if (this->gearServo < 0 && this->flapServo < 0)
  {
    return;
  }
  const uint64_t now = microsNow();
  if (now - this->servoPollAt < 200000)
  {
    return;
  }
  this->servoPollAt = now;
  g_msp.sendCommand(MSP_SERVO, NULL, 0);
}

void TSimData::onServos(const uint16_t* servos, int count)
{
  if (this->gearServo >= 0 && this->gearServo < count)
  {
    // INAV's gear output: 1000 up, 1500 or 2000 down. The handle is an int dataref: the float calls read 0 and
    // write nothing
    XPLMSetDatai(this->df_gear_handle, servos[this->gearServo] > 1250 ? 1 : 0);
  }
  if (this->flapServo >= 0 && this->flapServo < count)
  {
    const float span = (float)(this->flapServoFull - this->flapServoUp);
    if (fabsf(span) >= 1.0f)
    {
      XPLMSetDataf(this->df_flap_ratio, clampf((servos[this->flapServo] - this->flapServoUp) / span, 0.0f, 1.0f));
    }
  }
}

//==============================================================
//==============================================================
void TSimData::updateFromXPlane()
{
  const uint64_t t = microsNow();

  if ((t - this->gps_lastUpdate) >= (1000000 / GPS_RATE_HZ))
  {
    // stepped by the period, so the rate stays at GPS_RATE_HZ whatever X-Plane's frame period
    this->gps_lastUpdate += 1000000 / GPS_RATE_HZ;
    if ((t - this->gps_lastUpdate) >= (1000000 / GPS_RATE_HZ))
    {
      this->gps_lastUpdate = t;
    }
    this->GPSHasNewData = true;

    this->lattitude = XPLMGetDatad(this->df_lattitude);
    this->longitude = XPLMGetDatad(this->df_longitude);
    this->elevation = XPLMGetDatad(this->df_elevation);
    //this->agl = XPLMGetDataf(this->df_agl);

    this->local_vx = XPLMGetDataf(this->df_local_vx);
    this->local_vy = XPLMGetDataf(this->df_local_vy);
    this->local_vz = XPLMGetDataf(this->df_local_vz);

    this->speed = XPLMGetDataf(this->df_speed);
    this->airspeed = XPLMGetDataf(this->df_airspeed);
    this->course = XPLMGetDataf(this->df_hpath);
  }
  
  this->roll = XPLMGetDataf(this->df_roll);
  this->pitch = XPLMGetDataf(this->df_pitch);
  this->yaw = XPLMGetDataf(this->df_yaw);

  float kick = 0;
  if (this->autolaunch_kickStart != 0)
  {
    uint32_t t = GetTickCount();
    uint32_t dt = t - this->autolaunch_kickStart;
    if (dt > 1000)
    {
      this->autolaunch_kickStart = 0;
    }
    else
    {
      kick = 4 * sin(dt / 180.0f * 3.14f);
    }
  }
  this->accel_x = XPLMGetDataf(this->df_accel_x) + kick;
  this->accel_y = XPLMGetDataf(this->df_accel_y);
  this->accel_z = XPLMGetDataf(this->df_accel_z);

  this->gyro_x = XPLMGetDataf(this->df_gyro_x);
  this->gyro_y = XPLMGetDataf(this->df_gyro_y);
  this->gyro_z = XPLMGetDataf(this->df_gyro_z);

  this->baro = XPLMGetDataf(this->df_baro);
}

//==============================================================
//==============================================================
// The controls X-Plane gets each frame are interpolated between the two commands around that long ago, so the delay
// is exact whatever the frame rate. Fitted per axis to the T-28 logs' command-to-response times (servo_response.py,
// measured: real 50 ms roll, 46 pitch over 15 steps each; HITL with no delay 35/45, the link's own latency standing in
// for the 50 Hz PWM frame). Writable as the datarefs inav_hitl/servo_dead_time_us (pitch, yaw, throttle) and
// inav_hitl/servo_dead_time_roll_us
static int servoDeadTimeUs = 2000;
static int servoDeadTimeRollUs = 18000;

static int getServoDeadTime(void * inRefcon)
{
  return *(int *)inRefcon;
}

static void setServoDeadTime(void * inRefcon, int inValue)
{
  *(int *)inRefcon = inValue < 0 ? 0 : (inValue > 100000 ? 100000 : inValue);
}

void TSimData::registerDatarefs()
{
  this->dr_servo_dead_time = XPLMRegisterDataAccessor("inav_hitl/servo_dead_time_us", xplmType_Int, 1,
    getServoDeadTime, setServoDeadTime, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &servoDeadTimeUs, &servoDeadTimeUs);
  this->dr_servo_dead_time_roll = XPLMRegisterDataAccessor("inav_hitl/servo_dead_time_roll_us", xplmType_Int, 1,
    getServoDeadTime, setServoDeadTime, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &servoDeadTimeRollUs, &servoDeadTimeRollUs);
}

void TSimData::unregisterDatarefs()
{
  XPLMUnregisterDataAccessor(this->dr_servo_dead_time);
  XPLMUnregisterDataAccessor(this->dr_servo_dead_time_roll);
}

void TSimData::applyControls(const TControls& c)
{
  XPLMSetDatai(this->df_override_joystick, 1);
  XPLMSetDataf(this->df_control_throttle, (c.throttle + 500.0f) / 1000.0f);
  XPLMSetDataf(this->df_control_roll, c.roll / 500.0f);
  XPLMSetDataf(this->df_control_pitch, -(c.pitch / 500.0f));
  XPLMSetDataf(this->df_control_yaw, -(c.yaw / 500.0f));
}

void TSimData::sendToXPlane()
{
  const TControls c = { microsNow(), this->control_throttle, this->control_roll, this->control_pitch, this->control_yaw };
  if (!this->isT28)
  {
    this->controlCount = 0;
    this->applyControls(c);
    return;
  }
  if (this->controlCount == 32)
  {
    this->controlHead = (this->controlHead + 1) % 32;
    this->controlCount--;
  }
  this->controlQueue[(this->controlHead + this->controlCount) % 32] = c;
  this->controlCount++;
}

// the command as it was deadUs ago: interpolated between the two around that instant, the last one when none is newer
float TSimData::delayedControl(uint64_t now, int deadUs, int16_t TControls::*field, int& oldest)
{
  int due = -1;
  for (int i = 0; i < this->controlCount; i++)
  {
    if (now - this->controlQueue[(this->controlHead + i) % 32].t < (uint64_t)deadUs)
    {
      break;
    }
    due = i;
  }
  if (due < 0)
  {
    return NAN;
  }
  if (due < oldest)
  {
    oldest = due;
  }
  const TControls& c0 = this->controlQueue[(this->controlHead + due) % 32];
  if (due + 1 >= this->controlCount)
  {
    return c0.*field;
  }
  const TControls& c1 = this->controlQueue[(this->controlHead + due + 1) % 32];
  const float age0 = (float)(now - c0.t);
  const float age1 = (float)(now - c1.t);
  const float f = (age0 - deadUs) / fmaxf(1.0f, age0 - age1);
  return c0.*field + f * (c1.*field - c0.*field);
}

// every frame
void TSimData::applyDelayedControls()
{
  const uint64_t now = microsNow();
  int oldest = 32;
  const float roll = this->delayedControl(now, servoDeadTimeRollUs, &TControls::roll, oldest);
  const float pitch = this->delayedControl(now, servoDeadTimeUs, &TControls::pitch, oldest);
  const float yaw = this->delayedControl(now, servoDeadTimeUs, &TControls::yaw, oldest);
  const float throttle = this->delayedControl(now, servoDeadTimeUs, &TControls::throttle, oldest);
  if (isnan(roll) || isnan(pitch))
  {
    return;
  }
  const TControls c = { now, (int16_t)lroundf(throttle), (int16_t)lroundf(roll), (int16_t)lroundf(pitch), (int16_t)lroundf(yaw) };
  this->applyControls(c);
  // commands older than the oldest bracket still in use are done with; the newest one, once applied as is, too:
  // kept, it was written again every frame after the link dropped, over anything else driving the aircraft
  if (oldest >= this->controlCount - 1)
  {
    this->controlCount = 0;
  }
  else if (oldest > 0 && oldest < 32)
  {
    this->controlHead = (this->controlHead + oldest) % 32;
    this->controlCount -= oldest;
  }
}

//==============================================================
//==============================================================
void TSimData::updateFromINAV(const TMSPSimulatorFromINAV* data)
{
  this->control_throttle = data->throttle;
  this->control_roll = data->roll;
  this->control_pitch = data->pitch;
  this->control_yaw = data->yaw;

  this->isAirplane = (data->debugIndex & FIF_IS_AIRPLANE) != 0;
  bool prevArmed = this->isArmed;
  this->isArmed = (data->debugIndex & FIF_ARMED) != 0;
  this->isOSDDisabled = (data->debugIndex & FIF_OSD_DISABLED) != 0;
  this->isSupportedOSDNotFound = (data->debugIndex & FIF_ANALOG_OSD_NOT_FOUND) != 0;

  if (this->isArmed && !prevArmed)
  {
    disableBrakes();
    g_osd.setHome();
  }

  g_stats.debug[data->debugIndex & 7] = data->debugValue;

  this->estimated_attitude_roll = (int16_t)data->estimated_attitude_roll;
  this->estimated_attitude_pitch = (int16_t)data->estimated_attitude_pitch;
  this->estimated_attitude_yaw = (uint16_t)data->estimated_attitude_yaw;
  g_graph.addEstimatedAttitudeYPR(estimated_attitude_yaw, estimated_attitude_pitch, estimated_attitude_roll);

  g_graph.addDebug(data->debugIndex & 7, (float)data->debugValue);
  g_graph.addOutputYPR(this->control_yaw, this->control_pitch, this->control_roll);

  if ((data->debugIndex & 7 ) == 7)
  {
    g_map.addDebug(g_stats.debug[0], g_stats.debug[1]);
  }

  uint32_t t = GetTickCount();
  uint32_t delta = t - this->lastUpdateMS;
  if ((this->lastUpdateMS != 0) && (delta < 300))
  {
    g_graph.addUpdatePeriodMS(delta);
  }
  this->lastUpdateMS = t;
}

//==============================================================
//==============================================================
void TSimData::sendToINAV()
{
  TMSPSimulatorToINAV data;

  data.version = MSP_SIMULATOR_VERSION;

  data.flags = SIMU_ENABLE |
    ((this->batEmulation != BATTERY_NONE) ? SIMU_SIMULATE_BATTERY : 0) |
    (this->muteBeeper ? SIMU_MUTE_BEEPER : 0) |
    (this->attitude_use_sensors ? SIMU_USE_SENSORS : 0) |
    (this->GPSHasNewData && !this->gps_timeout ? SIMU_HAS_NEW_GPS_DATA : 0) |
    SIMU_EXT_BATTERY_VOLTAGE |
    (this->simulatePitot ? SIMU_AIRSPEED : 0) |
    SIMU_EXTENDED_FLAGS;

  data.flags2 = (this->gps_timeout ? SIMU2_GPS_TIMEOUT >> 8 : 0) |
    (this->simulatePitotFailureHW ? SIMU2_PITOT_FAILURE >> 8 : 0);

  this->GPSHasNewData = false;

  data.fix = this->gps_fix;
  data.numSat = (uint8_t)this->gps_numSat;
  data.lat = (int32_t)round(this->lattitude * 10000000);
  data.lon = (int32_t)round(this->longitude * 10000000);      
  data.alt = (int32_t)round(this->elevation * 100); //expected by inav: elevation in cm
  data.speed = (int16_t)round(this->speed * 100); //expected by inav: ground speed cm/sec   
  data.airspeed = (uint16_t)round(this->airspeed * 100); //expected by inav: ground speed cm/sec
  if (this->simulatePitotFailure60)
  {
    data.airspeed = (uint16_t)round(1777); 
  }
  data.course = (int16_t)round(this->course * 10);  // expected by inav: deg * 10
  if (data.course < 0) data.course += 3600;

  data.velNED[0] = (int16_t)round(-this->local_vz*100); // nedVelNorth;  
  data.velNED[1] = (int16_t)round( this->local_vx*100); //nedVelEast
  data.velNED[2] = (int16_t)round(-this->local_vy*100); //nedVelDown

  if (this->gps_glitch != GPS_GLITCH_NONE)
  {
    if (this->glitch_lattitude == 0)
    {
      this->glitch_lattitude = this->lattitude;
      this->glitch_longitude = this->longitude;
      this->glitch_elevation = this->elevation;
    }
  }
  else
  {
    this->glitch_lattitude = 0;
    this->glitch_longitude = 0;
    this->glitch_elevation = 0;
  }

  if (this->gps_glitch == GPS_GLITCH_FREEZE)
  {
    data.lat = (int32_t)round(this->glitch_lattitude * 10000000);
    data.lon = (int32_t)round(this->glitch_longitude * 10000000);
    data.alt = (int32_t)round(this->glitch_elevation * 100);
    data.speed = 0;
    data.velNED[0] = 0;
    data.velNED[1] = 0;
    data.velNED[2] = 0;
  }
  else if (this->gps_glitch == GPS_GLITCH_OFFSET)
  {
    data.lat = (int32_t)round((this->lattitude + 5 / 111.32) * 10000000);
    data.lon = (int32_t)round(this->longitude * 10000000);
    data.alt = (int32_t)round(this->elevation * 100);
  }
  else if (this->gps_glitch == GPS_GLITCH_LINEAR)
  {
    float k = GetTickCount() / 100000.0f;
    k -= (int)k;
    data.lat = (int32_t)round((this->lattitude + 1 * k / 111.32) * 10000000);
    data.lon = (int32_t)round(this->longitude * 10000000);
    data.alt = (int32_t)round(this->elevation * 100);
  }
  else if (this->gps_glitch == GPS_GLITCH_ALTITUDE)
  {
    float k = GetTickCount() / 100000.0f;
    k -= (int)k;
    data.alt = (int32_t)round(this->glitch_elevation * 100 + k * 100000);
    data.velNED[2] = -(int16_t)round(k * 100000);
  }
  else if ((this->gps_glitch == GPS_GLITCH_CIRCLE) || (this->gps_glitch == GPS_GLITCH_CIRCLE_ALTITUDE) || (this->gps_glitch == GPS_GLITCH_CIRCLE_ALTITUDE_5))
  {
    float k = GetTickCount() / 100000.0f;
    k -= (int)k;
    double a = k * 2 * CONST_PI;
    data.lat = (int32_t)round((this->glitch_lattitude + 1 * sin(a) / 111.32) * 10000000);
    data.lon = (int32_t)round((this->glitch_longitude + 1 * cos(a) / 111.32) * 10000000);
    data.alt = (int32_t)round(this->glitch_elevation * 100 + ((this->gps_glitch == GPS_GLITCH_CIRCLE_ALTITUDE) ? k * 100000 : 0));
    data.velNED[0] = (int16_t)round(sin(a + CONST_PI / 2) * 6283);
    data.velNED[1] = (int16_t)round(cos(a + CONST_PI / 2) * 6283);
    data.velNED[2] = (this->gps_glitch == GPS_GLITCH_CIRCLE_ALTITUDE) ? -(int16_t)round(k * 100000) : 0;
    data.course = (int16_t)round((360 - (a * 180 / CONST_PI)) * 10);  // expected by inav: deg * 10
    if (data.course > 3600) data.course -= 3600;
    data.speed = 6283;
  }

  //expected order of rotation from local to global: roll, pitch, yaw
  data.roll =  (int16_t)round( this->roll * 10);  //expected by inav: left wing down - negative roll, 1 degree = 10, values range: -1800...1800
  data.pitch = (int16_t)round(-this->pitch * 10);  //expected by inav: stick down/nose up - negative pitch, upside-down: stick down/nose up - positiive pitch, 1 degree = 10 , values range: -1800...1800
  data.yaw =   (int16_t)round( this->yaw * 10);  //expected by inav: rotate clockwise( top view) - positive yaw+, 1 degreee = 10 , values range: 0...3600 , north = 0
  if (data.yaw < 0) data.yaw += 3600;

  g_graph.addAttitudeYPR(data.yaw, data.pitch, data.roll);

  data.accel_x = clampToInt16( -this->accel_x * 1000 );  //expected by inav: forward - positive   
  // g_side has the opposite sign to INAV's lateral axis: sent as it was, INAV's own AHRS read gentle banks as level
  data.accel_y = clampToInt16( -this->accel_y * 1000 );  //expected by inav: right - negative
  data.accel_z = clampToInt16(  this->accel_z * 1000 );  //expected by inav: 1.0f in stable position (1G)

  g_graph.addACC(data.accel_x / 1000.0f, data.accel_y / 1000.0f, data.accel_z / 1000.0f);

  g_stats.dbg_acc_x = data.accel_x / 1000.0f;
  g_stats.dbg_acc_y = data.accel_y / 1000.0f;              
  g_stats.dbg_acc_z = data.accel_z / 1000.0f;

  data.gyro_x = clampToInt16(  this->gyro_x * 16.0f );  //expected by inav: roll left wing down rotation -> negative
  data.gyro_y = clampToInt16( -this->gyro_y * 16.0f );  //expected by inav: pitch up rotation -> negative, 1 deerees per second
  data.gyro_z = clampToInt16( -this->gyro_z * 16.0f );  //expected by inav: yaw clockwise rotation (top view) ->negative

  // INAV's motor command, 1000-2000 us, as the T-28's logs give it
  const float motor = 1500.0f + this->control_throttle;
  // the logs' noise is the flying aircraft's (motor, airflow): at rest the gyro is clean, and INAV calibrates it there
  if (this->sensorNoise && this->isArmed)
  {
    // 5-60 Hz noise of the T-28's filtered gyro (deg/s) and accelerometer (m/s2), at 1000 us and per 100 us more
    const float g = fmaxf(motor - 1000.0f, 0.0f) / 100.0f;
    data.gyro_x = clampToInt16(data.gyro_x + this->gauss(0.946f + 0.286f * g) * 16.0f);
    data.gyro_y = clampToInt16(data.gyro_y + this->gauss(0.318f + 0.103f * g) * 16.0f);
    data.gyro_z = clampToInt16(data.gyro_z + this->gauss(0.160f + 0.076f * g) * 16.0f);
    data.accel_x = clampToInt16(data.accel_x + this->gauss(0.0282f + 0.0117f * g) / 9.80665f * 1000.0f);
    data.accel_y = clampToInt16(data.accel_y + this->gauss(0.0478f + 0.0142f * g) / 9.80665f * 1000.0f);
    data.accel_z = clampToInt16(data.accel_z + this->gauss(0.1121f + 0.0467f * g) / 9.80665f * 1000.0f);
  }

  g_graph.addGyro(data.gyro_x / 16.0f, data.gyro_y / 16.0f, data.gyro_z / 16.0f);

  g_stats.dbg_gyro_x = data.gyro_x / 16.0f;  
  g_stats.dbg_gyro_y = data.gyro_y / 16.0f;  
  g_stats.dbg_gyro_z = data.gyro_z / 16.0f;  

  data.baro = (int32_t)round(this->baro * 3386.39f);
  // 12 Pa is about 1 m near sea level
  if (this->sensorNoise)
  {
    // 0.18 m of noise above 1 Hz on the T-28's baro
    data.baro = (int32_t)round(data.baro + this->gauss(0.18f) * 12.0f);
  }
  if (this->baroDynamic)
  {
    // the T-28's baro read 0.03 m lower per (m/s)^2 of airspeed: dynamic pressure inside the fuselage
    data.baro = (int32_t)round(data.baro + 0.030f * this->airspeed * this->airspeed * 12.0f);
  }

  // the earth's field as a compass sees it, not a horizontal vector to true north: declination from X-Plane
  // (magnetic_variation is negative to the east), inclination from a dipole about the geomagnetic pole (2025:
  // 80.7 N, 72.7 W), a few degrees off the real one. Up is negative here: the T-28 read z -867 flying level
  const double rad = 3.14159265358979 / 180.0;
  const double latR = XPLMGetDatad(this->df_lattitude) * rad;
  const double lonR = XPLMGetDatad(this->df_longitude) * rad;
  const double poleLat = 80.7 * rad, poleLon = -72.7 * rad;
  const double geomagLat = asin(sin(latR) * sin(poleLat) + cos(latR) * cos(poleLat) * cos(lonR - poleLon));
  const double incl = atan(2.0 * tan(geomagLat));
  const double decl = -XPLMGetDataf(this->df_magvar) * rad;
  float north[3];
  north[0] = (float)(cos(incl) * cos(decl));
  north[1] = (float)(cos(incl) * sin(decl));
  north[2] = (float)(-sin(incl));

  float quat[4];
  this->computeQuaternionFromRPY(quat, data.roll, data.pitch, data.yaw );
  this->transformVectorEarthToBody(north, quat);

  if (!this->simulate_mag_failure)
  {
    data.mag_x = clampToInt16(north[0] * 16000.0f);
    data.mag_y = clampToInt16(north[1] * 16000.0f);
    data.mag_z = clampToInt16(north[2] * 16000.0f);

    // the T-28's figures are in INAV units against its field of 1051; this field is 800 after INAV's /20
    const float k = 800.0f / 1051.0f * 20.0f;
    if (this->sensorNoise && this->isArmed)
    {
      data.mag_x = clampToInt16(data.mag_x + this->gauss(8.8f) * k);
      data.mag_y = clampToInt16(data.mag_y + this->gauss(13.3f) * k);
      data.mag_z = clampToInt16(data.mag_z + this->gauss(25.3f) * k);
    }
    if (this->magCurrent && this->isArmed)
    {
      // motor current as on the T-28, from its command and the airspeed (fit over its flights, 2.7 A rms)
      const float m = fmaxf(motor - 1000.0f, 0.0f) / 1000.0f;
      const float v = this->airspeed / 20.0f;
      const float amp = fmaxf(0.3f, 3.98f - 8.08f * m + 92.45f * m * m - 14.4f * m * m * m - 1.12f * v - 14.38f * m * v);
      // the compass moved per ampere, plus the offset its calibration left (body frame, INAV units)
      data.mag_x = clampToInt16(data.mag_x + 15.92f * amp * k);
      data.mag_y = clampToInt16(data.mag_y + (-13.52f * amp - 47.0f) * k);
      data.mag_z = clampToInt16(data.mag_z + (9.42f * amp - 4.0f) * k);
    }
  }
  else
  {
    data.mag_x = 0;
    data.mag_y = 0;
    data.mag_z = 0;
  }

  this->recalculateBattery();
  data.vbat = (uint8_t)round((this->escActive ? ESC_VOC - ESC_RBATT_OHM * this->escCurrent : this->battery_chargeV) * 10);

  g_msp.sendCommand(MSP_SIMULATOR, &data, sizeof(data));
}

//==============================================================
//==============================================================
void TSimData::computeQuaternionFromRPY( float* quat, int16_t roll, int16_t pitch, int16_t yaw)
{
  if (roll > 1800) roll -= 3600;
  if (pitch > 1800) pitch -= 3600;
  if (yaw > 1800) yaw -= 3600;

  const float cosRoll = cosf(DECIDEGREES_TO_RADIANS(roll) * 0.5f);
  const float sinRoll = sinf(DECIDEGREES_TO_RADIANS(roll) * 0.5f);

  const float cosPitch = cosf(DECIDEGREES_TO_RADIANS(pitch) * 0.5f);
  const float sinPitch = sinf(DECIDEGREES_TO_RADIANS(pitch) * 0.5f);

  const float cosYaw = cosf(DECIDEGREES_TO_RADIANS(-yaw) * 0.5f);
  const float sinYaw = sinf(DECIDEGREES_TO_RADIANS(-yaw) * 0.5f);

  quat[0] = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  quat[1] = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  quat[2] = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  quat[3] = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

//==============================================================
//==============================================================
void TSimData::transformVectorEarthToBody(float* v, float* quat)
{
  // HACK: This is needed to correctly transform from NED (sensor frame) to NEU (navigation)
  v[1] = -v[1];

  // From earth frame to body frame
  this->quaternionRotateVector(v, v, quat);
}

//==============================================================
//==============================================================
void TSimData::quaternionRotateVector(float* result, const float* vect, const float* quat)
{
  float vectQuat[4];
  float refConj[4];

  vectQuat[0] = 0;
  vectQuat[1] = vect[0];
  vectQuat[2] = vect[1];
  vectQuat[3] = vect[2];

  this->quaternionConjugate(refConj, quat);
  this->quaternionMultiply(vectQuat, refConj, vectQuat);
  this->quaternionMultiply(vectQuat, vectQuat, quat);

  result[0] = vectQuat[1];
  result[1] = vectQuat[2];
  result[2] = vectQuat[3];
}

//==============================================================
//==============================================================
void TSimData::quaternionConjugate(float* result, const float* q)
{
  result[0] = q[0];
  result[1] = -q[1];
  result[2] = -q[2];
  result[3] = -q[3];
}

//==============================================================
//==============================================================
void TSimData::quaternionMultiply(float* result, const float* a, const float* b)
{
  float p[4];

  p[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  p[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  p[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  p[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

  result[0] = p[0];
  result[1] = p[1];
  result[2] = p[2];
  result[3] = p[3];
}

//==============================================================
//==============================================================
void TSimData::disconnect()
{
  TMSPSimulatorToINAV data;
  data.version = 1;
  data.flags = 0;
  g_msp.sendCommand(MSP_SIMULATOR, &data, sizeof(data));

  this->control_throttle = -500;
  this->control_roll = 0;
  this->control_pitch = 0;
  this->control_yaw = 0;

  // at once: the plugin may be going away before the next frame
  this->controlCount = 0;
  const TControls c = { microsNow(), this->control_throttle, 0, 0, 0 };
  this->applyControls(c);
}

//==============================================================
//==============================================================
void TSimData::setBateryEmulation(TBatteryEmulationType s)
{
  this->batEmulation = s;
  this->battery_lastUpdate = GetTickCount();
  this->battery_chargeV = (s == BATTERY_DISCHAGED) ? 9.8f : 12.6f;
}

//==============================================================
//==============================================================
void TSimData::recalculateBattery()
{
  uint32_t t = GetTickCount();
  uint32_t dt = t - this->battery_lastUpdate;
  this->battery_lastUpdate = t;
  if (dt > 1000) dt = 1000;

  float k = (this->control_throttle + 500.0f) / 1000.0f;
  if (k < 0) k = 0;
  if (k > 1.0f) k = 1.0f;

  //k=k*k;
  //use linear dependency for simplicity

  //12.6-9.6 = 3V
  float n = 0;
  switch (this->batEmulation)
  {
  case BATTERY_3MIN:
    n = 3.0f;
    break;
  case BATTERY_10MIN:
    n = 10.0f;
    break;
  case BATTERY_30MIN:
    n = 30.0f;
    break;
  }

  if (n > 0)
  {
    this->battery_chargeV -= k * dt * 3.0f / (1000 * 60 * n);
  }

  if (this->battery_chargeV < 9.6f)
  {
    this->battery_chargeV = 9.6f;
  }
}

//==============================================================
//==============================================================
void TSimData::loadConfig(mINI::INIStructure& ini)
{
  if (ini[SETTINGS_SECTION].has(SETTINGS_GPS_NUMSAT))
  {
    this->gps_numSat = atoi( ini[SETTINGS_SECTION][SETTINGS_GPS_NUMSAT].c_str() );
    if (this->gps_numSat == 0)
    {
      this->gps_fix = GPS_NO_FIX;
    }
    else if (gps_numSat == 12)
    {
      this->gps_fix = GPS_FIX_3D;
    }
    else
    {
      this->gps_fix = GPS_FIX_3D;
      this->gps_numSat = 12;
    }
  }

  this->gps_timeout = ini[SETTINGS_SECTION].has(SETTINGS_GPS_TIMEOUT) && (ini[SETTINGS_SECTION][SETTINGS_GPS_TIMEOUT] == "1");
  this->gps_glitch = GPS_GLITCH_NONE;
  if (ini[SETTINGS_SECTION].has(SETTINGS_GPS_GLITCH))
  {
    this->gps_glitch = atoi(ini[SETTINGS_SECTION][SETTINGS_GPS_GLITCH].c_str());
  }

  this->simulate_mag_failure = ini[SETTINGS_SECTION].has(SETTINGS_MAG_FAILURE) && (ini[SETTINGS_SECTION][SETTINGS_MAG_FAILURE] == "1");

  this->attitude_use_sensors = ini[SETTINGS_SECTION].has(SETTINGS_ATTITUDE_USE_SENSORS) && (ini[SETTINGS_SECTION][SETTINGS_ATTITUDE_USE_SENSORS] == "1");

  if (ini[SETTINGS_SECTION].has(SETTINGS_BATTERY_EMULATION))
  {
    this->batEmulation = (TBatteryEmulationType)atoi(ini[SETTINGS_SECTION][SETTINGS_BATTERY_EMULATION].c_str());
    if (this->batEmulation < BATTERY_NONE || this->batEmulation > BATTERY_30MIN)
    {
      this->batEmulation = BATTERY_INFINITE;
    }
  }

  this->muteBeeper = ini[SETTINGS_SECTION].has(SETTINGS_MUTE_BEEPER) && (ini[SETTINGS_SECTION][SETTINGS_MUTE_BEEPER] != "0");
  this->fixNoon = !ini[SETTINGS_SECTION].has(SETTINGS_FIX_NOON) || (ini[SETTINGS_SECTION][SETTINGS_FIX_NOON] != "0");
  this->sensorNoise = ini[SETTINGS_SECTION].has(SETTINGS_SENSOR_NOISE) && (ini[SETTINGS_SECTION][SETTINGS_SENSOR_NOISE] == "1");
  this->magCurrent = ini[SETTINGS_SECTION].has(SETTINGS_MAG_CURRENT) && (ini[SETTINGS_SECTION][SETTINGS_MAG_CURRENT] == "1");
  this->baroDynamic = ini[SETTINGS_SECTION].has(SETTINGS_BARO_DYNAMIC) && (ini[SETTINGS_SECTION][SETTINGS_BARO_DYNAMIC] == "1");
  this->fastLink = ini[SETTINGS_SECTION].has(SETTINGS_FAST_LINK) && (ini[SETTINGS_SECTION][SETTINGS_FAST_LINK] == "1");
  this->escModel = !ini[SETTINGS_SECTION].has(SETTINGS_ESC_MODEL) || (ini[SETTINGS_SECTION][SETTINGS_ESC_MODEL] != "0");
  this->gearServo = ini[SETTINGS_SECTION].has(SETTINGS_GEAR_SERVO) ? atoi(ini[SETTINGS_SECTION][SETTINGS_GEAR_SERVO].c_str()) : -1;
  this->flapServo = ini[SETTINGS_SECTION].has(SETTINGS_FLAP_SERVO) ? atoi(ini[SETTINGS_SECTION][SETTINGS_FLAP_SERVO].c_str()) : -1;
  this->flapServoUp = ini[SETTINGS_SECTION].has(SETTINGS_FLAP_SERVO_UP) ? atoi(ini[SETTINGS_SECTION][SETTINGS_FLAP_SERVO_UP].c_str()) : 1000;
  this->flapServoFull = ini[SETTINGS_SECTION].has(SETTINGS_FLAP_SERVO_FULL) ? atoi(ini[SETTINGS_SECTION][SETTINGS_FLAP_SERVO_FULL].c_str()) : 2000;

  this->simulatePitot = !ini[SETTINGS_SECTION].has(SETTINGS_SIMULATE_PITOT) || (ini[SETTINGS_SECTION][SETTINGS_SIMULATE_PITOT] != "0");
  this->simulatePitotFailureHW = ini[SETTINGS_SECTION].has(SETTINGS_SIMULATE_PITOT_FAILURE_HW) && (ini[SETTINGS_SECTION][SETTINGS_SIMULATE_PITOT_FAILURE_HW] == "1");
  this->simulatePitotFailure60 = ini[SETTINGS_SECTION].has(SETTINGS_SIMULATE_PITOT_FAILURE_60) && (ini[SETTINGS_SECTION][SETTINGS_SIMULATE_PITOT_FAILURE_60] == "1");
  if (this->simulatePitot)
  {
    this->simulatePitotFailureHW = false;
    this->simulatePitotFailure60 = false;
  }
  if (this->simulatePitotFailureHW) this->simulatePitotFailure60 = false;
}

//==============================================================
//==============================================================
void TSimData::saveConfig(mINI::INIStructure& ini)
{
  ini[SETTINGS_SECTION][SETTINGS_GPS_NUMSAT] = std::to_string(this->gps_numSat);
  ini[SETTINGS_SECTION][SETTINGS_ATTITUDE_USE_SENSORS] = std::to_string(this->attitude_use_sensors ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_GPS_TIMEOUT] = std::to_string(this->gps_timeout ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_GPS_GLITCH] = std::to_string(this->gps_glitch);
  ini[SETTINGS_SECTION][SETTINGS_MAG_FAILURE] = std::to_string(this->simulate_mag_failure ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_BATTERY_EMULATION] = std::to_string(this->batEmulation);
  ini[SETTINGS_SECTION][SETTINGS_MUTE_BEEPER] = std::to_string(this->muteBeeper ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_FIX_NOON] = std::to_string(this->fixNoon ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_SENSOR_NOISE] = std::to_string(this->sensorNoise ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_MAG_CURRENT] = std::to_string(this->magCurrent ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_BARO_DYNAMIC] = std::to_string(this->baroDynamic ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_FAST_LINK] = std::to_string(this->fastLink ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_ESC_MODEL] = std::to_string(this->escModel ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_GEAR_SERVO] = std::to_string(this->gearServo);
  ini[SETTINGS_SECTION][SETTINGS_FLAP_SERVO] = std::to_string(this->flapServo);
  ini[SETTINGS_SECTION][SETTINGS_FLAP_SERVO_UP] = std::to_string(this->flapServoUp);
  ini[SETTINGS_SECTION][SETTINGS_FLAP_SERVO_FULL] = std::to_string(this->flapServoFull);
  ini[SETTINGS_SECTION][SETTINGS_SIMULATE_PITOT] = std::to_string(this->simulatePitot ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_SIMULATE_PITOT_FAILURE_HW] = std::to_string(this->simulatePitotFailureHW ? 1 : 0);
  ini[SETTINGS_SECTION][SETTINGS_SIMULATE_PITOT_FAILURE_60] = std::to_string(this->simulatePitotFailure60 ? 1 : 0);
}


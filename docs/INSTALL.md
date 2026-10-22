# Installation and first flight

What you need: X-Plane 11 on Windows (11.55 was used; the [demo](https://www.x-plane.com/desktop/try-it/older/) works
for 15 minutes at a time), a flight controller running INAV 8 or newer with the simulator enabled (every official
build has it), a USB cable. No receiver, battery or radio: the plugin plays the radio.

## 1. The aircraft and the plugin

Copy `aircraft/T28_RC_1200` into `X-Plane 11/Aircraft/Extra Aircraft/`. The plugin lives inside the aircraft folder
(`plugins/INAV-X-Plane-HITL/64/win.xpl` and its `assets`), as RomanLut's original does: it loads with the aircraft.
In X-Plane tick *Show extra aircraft from older versions* if the T-28 does not appear, and set *Settings > General >
Flight models per frame* to 10 (a 1.2 m model is small for X-Plane's default).

The plugin here is RomanLut's 1.5.0 with the changes listed in [PLUGIN.md](PLUGIN.md); to build it yourself see the
end of that page.

## 2. INAV

Configure the board as for a real aircraft of this kind. The settings below are what the T-28 wants; put them in the
CLI (adapt the AUX channels to your taste, they are what the plugin's radio will drive):

```
set platform_type = AIRPLANE
set receiver_type = SIM
feature GPS
feature OSD
set has_flaps = ON
# modes: ARM on AUX1 high; ANGLE on AUX2 low, NAV ALTHOLD mid, NAV CRUISE high; NAV LAUNCH AUX3 high; NAV RTH AUX4 high
aux 0 0 0 1700 2100
aux 1 1 1 900 2100
aux 2 12 1 900 1300
aux 3 72 2 1700 2100
aux 4 42 8 1400 1600
aux 5 43 8 1700 2100
# gear on INAV servo 3 and flaps on servo 4, both from AUX3 (channel 7, 1000 / 1500 / 2000): servo 3 goes 1000 up,
# 1500 and 2000 down; servo 4 goes 1500 up, 1600 half, 1700 full. The plugin reads these two servos and moves X-Plane's
# gear handle and flaps (gear_servo, flap_servo, flap_servo_up, flap_servo_full below).
servo 4 1000 2000 1600 100
smix 4 3 10 100 0 -1
smix 5 4 10 100 0 -1
logic 0 1 -1 2 2 41 0 1 0
logic 1 1 0 38 0 7 0 1500 0
logic 2 1 -1 2 2 41 0 3 0
logic 3 1 2 38 0 7 0 2000 0
# the real aircraft's flap compensation: +4 % of the flap channel on the elevator, see MODEL.md
smix 8 1 15 4 0 -1
# a fake rangefinder, so the plugin's one is taken
set rangefinder_hardware = FAKE
save
```

Then a safehome at your runway with a landing approach (`safehome` and `fwapproach`), or a mission ending with a
landing: the T-28 was tuned with `nav_fw_alt_control_response = 100` and the stock INAV 10 landing settings except
`nav_fw_land_approach_length = 12000` (a 120 m final from 8 m suits its glide).

**Link rate.** Stock INAV handles one MSP command per serial task run, 100 a second. The plugin's default *Link
rate > Safe for stock INAV* sends 62 frames a second. *Every X-Plane frame* sends one per frame (120 a second here)
and needs an INAV build with [iNavFlight/inav#12177](https://github.com/iNavFlight/inav/pull/12177), which takes
several commands per run in HITL; on stock firmware it overflows the queue and the attitude freezes.

## 3. Plugin settings

The plugin writes `X-Plane 11/Output/preferences/inavhitl.ini`. Two keys are only there (no menu):

```
gear_servo=3
flap_servo=4
flap_servo_up=1500
flap_servo_full=1700
```

the INAV servo outputs (0-based, as in `smix`) that move X-Plane's gear handle (down above 1250 us) and flaps, and
the flap servo's output with the flaps up and fully down: with the mixer above, servo 4 sits at 1500 / 1600 / 1700 for
flaps up / half / full. -1 turns an output off. The rest
is in the *Plugins > INAV HITL/SITL* menu ([PLUGIN.md](PLUGIN.md)). For the T-28 as it was tuned: *Motor >
Brushless and ESC of the T-28* (default), *Time of day > Always noon* (default), *Sensors > Ideal* or *Noise from the
T-28 logs*, *Compass > Normal* or *Pulled by motor current*.

## 4. Fly

1. Start X-Plane with the T-28 on a runway, engine off.
2. Connect the board by USB. *Plugins > INAV HITL/SITL > Link > Connect to Flight Controller* (it probes the serial
   ports) or, if you run INAV's SITL, *Connect to SITL > UART1*.
3. The plugin now feeds the board every frame and reads back what INAV wants of the controls. Arm with the modes'
   channels from the plugin's radio page, or drive the board from the INAV Configurator's receiver tab... the simplest
   is a mission or an RTH that lands: arm, NAV LAUNCH, let it go.

Without the mouse: the X-Plane commands `inav_hitl/connect_sitl_uart1` .. `uart8` and `inav_hitl/disconnect` can be
bound to a key or sent over UDP (`CMND`), handy for scripted test campaigns.

## Troubleshooting

- *SENSORS_CALIBRATING* never clears: the model moved on the ground while the board calibrated (a jump onto its wheels
  when the gear came down): put the gear down before arming, or let INAV retry.
- The attitude freezes for seconds: the link rate is too high for the firmware, go back to *Safe*.
- The aircraft rolls hard when placed by a script: X-Plane's first samples after a placement are garbage; wait a
  second before taking the controls.

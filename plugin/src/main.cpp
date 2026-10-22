#include "config.h"

#define MSP_TIMEOUT_US  1000000u
// stock INAV answers one MSP command per 100 Hz serial run: 62 frames a second leave room for a ground station
#define MSP_PERIOD_SAFE_US  16000u
// a frame per X-Plane frame up to 250 Hz, for a build that takes several commands per run (inav #12177)
#define MSP_PERIOD_FAST_US  4000u

#include "menu.h"
#include "simData.h"
#include "msp.h"
#include "stats.h"
#include "util.h"
#include "osd.h"
#include "graph.h"
#include "sound.h"
#include "map.h"

uint64_t lastUpdateTime;
bool should_wait;
bool firstRender = true;

XPLMFlightLoopID loopId;

mINI::INIStructure ini;

//==============================================================
//==============================================================
void cbConnect(TCBConnectParm state)
{
  g_menu._cbConnect(state);
  g_osd.cbConnect(state);
  g_simData.setBateryEmulation(g_simData.batEmulation); //rechange battery
  lastUpdateTime = microsNow();
  should_wait = false;
}

//==============================================================
//==============================================================
void cbMessage(int code, const uint8_t* messageBuffer, int length)
{
  if (code == MSP_SIMULATOR)
  {
    if (length < MSP_SIMULATOR_RESPOSE_MIN_LENGTH)
    {
      g_menu.actionDisconnect();
      playSound("assets\\unsupported_firmware.wav");
      LOG("Unsupported firmware version, MSP_SIMULATOR response length: %d", length);
      g_osd.showMsg("UNSUPPORTED FIRMWARE");
    }
    else
    {
      TMSPSimulatorFromINAV msg;
      memset(&msg, 0, sizeof(msg));
      if (length > sizeof(msg)) return;
      memcpy(&msg, messageBuffer, length);
      g_simData.updateFromINAV(&msg);

      if (!g_simData.isAirplane)
      {
        g_menu.actionDisconnect();
        playSound("assets\\unsupported.wav");
        LOG("Unsupported aircraft type");
        g_osd.showMsg("UNSUPPORTED AIRCRAFT TYPE");
      }
      else
      {
        if (g_simData.isOSDDisabled)
        {
          g_osd.showMsg("OSD IS DISABLED:\n\nCONFIURATION\n->OTHER FEATURES\n->OSD");
        }
        else if (g_simData.isSupportedOSDNotFound)
        {
          g_osd.showMsg("SUPPORTED OSD TYPE\nNOT FOUND OR DISABLED\n\nCHECK REQUIREMENTS\nON PROJECT PAGE");
        }

        g_simData.sendToXPlane();

        g_osd.updateFromINAV((const TMSPSimulatorFromINAV*)messageBuffer);

        should_wait = false;
      }
    }
  }
  else if (code == MSP_DEBUGMSG)
  {
    LOG("INAV: %s\n", (const char*)messageBuffer);
  }
  else if (code == MSP_WP_GETINFO)
  {
    g_map.onWPInfo((const TMSPWPInfo*)messageBuffer);
  }
  else if (code == MSP_SERVO)
  {
    g_simData.onServos((const uint16_t*)messageBuffer, length / 2);
  }
  else if (code == MSP_WP)
  {
    g_map.onWP((const TMSPWP*)messageBuffer);
  }

}

//==============================================================
//==============================================================
// this flightloop callback will be called every frame to update the targets
float floop_cb(float elapsed1, float elapsed2, int ctr, void* refcon)
{
  g_msp.loop();
  g_simData.keepNoon();
  g_simData.updateMotor();
  g_simData.applyDelayedControls();

  if (g_msp.isConnected())
  {
    const uint64_t now = microsNow();
    if ((now - lastUpdateTime) >= (should_wait ? MSP_TIMEOUT_US : (g_simData.fastLink ? MSP_PERIOD_FAST_US : MSP_PERIOD_SAFE_US)))
    {
      g_simData.updateFromXPlane();
      g_simData.sendToINAV();
      g_simData.pollServos();
      lastUpdateTime = now;
      //should_wait = true; do not wait for answer, send next state after 10us passed
    }
  }

  g_stats.cycles++;
  g_stats.loop();
  return -1;
}

//==============================================================
//==============================================================
// The OSD and the graph are drawn in a borderless full-screen overlay window: X-Plane 11.50+ on Vulkan never calls
// the legacy xplm_Phase_Window draw callback, while plugin windows are composited under both OpenGL and Vulkan
static XPLMWindowID overlayWindow = NULL;

static void overlayDraw(XPLMWindowID inWindowID, void * inRefcon)
{
  int l, t, r, b;
  XPLMGetScreenBoundsGlobal(&l, &t, &r, &b);
  XPLMSetWindowGeometry(inWindowID, l, t, r, b);

  if (firstRender)
  {
    int sx, sy;
    XPLMGetScreenSize(&sx, &sy);
    LOG("overlay: screen bounds %d %d %d %d, screen size %d x %d", l, t, r, b, sx, sy);
    setView();
    g_osd.setHome();
    firstRender = false;
  }

  // XPLMSetGraphicsState caches the state it last set; under Vulkan the GL bridge context does not match X-Plane's
  // cache, so it is reset first and the OSD's own call below really enables texturing and blending
  XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // the OSD code draws in X-Plane window coordinates from (0, 0); the window callback works in global boxels
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef((float)l, (float)b, 0.0f);
  g_osd.drawCallback();
  g_graph.drawCallback();
  glPopMatrix();

  XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
}

static int overlayMouse(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void * inRefcon)
{
  return 0;   // clicks go through to X-Plane
}

static int overlayWheel(XPLMWindowID inWindowID, int x, int y, int wheel, int clicks, void * inRefcon)
{
  return 0;
}

static XPLMCursorStatus overlayCursor(XPLMWindowID inWindowID, int x, int y, void * inRefcon)
{
  return xplm_CursorDefault;
}

static void overlayKey(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void * inRefcon, int losingFocus)
{
}

static void createOverlayWindow()
{
  int l, t, r, b;
  XPLMGetScreenBoundsGlobal(&l, &t, &r, &b);

  XPLMCreateWindow_t params;
  memset(&params, 0, sizeof(params));
  params.structSize = sizeof(params);
  params.left = l;
  params.top = t;
  params.right = r;
  params.bottom = b;
  params.visible = 1;
  params.drawWindowFunc = overlayDraw;
  params.handleMouseClickFunc = overlayMouse;
  params.handleRightClickFunc = overlayMouse;
  params.handleMouseWheelFunc = overlayWheel;
  params.handleKeyFunc = overlayKey;
  params.handleCursorFunc = overlayCursor;
  params.refcon = NULL;
  params.decorateAsFloatingWindow = xplm_WindowDecorationNone;
  params.layer = xplm_WindowLayerFlightOverlay;
  overlayWindow = XPLMCreateWindowEx(&params);
}

//==============================================================
//==============================================================
PLUGIN_API int XPluginStart(
	char *		outName,
	char *		outSig,
	char *		outDesc)
{
  LOG("Plugin start");

	strcpy(outName, "INAV HITL");
	strcpy(outSig, "https://github.com/RomanLut/INAV-X-Plane-HITL");
	strcpy(outDesc, "INAV Hardware In The Loop");

  g_osd.init();

  createOverlayWindow();

	return 1;
}

//==============================================================
//==============================================================
PLUGIN_API void	XPluginStop(void)
{
  LOG("Plugin stop");

  if (overlayWindow != NULL)
  {
    XPLMDestroyWindow(overlayWindow);
    overlayWindow = NULL;
  }

  g_osd.destroy();
  g_sound.destroy();
  g_map.destroy();

}

//==============================================================
//==============================================================
bool buildIniFileName( char* iniFileName )
{
  XPLMGetPrefsPath(iniFileName);
  char* p = XPLMExtractFileAndPath(iniFileName);
  if (!p) return false;
  strncat(iniFileName, XPLMGetDirectorySeparator(), MAX_PATH);
  strcat(iniFileName, "inavhitl.ini");
  return true;
}

//==============================================================
//==============================================================
bool loadIniFile()
{
  char iniFileName[MAX_PATH];
  if (!buildIniFileName(iniFileName))
  {
    g_menu.updateAll();
    return false;
  }

  mINI::INIFile iniFile(iniFileName);
  iniFile.read(ini);

  g_simData.loadConfig(ini);
  g_osd.loadConfig(ini);
  g_graph.loadConfig(ini);
  g_map.loadConfig(ini);
  g_menu.loadConfig(ini);

  g_menu.updateAll();

  return true;
}


//==============================================================
//==============================================================
bool saveIniFile()
{
  g_simData.saveConfig(ini);
  g_osd.saveConfig(ini);
  g_graph.saveConfig(ini);
  g_map.saveConfig(ini);
  g_menu.saveConfig(ini);

  char iniFileName[MAX_PATH];
  if (!buildIniFileName(iniFileName)) return false;

  mINI::INIFile iniFile(iniFileName);
  iniFile.generate(ini);
  return true;
}

//==============================================================
//==============================================================
PLUGIN_API int XPluginEnable(void)
{
  LOG("Plugin enable");

  g_stats.init();
  g_simData.init(); //initialize before memu
  g_simData.updateFromXPlane();

  g_map.init();

  g_menu.createMenu();
  g_simData.registerDatarefs();

  XPLMCreateFlightLoop_t params;
  params.structSize = sizeof(XPLMCreateFlightLoop_t);
  params.callbackFunc = &floop_cb;
  params.phase = xplm_FlightLoop_Phase_AfterFlightModel;
  params.refcon = NULL;
  loopId = XPLMCreateFlightLoop(&params);
  XPLMScheduleFlightLoop(loopId, -1, true);

  g_sound.init();

  playSound("assets\\ready_to_connect.wav");

  loadIniFile();

  return 1;
}

//==============================================================
//==============================================================
PLUGIN_API void XPluginDisable(void)
{
  LOG("Plugin disable");

  g_stats.close();

  // X-Plane's own motor back, or the prop would keep the last torque written
  if (g_simData.escActive)
  {
    XPLMSetDatai(g_simData.df_override_torque, 0);
    g_simData.escActive = false;
  }

  if (g_msp.isConnected())
  {
    g_simData.disconnect();
    g_msp.disconnect();
  }
  g_simData.unregisterDatarefs();
  g_menu.destroyMenu();

  XPLMDestroyFlightLoop(loopId);
}

//==============================================================
//==============================================================
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam)
{
  if (inMsg == XPLM_MSG_AIRPORT_LOADED)
  {
    firstRender = true;
  }
}



// LANDING THOTTLE MANAGER

// This plugin will manage the throttle and reverse thrust during landing
// which helps when using VR as the pilot can concentrate on getting the threshold
// speed right and looking out of the window at the runway, rather than
// fumbling for the throttle levers and the reverse-thrust hotspot

// when the plugin is enabled (e.g. using a button press) it checks if the
// landing conditions are met, for example less than 160KIAS, flaps at 18+ deg, gears are down
// and 500ft or less above ground.
// if the conditions are met it will reduce throttle to idle and then wait for all
// wheels to be on the ground.
// when all wheels are down the reverse thrust is applied until a speed of 60KIAS
// is reached at which point reverse thrust is disabled and the throttle
// returned to idle
// if the conditions are not met to enable the plugin then voice guidance will be given
// as to which conditions are not being met

#include "LandingThrottleManager.h"
#include "Diagnostic.h"

#define MODULE_NAME "Landing Throttle Manager"

// configuration section
// minimum speed in knots at which the reverse thrust can be enabled
#define MIN_SPEED_REVERSE_THRUST 60.0f
// maximum speed in knots at which the manager can be enabled
#define MAX_AIRSPEED 160.0f
// minimum flap angle at which the manager can be enabled
#define MIN_FLAP_ANGLE 18.0f
// maximum height above ground in meters at which the manager can be enabled
#define MAX_ALTITUDE 152.4f

// time between executions of the state machine, in seconds
#define STATE_MACHINE_EXECUTION_INTERVAL 0.25f
// the ratio of the gears when they are down
#define GEAR_DOWN_RATIO 1.0f

// menu item IDs
#define MENU_ITEM_ID_ENABLE 1
#define MENU_ITEM_ID_STOP   2

// state machine states
typedef enum _states_t
{
  WAIT_FOR_USER,
  START,
  THROTTLE_DOWN,
  WAIT_FOR_IDLE_THROTTLE,
  WAIT_FOR_TOUCHDOWN,
  APPLY_REVERSE,
  WAIT_FOR_END_OF_REVERSE
} states_t;

// identifiers of known aircraft
typedef enum _aircraft_id_t
{
  AIRCRAFT_UNKNOWN,
  AIRCRAFT_XCRAFTS_ERJ_FAMILY
} aircraft_id_t;

// describes a know aircraft
typedef struct _known_aircraft_t
{
  aircraft_id_t Id;
  char *DescriptionMatch;
  char *UserFriendlyName;
} known_aircraft_t;

// commands and data references that we need
static XPLMCommandRef ReverseThrustCmd = NULL;
static XPLMCommandRef ThrottleDownCmd = NULL;
static XPLMDataRef    ThrottleRatioRef = NULL;
static XPLMDataRef    IndicatedAirSpeedRef = NULL;
static XPLMDataRef    AllWheelsOnGroundRef = NULL;
static XPLMDataRef    FlapsAngleRef = NULL;
static XPLMDataRef    GearDeployRatioRef = NULL;
static XPLMDataRef    AltitudeAboveGroundRef = NULL;

// custom commands
static XPLMCommandRef EnableCmd = NULL;

// the current state of the state machine
static states_t CurrentState = WAIT_FOR_USER;
// flag to indicate if the user has requested deactivation of the manager
static bool DeactivationRequested = FALSE;
// prototype for the function that handles menu choices
static void	MenuHandlerCallback(void *inMenuRef, void *inItemRef);
// flag to indicate if we are ready for use
static bool Ready = FALSE;

// all the known aircraft
static _known_aircraft_t KnownAircrafts[] =
{
  {AIRCRAFT_XCRAFTS_ERJ_FAMILY, "x-crafts erj", "X-Crafts ERJ Family"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////////
// INTERNAL FUNCTIONS

// execute the state machine, called periodically by x-plane
// returns the number of seconds to the next execution
static float StateMachine
  (
  float elapsedMe,
  float elapsedSim,
  int counter,
  void *refcon
  )
{
  if (Ready == FALSE) return STATE_MACHINE_EXECUTION_INTERVAL;

  switch (CurrentState)
  {
    // the current state needs to be set to START to exit
    // this state
  case WAIT_FOR_USER:
    break;

    // start the manager
  case START:
  {
    float ThrottleRatio = XPLMGetDataf(ThrottleRatioRef);

    if (ThrottleRatio > 0)
    {
      CurrentState = THROTTLE_DOWN;
#if DIAGNOSTIC == 1
      Diagnostic_printf("Going to throttle down as we are not at idle throttle\n");
#endif // DIAGNOSTIC
    }
    else
    {
      CurrentState = WAIT_FOR_TOUCHDOWN;
#if DIAGNOSTIC == 1
      Diagnostic_printf("Already at idle throttle, waiting for touch down of all three wheels\n");
#endif // DIAGNOSTIC
    }
  }
  break;

  // start throttling down
  case THROTTLE_DOWN:
  {
#if DIAGNOSTIC == 1
    Diagnostic_printf("Throttling down, waiting for idle throttle\n");
#endif // DIAGNOSTIC
    XPLMCommandBegin(ThrottleDownCmd);
    CurrentState = WAIT_FOR_IDLE_THROTTLE;
  }
  break;

  // waiting for the throttle to reach idle
  case WAIT_FOR_IDLE_THROTTLE:
  {
    if (DeactivationRequested == TRUE)
    {
      XPLMCommandEnd(ThrottleDownCmd);
      DeactivationRequested = FALSE;
      CurrentState = WAIT_FOR_USER;
#if DIAGNOSTIC == 1
      Diagnostic_printf("Deactivation while waiting for idle throttle\n");
#endif // DIAGNOSTIC
    }
    else
    {
      float ThrottleRatio = XPLMGetDataf(ThrottleRatioRef);
      if (ThrottleRatio == 0)
      {
        XPLMCommandEnd(ThrottleDownCmd);
#if DIAGNOSTIC == 1
        Diagnostic_printf("Throttle now at idle, waiting for touch down of all three wheels\n");
#endif // DIAGNOSTIC
        CurrentState = WAIT_FOR_TOUCHDOWN;
      }
    }
  }
  break;

  // wait for all of the wheels to touch the ground so we don't slam
  // the aircraft into the ground with reverse thrust
  case WAIT_FOR_TOUCHDOWN:
  {
    if (DeactivationRequested == TRUE)
    {
      XPLMCommandEnd(ReverseThrustCmd);
      DeactivationRequested = FALSE;
      CurrentState = WAIT_FOR_USER;
#if DIAGNOSTIC == 1
      Diagnostic_printf("Deactivation while waiting for touch down\n");
#endif // DIAGNOSTIC
    }
    else
    {
      int AllWheelsOnGround = XPLMGetDatai(AllWheelsOnGroundRef);
      if (AllWheelsOnGround == TRUE)
      {
#if DIAGNOSTIC == 1
        Diagnostic_printf("All wheels on ground, applying reverse thrust\n");
#endif // DIAGNOSTIC
        CurrentState = APPLY_REVERSE;
      }
    }
  }
  break;

  // apply the reverse thrust
  case APPLY_REVERSE:
  {
    float IndicatedAirSpeed = XPLMGetDataf(IndicatedAirSpeedRef);
    if (IndicatedAirSpeed > MIN_SPEED_REVERSE_THRUST)
    {
      XPLMCommandBegin(ReverseThrustCmd);
#if DIAGNOSTIC == 1
      Diagnostic_printf("Indicated air speed=%f which is above the minimum of %f, waiting for end condition\n", IndicatedAirSpeed, MIN_SPEED_REVERSE_THRUST);
#endif // DIAGNOSTIC
      CurrentState = WAIT_FOR_END_OF_REVERSE;
    }
    else
    {
      CurrentState = WAIT_FOR_USER;
    }
  }
  break;

  // wait for the right conditions to terminate the reverse thrust
  case WAIT_FOR_END_OF_REVERSE:
  {
    if (DeactivationRequested == TRUE)
    {
      XPLMCommandEnd(ReverseThrustCmd);
      DeactivationRequested = FALSE;
      CurrentState = WAIT_FOR_USER;
#if DIAGNOSTIC == 1
      Diagnostic_printf("Deactivation while waiting for end of reverse thrust\n");
#endif // DIAGNOSTIC
    }
    else
    {
      float IndicatedAirSpeed = XPLMGetDataf(IndicatedAirSpeedRef);
      if (IndicatedAirSpeed <= MIN_SPEED_REVERSE_THRUST)
      {
        XPLMCommandEnd(ReverseThrustCmd);
#if DIAGNOSTIC == 1
        Diagnostic_printf("Indicated air speed is %f, which is less than %f, end of reverse thrust\n", IndicatedAirSpeed, MIN_SPEED_REVERSE_THRUST);
#endif // DIAGNOSTIC
        CurrentState = WAIT_FOR_USER;
      }
    }
  }
  break;
  }

  return STATE_MACHINE_EXECUTION_INTERVAL;
}

// enables the manager
static void Enable
(
  void
)
{
  if (Ready == FALSE) return;

  // trigger the state machine
  if (CurrentState == WAIT_FOR_USER)
  {
    float IndicatedAirSpeed = XPLMGetDataf(IndicatedAirSpeedRef);
    float FlapAngles[1];
    XPLMGetDatavf(FlapsAngleRef, FlapAngles, 0, 1);
    float GearDeployRatio[1];
    XPLMGetDatavf(GearDeployRatioRef, GearDeployRatio, 0, 1);
    float AltitudeAboveGround = XPLMGetDataf(AltitudeAboveGroundRef);

#if DIAGNOSTIC == 1
    Diagnostic_printf("Enable requested by user\n");
    Diagnostic_printf("Current IAS=%f (require %f or below)\n", IndicatedAirSpeed, MAX_AIRSPEED);
    Diagnostic_printf("Current flap angle=%f (require %f or above)\n", FlapAngles[0], MIN_FLAP_ANGLE);
    Diagnostic_printf("Current gears are down=%s (require yes)\n", GearDeployRatio[0] == GEAR_DOWN_RATIO ? "yes" : "no");
    Diagnostic_printf("Current altitude=%fm (require %fm or below)\n", AltitudeAboveGround, MAX_ALTITUDE);
#endif // DIAGNOSTIC

    if ((IndicatedAirSpeed <= MAX_AIRSPEED) && (FlapAngles[0] >= MIN_FLAP_ANGLE) && (GearDeployRatio[0] == GEAR_DOWN_RATIO) && (AltitudeAboveGround <= MAX_ALTITUDE))
    {
      DeactivationRequested = FALSE;
      CurrentState = START;
#if DIAGNOSTIC == 1
      Diagnostic_printf("Conditions met, now enabled\n");
#endif // DIAGNOSTIC
    }
    else
    {
      char Errors[256] = "";
      if (IndicatedAirSpeed > MAX_AIRSPEED) strcat_s(Errors, 256, " Airspeed too high");
      if (FlapAngles[0] < MIN_FLAP_ANGLE) strcat_s(Errors, 256, " Flaps too low");
      if (GearDeployRatio[0] != GEAR_DOWN_RATIO) strcat_s(Errors, 256, " Gear not down");
      if (AltitudeAboveGround > MAX_ALTITUDE) strcat_s(Errors, 256, " Altitude too high");
      if (strlen(Errors) > 0) XPLMSpeakString(Errors);
    }
  }
  else
  {
    XPLMSpeakString("Already enabled");
  }
}

// handles the enable command
static int EnableCmdHandler
(
  XPLMCommandRef inCommand,
  XPLMCommandPhase inPhase,
  void *inRefcon
)
{
  // If inPhase == 0 the command is executed once on button down.
  if (inPhase == 0)
  {
    if (Ready == FALSE)
    {
      XPLMSpeakString("Plugin failed to load, check the aircraft is known");
      return 0;
    }

    Enable();
  }

  // disable further processing of this command
  return 0;
}

// called when the user chooses a menu item
static void MenuHandlerCallback
(
  void *inMenuRef,
  void *inItemRef
)
{
  if (Ready == FALSE)
  {
    XPLMSpeakString("Plugin failed to load, check the aircraft is known");
    return;
  }

  // user chose to arm the manager
  if ((int)inItemRef == MENU_ITEM_ID_ENABLE)
  {
    Enable();
  }
  // user choose to stop the manager
  else if ((int)inItemRef == MENU_ITEM_ID_STOP)
  {
    if (CurrentState != WAIT_FOR_USER)
    {
      DeactivationRequested = TRUE;
#if DIAGNOSTIC == 1
      Diagnostic_printf("User requested deactivation\n");
#endif // DIAGNOSTIC
    }
  }
}

// determines the currently loaded aircraft
static aircraft_id_t DetectAircraft
(
  void
)
{
  // is a description for the aircraft defined? if not then we can't tell what it is
  XPLMDataRef AircraftDescriptionRef = XPLMFindDataRef("sim/aircraft/view/acf_descrip");
  if (AircraftDescriptionRef != NULL)
  {
    // get aircraft description and convert to lower case
    char Description[256];
    XPLMGetDatab(AircraftDescriptionRef, (void *)Description, 0, 256);
    for (int c = 0; c < strlen(Description); c++) Description[c] = tolower(Description[c]);

#if DIAGNOSTIC == 1
    Diagnostic_printf("Aircraft loaded = '%s'\n", Description);
#endif // DIAGNOSTIC

    int NumKnownAircraft = sizeof(KnownAircrafts) / sizeof(known_aircraft_t);
#if DIAGNOSTIC == 1
    Diagnostic_printf("We know about %d different aircraft, searching for match\n", NumKnownAircraft);
#endif // DIAGNOSTIC

    for (int a = 0; a < NumKnownAircraft; a++)
    {
      if (strstr(Description, KnownAircrafts[a].DescriptionMatch) != NULL)
      {
#if DIAGNOSTIC == 1
        Diagnostic_printf("Found match for aircraft: %s\n", KnownAircrafts[a].UserFriendlyName);
#endif // DIAGNOSTIC
        return KnownAircrafts[a].Id;
      }
    }
  }

  return AIRCRAFT_UNKNOWN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE API

// initalizes the module
// returns TRUE for success, FALSE for error
int LandingThrottleManager_Init
  (
  XPLMMenuID ParentMenuId
  )
{
  XPLMMenuID myMenu;
  int mySubMenuItem;

  // not ready until we know what aircraft will be used
  Ready = FALSE;

  mySubMenuItem = XPLMAppendMenuItem(
    ParentMenuId,
    MODULE_NAME,
    0,
    1);

  myMenu = XPLMCreateMenu(
    MODULE_NAME,
    ParentMenuId,
    mySubMenuItem,
    MenuHandlerCallback,
    0
  );

  // Append menu items to our submenu
  XPLMAppendMenuItem(
    myMenu,
    "Enable",
    (void *)MENU_ITEM_ID_ENABLE,
    1);
  XPLMAppendMenuItem(
    myMenu,
    "Stop and disable",
    (void *)MENU_ITEM_ID_STOP,
    1);

  // create custom command
  char CmdName[100];
  sprintf_s(CmdName, 100, "%s//%s//Enable", PLUGIN_NAME, MODULE_NAME);
  char CmdDesc[100];
  sprintf_s(CmdDesc, 100, "Enable the throttle manager (%s-%s)", PLUGIN_NAME, MODULE_NAME);
  EnableCmd = XPLMCreateCommand(CmdName, CmdDesc);
  XPLMRegisterCommandHandler(
    EnableCmd,         // in Command name
    EnableCmdHandler,  // in Handler
    1,                 // Receive input before plugin windows.
    (void *)0);        // inRefcon.

  // initialize state machine
  CurrentState = WAIT_FOR_USER;

  // register the state machine callback
  XPLMRegisterFlightLoopCallback(StateMachine, STATE_MACHINE_EXECUTION_INTERVAL, NULL);

  return TRUE;
}

// called when a message is received from X-plane
void LandingThrottleManager_ReceiveMessage
  (
  XPLMPluginID inFromWho,
  int	inMessage,
  void *inParam
  )
{
  // a new aircraft has been loaded, check if we know it and if so access the data refs and commands we need
  if (inMessage == XPLM_MSG_PLANE_LOADED)
  {
    Ready = FALSE;

    aircraft_id_t AircraftId = DetectAircraft();

    switch (AircraftId)
    {
    case AIRCRAFT_XCRAFTS_ERJ_FAMILY:
    {
      // get commands
      ReverseThrustCmd = XPLMFindCommand("sim/engines/thrust_reverse_hold");
      if (ReverseThrustCmd == NULL)
      {
        return;
      }
      ThrottleDownCmd = XPLMFindCommand("sim/engines/throttle_down");
      if (ThrottleDownCmd == NULL)
      {
        return;
      }

      // get datarefs
      ThrottleRatioRef = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");
      if (ThrottleRatioRef == NULL)
      {
        return;
      }
      IndicatedAirSpeedRef = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed2");
      if (IndicatedAirSpeedRef == NULL)
      {
        return;
      }
      AllWheelsOnGroundRef = XPLMFindDataRef("sim/flightmodel/failures/onground_all");
      if (AllWheelsOnGroundRef == NULL)
      {
        return;
      }
      FlapsAngleRef = XPLMFindDataRef("sim/flightmodel2/wing/flap1_deg");
      if (FlapsAngleRef == NULL)
      {
        return;
      }

      GearDeployRatioRef = XPLMFindDataRef("sim/flightmodel2/gear/deploy_ratio");
      if (GearDeployRatioRef == NULL)
      {
        return;
      }

      AltitudeAboveGroundRef = XPLMFindDataRef("sim/flightmodel2/position/y_agl");
      if (AltitudeAboveGroundRef == NULL)
      {
        return;
      }
    }
#if DIAGNOSTIC == 1
    Diagnostic_printf("Ready to go\n");
#endif // DIAGNOSTIC
    Ready = TRUE;
    break;

    case AIRCRAFT_UNKNOWN:
      break;
    }
  }
}

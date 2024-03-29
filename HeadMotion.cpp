// HEAD MOTION

// Moves the pilots head in response to turbulence and touch down

#include <math.h>
#include "HeadMotion.h"
#include "Diagnostic.h"

#define MODULE_NAME "Head Motion"

// time between executions of the state machine, in seconds
#define STATE_MACHINE_EXECUTION_INTERVAL_NORMAL      0.250f
#define STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE 0.005f

// menu item IDs
#define MENU_ITEM_ID_TOUCHDOWN_ENABLE 1

// custom commands
static XPLMCommandRef DisableTouchDownCmd = NULL;

// commands and data references that we need
static XPLMDataRef    PilotXRef                 = NULL;
static XPLMDataRef    PilotYRef                 = NULL;
static XPLMDataRef    PilotZRef                 = NULL;
static XPLMDataRef    PilotHeadingRef           = NULL;
static XPLMDataRef    PilotPitchRef             = NULL;
static XPLMDataRef    PilotRollRef              = NULL;
static XPLMDataRef    AnyWheelOnGroundRef       = NULL;
static XPLMDataRef    UpwardGearGroundForceNRef = NULL;
static XPLMDataRef    TotalDownwardGForceRef    = NULL;
static XPLMDataRef    GearVerticalForceNmRef    = NULL;
static XPLMDataRef    VerticalSpeedRef          = NULL;
static XPLMDataRef    FlightTimeRef             = NULL;
static XPLMDataRef    AllWheelsOnGroundRef      = NULL;
static XPLMCommandRef RegularDownCmd            = NULL;
static XPLMCommandRef RegularUpCmd              = NULL;
static XPLMCommandRef FastDownCmd               = NULL;
static XPLMCommandRef FastUpCmd                 = NULL;

// prototype for the function that handles menu choices
static void	MenuHandlerCallback(void *inMenuRef, void *inItemRef);

// state machine states
typedef enum _states_t
{
  START,
  WAIT_FOR_FLYING,
  WAIT_FOR_LANDING,
  TOUCHDOWN,
  MOVE_UP,
  RESTORING_POSITION,
  WAIT_FOR_NOSE
} states_t;

typedef struct _pilots_head_t
{
  double x;
  double y;
  double z;
  double Heading;
  double Pitch;
  double Roll;
} pilots_head_t;

// the current state of the state machine
static states_t CurrentState;
// flag to indicate if we are ready for use
static bool Ready = FALSE;
static float TouchdownTime;
static float BottomTime;
static pilots_head_t InitialHeadPosition;
static double LandingShakeAmplitude;
static double TargetPilotY;
static XPLMCommandRef UpCommand;
static XPLMCommandRef DownCommand;
static bool HaveInitialHeadPosition;
static bool FastMovement;
static bool Enabled;
static XPLMMenuID myMenu;
static int MenuItem_Enable;
static bool Terminate_Motion;
static double PreviousFlightTime;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// INTERNAL FUNCTIONS

// enables the touch down motion
static void DisableTouchDownMotion
  (
  void
  )
{
}

// gets the current position of the pilots' head
static void GetHeadPosition
  (
  pilots_head_t *Position  // filled with the current position
  )
{
  Position->x       = XPLMGetDataf(PilotXRef);
  Position->y       = XPLMGetDataf(PilotYRef);
  Position->z       = XPLMGetDataf(PilotZRef);
  Position->Heading = XPLMGetDataf(PilotHeadingRef);
  Position->Pitch   = XPLMGetDataf(PilotPitchRef);
  Position->Roll    = XPLMGetDataf(PilotRollRef);
}

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
  float GearForces[3];
  float NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_NORMAL;
  float FlightTime = XPLMGetDataf(FlightTimeRef);

  // new aircraft or location loaded, initialize
  if (FlightTime < PreviousFlightTime)
  {
    Ready = TRUE;
    CurrentState = START;
    HaveInitialHeadPosition = FALSE;
#if DIAGNOSTIC == 1
    Diagnostic_printf("Start\n");
#endif // DIAGNOSTIC
  }
  PreviousFlightTime = FlightTime;

  if (Enabled == FALSE) return NextInterval;
  if (Ready == FALSE) return NextInterval;

  switch (CurrentState)
  {
    case START:
      // wait for x-plane to drop the plane onto the ground at the start
      // of the simulation
      if (FlightTime >= 3.0)
      {
#if DIAGNOSTIC == 1
        Diagnostic_printf("Waiting for X-plane to finish initial aircraft drop\n");
#endif // DIAGNOSTIC
        CurrentState = WAIT_FOR_FLYING;
        Terminate_Motion = FALSE;
      }
      break;

    case WAIT_FOR_FLYING:
      if (Terminate_Motion)
      {
        // do nothing, keep in this state
      }
      else
      {
        // wait for all wheels off the ground
        if (XPLMGetDatai(AnyWheelOnGroundRef) == FALSE)
        {
#if DIAGNOSTIC == 1
          Diagnostic_printf("All wheels off the ground, waiting for landing...\n");
#endif // DIAGNOSTIC
          CurrentState = WAIT_FOR_LANDING;
          if (HaveInitialHeadPosition == FALSE)
          {
            GetHeadPosition(&InitialHeadPosition);
          }
        }
      }
      break;

    case WAIT_FOR_LANDING:
      if (Terminate_Motion)
      {
        CurrentState = WAIT_FOR_FLYING;
      }
      else
      {
        // at least one wheel on the ground
        if (XPLMGetDatai(AnyWheelOnGroundRef) == TRUE)
        {
#if DIAGNOSTIC == 1
          Diagnostic_printf("At least one wheel on the ground, start landing head motion\n");
          XPLMGetDatavf(GearVerticalForceNmRef, GearForces, 0, 3);
          Diagnostic_printf("%fN %fG %fNm %fNm %fNm\n", XPLMGetDataf(UpwardGearGroundForceNRef), XPLMGetDataf(TotalDownwardGForceRef), GearForces[0], GearForces[1], GearForces[2]);
#endif // DIAGNOSTIC
          CurrentState = TOUCHDOWN;
          TouchdownTime = XPLMGetElapsedTime();
#if DIAGNOSTIC == 1
          Diagnostic_printf("Got head height of = %f\n", InitialHeadPosition.y);
#endif // DIAGNOSTIC

          double VerticalSpeedMS = fabs(XPLMGetDataf(VerticalSpeedRef));

          // greater than 2m/s is considered a hard landing:
          // https://en.wikipedia.org/wiki/Hard_landing#:~:text=Landing%20is%20the%20final%20phase,classed%20by%20crew%20as%20hard.
          // normal descent rate is 60-180FPM (0.3-0.9m/s). Over 240FPM (1.2m/s) is hard and requires an inspection:
          // https://www.boldmethod.com/learn-to-fly/aerodynamics/why-its-hard-to-land-smooth-in-empty-jets/

          // scale shaking amplitude to vertical speed
          char msg[100];
          sprintf_s(msg, 100, "%.1f meters per second", VerticalSpeedMS);
#if DIAGNOSTIC == 1
          Diagnostic_printf("%s\n", msg);
          //XPLMSpeakString(msg);
#endif // DIAGNOSTIC

          if (VerticalSpeedMS > 0.5)
          {
            DownCommand = RegularDownCmd;
            UpCommand = RegularUpCmd;
            // not enough FPS in VR for the fast motion to work reliably
            //DownCommand = FastDownCmd;
            //UpCommand = FastUpCmd;
          }
          else
          {
            DownCommand = RegularDownCmd;
            UpCommand = RegularUpCmd;
          }

          // scale shaking amplitude so 0.0 -> 0.7 m/s = shake amplitude 0.0 -> 0.08 m

          double Slope = 0.7 / 0.08;
          LandingShakeAmplitude = VerticalSpeedMS / Slope;
          if (LandingShakeAmplitude < 0) LandingShakeAmplitude = 0;

#if DIAGNOSTIC == 1
          Diagnostic_printf("Landing shake amplitude = %fm\n", LandingShakeAmplitude);
#endif // DIAGNOSTIC

          if (LandingShakeAmplitude > 0)
          {
            TargetPilotY = InitialHeadPosition.y - LandingShakeAmplitude;
            XPLMCommandBegin(DownCommand);
#if DIAGNOSTIC == 1
            Diagnostic_printf("Moving head down, target position of %f\n", TargetPilotY);
#endif // DIAGNOSTIC

            NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
          }
          else
          {
            CurrentState = WAIT_FOR_FLYING;
          }
        }
      }
      break;

    case TOUCHDOWN:
      if (Terminate_Motion)
      {
        XPLMCommandEnd(DownCommand);
        CurrentState = WAIT_FOR_FLYING;
      }
      else
      {
        double CurrentPilotY = XPLMGetDataf(PilotYRef);

        if (CurrentPilotY <= TargetPilotY)
        {
          XPLMCommandEnd(DownCommand);
#if DIAGNOSTIC == 1
          Diagnostic_printf("Bottom of bounce, current position is %f, going back to %f\n", CurrentPilotY, InitialHeadPosition.y);
#endif // DIAGNOSTIC
          CurrentState = MOVE_UP;
          BottomTime = XPLMGetElapsedTime();
        }

        NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
      }
      break;

    case MOVE_UP:
      if (Terminate_Motion)
      {
        CurrentState = WAIT_FOR_FLYING;
      }
      else
      {
        XPLMCommandBegin(UpCommand);
        CurrentState = RESTORING_POSITION;
        NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
      }
      break;

    case RESTORING_POSITION:
      if (Terminate_Motion)
      {
        XPLMCommandEnd(UpCommand);
        CurrentState = WAIT_FOR_FLYING;
      }
      else
      {
        double CurrentPilotY = XPLMGetDataf(PilotYRef);

        if (CurrentPilotY >= InitialHeadPosition.y)
        {
#if DIAGNOSTIC == 1
          Diagnostic_printf("End of movement, current position is %f\n", CurrentPilotY);
#endif // DIAGNOSTIC
          XPLMCommandEnd(UpCommand);

          // check if nose wheel is down
          XPLMGetDatavf(GearVerticalForceNmRef, GearForces, 0, 3);
          // nose wheel is not down
          if (GearForces[0] == 0)
          {
            CurrentState = WAIT_FOR_NOSE;
          }
          else
          {
            CurrentState = WAIT_FOR_FLYING;
          }
        }
        else
        {
          NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
        }
      }
      break;

    case WAIT_FOR_NOSE:
      if (Terminate_Motion)
      {
        CurrentState = WAIT_FOR_FLYING;
      }
      else
      {
        // wait for all wheels on the ground
        if (XPLMGetDatai(AllWheelsOnGroundRef) == TRUE)
        {
          // small bump
          TargetPilotY = InitialHeadPosition.y - 0.005;
          XPLMCommandBegin(DownCommand);
#if DIAGNOSTIC == 1
          Diagnostic_printf("Nose down so moving head down, target position of %f\n", TargetPilotY);
#endif // DIAGNOSTIC

          CurrentState = TOUCHDOWN;
          NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
        }
      }
      break;
  }

  return NextInterval;
}

// handles the release command
static int ReleaseCmdHandler
(
  XPLMCommandRef inCommand,
  XPLMCommandPhase inPhase,
  void *inRefcon
)
{
  // If inPhase == 0 the command is executed once on button down.
  if (inPhase == 0)
  {
    DisableTouchDownMotion();
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
  // user chose to release the parking brake
  if ((int)inItemRef == MENU_ITEM_ID_TOUCHDOWN_ENABLE)
  {
    Enabled = !Enabled;

    XPLMCheckMenuItem(myMenu, MenuItem_Enable, Enabled ? xplm_Menu_Checked : xplm_Menu_Unchecked);
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE API

// initalizes the module
// returns TRUE for success, FALSE for error
int HeadMotion_Init
  (
  XPLMMenuID ParentMenuId
  )
{
  Enabled = FALSE;

  int mySubMenuItem = XPLMAppendMenuItem(
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
  MenuItem_Enable = XPLMAppendMenuItem(
    myMenu,
    "Enable touch-down motion",
    (void *)MENU_ITEM_ID_TOUCHDOWN_ENABLE,
    1);

  if (Enabled == TRUE)
  {
    XPLMCheckMenuItem(myMenu, MenuItem_Enable, xplm_Menu_Checked);
  }

  // get datarefs
  PilotXRef = XPLMFindDataRef("sim/graphics/view/pilots_head_x");
  if (PilotXRef == NULL)
  {
    return FALSE;
  }
  PilotYRef = XPLMFindDataRef("sim/graphics/view/pilots_head_y");
  if (PilotYRef == NULL)
  {
    return FALSE;
  }
  PilotZRef = XPLMFindDataRef("sim/graphics/view/pilots_head_z");
  if (PilotZRef == NULL)
  {
    return FALSE;
  }
  PilotHeadingRef = XPLMFindDataRef("sim/graphics/view/pilots_head_psi");
  if (PilotHeadingRef == NULL)
  {
    return FALSE;
  }
  PilotPitchRef = XPLMFindDataRef("sim/graphics/view/pilots_head_the");
  if (PilotPitchRef == NULL)
  {
    return FALSE;
  }
  PilotRollRef = XPLMFindDataRef("sim/graphics/view/pilots_head_phi");
  if (PilotRollRef == NULL)
  {
    return FALSE;
  }
  AnyWheelOnGroundRef = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
  if (AnyWheelOnGroundRef == NULL)
  {
    return FALSE;
  }
  AllWheelsOnGroundRef = XPLMFindDataRef("sim/flightmodel/failures/onground_all");
  if (AllWheelsOnGroundRef == NULL)
  {
    return FALSE;
  }

  UpwardGearGroundForceNRef = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
  if (UpwardGearGroundForceNRef == NULL)
  {
    return FALSE;
  }
  TotalDownwardGForceRef = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");
  if (TotalDownwardGForceRef == NULL)
  {
    return FALSE;
  }
  GearVerticalForceNmRef = XPLMFindDataRef("sim/flightmodel2/gear/tire_vertical_force_n_mtr");
  if (GearVerticalForceNmRef == NULL)
  {
    return FALSE;
  }
  VerticalSpeedRef = XPLMFindDataRef("sim/flightmodel/position/local_vy");
  if (VerticalSpeedRef == NULL)
  {
    return FALSE;
  }
  FlightTimeRef = XPLMFindDataRef("sim/time/total_flight_time_sec");
  if (FlightTimeRef == NULL)
  {
    return FALSE;
  }
  RegularDownCmd = XPLMFindCommand("sim/general/down");
  if (RegularDownCmd == NULL)
  {
    return FALSE;
  }
  RegularUpCmd = XPLMFindCommand("sim/general/up");
  if (RegularUpCmd == NULL)
  {
    return FALSE;
  }
  FastDownCmd = XPLMFindCommand("sim/general/down_fast");
  if (FastDownCmd == NULL)
  {
    return FALSE;
  }
  FastUpCmd = XPLMFindCommand("sim/general/up_fast");
  if (FastUpCmd == NULL)
  {
    return FALSE;
  }

  // register the state machine callback
  XPLMRegisterFlightLoopCallback(StateMachine, STATE_MACHINE_EXECUTION_INTERVAL_NORMAL, NULL);

  HaveInitialHeadPosition = FALSE;

  return TRUE;
}

// called when a message is received from X-plane
void HeadMotion_ReceiveMessage
  (
  XPLMPluginID inFromWho,
  int	inMessage,
  void *inParam
  )
{
  // a new aircraft has been loaded
  if (inMessage == XPLM_MSG_PLANE_LOADED)
  {
//#if DIAGNOSTIC == 1
    //Diagnostic_printf("Ready to go\n");
//#endif // DIAGNOSTIC
    //Ready = TRUE;
    //CurrentState = START;
    //HaveInitialHeadPosition = FALSE;
    PreviousFlightTime = 10;
  }
  else if ((inMessage == XPLM_MSG_PLANE_UNLOADED) || (inMessage == XPLM_MSG_PLANE_CRASHED))
  {
#if DIAGNOSTIC == 1
    Diagnostic_printf("Aircraft crashed or unloaded\n");
#endif // DIAGNOSTIC
    Terminate_Motion = TRUE;
  }
}

// HEAD MOTION

// Moves the pilots head in response to turbulence and touch down

#include <math.h>
#include "HeadMotion.h"
#include "Diagnostic.h"

#define MODULE_NAME "Head Motion"

// time between executions of the state machine, in seconds
#define STATE_MACHINE_EXECUTION_INTERVAL_NORMAL 0.25f
#define STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE 0.01f
// set to 1 to enable diagnostic output to Log.txt
#define DIAGNOSTIC 1

// menu item IDs
#define MENU_ITEM_ID_TOUCHDOWN_DISABLE 1

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
static XPLMCommandRef DownFastCmd               = NULL;
static XPLMCommandRef UpFastCmd                 = NULL;

// prototype for the function that handles menu choices
static void	MenuHandlerCallback(void *inMenuRef, void *inItemRef);

// state machine states
typedef enum _states_t
{
  START,
  WAIT_FOR_TAKEOFF,
  WAIT_FOR_LANDING,
  TOUCHDOWN,
  RESTORING_POSITION
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
static pilots_head_t InitialHeadPosition;
static double LandingShakeAmplitude;
static double TargetPilotY;
static XPLMCommandRef ActiveMovementCommand;


////////////////////////////////////////////////////////////////////////////////////////////////////////
// INTERNAL FUNCTIONS

/*// rotates the pilots head up or down
static void MovePilotsHeadUpDown
(
  double Degrees  // negative is down, positive is up
)
{
  double PilotPitch = XPLMGetDataf(PilotPitchRef);
  XPLMSetDataf(PilotPitchRef, PilotPitch + Degrees);
}

// raises or lowers the pilots head
static void RaiseLowerPilotsHead
  (
  double Meters  // negative is down, positie is up
  )
{
  double PilotY = XPLMGetDataf(PilotYRef);
  XPLMSetDataf(PilotYRef, PilotY + Meters);
}

// sets the height of the pilots head
static void SetPilotHeadHeight
  (
  double Meters  // height to set
  )
{
  XPLMSetDataf(PilotYRef, Meters);
}*/

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

// starts the pilots head motion
static void StartMotion
  (
  double TargetPilotY  // target Y position for pilot's head
  )
{
  double CurrentPilotY = XPLMGetDataf(PilotYRef);
  if (TargetPilotY < CurrentPilotY)
  {
    ActiveMovementCommand = DownFastCmd;
    XPLMCommandBegin(ActiveMovementCommand);
  }
  else if (TargetPilotY > CurrentPilotY)
  {
    ActiveMovementCommand = UpFastCmd;
    XPLMCommandBegin(ActiveMovementCommand);
  }
}

// stops the current motion
static void StopMotion
  (
  void
  )
{
  if (ActiveMovementCommand != NULL)
  {
    XPLMCommandEnd(ActiveMovementCommand);
    ActiveMovementCommand = NULL;
  }
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

  if (Ready == FALSE) return NextInterval;

  switch (CurrentState)
  {
    case START:
      // wait for x-plane to drop the plane onto the ground at the start
      // of the simulation
      if (elapsedSim >= 2.0)
      {
#if DIAGNOSTIC == 1
        Diagnostic_printf("Waiting for X-plane to finish initial aircraft drop\n");
#endif // DIAGNOSTIC
        CurrentState = WAIT_FOR_TAKEOFF;
      }
      break;

    case WAIT_FOR_TAKEOFF:
      // wait for all wheels off the ground
      if (XPLMGetDatai(AnyWheelOnGroundRef) == FALSE)
      {
#if DIAGNOSTIC == 1
        Diagnostic_printf("All wheels off the ground, waiting for landing...\n");
#endif // DIAGNOSTIC
        CurrentState = WAIT_FOR_LANDING;
      }
      break;

    case WAIT_FOR_LANDING:
      // at least one wheel on the ground
      if (XPLMGetDatai(AnyWheelOnGroundRef) == TRUE)
      {
#if DIAGNOSTIC == 1
        Diagnostic_printf("At least one wheel on the ground, start landing head motion\n");
        XPLMGetDatavf(GearVerticalForceNmRef, GearForces, 0, 3);
        Diagnostic_printf("%fN %fG %fNm %fNm %fNm\n", XPLMGetDataf(UpwardGearGroundForceNRef), XPLMGetDataf(TotalDownwardGForceRef), GearForces[0], GearForces[1], GearForces[2]);
#endif // DIAGNOSTIC
        CurrentState = TOUCHDOWN;
        TouchdownTime = elapsedSim;
        GetHeadPosition(&InitialHeadPosition);
#if DIAGNOSTIC == 1
        Diagnostic_printf("Got head height of = %f\n", InitialHeadPosition.y);
#endif // DIAGNOSTIC

        double TouchDownG = XPLMGetDataf(TotalDownwardGForceRef) - 1.0;

        // scale shaking amplitude so TouchdownG 0.0 -> 2.0 = shake amplitude 0.0 -> 0.05

        double Slope = 2.0 / 0.05;
        LandingShakeAmplitude = TouchDownG / Slope;

#if DIAGNOSTIC == 1
        Diagnostic_printf("Landing shake amplitude = %fm\n", LandingShakeAmplitude);
#endif // DIAGNOSTIC

        //LandingShakeAmplitude = 0.05;
        //LandingShakeAmplitude = 0.005;

        // https://www.calculushowto.com/calculus-definitions/damped-sine-wave/
        // https://www.desmos.com/calculator
        // y\ =\ -\left(0.1e^{-2x}\cdot\cos\left(30x\right)\right)

        double PilotYOffset = -(LandingShakeAmplitude * exp(-2 * (elapsedSim - TouchdownTime)) * cos(30 * (elapsedSim - TouchdownTime)));
        TargetPilotY = InitialHeadPosition.y + PilotYOffset;
        StartMotion(TargetPilotY);

        NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
      }
      break;

    case TOUCHDOWN:
      // if it's been 1.5 seconds since first wheel down then
      // end head motion
      if (elapsedSim > (TouchdownTime + 1.5))
      {
#if DIAGNOSTIC == 1
        Diagnostic_printf("On ground long enough, restoring y to %f\n", InitialHeadPosition.y);
#endif // DIAGNOSTIC

        StartMotion(InitialHeadPosition.y);
        CurrentState = RESTORING_POSITION;

        NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
      }
      else
      {
        // check if we need to change the direction of movement
        double CurrentPilotY = XPLMGetDataf(PilotYRef);
        bool ChangeDirection = FALSE;
        if (ActiveMovementCommand == DownFastCmd)
        {
          if (CurrentPilotY <= TargetPilotY)
          {
            XPLMCommandEnd(ActiveMovementCommand);
            ChangeDirection = TRUE;
            ActiveMovementCommand = NULL;
          }
        }
        else if (ActiveMovementCommand == UpFastCmd)
        {
          if (CurrentPilotY >= TargetPilotY)
          {
            XPLMCommandEnd(ActiveMovementCommand);
            ChangeDirection = TRUE;
            ActiveMovementCommand = NULL;
          }
        }

        // change direction of movement
        if (ChangeDirection)
        {
          double PilotYOffset = -(LandingShakeAmplitude * exp(-2 * (elapsedSim - TouchdownTime)) * cos(30 * (elapsedSim - TouchdownTime)));
          TargetPilotY = InitialHeadPosition.y + PilotYOffset;
          StartMotion(TargetPilotY);
        }
        NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
      }
      break;

    case RESTORING_POSITION:
      {
        double CurrentPilotY = XPLMGetDataf(PilotYRef);

        if (ActiveMovementCommand == DownFastCmd)
        {
          if (CurrentPilotY <= InitialHeadPosition.y)
          {
#if DIAGNOSTIC == 1
            Diagnostic_printf("Moving head down, reached initial position of %f (current = %f)\n", InitialHeadPosition.y, CurrentPilotY);
#endif // DIAGNOSTIC
            XPLMCommandEnd(ActiveMovementCommand);
            ActiveMovementCommand = NULL;
            CurrentState = WAIT_FOR_TAKEOFF;
          }
          else
          {
            NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
          }
        }
        else if (ActiveMovementCommand == UpFastCmd)
        {
          if (CurrentPilotY >= InitialHeadPosition.y)
          {
#if DIAGNOSTIC == 1
            Diagnostic_printf("Moving head up, reached initial position of %f (current = %f)\n", InitialHeadPosition.y, CurrentPilotY);
#endif // DIAGNOSTIC
            XPLMCommandEnd(ActiveMovementCommand);
            ActiveMovementCommand = NULL;
            CurrentState = WAIT_FOR_TAKEOFF;
          }
          else
          {
            NextInterval = STATE_MACHINE_EXECUTION_INTERVAL_PERFORMANCE;
          }
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
  if ((int)inItemRef == MENU_ITEM_ID_TOUCHDOWN_DISABLE)
  {
    DisableTouchDownMotion();
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
  XPLMMenuID myMenu;
  int mySubMenuItem;

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
    "Disable touch-down motion",
    (void *)MENU_ITEM_ID_TOUCHDOWN_DISABLE,
    1);

  // create custom command
  char CmdName[100];
  sprintf_s(CmdName, 100, "%s//%s//Disable touch-down motion", PLUGIN_NAME, MODULE_NAME);
  char CmdDesc[100];
  sprintf_s(CmdDesc, 100, "Disable touch-down motion (%s-%s)", PLUGIN_NAME, MODULE_NAME);
  DisableTouchDownCmd = XPLMCreateCommand(CmdName, CmdDesc);
  XPLMRegisterCommandHandler(
    DisableTouchDownCmd, // in Command name
    ReleaseCmdHandler,   // in Handler
    1,                   // Receive input before plugin windows
    (void *)0);          // inRefcon

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
  DownFastCmd = XPLMFindCommand("sim/general/down_fast");
  if (DownFastCmd == NULL)
  {
    return FALSE;
  }
  UpFastCmd = XPLMFindCommand("sim/general/up_fast");
  if (UpFastCmd == NULL)
  {
    return FALSE;
  }

  // register the state machine callback
  XPLMRegisterFlightLoopCallback(StateMachine, STATE_MACHINE_EXECUTION_INTERVAL_NORMAL, NULL);

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
#if DIAGNOSTIC == 1
    Diagnostic_printf("Ready to go\n");
#endif // DIAGNOSTIC
    Ready = TRUE;
    CurrentState = START;
    GetHeadPosition(&InitialHeadPosition);
  }
  else if ((inMessage == XPLM_MSG_PLANE_UNLOADED) || (inMessage == XPLM_MSG_PLANE_CRASHED))
  {
    CurrentState = START;
//#if DIAGNOSTIC == 1
//    Diagnostic_printf("Plane crashed or unloaded, stopped\n");
//#endif // DIAGNOSTIC
//    Ready = FALSE;
    StopMotion();
  }
}

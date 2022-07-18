// PARKING BRAKE

// This plugin will release the parking brake
// Needed for gliders that don't have a brake control in the cockpit

#include "ParkingBrake.h"
#include "Diagnostic.h"

#define MODULE_NAME "Parking Brake"

// menu item IDs
#define MENU_ITEM_ID_RELEASE 1

// commands and data references that we need
static XPLMCommandRef BrakeMaxCmd = NULL;

// custom commands
static XPLMCommandRef ReleaseCmd = NULL;

// prototype for the function that handles menu choices
static void	MenuHandlerCallback(void *inMenuRef, void *inItemRef);


////////////////////////////////////////////////////////////////////////////////////////////////////////
// INTERNAL FUNCTIONS

// releases the parking brake
static void ReleaseBrake
  (
  void
  )
{
  XPLMCommandBegin(BrakeMaxCmd);
  XPLMCommandEnd(BrakeMaxCmd);

  XPLMSpeakString("Brake released");
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
    ReleaseBrake();
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
  if ((int)inItemRef == MENU_ITEM_ID_RELEASE)
  {
    ReleaseBrake();
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE API

// initalizes the module
// returns TRUE for success, FALSE for error
int ParkingBrake_Init
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
    "Release",
    (void *)MENU_ITEM_ID_RELEASE,
    1);

  // create custom command
  char CmdName[100];
  sprintf_s(CmdName, 100, "%s//%s//Release", PLUGIN_NAME, MODULE_NAME);
  char CmdDesc[100];
  sprintf_s(CmdDesc, 100, "Release the brake (%s-%s)", PLUGIN_NAME, MODULE_NAME);
  ReleaseCmd = XPLMCreateCommand(CmdName, CmdDesc);
  XPLMRegisterCommandHandler(
    ReleaseCmd,         // in Command name
    ReleaseCmdHandler,  // in Handler
    1,                 // Receive input before plugin windows.
    (void *)0);        // inRefcon.

  // get commands
  BrakeMaxCmd = XPLMFindCommand("sim/flight_controls/brakes_max");
  if (BrakeMaxCmd == NULL)
  {
    return FALSE;
  }

  return TRUE;
}

// called when a message is received from X-plane
void ParkingBrake_ReceiveMessage
  (
  XPLMPluginID inFromWho,
  int	inMessage,
  void *inParam
  )
{
}

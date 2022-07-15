// XVR Tools
// Version 1.0.0
// (C) andy@britishideas.com 2022, free for personal use, no commercial use
// For X-Plane 11.55

// This plugin provides various features and tools that are useful when using
// X-Plane in pure VR

#include <stdio.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include "Diagnostic.h"
#include "Global.h"

#include "LandingThrottleManager.h"
#include "ParkingBrake.h"
#include "HeadMotion.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////
// INTERNAL FUNCTIONS


////////////////////////////////////////////////////////////////////////////////////////////////////////
// PLUGIN API

// called from x-plane to initialize the plugin
// returns TRUE for success, FALSE for error
PLUGIN_API int XPluginStart
  (
  char *outName,  // on return filled with user-friendly plugin name
  char *outSig,   // on return filled with a unique signature
  char *outDesc   // on return filled with a description or error message
  )
{
  XPLMMenuID myMenu;
  int	mySubMenuItem;

  Diagnostic_printf("%s version %d.%d.%d\n", PLUGIN_NAME, PLUGIN_VERSION_MAJOR, PLUGIN_VERSION_MINOR, PLUGIN_VERSION_DOT);
  Diagnostic_printf("%s\n", PLUGIN_COPYRIGHT);

  // Provide our plugin's profile to the plugin system
  strcpy_s(outName, 256, PLUGIN_NAME);
  strcpy_s(outSig, 256, "britishideas.assistants.xvrtools");
  strcpy_s(outDesc, 256, "Tools for VR users");

	// First we put a new menu item into the plugin menu.
	// This menu item will contain a submenu for us
	mySubMenuItem = XPLMAppendMenuItem(
		XPLMFindPluginsMenu(),	     // Put in plugins menu
    PLUGIN_NAME,	               // Item Title
		0,						               // Item Ref
		1);						               // Force English
	
	// Now create a submenu attached to our menu item
	myMenu = XPLMCreateMenu(
    PLUGIN_NAME,
		XPLMFindPluginsMenu(), 
		mySubMenuItem, 			    // Menu Item to attach to
		NULL,	  // The handler
		0);						          // Handler Ref

  if (!LandingThrottleManager_Init(myMenu))
  {
    return FALSE;
  }

  if (!ParkingBrake_Init(myMenu))
  {
    return FALSE;
  }

  if (!HeadMotion_Init(myMenu))
  {
    return FALSE;
  }

  return TRUE;
}

PLUGIN_API void	XPluginStop
  (
  void
  )
{
}

PLUGIN_API void XPluginDisable
  (
  void
  )
{
}

PLUGIN_API int XPluginEnable
  (
  void
  )
{
  return 1;
}

// called when a message is received from X-plane
PLUGIN_API void XPluginReceiveMessage
  (
  XPLMPluginID inFromWho,
  int	inMessage,
  void *inParam
  )
{
  LandingThrottleManager_ReceiveMessage(inFromWho, inMessage, inParam);
  ParkingBrake_ReceiveMessage(inFromWho, inMessage, inParam);
  HeadMotion_ReceiveMessage(inFromWho, inMessage, inParam);
}

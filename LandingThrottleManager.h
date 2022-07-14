#ifndef _LANDINGTHROTTLEMANAGERH_
#define _LANDINGTHROTTLEMANAGERH_

#include "Global.h"

// initalizes the module
// returns TRUE for success, FALSE for error
extern int LandingThrottleManager_Init
  (
  XPLMMenuID ParentMenuId
  );

// called when a message is received from X-plane
extern void LandingThrottleManager_ReceiveMessage
  (
  XPLMPluginID inFromWho,
  int	inMessage,
  void *inParam
  );

#endif // _LANDINGTHROTTLEMANAGERH_

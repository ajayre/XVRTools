#ifndef _PARKINGBRAKEH_
#define _PARKINGBRAKEH_

#include "Global.h"

// initalizes the module
// returns TRUE for success, FALSE for error
extern int ParkingBrake_Init
  (
  XPLMMenuID ParentMenuId
  );

// called when a message is received from X-plane
extern void ParkingBrake_ReceiveMessage
  (
  XPLMPluginID inFromWho,
  int	inMessage,
  void *inParam
  );

#endif // _PARKINGBRAKEH_

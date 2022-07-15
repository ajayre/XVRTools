#ifndef _HEADMOTIONH_
#define _HEADMOTIONH_

#include "Global.h"

// initalizes the module
// returns TRUE for success, FALSE for error
extern int HeadMotion_Init
  (
  XPLMMenuID ParentMenuId
  );

// called when a message is received from X-plane
extern void HeadMotion_ReceiveMessage
  (
  XPLMPluginID inFromWho,
  int	inMessage,
  void *inParam
  );

#endif // _HEADMOTIONH_

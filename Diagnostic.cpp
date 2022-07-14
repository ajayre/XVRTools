#include "Diagnostic.h"

// prints a diagnostic line to Log.txt
// accepts the same arguments as printf
void Diagnostic_printf
  (
  char *str,
  ...
  )
{
  va_list lst;
  char line[256];

  va_start(lst, str);
  vsprintf_s(line, 256, str, lst);
  XPLMDebugString(PLUGIN_NAME);
  XPLMDebugString(": ");
  XPLMDebugString(line);
  va_end(lst);
}

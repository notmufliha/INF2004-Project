#include "s5_bridge.h"
/* These are deliberately fail-closed. The real Buddy 2/3 implementation
 * must override them, rather than silently pretending motion succeeded. */
__attribute__((weak)) bool s5_bridge_ready(void) { return false; }
__attribute__((weak)) bool s5_bridge_request(S5Command cmd, const S5Profile *p)
{ (void)cmd; (void)p; return false; }
__attribute__((weak)) bool s5_bridge_stopped(void) { return false; }
__attribute__((weak)) bool s5_bridge_motion_complete(void) { return false; }
__attribute__((weak)) bool s5_bridge_line_found(void) { return false; }
__attribute__((weak)) void s5_bridge_emergency_stop(void) { }

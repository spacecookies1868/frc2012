#ifndef __DEBUGGING__
#define __DEBUGGING__

#include "WPILib.h"

#define TRACE_ENTRY 1

#if TRACE_ENTRY
#define ENTRY(s) { fprintf( stdout, "%s: %s, %d\n", s, __FUNCTION__, __LINE__ ); }
#else
#define ENTRY(s)
#endif

#define CHECK_NULL(p) if (!(p)) { fprintf( stderr, "NULL pointer: %s, %d\n", __FUNCTION__, __LINE__ ); }

#define DO_PERIODIC( period_count, stmt ) do { static int __count__ = 0; if (( __count__++ % ( period_count )) == 0 ) { stmt; }} while(0)

#define NEW_JOYSTICKS 1
#define USE_CAMERA 1

#define USE_CAMERA 1

#endif

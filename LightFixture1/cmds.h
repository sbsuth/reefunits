#if DEF_CMD_MACROS

#define MAX_ARGS 1
#define MAX_CMD_LEN 6

#else

X(  CmdPing,        "ping", false, CmdArgNone )
X(  CmdStop,        "stop", false, CmdArgNone )
X(  CmdUp,          "up",   false, CmdArgInt )
X(  CmdDown,        "down", false, CmdArgInt )
X(  CmdCalibrate,   "cal",  true,  CmdArgNone )
X(  CmdGetHeight,   "gh",   false, CmdArgNone )
X(  CmdRFState,     "rfs",  false, CmdArgNone )
X(  CmdDim,         "dim",  false, CmdArgInt )
X(  CmdNone,        "nop",  false, CmdArgNone )

#endif


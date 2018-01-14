#if DEF_CMD_MACROS

#define MAX_ARGS 1
#define MAX_CMD_LEN 5 // Not including 0.

#else

X(  CmdPing,        "ping", false,  CmdArgNone )
X(  CmdAllOff,      "aoff", false,  CmdArgNone )
X(  CmdPumpOn,      "pon",  true,   CmdArgInt )
X(  CmdPumpOff,     "poff", false,  CmdArgInt )
X(  CmdStatus,      "stat", false,  CmdArgNone )
X(  CmdROOn,        "roon", false,  CmdArgInt )
X(  CmdFill,        "fill", false,  CmdArgInt )
X(  CmdExtPause,    "paus", false,  CmdArgInt )
X(  CmdNone,        "nop",  false,  CmdArgNone )

#endif


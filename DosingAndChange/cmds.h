#if DEF_CMD_MACROS

#define MAX_ARGS 2
#define MAX_CMD_LEN 4 // Not including 0.

#else

X(  CmdPing,        "ping", false,  CmdArgNone )
X(  CmdPumpOn,      "pon",  false,  CmdArgInt )
X(  CmdPumpOff,     "poff", false,  CmdArgInt )
X(  CmdStatus,      "stat", false,  CmdArgNone )
X(  CmdNone,        "nop",  false,  CmdArgNone )

#endif


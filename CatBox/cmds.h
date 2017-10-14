#if DEF_CMD_MACROS

#define MAX_ARGS 1
#define MAX_CMD_LEN 5 // Not including 0.

#else

X(  CmdPing,        "ping", false,  CmdArgNone )
X(  CmdStatus,      "stat", false,  CmdArgNone )
X(  CmdOn,          "on",   false,  CmdArgNone )
X(  CmdOff,         "off",  false,  CmdArgNone )
X(  CmdNone,        "nop",  false,  CmdArgNone )

#endif


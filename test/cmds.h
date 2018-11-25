#if DEF_CMD_MACROS

#define MAX_ARGS 1
#define MAX_CMD_LEN 5 // Not including 0.

#else

X(  CmdPing,        "ping", false,  CmdArgNone )
X(  CmdSetRes,      "res",  false,  CmdArgInt )
X(  CmdGetTemp,     "get",  false,  CmdArgInt )
X(  CmdNone,        "nop",  false,  CmdArgNone )

#endif


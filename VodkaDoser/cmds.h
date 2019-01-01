#if DEF_CMD_MACROS

#define MAX_ARGS 1
#define MAX_CMD_LEN 5 // Not including 0.

#else

X(  CmdPing,        "ping", false,  CmdArgNone )
X(  CmdPumpOn,      "pon",  true,   CmdArgInt )
X(  CmdPumpSpeed,   "pspd", true,   CmdArgInt )
X(  CmdDispense,    "disp",  true,   CmdArgInt )
X(  CmdCal,         "calb",  true,   CmdArgNone )
X(  CmdCalRslt,     "calr",  true,   CmdArgInt )
X(  CmdNone,        "nop",  false,  CmdArgNone )

#endif


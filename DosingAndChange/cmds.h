#if DEF_CMD_MACROS

#define MAX_ARGS 2
#define MAX_CMD_LEN 4 // Not including 0.

#else

X(  CmdPing,        "ping", false,  CmdArgNone )
X(  CmdStatus,      "stat", false,  CmdArgNone )
X(  CmdEnable,      "en",   false,  CmdArgInt )
X(  CmdResetAll,    "rall", false,  CmdArgNone )
X(  CmdDispense,    "disp", true,   CmdArgInt )
X(  CmdStepsPerMl,  "cals", true,   CmdArgInt )
X(  CmdCal,         "calb", true,   CmdArgNone )
X(  CmdCalRslt,     "calr", true,   CmdArgInt )
X(  CmdNone,        "nop",  false,  CmdArgNone )

#endif


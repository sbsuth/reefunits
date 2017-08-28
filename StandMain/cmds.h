#if DEF_CMD_MACROS

#define MAX_ARGS 2
#define MAX_CMD_LEN 5 // Not including 0.

#else

X(  CmdPing,            "ping", false,  CmdArgNone )
X(  CmdStatus,          "stat", false,  CmdArgNone )
X(  CmdSw,              "sw",   true,   CmdArgInt )
X(  CmdPumpSpeed,       "pspd", true,   CmdArgInt )
X(  CmdSaveSettings,    "sset", false,  CmdArgNone )
X(  CmdRestoreSettings, "rset", false,  CmdArgNone )
X(  CmdNone,            "nop",  false,  CmdArgNone )

#endif


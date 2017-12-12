#if DEF_CMD_MACROS

#define MAX_ARGS 3
#define MAX_CMD_LEN 5 // Not including 0.

#else

X(  CmdPing,            "ping", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdStatus,          "stat", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdPumpStatus,      "pstat", false, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdSw,              "sw",   true,   CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdPumpSpeed,       "pspd", true,   CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdSaveSettings,    "sset", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdRestoreSettings, "rset", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdPumpMode,        "mset", true,   CmdArgInt,  CmdArgFloat,CmdArgFloat )
X(  CmdTempShutdown,    "tshut", true,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdCalEC,           "cale", false,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdCalPH,           "calp", false,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdHeat,            "heat", false,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdSetTargetTemp,   "stt",  false,  CmdArgFloat,CmdArgInt,  CmdArgFloat )
X(  CmdCalTemp,         "calt", true,   CmdArgInt,  CmdArgFloat, CmdArgNone )
X(  CmdNone,            "nop",  false,  CmdArgNone, CmdArgNone, CmdArgNone )

#endif


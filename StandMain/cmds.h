#if DEF_CMD_MACROS

#define MAX_ARGS 3
#define MAX_CMD_LEN 5 // Not including 0.
#define MAX_STR_ARG_LEN 16

#else

X(  CmdPing,            "ping", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdStatus,          "stat", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdPumpStatus,      "pstat", false, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdSw,              "sw",   true,   CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdPumpSpeed,       "pspd", true,   CmdArgInt,  CmdArgInt,  CmdArgNone )
X(  CmdSaveSettings,    "sset", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdRestoreSettings, "rset", false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdPumpMode,        "mset", true,   CmdArgInt,  CmdArgFloat,CmdArgFloat )
X(  CmdPumpCurSet,      "cset", true,   CmdArgInt,  CmdArgInt,  CmdArgInt )
X(  CmdTempShutdown,    "tshut", false, CmdArgInt,  CmdArgInt,  CmdArgInt )
X(  CmdCalEC,           "cale", false,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdCalPH,           "calp", false,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdHeat,            "heat", false,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdSetTargetTemp,   "stt",  false,  CmdArgFloat,CmdArgInt,  CmdArgFloat )
X(  CmdCalTemp,         "calt", false,   CmdArgFloat,CmdArgNone, CmdArgNone )
X(  CmdRenewRadio,      "rr",   false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdCalConsts,       "cc",   false,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdProbeCmd,        "pc",   true,   CmdArgStr,  CmdArgNone, CmdArgNone )
X(  CmdTC,              "tc",   true,   CmdArgInt,  CmdArgFloat, CmdArgNone )
X(  CmdNone,            "nop",  false,  CmdArgNone, CmdArgNone, CmdArgNone )

#endif


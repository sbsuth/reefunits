#if DEF_CMD_MACROS

#define MAX_ARGS 3
#define MAX_CMD_LEN 6

#else

X(  CmdPing,        "ping", false, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdStop,        "stop", false, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdUp,          "up",   false, CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdDown,        "down", false, CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdCalibrate,   "cal",  true,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdGetHeight,   "gh",   false, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdRFState,     "rfs",  false, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdDim,         "dim",  false, CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdSetLoc,      "sloc", false, CmdArgFloat,CmdArgFloat,CmdArgInt )
X(  CmdSetTime,     "stime", false, CmdArgLong, CmdArgNone,  CmdArgNone )
X(  CmdStat,        "stat", false, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdGetCurVals,  "gvals", false, CmdArgNone, CmdArgNone, CmdArgNone)
X(  CmdSetMaxPct,   "spct", true,  CmdArgInt,  CmdArgInt,  CmdArgNone )
X(  CmdGetMaxPcts,  "gpct", true,  CmdArgNone,  CmdArgNone,  CmdArgNone )
X(  CmdSetLevel,    "slev", true,  CmdArgInt,  CmdArgFloat,  CmdArgNone )
X(  CmdSetMode,     "mode", false,  CmdArgInt,  CmdArgInt,  CmdArgNone )
X(  CmdGetCycle,    "gcyc", false,  CmdArgNone,  CmdArgNone,  CmdArgNone )
X(  CmdDumpDay,     "dday", false,  CmdArgInt,  CmdArgNone,  CmdArgNone )
X(  CmdSaveSettings,"sset", false,  CmdArgNone, CmdArgNone, CmdArgNone)
X(  CmdRestoreSettings,"rset", false,  CmdArgNone, CmdArgNone, CmdArgNone)
X(  CmdNone,        "nop",  false, CmdArgNone, CmdArgNone, CmdArgNone)

#endif


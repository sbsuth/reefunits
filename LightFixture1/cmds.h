#if DEF_CMD_MACROS

#define MAX_ARGS 4
#define MAX_CMD_LEN 6

#else

X(  CmdPing,        "ping", false, CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdStop,        "stop", false, CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdUp,          "up",   false, CmdArgInt,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdDown,        "down", false, CmdArgInt,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdCalibrate,   "cal",  true,  CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdGetHeight,   "gh",   false, CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdRFState,     "rfs",  false, CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdDim,         "dim",  false, CmdArgInt,  CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdSetLoc,      "sloc", false, CmdArgFloat,CmdArgFloat,CmdArgInt, CmdArgNone )
X(  CmdSetTime,     "stime", false, CmdArgLong, CmdArgNone,  CmdArgNone, CmdArgNone )
X(  CmdStat,        "stat", false, CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone )
X(  CmdGetCurVals,  "gvals", false, CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone)
X(  CmdSetMaxPct,   "spct", true,  CmdArgInt,  CmdArgInt,  CmdArgNone, CmdArgNone )
X(  CmdGetMaxPcts,  "gpct", true,  CmdArgNone,  CmdArgNone,  CmdArgNone, CmdArgNone )
X(  CmdSetMaxPctssA, "spcta", true,  CmdArgInt,  CmdArgInt,  CmdArgInt, CmdArgInt )
X(  CmdSetMaxPctssB, "spctb", true,  CmdArgInt,  CmdArgInt,  CmdArgInt, CmdArgInt )
X(  CmdSetLevel,    "slev", false,  CmdArgInt,  CmdArgInt,  CmdArgFloat, CmdArgNone )
X(  CmdSetMode,     "mode", false,  CmdArgInt,  CmdArgNone,  CmdArgNone, CmdArgNone )
X(  CmdSetSpectrum, "spec", false,  CmdArgInt,  CmdArgNone,  CmdArgNone, CmdArgNone )
X(  CmdSetDay,      "sday", false,  CmdArgLong,  CmdArgLong,  CmdArgNone, CmdArgNone )
X(  CmdGetCycle,    "gcyc", false,  CmdArgNone,  CmdArgNone,  CmdArgNone, CmdArgNone )
X(  CmdDumpDay,     "dday", false,  CmdArgInt,  CmdArgNone,  CmdArgNone, CmdArgNone )
X(  CmdSaveSettings,"sset", false,  CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone)
X(  CmdRestoreSettings,"rset", false,  CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone)
X(  CmdNone,        "nop",  false, CmdArgNone, CmdArgNone, CmdArgNone, CmdArgNone)

#endif


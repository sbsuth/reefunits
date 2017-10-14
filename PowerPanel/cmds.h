#if DEF_CMD_MACROS

#define MAX_ARGS 1
#define MAX_CMD_LEN 4 // Not including 0.

#else

X(  CmdPing,        "ping", false, CmdArgNone )
X(  CmdAllOff,      "aoff", false, CmdArgNone )
X(  CmdOn,          "on",   false, CmdArgInt )
X(  CmdOff,         "off",  false, CmdArgInt )
X(  CmdStatus,      "stat", false, CmdArgNone )
X(  CmdSaveSettings,"sset", false,  CmdArgNone )
X(  CmdRestoreSettings,"rset", false,  CmdArgNone )
X(  CmdNone,        "nop",  false, CmdArgNone )

#endif


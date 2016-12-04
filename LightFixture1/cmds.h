#if DEF_CMD_TYPES

#define MAX_ARGS 2
#define MAX_CMD_LEN 6

enum CommandKind {
	CmdPing,
	CmdStop,
	CmdUp,
	CmdDown,
	CmdCalibrate,
	CmdGetHeight,
	CmdRFState,
	CmdNone
};

#endif
#if DEF_CMDS

const char string_0[] PROGMEM = "ping";
const char string_1[] PROGMEM = "stop";
const char string_2[] PROGMEM = "up"; 
const char string_3[] PROGMEM = "down";
const char string_4[] PROGMEM = "cal";
const char string_5[] PROGMEM = "gh";
const char string_6[] PROGMEM = "rfs";
const char string_7[] PROGMEM = "nop";

const CommandDescr PROGMEM g_commandDescrs[] = {
    { CmdPing,      string_0, false  },
    { CmdStop,      string_1, false  },
    { CmdUp,        string_2, false,  CmdArgInt },
    { CmdDown,      string_3, false,  CmdArgInt },
    { CmdCalibrate, string_4, true   },
    { CmdGetHeight, string_5, false   },
    { CmdRFState,   string_6, false   },
    { CmdNone,      string_7, false  },
};
#endif

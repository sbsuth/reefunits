#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>
#include "common.h"

#define DEF_CMD_MACROS 1
#include "cmds.h"
#undef DEF_CMD_MACROS

#if MAX_ARGS==1
#define X(a,b,c,d) a,
#elif MAX_ARGS==2
#define X(a,b,c,d,e) a,
#elif MAX_ARGS==3
#define X(a,b,c,d,e,f) a,
#else
#error "Unsupported MAX_ARGS"
#endif
enum CommandKind {
    #include "cmds.h"
};
#undef X

#if !USE_JSON
class JsonObject;
#endif

enum CommandArgKind {
    CmdArgNone,
    CmdArgInt,
    CmdArgFloat,
    CmdArgStr,
};

class InStream;
class CommandParser;

class CommandArg {
  public:
    CommandArg( CommandArgKind k=CmdArgNone )
        : m_kind(k), m_set(false)
    {}
    CommandArgKind    kind() {
        return m_kind;
    }
    void reset() {
        m_set = false;
        m_kind = CmdArgNone;
    }
    bool isSet() const {
        return m_set;
    }
    void setKind( CommandArgKind k ) {
        m_kind = k;
    }
    void setInt( int v ) {
        m_kind = CmdArgInt;
        m_val.i = v;
        m_set = true;
    }
    bool getInt( int& v ) {
        if (m_kind == CmdArgInt) {
            v = m_val.i;
            return true;
        } else {
            return false;
        }
    }
    void setFloat( float v ) {
        m_kind = CmdArgFloat;
        m_val.f = v;
        m_set = true;
    }
    bool getFloat( float& v ) {
        if (m_kind == CmdArgFloat) {
            v = m_val.f;
            return true;
        } else {
            return false;
        }
    }
    void setStr( char* v ) {
        m_kind = CmdArgStr;
        m_val.s = v;
        m_set = true;
    }
    bool getStr( const char*& v ) {
        if (m_kind == CmdArgStr) {
            v = m_val.s;
            return true;
        } else {
            return false;
        }
    }
  protected:
    CommandArgKind    m_kind;
    bool    m_set;
    union {
        int     i;
        float   f;
        char*   s;
    }   m_val;
};

class Command {
  public:

    Command() 
        : m_parser(0)
    {
        reset();
    }

    void reset() {
        m_kind = CmdNone;
        m_id = 0;
        for ( int i=0; i < MAX_ARGS; i++ )
            m_args[i].reset();
    }

    CommandKind kind() const {
        return m_kind;
    }
    void setKind( CommandKind k) {
        m_kind = k;
    }

    short ID() const {
        return m_id;
    }
    void setID( short i ) {
        m_id = i;
    }

    void printName();

    uchar numArgs() const {
        uchar i;
        for ( i=0; i < MAX_ARGS; i++ ) {
            if (!m_args[i].isSet())
                break;
        }
        return i;
    }
    CommandArg* arg( uchar i ) {
        if (i >= MAX_ARGS)
            return 0;
        CommandArg* arg = &m_args[i];
        return arg;
    }
    CommandParser* parser() {
        return m_parser;
    }
    void setParser( CommandParser* p ) {
        m_parser = p;
    }
    inline void ack();
    inline void ack( JsonObject& json );
    inline void ack( JsonArray& json );
    void ack( bool val );
    void ack( int val );
    void ack( float val );
    void ack( const char* val );
  protected:
    CommandKind m_kind;
    short       m_id;
    CommandArg  m_args[MAX_ARGS];
    CommandParser* m_parser;
};

// A CommandDescr is presumed to be in PROGMEM, so all the accessors read values that way.
struct CommandDescr 
{
    CommandKind kind() const {
        unsigned char k = pgm_read_byte( &m_kind );
        return (CommandKind)k;
    }
    bool hasID() const {
        unsigned char hid = pgm_read_byte( &m_hasID );
        return hid;
    }
    char* getStr( char* s, short len ) const {
        char* sptr = (char*)pgm_read_ptr( &this->m_str );
        strncpy_P( s, sptr, len-1 );
        s[len-1] = 0;
        return s;
    }
    void printStr() const {
        char c;
        char* sptr = (char*)pgm_read_ptr( &this->m_str );
        while ((c=pgm_read_byte(sptr))) {
            Serial.print(c);
            sptr++;
        }
    }

    ushort numArgs() const {
        short n = 0;
        for ( short i=0; i < MAX_ARGS; i++ ) {
            CommandArgKind k = argKind(i);
            if (k == CmdArgNone)
                return i;
        }
        return MAX_ARGS;
    }
    CommandArgKind argKind( ushort i ) const {
        if (i < MAX_ARGS) {
            unsigned char k = pgm_read_byte( &m_args[i] );
            return (CommandArgKind)k;
        } else {
            return CmdArgNone;
        }
    }
    unsigned char m_kind;
    const char*  m_str;
    unsigned char m_hasID;
    unsigned char m_args[MAX_ARGS];
};

// Define PROGMEM string vars storing command name strings.
// THese must be separate decls for each one if they're to be part of
// a progmem array of structs that references them.
#if MAX_ARGS==1
#define X(a,b,c,d) const char string_##a[] PROGMEM = b;
#elif MAX_ARGS==2
#define X(a,b,c,d,e) const char string_##a[] PROGMEM = b;
#elif MAX_ARGS==3
#define X(a,b,c,d,e,f) const char string_##a[] PROGMEM = b;
#else
#error "Unsupported MAX_ARGS"
#endif
#include "cmds.h"
#undef X

// Define CommandDescr structs
#if MAX_ARGS==1
#define X(a,b,c,d) { a,      string_##a, c, d  },
#elif MAX_ARGS==2
#define X(a,b,c,d,e) { a,      string_##a, c, d, e  },
#elif MAX_ARGS==3
#define X(a,b,c,d,e,f) { a,      string_##a, c, d, e, f  },
#else
#error "Unsupported MAX_ARGS"
#endif

const CommandDescr PROGMEM g_commandDescrs[] = {
    #include "cmds.h"
};
#undef X

class InStream 
{
  public:
    InStream() {}
    virtual bool read( char*)=0;
    virtual bool write( char c );
    bool write( const char* s ) {
        while (*s) {
            if (!write(*s++))
                return false;
        }
        return true;
    }
    bool write_pgm( const char* s ) {
        char c;
        while ((c=pgm_read_byte(s))) {
            bool ok = write(c);
            if (!ok)
                return false;
            s++;
        }
        return true;
    }
    virtual void ack( Command* cmd ) {}
    virtual void ack( Command* cmd, JsonObject& json ) {
        ack(cmd);
    }
    virtual void ack( Command* cmd, JsonArray& json ) {
        ack(cmd);
    }
    virtual void pingActivity() {}
};

#if USE_STDIO
class StdInStream : public InStream 
{
  public:
    StdInStream()
    {
        fcntl (0, F_SETFL, O_NONBLOCK);
    }
    bool read( char* c ) {
        int r =  ::read (0, c, 1);
        return (r > 0);
    }
    bool write( char c ) {
        size_t n = ::write( 2, &c, 1 );
        return (n == 1);
    }
};
#endif

#if defined(ARDUINO)
class ArduinoSerialIO : public InStream 
{
  public:
    ArduinoSerialIO()
    {
    }
    bool read( char* c ) {
		if (Serial.available() > 0) {
			*c = Serial.read();
			return true;
		} else {
			return false;
		}
	}
    bool write( char c ) {
        Serial.write(c);
        return true;
    }
    virtual void ack( Command* cmd, JsonObject& json );
    virtual void ack( Command* cmd, JsonArray& json );
};
#endif

#if defined(__RF24NETWORK_H__)
class RF24SerialIO : public InStream 
{
  public:
    RF24SerialIO( RF24Network* network )
		: m_network(network)
    {
    }
    bool read( char* c ) {
		if (m_network->available()) {
			m_network->read( m_header, c, 1 );
			return true;
		} else {
			return false;
		}
	}
    bool write( char c ) {
		if (m_network->available()) {
            // Write to the same place we just read from.
		    return m_network->write( m_header, &c, 1 );
		} else {
			return false;
		}
    }
  protected:
    RF24Network* m_network;
    RF24NetworkHeader m_header;
};
#endif

#if defined(RF24Ethernet_h)
class EthernetSerialIO : public InStream 
{
  public:
    EthernetSerialIO( RF24IPInterface* interface )
		: m_interface(interface)
    {
    }
    bool read( char* c );
    bool write( char c ) {
        // write to the same client we just read from.
        m_client.write(c);
        return true;
    }
    virtual void ack( Command* cmd );
    virtual void ack( Command* cmd, JsonObject& json );
    virtual void ack( Command* cmd, JsonArray& json );

    EthernetClient& client() {
        return m_client;
    }
    virtual void pingActivity();
        
  protected:
    RF24IPInterface* m_interface;
    EthernetClient m_client; // Make it simple to get client from last command.
};
#endif


class CommandParser
{
  public:
    enum {MaxTokenLen=32};

    CommandParser( const CommandDescr* descrs, InStream* s=0 )
        : m_descrs(descrs)
        , m_stream(s)
        , m_state(ExpCmd)
        , m_tokenOK(false)
        , m_descr(0)
        , m_httpHeader(0)
        , m_lastWasCR(false)
    {
        reset();
    }

    Command* getCommand( Command* cmd, bool &error );

    void reset() {
        m_state = ExpCmd;
        m_tokenOK = false;
        m_descr = 0;
        m_token[MaxTokenLen] = 0;
        m_token[0] = 0;
        m_argNum = 0;
    }

    bool tokenOK() {
        return m_tokenOK;
    }

    uchar inHTTPHeader() {
        return m_httpHeader;
    }
    void setHTTPHeader( uchar h) {
        m_httpHeader = h;
    }

    InStream* stream() {
        return m_stream;
    }
    void setStream( InStream* s ) {
        m_stream = s;
    }

    const CommandDescr* getDescr( CommandKind k ) const ;
        
  protected:
    enum State { ExpCmd, ExpId, ExpInt, ExpFloat, ExpStr, ExpHTTP };

    bool setExpArg( int index, bool eol, bool& error );
    bool nextToken( bool& eol, bool& error );
    bool decodeCommand( bool &error );
    bool decodeInt( int& i );
    bool decodeFloat( float& f );
    void processHTTPHeader( bool eol );

    const CommandDescr* m_descrs;
    InStream* m_stream;
    State m_state;
    bool  m_tokenOK;
    int   m_argNum;
    char m_token[MaxTokenLen+1];
    const CommandDescr* m_descr;
    uchar m_httpHeader;
    bool  m_lastWasCR;
};

// Inline functions with order issues.
inline void Command::ack()
{
    parser()->stream()->ack(this);
}

inline void Command::ack( JsonObject& json )
{
    parser()->stream()->ack(this,json);
}

inline void Command::ack( JsonArray& json )
{
    parser()->stream()->ack(this,json);
}


#endif


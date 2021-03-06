#if USE_RF24
#include <RF24Interface.h>
#endif
#if USE_JSON
#include <ArduinoJson.h>
#endif
#include <Command.h>

//#define DEBUG_TOKEN 1
//#define DEBUG_CHAR 1
//#define DEBUG_HTTP 1
//#define DEBUG_ACK 1

// Static variable decls.

// Convert to lower case with optional enable.
char lc(char c, bool ic=false) {
    if (ic && (c >= 'A') && (c <= 'Z'))
        return c + ('a'-'A');
    else
        return c;
}

// String match with option to ignore case.
bool str_match( const char* cp1, const char* cp2, bool ic )
{
    while ( *cp1 && *cp2 && (lc(*cp1,ic) == lc(*cp2,ic))) {
        cp1++;
        cp2++;
    }
    return (*cp1 == *cp2);
}

bool CommandParser::decodeCommand( bool &error )
{
    for ( const CommandDescr* descr = m_descrs; descr->kind() != CmdNone; descr++ ) {
        const char* cp1 = m_token;
        char descrStr[MAX_CMD_LEN+1];
        const char* cp2 = descr->getStr( descrStr, MAX_CMD_LEN+1 );
        if (str_match(cp1,cp2,false)) {
            m_descr = descr;
            return true;
        }
    }
    error = true;
    return false;
}

const CommandDescr* CommandParser::getDescr( CommandKind kind ) const
{
    for ( const CommandDescr* descr = m_descrs; descr->kind() != CmdNone; descr++ ) {
        if (descr->kind() == kind)
            return descr;
    }
    return 0;
}

template <typename T>
bool CommandParser::decodeInt( T& i )
{
    T v = 0;
    char* cp = m_token;
    bool s = (*m_token == '-');
    if (s)
        cp++;

    while ((*cp >= '0') && (*cp <= '9')) {
        v = (v*10) + (*cp - '0');
        cp++;
    }

    if (s)
        i = -v;
    else
        i = v;

    return !(*cp);
}

bool CommandParser::decodeFloat( float& f )
{
    char* cp = m_token;
    bool s = (*m_token == '-');
    if (s)
        cp++;

    f = 0.0;

    while (*cp && (*cp >= '0') && (*cp <= '9')) {
        f = (f*10) + (*cp - '0');
        cp++;
    }
    if (*cp == '.') {
        cp++;
        float d = 0.1;
        while (*cp && (*cp >= '0') && (*cp <= '9')) {
            f += d * (*cp - '0');
            d /= 10.0;
            cp++;
        }
    }
    
    if (s)
        f = -f;

    return !(*cp);
}

bool CommandParser::setExpArg( int index, bool eol, bool& error )
{
    if (index >= m_descr->numArgs()) {
        reset();
        m_state = ExpCmd;
        return true;
    }
    if (eol) {
        error = true;
        reset();
        return false;
    }
    switch (m_descr->argKind(m_argNum)) {
        case CmdArgInt:
            m_state = ExpInt;
            break;
        case CmdArgLong:
            m_state = ExpLong;
            break;
        case CmdArgFloat:
            m_state = ExpFloat;
            break;
        case CmdArgStr:
            m_state = ExpStr;
            break;
        default:
            reset();
            m_state = ExpCmd;
            break;
    }
    return true;
}

bool CommandParser::nextToken( bool& eol, bool& error ) 
{
    char c;
    if (m_tokenOK) {
        m_tokenOK = false;
        m_token[0] = 0;
    }
        
    bool start = true;
    int ic = 0;
    char* pc = &m_token[0];
    while ((ic < MaxTokenLen) && *pc) {
        ic++;
        pc++;
        start = false;
    }
    while (!m_tokenOK && m_stream->read(&c) && (ic < MaxTokenLen)) {
        bool space = (c == ' ') || (c == '\t');

        // Treat \r, \n, or \r\n as eol.
        // We do this by recognizing recognizing either \r, or \n when
        // not followed by \r as eol.  And, by ignoring \n after \r.
        if (c == '\n' && m_lastWasCR) {
            m_lastWasCR = false;
            continue;
        }
        eol = ((c == '\r') || (c == '\n'));
        m_lastWasCR = (c == '\r');
        if (inHTTPHeader()) {
            if (start && eol) {
                setHTTPHeader(0); // Blank line terminates http header.
            }
        } else {
            if ((space || eol) && start) 
                continue; // Strip leading ws.
        }
        start = false;

        if (space || eol) {
            m_tokenOK = true;
            *pc = 0;
            #if DEBUG_TOKEN
            Serial.print(F("TOKEN: ")); Serial.println(m_token); 
            #endif
        } else {
            *pc++ = c;
            ic++;
        }
    }
    *pc = 0;
    if (!m_tokenOK && (ic == MaxTokenLen)) {
        error = true;
    }

    processHTTPHeader(eol);

    return m_tokenOK;
}

// If the token is good, and its an HTTP command, go into HTTP mode.
// If in HTTP mode, if the token is the http command header token, 
// to into "reading command from HTTP header" mode.
void CommandParser::processHTTPHeader( bool eol )
{
    if (!m_tokenOK)
        return;


    if (   str_match(m_token,"get",true)
        || str_match(m_token,"post",true)) {
        #if DEBUG_HTTP
        Serial.print(F("Start HTTP ")); Serial.println(m_token);
        #endif
        setHTTPHeader(1);
        m_state = ExpHTTP;
    } else if ( str_match(m_token,"suth-cmd:",false) ) {
        setHTTPHeader(2);
        reset();
        m_tokenOK = true; // Handled specially as a "continue" in caller.
        #if DEBUG_HTTP
        Serial.println(F("Start suth-cmd: "));
        #endif
    } else if ((inHTTPHeader()==2) && eol) {
        #if DEBUG_HTTP
        Serial.println(F("End suth-cmd"));
        #endif
        setHTTPHeader(1);
    } else if (inHTTPHeader()==1) {
        // Ignore all tokens in cmd processor while in header, but not in command.
        #if DEBUG_HTTP
        Serial.print(F("Ignore header token ")); Serial.println(m_token);
        #endif
        if (m_state != ExpHTTP)
            reset();
    }
}


Command* CommandParser::getCommand( Command* cmd, bool& error, unsigned timeout )
{
    bool eol = false;
    unsigned long lastTime = millis() + timeout;

    cmd->setParser(this);

    do {
        if ( !nextToken( eol, error ) ) {
            if (error)
                reset();
            return 0;
        }
        switch (m_state) {
            case ExpHTTP:
                if (!inHTTPHeader())
                    m_state = ExpCmd;
                break;
            case ExpCmd:
                if ((inHTTPHeader() == 2) && !m_token[0]) {
                    // Wierd state after finding suth-cmd.  Want to read next token.
                    continue;
                }
                if ( !decodeCommand( error ) )  {
                    if (error)
                        reset();
                    if (inHTTPHeader()) {
                        break;
                    } else {
                        return 0;
                    }
                }
                cmd->setKind( m_descr->kind() );
                m_argNum = 0;
                if (m_descr->hasID()) {
                    if (eol) {
                        error = true;
                        reset();
                        return 0;
                    }
                    m_state = ExpId;;
                } else if (!setExpArg(0,eol,error)) {
                    reset();
                    return 0;
                }
                break;
            case ExpId: {
                int id = 0;
                if ( decodeInt( id ) ) {
                    cmd->setID(id);
                    if (!setExpArg(0,eol,error)) {
                        reset();
                        return 0;
                    }
                } else {
                    error = true;
                    reset();
                    return 0;
                }
                break;
            }
            case ExpInt: {
                int i = 0;
                if ( decodeInt( i ) ) {
                    cmd->arg(m_argNum)->setInt(i);
                    if ( !setExpArg( ++m_argNum,eol,error )) {
                        reset();
                        return 0;
                    }
                } else {
                    error = true;
                    reset();
                    return 0;
                }
                break;
            }
            case ExpLong: {
                long i = 0;
                if ( decodeInt( i ) ) {
                    cmd->arg(m_argNum)->setLong(i);
                    if ( !setExpArg( ++m_argNum,eol,error )) {
                        reset();
                        return 0;
                    }
                } else {
                    error = true;
                    reset();
                    return 0;
                }
                break;
            }
            case ExpFloat: {
                float f = 0.0;
                if ( decodeFloat( f ) ) {
                    cmd->arg(m_argNum)->setFloat(f);
                    if ( !setExpArg( ++m_argNum,eol,error )) {
                        reset();
                        return 0;
                    }
                } else {
                    error = true;
                    reset();
                    return 0;
                }
                break;
            }
            case ExpStr:
                cmd->arg(m_argNum)->setStr( m_token );
                if ( !setExpArg( ++m_argNum,eol,error )) {
                    reset();
                    return 0;
                }
                break;
        }
        if (!error) {
            if (inHTTPHeader()==1) 
                m_state = ExpHTTP;
            
            if (m_state == ExpCmd)
                return cmd;
        } else {
            return 0;
        }
    } while (millis() < lastTime);

    return 0;
}

#if USE_JSON
#if 0
// Header for json response up to point requiring content length.
static const char json_resp_header[] PROGMEM =
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: application/jsonrequest\r\n"
  "Content-Length: ";
#else
// Header for json response up to point requiring content length.
static const char json_resp_header[] PROGMEM =
  "Content-Length: ";
#endif
#endif

#if USE_RF24
void EthernetSerialIO::pingActivity() {
    m_interface->pingHeartbeat();
}

bool EthernetSerialIO::read( char* c ) {
    bool wasConnected = m_client.connected();
    if (!wasConnected) {
        // We keep a copy, and set it globally so others can see we're connected.
        m_client = m_interface->getServer().available();
        m_interface->setIncomingClient( m_client );
    }
    int n = m_client.available();
    if (n > 0) {
        #if DEBUG_CONNECT
        if (!wasConnected) {Serial.print("New connection on read.");Serial.print(n);Serial.println(" available");}
        #endif
        *c = m_client.read();
        #if DEBUG_CHAR
        Serial.print("CHAR: '");Serial.print(*c);Serial.println("'");
        #endif
        pingActivity();
        return true;
    } else if ( m_client && m_client.connected() && ((millis() - m_interface->lastHeartbeat()) > m_connectionTimeout)) {
        // If connected with no activity past a timeout, disconnect.
        disconnect(true);
        return false;
    } else {
        return false;
    }
}

void EthernetSerialIO::ack( Command* cmd )
{
    const char* str = json_resp_header;
    write_pgm(str);
    client().print( 0 );
    client().print( "\r\n\r\n" );
    #if DEBUG_ACK
    Serial.print(F("Send empty ack for cmd #")); Serial.println((int)cmd->kind());
    #endif
    disconnect(false);
    pingActivity();
}

#if USE_JSON
void EthernetSerialIO::ack( Command* cmd, JsonObject& json )
{
    write_pgm(json_resp_header);
    client().print( json.measureLength() );
    client().print( "\r\n\r\n" );
    #if DEBUG_ACK
    Serial.print(F("Send ack for cmd #")); Serial.println((int)cmd->kind());
    #endif
    json.printTo(this->client());
    disconnect(false);
    pingActivity();
}
void EthernetSerialIO::ack( Command* cmd, JsonArray& json )
{
    write_pgm(json_resp_header);
    client().print( json.measureLength() );
    client().print( "\r\n\r\n" );
    #if DEBUG_ACK
    Serial.print(F("Send ack for cmd #")); Serial.println((int)cmd->kind());
    #endif
    json.printTo(this->client());
    disconnect(false);
    pingActivity();
}
#endif

void EthernetSerialIO::disconnect( bool unnatural )
{
    m_client.stop();
    m_client = EthernetClient();
    m_interface->setIncomingClient( m_client );
    m_interface->logDisconnect( unnatural );
}

#endif

#if USE_JSON
#if defined(ARDUINO)
void ArduinoSerialIO::ack( Command* cmd, JsonObject& json )
{
    #if USE_JSON
    json.printTo(Serial);
    Serial.println("");
    #endif
}
void ArduinoSerialIO::ack( Command* cmd, JsonArray& json )
{
    #if USE_JSON
    json.printTo(Serial);
    Serial.println("");
    #endif
}
#endif
#endif

void Command::printName()
{
    if (!m_parser)
        return;
    const CommandDescr* descr = m_parser->getDescr(m_kind);
    if (!descr)
        return;

    descr->printStr();
}

#if 0
int main( int argc, char** argv) 
{
    CommandParser parser( g_commandDescrs );
    StdInStream sis;
    Command cmd;
    bool done = false;
    while (!done) {
        bool error = false;
        if ( parser.getCommand( sis, &cmd, error ) ) {
            switch ( cmd.kind() ) {
                case CmdNone:
                    printf("Got 'None'\n");
                    break;
                case CmdPing:
                    printf("Got 'Ping'\n");
                    break;
                case CmdStop:
                    printf("Got 'Stop'\n");
                    done = true;
                    break;
                case CmdUp: {
                    int pct;
                    cmd.arg(0)->getInt(pct);
                    printf("Got 'Up', id=%d, pct=%d\n", (int)cmd.ID(), pct);
                    break;
                }
                case CmdDown: {
                    float pct;
                    cmd.arg(0)->getFloat(pct);
                    printf("Got 'Down', id=%d, pct=%f\n", (int)cmd.ID(), pct);
                    break;
                }
                case CmdCalibrate:
                    printf("Got 'Calibrate, id=%d'\n", (int)cmd.ID());
                    break;
                default:
                    printf("Unrecognized cmd\n");
                    break;
            }
            parser.reset();
        } else if (error) {
            printf("Error in command\n");
            parser.reset();
        }
    }
}
#endif


#if USE_JSON
//
// Utilities to ack simple values in Json wrappers.
// //
#if defined(ARDUINO)
void Command::ack( bool val )
{
    #if USE_JSON
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["value"] = val;
    ack( json );
    #endif
}

void Command::ack( int val )
{
    #if USE_JSON
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["value"] = val;
    ack( json );
    #endif
}
#endif
void Command::ack( float val )
{
    #if USE_JSON
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["value"] = val;
    ack( json );
    #endif
}

void Command::ack( const char* val )
{
    #if USE_JSON
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["value"] = val;
    ack( json );
    #endif
}
void Command::ack( const String& val )
{
    #if USE_JSON
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["value"] = val;
    ack( json );
    #endif
}
#endif

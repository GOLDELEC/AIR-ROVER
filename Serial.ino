// AIR ROVER smart car
// copyright GOLELEC 2021
// goldelec.com
// This code is public domain, enjoy!


#define RX_BUFFER_SIZE 256
#define UART_NUMBER 1
#define TX_BUFFER_SIZE 256
#define INBUF_SIZE 64


static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

// Capability is bit flags; defines should be 1, 2, 4, 8...
#define BIND_CAPABLE 0; 

const uint32_t PROGMEM capability = 0+BIND_CAPABLE;

#ifdef DEBUGMSG
  #define DEBUG_MSG_BUFFER_SIZE 128
  static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
  static uint8_t head_debug;
  static uint8_t tail_debug;
#endif

#define MSP_VERSION              0

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number

#define MSP_SET_RAW_RC_TINY      150   //in message          4 rc chan

#define MSP_AIR                  199   //out meassage         device state


#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)

//下一步骤
#define MSP_COOKING_NEXT             215
//上一步骤
#define MSP_COOKING_PRE              216
//暂停
#define MSP_COOKING_PAUSE            217
//继续
#define MSP_COOKING_RESUME           218
//重置
#define MSP_COOKING_RESET            219
//设置新的菜谱时序
#define MSP_COOKING_SET_COOKING      220
//跳转到指定步骤
#define MSP_COOKING_JUMP             221
//让定时器向app上报状态
#define MSP_COOKING_STATUS           230

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];


#define CURRENTPORT 0

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);UartSendData();
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialCom() {
  uint8_t c,n;  
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

  for(n=0;n<UART_NUMBER;n++) {
    while (SerialAvailable(CURRENTPORT)) {
			
      uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[CURRENTPORT]-serialTailTX[CURRENTPORT]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(CURRENTPORT);


      // regular data handling to detect and handle MSP and other data
      if (c_state[CURRENTPORT] == IDLE) {
        c_state[CURRENTPORT] = (c=='$') ? HEADER_START : IDLE;
        if (c_state[CURRENTPORT] == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
      } else if (c_state[CURRENTPORT] == HEADER_START) {
        c_state[CURRENTPORT] = (c=='M') ? HEADER_M : IDLE;
      } else if (c_state[CURRENTPORT] == HEADER_M) {
        c_state[CURRENTPORT] = (c=='<') ? HEADER_ARROW : IDLE;
      } else if (c_state[CURRENTPORT] == HEADER_ARROW) {
        if (c > INBUF_SIZE) {  // now we are expecting the payload size
          c_state[CURRENTPORT] = IDLE;
          continue;
        }
        dataSize[CURRENTPORT] = c;
        offset[CURRENTPORT] = 0;
        checksum[CURRENTPORT] = 0;
        indRX[CURRENTPORT] = 0;
        checksum[CURRENTPORT] ^= c;
        c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow
      } else if (c_state[CURRENTPORT] == HEADER_SIZE) {
        cmdMSP[CURRENTPORT] = c;
        checksum[CURRENTPORT] ^= c;
        c_state[CURRENTPORT] = HEADER_CMD;
      } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
        checksum[CURRENTPORT] ^= c;
        inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
      } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
        if (checksum[CURRENTPORT] == c) {  // compare calculated and transferred checksum
          //debugWrite("C:");
					//debugWrite(cmdMSP[CURRENTPORT]);
				
          evaluateCommand();  // we got a valid packet, evaluate it
        }
        c_state[CURRENTPORT] = IDLE;
      }

    }
  }
}

void evaluateCommand() {
  unsigned char auxChannels;
  unsigned char aux;

#define MSP_COOKING_NEXT             215
#define MSP_COOKING_PRE              216
#define MSP_COOKING_PAUSE            217
#define MSP_COOKING_RESUME           218
#define MSP_COOKING_RESET            219
#define MSP_COOKING_SET_COOKING      220
  
  switch(cmdMSP[CURRENTPORT]) {
   case MSP_COOKING_NEXT:
     break;
   case MSP_COOKING_PRE:
     break;
   case MSP_COOKING_PAUSE:
     break;
   case MSP_COOKING_RESUME:
     break;
   case MSP_COOKING_RESET:
     break;
   case MSP_COOKING_STATUS:
     break;
   case MSP_COOKING_JUMP:
     break;
   case MSP_COOKING_SET_COOKING:     
     break;

	 
   case MSP_SET_RAW_RC:
     for(uint8_t i=0;i<8;i++) {
       read16();
     }
     
     failsafeCnt = 0;
     
     headSerialReply(0);
     break;
 
	 case MSP_SET_RAW_RC_TINY:
		for(uint8_t i = 0;i < 4;i++) {
			serialRcValue[i] = 1000 + read8() * 4;
		}

		auxChannels = read8();

		aux = (auxChannels & 0xc0) >> 6;

		if(aux == 0){
			serialRcValue[4] = 1000;
		}
		else if(aux == 1){
			serialRcValue[4] = 1500;
		}
		else{
			serialRcValue[4] = 2000;
		}


		aux = (auxChannels & 0x30) >> 4;

		if(aux == 0){
			serialRcValue[5] = 1000;
		}
		else if(aux == 1){
			serialRcValue[5] = 1500;
		}
		else{
			serialRcValue[5] = 2000;
		}


		aux = (auxChannels & 0x0c) >> 2;

		if(aux == 0){
			serialRcValue[6] = 1000;
		}
		else if(aux == 1){
			serialRcValue[6] = 1500;
		}
		else{
			serialRcValue[6] = 2000;
		}

		aux = (auxChannels & 0x03);

		if(aux == 0){
			serialRcValue[7] = 1000;
		}
		else if(aux == 1){
			serialRcValue[7] = 1500;
		}
		else{
			serialRcValue[7] = 2000;
		}

		failsafeCnt = 0;

		

				 
		return;
			
		/*
		headSerialReply(8);
		
		for(uint8_t i = 0; i < 4; i++) {
			serialize16(serialRcValue[i]);
		}*/
		break;

	case MSP_AIR:
     headSerialReply(14);
     serialize8(0x08);  //serialize8(flightState);
     serialize16(100); //version
     serialize16(0);  //serialize16((int16_t)EstAlt);
     for(uint8_t i=0;i<2;i++) serialize16(0);//for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
     serialize16(0);//serialize16((int16_t)AltHold);
     serialize8(vbat);
     serialize8(0);//serialize8((int8_t)(conf.angleTrim[PITCH]));
     serialize8(0);//serialize8((int8_t)(conf.angleTrim[ROLL]));
     break;

   case MSP_IDENT:
     headSerialReply(7);
     serialize8(110);   // multiwii version
     serialize8(MULTITYPE); // type of multicopter
     serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
     serialize32(pgm_read_dword(&(capability)));        // "capability"
     break;
   case MSP_STATUS:
     headSerialReply(11);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
     serialize32(0);
     serialize8(0);   // current setting
     break;
   case MSP_EEPROM_WRITE:

     headSerialReply(0);
     break;
   case MSP_DEBUG:
     headSerialReply(8);
     for(uint8_t i=0;i<4;i++) {
       serialize16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;
   #ifdef DEBUGMSG
   case MSP_DEBUGMSG:
     {
       uint8_t size = debugmsg_available();
       if (size > 16) size = 16;
       headSerialReply(size);
       debugmsg_serialize(size);
     }
     break;
   #endif
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  #ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
    switch (sr) {
    // Note: we may receive weird characters here which could trigger unwanted features during flight.
    //       this could lead to a crash easily.
    //       Please use if (!f.ARMED) where neccessary
      #ifdef LCD_CONF
        case 's':
        case 'S':
          if (!f.ARMED) configurationLoop();
          break;
      #endif
      #ifdef LOG_PERMANENT_SHOW_AT_L
        case 'L':
          if (!f.ARMED) dumpPLog(1);
          break;
        #endif
        #if defined(LCD_TELEMETRY) && defined(LCD_TEXTSTAR)
        case 'A': // button A press
          toggle_telemetry(1);
          break;
        case 'B': // button B press
          toggle_telemetry(2);
          break;
        case 'C': // button C press
          toggle_telemetry(3);
          break;
        case 'D': // button D press
          toggle_telemetry(4);
          break;
        case 'a': // button A release
        case 'b': // button B release
        case 'c': // button C release
        case 'd': // button D release
          break;
      #endif
      #ifdef LCD_TELEMETRY
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
      #if defined(LOG_VALUES) || defined(DEBUG)
        case 'R':
      #endif
      #ifdef DEBUG
        case 'F':
      #endif
          toggle_telemetry(sr);
          break;
      #endif // LCD_TELEMETRY
    }
  #endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX[CURRENTPORT];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][CURRENTPORT] = a;
  checksum[CURRENTPORT] ^= a;
  serialHeadTX[CURRENTPORT] = t;
}

void UartSendData() {
	UCSR0B |= (1<<UDRIE0);  //enable transmitter UDRE interrupt
}

static void inline SerialOpen(uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  UCSR0A  = (1<<U2X0);  //Bit 1 – U2Xn: Double the USART Transmission Speed, Write this bit to zero when using synchronous operation

	UBRR0H = h; 
	UBRR0L = l; 

	UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); //1<<RXEN0: enable RX; 1<<TXEN0: enable TX; 1<<RXCIE0: Enable the USART Recieve Complete interrupt 


	/*
	uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);

	UBRR0H = h;  
  UBRR0L = l;

	UCSR0A &= ~(_BV(U2X0)); //Bit 1 – U2Xn: Double the USART Transmission Speed, Write this bit to zero when using synchronous operation
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8-bit data  UCSZn1:0: Character Size 
	
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX  
  UCSR0B |= (1 << RXCIE0);// | (1 << TXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC), its like GUI event but for microcontrollers
 */ 
}

static void inline SerialEnd() {
	UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0));
}

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;  
  serialHeadRX[portnum] = h;
}

ISR(USART_UDRE_vect) {  // Serial 0 on a UNO
  uint8_t t = serialTailTX[0];
  if (serialHeadTX[0] != t) {
    if (++t >= TX_BUFFER_SIZE) t = 0;
    UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring
    serialTailTX[0] = t;
  }
  if (t == serialHeadTX[0]) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
}

ISR(USART_RX_vect)  { store_uart_in_buf(UDR0, 0); sei();}

uint8_t SerialRead(uint8_t port) {
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
	uint8_t available = (serialHeadRX[port] - serialTailRX[port])%RX_BUFFER_SIZE;

  return available;
}

void SerialWrite(uint8_t port,uint8_t c){
  serialize8(c);UartSendData();
}

#ifdef DEBUGMSG
void debugPrint(const char *str) {
  while(*str) {
    debug_buf[head_debug++] = *str++;
    if (head_debug == DEBUG_MSG_BUFFER_SIZE) {
      head_debug = 0;
    }
  }
}

static uint8_t debugmsg_available() {
  if (head_debug >= tail_debug) {
    return head_debug-tail_debug;
  } else {
    return head_debug + (DEBUG_MSG_BUFFER_SIZE-tail_debug);
  }
}

static void debugmsg_serialize(uint8_t l) {
  for (uint8_t i=0; i<l; i++) {
    if (head_debug != tail_debug) {
      serialize8(debug_buf[tail_debug++]);
      if (tail_debug == DEBUG_MSG_BUFFER_SIZE) {
        tail_debug = 0;
      }
    } else {
      serialize8('\0');
    }
  }
}
#endif

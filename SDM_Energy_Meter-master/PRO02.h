#ifndef PRO02_h
#define PRO02_h
//------------------------------------------------------------------------------
#include <Arduino.h>
#include <SDM_Config_User.h>
#if defined ( USE_HARDWARESERIAL )
  #include <HardwareSerial.h>
#else
  #include <SoftwareSerial.h>
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if !defined ( SDM_UART_BAUD )
  #define SDM_UART_BAUD                               4800                      //  default baudrate
#endif

#if !defined ( DERE_PIN )
  #define DERE_PIN                                    NOT_A_PIN                 //  default digital pin for control MAX485 DE/RE lines (connect DE & /RE together to this pin)
#endif

#if defined ( USE_HARDWARESERIAL )

  #if !defined ( SDM_UART_CONFIG )
    #define SDM_UART_CONFIG                           SERIAL_8N1                //  default hardware uart config
  #endif

  #if defined ( ESP8266 ) && !defined ( SWAPHWSERIAL )
    #define SWAPHWSERIAL                              0                         //  (only esp8266) when hwserial used, then swap uart pins from 3/1 to 13/15 (default not swap)
  #endif

  #if defined ( ESP32 )
    #if !defined ( SDM_RX_PIN )
      #define SDM_RX_PIN                              -1                        //  use default rx pin for selected port
    #endif
    #if !defined ( SDM_TX_PIN )
      #define SDM_TX_PIN                              -1                        //  use default tx pin for selected port
    #endif
  #endif

#else

  #if defined ( ESP8266 ) || defined ( ESP32 )
    #if !defined ( SDM_UART_CONFIG )
      #define SDM_UART_CONFIG                         SWSERIAL_8N1              //  default softwareware uart config for esp8266/esp32
    #endif
  #endif

//  #if !defined ( SDM_RX_PIN ) || !defined ( SDM_TX_PIN )
//    #error "SDM_RX_PIN and SDM_TX_PIN must be defined in SDM_Config_User.h for Software Serial option)"
//  #endif

  #if !defined ( SDM_RX_PIN )
    #define SDM_RX_PIN                                -1
  #endif
  #if !defined ( SDM_TX_PIN )
    #define SDM_TX_PIN                                -1
  #endif

#endif

#if !defined ( WAITING_TURNAROUND_DELAY )
  #define WAITING_TURNAROUND_DELAY                    200                       //  time in ms to wait for process current request
#endif

#if !defined ( RESPONSE_TIMEOUT )
  #define RESPONSE_TIMEOUT                            500                       //  time in ms to wait for return response from all devices before next request
#endif

#if !defined ( SDM_MIN_DELAY )
  #define SDM_MIN_DELAY                               20                        //  minimum value (in ms) for WAITING_TURNAROUND_DELAY and RESPONSE_TIMEOUT
#endif

#if !defined ( SDM_MAX_DELAY )
  #define SDM_MAX_DELAY                               5000                      //  maximum value (in ms) for WAITING_TURNAROUND_DELAY and RESPONSE_TIMEOUT
#endif

//------------------------------------------------------------------------------

#define PRO_ERR_NO_ERROR                              0                         //  no error
#define PRO_ERR_CRC_ERROR                             1                         //  crc error
#define PRO_ERR_WRONG_BYTES                           2                         //  bytes b0,b1 or b2 wrong
#define PRO_ERR_NOT_ENOUGHT_BYTES                     3                         //  not enough bytes from sdm
#define PRO_ERR_TIMEOUT                               4                         //  timeout

//------------------------------------------------------------------------------

#define FRAMESIZE                                     9                         //  size of out/in array
#define PRO_REPLY_BYTE_COUNT                          0x04                      //  number of bytes with data

#define PRO_B_01                                      0x02                      //  BYTE 1 -> slave address (default value 1 read from node 1)
#define PRO_B_02                                      0x03                      //  BYTE 2 -> function code (default value 0x04 read from 3X input registers)
#define PRO_B_05                                      0x00                      //  BYTE 5
#define PRO_B_06                                      0x02                      //  BYTE 6
                                                                                //  BYTES 3 & 4 (BELOW)


//---------------------------------------------------------------------------------------------
//      REGISTERS LIST FOR PRO02 DEVICE                                                          
//---------------------------------------------------------------------------------------------
//      REGISTER NAME                                 REGISTER ADDRESS              UNIT       
//---------------------------------------------------------------------------------------------
#define PRO_PHASE_1_VOLTAGE                           0x5002                    //  V         
#define PRO_PHASE_2_VOLTAGE                           0x5004                    //  V         
#define PRO_PHASE_3_VOLTAGE                           0x5006                    //  V          
#define PRO_PHASE_1_CURRENT                           0x500C                    //  A          
#define PRO_PHASE_2_CURRENT                           0x500E                    //  A          
#define PRO_PHASE_3_CURRENT                           0x5010                    //  A          
#define PRO_PHASE_1_POWER                             0x5014                    //  W          
#define PRO_PHASE_2_POWER                             0x5016                    //  W          
#define PRO_PHASE_3_POWER                             0x5018                    //  W          
#define PRO_PHASE_1_APPARENT_POWER                    0x5024                    //  VA         
#define PRO_PHASE_2_APPARENT_POWER                    0x5026                    //  VA         
#define PRO_PHASE_3_APPARENT_POWER                    0x5028                    //  VA         
#define PRO_PHASE_1_REACTIVE_POWER                    0x501C                    //  VAr        
#define PRO_PHASE_2_REACTIVE_POWER                    0x501E                    //  VAr        
#define PRO_PHASE_3_REACTIVE_POWER                    0x5020                    //  VAr        
#define PRO_PHASE_1_POWER_FACTOR                      0x502C                    //             
#define PRO_PHASE_2_POWER_FACTOR                      0x502E                    //             
#define PRO_PHASE_3_POWER_FACTOR                      0x5030                    //                       
#define PRO_TOTAL_ACTIVE_ENERGY                       0x5030                    //  kWh         
//---------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------------------------

class PRO02 {
  public:
#if defined ( USE_HARDWARESERIAL )                                              //  hardware serial
  #if defined ( ESP8266 )                                                       //  on esp8266
    PRO02(HardwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, bool swapuart = SWAPHWSERIAL);
  #elif defined ( ESP32 )                                                       //  on esp32
    PRO02(HardwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, int8_t rx_pin = SDM_RX_PIN, int8_t tx_pin = SDM_TX_PIN);
  #else                                                                         //  on avr
    PRO02(HardwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG);
  #endif
#else                                                                           //  software serial
  #if defined ( ESP8266 ) || defined ( ESP32 )                                  //  on esp8266/esp32
    PRO02(SoftwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, int8_t rx_pin = SDM_RX_PIN, int8_t tx_pin = SDM_TX_PIN);
  #else                                                                        //  on avr
    PRO02(SoftwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN);
  #endif
#endif
    virtual ~PRO02();

    void begin(void);
    float readVal(uint16_t reg, uint8_t node = PRO_B_01);                       //  read value from register = reg and from deviceId = node
    uint16_t getErrCode(bool _clear = false);                                   //  return last errorcode (optional clear this value, default flase)
    uint32_t getErrCount(bool _clear = false);                                  //  return total errors count (optional clear this value, default flase)
    uint32_t getSuccCount(bool _clear = false);                                 //  return total success count (optional clear this value, default false)
    void clearErrCode();                                                        //  clear last errorcode
    void clearErrCount();                                                       //  clear total errors count
    void clearSuccCount();                                                      //  clear total success count
    void setMsTurnaround(uint16_t _msturnaround = WAITING_TURNAROUND_DELAY);    //  set new value for WAITING_TURNAROUND_DELAY (ms), min=SDM_MIN_DELAY, max=SDM_MAX_DELAY
    void setMsTimeout(uint16_t _mstimeout = RESPONSE_TIMEOUT);                  //  set new value for RESPONSE_TIMEOUT (ms), min=SDM_MIN_DELAY, max=SDM_MAX_DELAY
    uint16_t getMsTurnaround();                                                 //  get current value of WAITING_TURNAROUND_DELAY (ms)
    uint16_t getMsTimeout();                                                    //  get current value of RESPONSE_TIMEOUT (ms)

  private:
#if defined ( USE_HARDWARESERIAL )
    HardwareSerial& proSer;
#else
    SoftwareSerial& proSer;
#endif

#if defined ( USE_HARDWARESERIAL )
    int _config = SDM_UART_CONFIG;
  #if defined ( ESP8266 )
    bool _swapuart = SWAPHWSERIAL;
  #elif defined ( ESP32 )
    int8_t _rx_pin = -1;
    int8_t _tx_pin = -1;
  #endif
#else
  #if defined ( ESP8266 ) || defined ( ESP32 )
    int _config = SDM_UART_CONFIG;
  #endif
    int8_t _rx_pin = -1;
    int8_t _tx_pin = -1; 
#endif
    long _baud = SDM_UART_BAUD;
    int _dere_pin = DERE_PIN;
    uint16_t readingerrcode = PRO_ERR_NO_ERROR;                                 //  4 = timeout; 3 = not enough bytes; 2 = number of bytes OK but bytes b0,b1 or b2 wrong, 1 = crc error
    uint16_t msturnaround = WAITING_TURNAROUND_DELAY;
    uint16_t mstimeout = RESPONSE_TIMEOUT;
    uint32_t readingerrcount = 0;                                               //  total errors counter
    uint32_t readingsuccesscount = 0;                                           //  total success counter
    uint16_t calculateCRC(uint8_t *array, uint8_t len);
    void flush(unsigned long _flushtime = 0);                                   //  read serial if any old data is available or for a given time in ms
    void dereSet(bool _state = LOW);                                            //  for control MAX485 DE/RE pins, LOW receive from SDM, HIGH transmit to SDM
};
#endif // SDM_h

#include "PRO01.h"
//------------------------------------------------------------------------------
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
PRO01::PRO01(HardwareSerial& serial, long baud, int dere_pin, int config, bool swapuart) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_swapuart = swapuart;
}
#elif defined ( ESP32 )
PRO01::PRO01(HardwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO01::PRO01(HardwareSerial& serial, long baud, int dere_pin, int config) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
}
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
PRO01::PRO01(SoftwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO01::PRO01(SoftwareSerial& serial, long baud, int dere_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
}
#endif
#endif

PRO01::~PRO01() {
}

void PRO01::begin(void) {
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
  proSer.begin(_baud, (SerialConfig)_config);
#elif defined ( ESP32 )
  proSer.begin(_baud, _config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud, _config);
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
  proSer.begin(_baud, (SoftwareSerialConfig)_config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud);
#endif
#endif

#if defined ( USE_HARDWARESERIAL ) && defined ( ESP8266 )
  if (_swapuart)
    proSer.swap();
#endif
  if (_dere_pin != NOT_A_PIN) {
    pinMode(_dere_pin, OUTPUT);                                                 //set output pin mode for DE/RE pin when used (for control MAX485)
  }
  dereSet(LOW);                                                                 //set init state to receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}

float PRO01::readVal(uint16_t reg, uint8_t node) {
  uint16_t temp;
  unsigned long resptime;
  uint8_t proarr[FRAMESIZE] = {node, PRO_B_02, 0, 0, PRO_B_05, PRO_B_06, 0, 0, 0};
  float res = NAN;
  uint16_t readErr = PRO_ERR_NO_ERROR;

  proarr[2] = highByte(reg);
  proarr[3] = lowByte(reg);

  temp = calculateCRC(proarr, FRAMESIZE - 3);                                   //calculate out crc only from first 6 bytes

  proarr[6] = lowByte(temp);
  proarr[7] = highByte(temp);

#if !defined ( USE_HARDWARESERIAL )
  proSer.listen();                                                              //enable softserial rx interrupt
#endif

  flush();                                                                      //read serial if any old data is available

  dereSet(HIGH);                                                                //transmit to SDM  -> DE Enable, /RE Disable (for control MAX485)

  delay(2);                                                                     //fix for issue (nan reading) by sjfaustino: https://github.com/reaper7/SDM_Energy_Meter/issues/7#issuecomment-272111524

  proSer.write(proarr, FRAMESIZE - 1);                                          //send 8 bytes

  proSer.flush();                                                               //clear out tx buffer

  dereSet(LOW);                                                                 //receive from SDM -> DE Disable, /RE Enable (for control MAX485)

  resptime = millis() + msturnaround;

  while (proSer.available() < FRAMESIZE) {
    if (resptime < millis()) {
      readErr = PRO_ERR_TIMEOUT;                                                //err debug (4)
      break;
    }
    yield();
  }

  if (readErr == PRO_ERR_NO_ERROR) {                                            //if no timeout...

    if (proSer.available() >= FRAMESIZE) {

      for(int n=0; n<FRAMESIZE; n++) {
        proarr[n] = proSer.read();
      }

      if (proarr[0] == node && proarr[1] == PRO_B_02 && proarr[2] == PRO_REPLY_BYTE_COUNT) {

        if ((calculateCRC(proarr, FRAMESIZE - 2)) == ((proarr[8] << 8) | proarr[7])) {  //calculate crc from first 7 bytes and compare with received crc (bytes 7 & 8)
          ((uint8_t*)&res)[3]= proarr[3];
          ((uint8_t*)&res)[2]= proarr[4];
          ((uint8_t*)&res)[1]= proarr[5];
          ((uint8_t*)&res)[0]= proarr[6];
        } else {
          readErr = PRO_ERR_CRC_ERROR;                                          //err debug (1)
        }

      } else {
        readErr = PRO_ERR_WRONG_BYTES;                                          //err debug (2)
      }

    } else {
      readErr = PRO_ERR_NOT_ENOUGHT_BYTES;                                      //err debug (3)
    }

  }

  flush(mstimeout);                                                             //read serial if any old data is available and wait for RESPONSE_TIMEOUT (in ms)
  
  if (proSer.available())                                                       //if serial rx buffer (after RESPONSE_TIMEOUT) still contains data then something spam rs485, check node(s) or increase RESPONSE_TIMEOUT
    readErr = PRO_ERR_TIMEOUT;                                                  //err debug (4) but returned value may be correct

  if (readErr != PRO_ERR_NO_ERROR) {                                            //if error then copy temp error value to global val and increment global error counter
    readingerrcode = readErr;
    readingerrcount++; 
  } else {
    ++readingsuccesscount;
  }

#if !defined ( USE_HARDWARESERIAL )
  proSer.stopListening();                                                       //disable softserial rx interrupt
#endif

  return (res);
}

uint16_t PRO01::getErrCode(bool _clear) {
  uint16_t _tmp = readingerrcode;
  if (_clear == true)
    clearErrCode();
  return (_tmp);
}

uint32_t PRO01::getErrCount(bool _clear) {
  uint32_t _tmp = readingerrcount;
  if (_clear == true)
    clearErrCount();
  return (_tmp);
}

uint32_t PRO01::getSuccCount(bool _clear) {
  uint32_t _tmp = readingsuccesscount;
  if (_clear == true)
    clearSuccCount();
  return (_tmp);
}

void PRO01::clearErrCode() {
  readingerrcode = PRO_ERR_NO_ERROR;
}

void PRO01::clearErrCount() {
  readingerrcount = 0;
}

void PRO01::clearSuccCount() {
  readingsuccesscount = 0;
}

void PRO01::setMsTurnaround(uint16_t _msturnaround) {
  if (_msturnaround < SDM_MIN_DELAY)
    msturnaround = SDM_MIN_DELAY;
  else if (_msturnaround > SDM_MAX_DELAY)
    msturnaround = SDM_MAX_DELAY;
  else
    msturnaround = _msturnaround; 
}

void PRO01::setMsTimeout(uint16_t _mstimeout) {
  if (_mstimeout < SDM_MIN_DELAY)
    mstimeout = SDM_MIN_DELAY;
  else if (_mstimeout > SDM_MAX_DELAY)
    mstimeout = SDM_MAX_DELAY;
  else
    mstimeout = _mstimeout; 
}

uint16_t PRO01::getMsTurnaround() {
  return (msturnaround);
}

uint16_t PRO01::getMsTimeout() {
  return (mstimeout);
}

uint16_t PRO01::calculateCRC(uint8_t *array, uint8_t len) {
  uint16_t _crc, _flag;
  _crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    _crc ^= (uint16_t)array[i];
    for (uint8_t j = 8; j; j--) {
      _flag = _crc & 0x0001;
      _crc >>= 1;
      if (_flag)
        _crc ^= 0xA001;
    }
  }
  return _crc;
}

void PRO01::flush(unsigned long _flushtime) {
  unsigned long flushtime = millis() + _flushtime;
  while (proSer.available() || flushtime >= millis()) {
    if (proSer.available())                                                     //read serial if any old data is available
      proSer.read();
    delay(1);
  }
}

void PRO01::dereSet(bool _state) {
  if (_dere_pin != NOT_A_PIN)
    digitalWrite(_dere_pin, _state);                                            //receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}


#include "PRO02.h"
//------------------------------------------------------------------------------
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
PRO02::PRO02(HardwareSerial& serial, long baud, int dere_pin, int config, bool swapuart) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_swapuart = swapuart;
}
#elif defined ( ESP32 )
PRO02::PRO02(HardwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO02::PRO02(HardwareSerial& serial, long baud, int dere_pin, int config) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
}
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
PRO02::PRO02(SoftwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO02::PRO02(SoftwareSerial& serial, long baud, int dere_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
}
#endif
#endif

PRO02::~PRO02() {
}

void PRO02::begin(void) {
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
  proSer.begin(_baud, (SerialConfig)_config);
#elif defined ( ESP32 )
  proSer.begin(_baud, _config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud, _config);
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
  proSer.begin(_baud, (SoftwareSerialConfig)_config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud);
#endif
#endif

#if defined ( USE_HARDWARESERIAL ) && defined ( ESP8266 )
  if (_swapuart)
    proSer.swap();
#endif
  if (_dere_pin != NOT_A_PIN) {
    pinMode(_dere_pin, OUTPUT);                                                 //set output pin mode for DE/RE pin when used (for control MAX485)
  }
  dereSet(LOW);                                                                 //set init state to receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}

float PRO02::readVal(uint16_t reg, uint8_t node) {
  uint16_t temp;
  unsigned long resptime;
  uint8_t proarr[FRAMESIZE] = {node, PRO_B_02, 0, 0, PRO_B_05, PRO_B_06, 0, 0, 0};
  float res = NAN;
  uint16_t readErr = PRO_ERR_NO_ERROR;

  proarr[2] = highByte(reg);
  proarr[3] = lowByte(reg);

  temp = calculateCRC(proarr, FRAMESIZE - 3);                                   //calculate out crc only from first 6 bytes

  proarr[6] = lowByte(temp);
  proarr[7] = highByte(temp);

#if !defined ( USE_HARDWARESERIAL )
  proSer.listen();                                                              //enable softserial rx interrupt
#endif

  flush();                                                                      //read serial if any old data is available

  dereSet(HIGH);                                                                //transmit to SDM  -> DE Enable, /RE Disable (for control MAX485)

  delay(2);                                                                     //fix for issue (nan reading) by sjfaustino: https://github.com/reaper7/SDM_Energy_Meter/issues/7#issuecomment-272111524

  proSer.write(proarr, FRAMESIZE - 1);                                          //send 8 bytes

  proSer.flush();                                                               //clear out tx buffer

  dereSet(LOW);                                                                 //receive from SDM -> DE Disable, /RE Enable (for control MAX485)

  resptime = millis() + msturnaround;

  while (proSer.available() < FRAMESIZE) {
    if (resptime < millis()) {
      readErr = PRO_ERR_TIMEOUT;                                                //err debug (4)
      break;
    }
    yield();
  }

  if (readErr == PRO_ERR_NO_ERROR) {                                            //if no timeout...

    if (proSer.available() >= FRAMESIZE) {

      for(int n=0; n<FRAMESIZE; n++) {
        proarr[n] = proSer.read();
      }

      if (proarr[0] == node && proarr[1] == PRO_B_02 && proarr[2] == PRO_REPLY_BYTE_COUNT) {

        if ((calculateCRC(proarr, FRAMESIZE - 2)) == ((proarr[8] << 8) | proarr[7])) {  //calculate crc from first 7 bytes and compare with received crc (bytes 7 & 8)
          ((uint8_t*)&res)[3]= proarr[3];
          ((uint8_t*)&res)[2]= proarr[4];
          ((uint8_t*)&res)[1]= proarr[5];
          ((uint8_t*)&res)[0]= proarr[6];
        } else {
          readErr = PRO_ERR_CRC_ERROR;                                          //err debug (1)
        }

      } else {
        readErr = PRO_ERR_WRONG_BYTES;                                          //err debug (2)
      }

    } else {
      readErr = PRO_ERR_NOT_ENOUGHT_BYTES;                                      //err debug (3)
    }

  }

  flush(mstimeout);                                                             //read serial if any old data is available and wait for RESPONSE_TIMEOUT (in ms)
  
  if (proSer.available())                                                       //if serial rx buffer (after RESPONSE_TIMEOUT) still contains data then something spam rs485, check node(s) or increase RESPONSE_TIMEOUT
    readErr = PRO_ERR_TIMEOUT;                                                  //err debug (4) but returned value may be correct

  if (readErr != PRO_ERR_NO_ERROR) {                                            //if error then copy temp error value to global val and increment global error counter
    readingerrcode = readErr;
    readingerrcount++; 
  } else {
    ++readingsuccesscount;
  }

#if !defined ( USE_HARDWARESERIAL )
  proSer.stopListening();                                                       //disable softserial rx interrupt
#endif

  return (res);
}

uint16_t PRO02::getErrCode(bool _clear) {
  uint16_t _tmp = readingerrcode;
  if (_clear == true)
    clearErrCode();
  return (_tmp);
}

uint32_t PRO02::getErrCount(bool _clear) {
  uint32_t _tmp = readingerrcount;
  if (_clear == true)
    clearErrCount();
  return (_tmp);
}

uint32_t PRO02::getSuccCount(bool _clear) {
  uint32_t _tmp = readingsuccesscount;
  if (_clear == true)
    clearSuccCount();
  return (_tmp);
}

void PRO02::clearErrCode() {
  readingerrcode = PRO_ERR_NO_ERROR;
}

void PRO02::clearErrCount() {
  readingerrcount = 0;
}

void PRO02::clearSuccCount() {
  readingsuccesscount = 0;
}

void PRO02::setMsTurnaround(uint16_t _msturnaround) {
  if (_msturnaround < SDM_MIN_DELAY)
    msturnaround = SDM_MIN_DELAY;
  else if (_msturnaround > SDM_MAX_DELAY)
    msturnaround = SDM_MAX_DELAY;
  else
    msturnaround = _msturnaround; 
}

void PRO02::setMsTimeout(uint16_t _mstimeout) {
  if (_mstimeout < SDM_MIN_DELAY)
    mstimeout = SDM_MIN_DELAY;
  else if (_mstimeout > SDM_MAX_DELAY)
    mstimeout = SDM_MAX_DELAY;
  else
    mstimeout = _mstimeout; 
}

uint16_t PRO02::getMsTurnaround() {
  return (msturnaround);
}

uint16_t PRO02::getMsTimeout() {
  return (mstimeout);
}

uint16_t PRO02::calculateCRC(uint8_t *array, uint8_t len) {
  uint16_t _crc, _flag;
  _crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    _crc ^= (uint16_t)array[i];
    for (uint8_t j = 8; j; j--) {
      _flag = _crc & 0x0001;
      _crc >>= 1;
      if (_flag)
        _crc ^= 0xA001;
    }
  }
  return _crc;
}

void PRO02::flush(unsigned long _flushtime) {
  unsigned long flushtime = millis() + _flushtime;
  while (proSer.available() || flushtime >= millis()) {
    if (proSer.available())                                                     //read serial if any old data is available
      proSer.read();
    delay(1);
  }
}

void PRO02::dereSet(bool _state) {
  if (_dere_pin != NOT_A_PIN)
    digitalWrite(_dere_pin, _state);                                            //receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}


#include "PRO03.h"
//------------------------------------------------------------------------------
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
PRO03::PRO03(HardwareSerial& serial, long baud, int dere_pin, int config, bool swapuart) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_swapuart = swapuart;
}
#elif defined ( ESP32 )
PRO03::PRO03(HardwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO03::PRO03(HardwareSerial& serial, long baud, int dere_pin, int config) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
}
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
PRO03::PRO03(SoftwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO03::PRO03(SoftwareSerial& serial, long baud, int dere_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
}
#endif
#endif

PRO03::~PRO03() {
}

void PRO03::begin(void) {
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
  proSer.begin(_baud, (SerialConfig)_config);
#elif defined ( ESP32 )
  proSer.begin(_baud, _config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud, _config);
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
  proSer.begin(_baud, (SoftwareSerialConfig)_config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud);
#endif
#endif

#if defined ( USE_HARDWARESERIAL ) && defined ( ESP8266 )
  if (_swapuart)
    proSer.swap();
#endif
  if (_dere_pin != NOT_A_PIN) {
    pinMode(_dere_pin, OUTPUT);                                                 //set output pin mode for DE/RE pin when used (for control MAX485)
  }
  dereSet(LOW);                                                                 //set init state to receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}

float PRO03::readVal(uint16_t reg, uint8_t node) {
  uint16_t temp;
  unsigned long resptime;
  uint8_t proarr[FRAMESIZE] = {node, PRO_B_02, 0, 0, PRO_B_05, PRO_B_06, 0, 0, 0};
  float res = NAN;
  uint16_t readErr = PRO_ERR_NO_ERROR;

  proarr[2] = highByte(reg);
  proarr[3] = lowByte(reg);

  temp = calculateCRC(proarr, FRAMESIZE - 3);                                   //calculate out crc only from first 6 bytes

  proarr[6] = lowByte(temp);
  proarr[7] = highByte(temp);

#if !defined ( USE_HARDWARESERIAL )
  proSer.listen();                                                              //enable softserial rx interrupt
#endif

  flush();                                                                      //read serial if any old data is available

  dereSet(HIGH);                                                                //transmit to SDM  -> DE Enable, /RE Disable (for control MAX485)

  delay(2);                                                                     //fix for issue (nan reading) by sjfaustino: https://github.com/reaper7/SDM_Energy_Meter/issues/7#issuecomment-272111524

  proSer.write(proarr, FRAMESIZE - 1);                                          //send 8 bytes

  proSer.flush();                                                               //clear out tx buffer

  dereSet(LOW);                                                                 //receive from SDM -> DE Disable, /RE Enable (for control MAX485)

  resptime = millis() + msturnaround;

  while (proSer.available() < FRAMESIZE) {
    if (resptime < millis()) {
      readErr = PRO_ERR_TIMEOUT;                                                //err debug (4)
      break;
    }
    yield();
  }

  if (readErr == PRO_ERR_NO_ERROR) {                                            //if no timeout...

    if (proSer.available() >= FRAMESIZE) {

      for(int n=0; n<FRAMESIZE; n++) {
        proarr[n] = proSer.read();
      }

      if (proarr[0] == node && proarr[1] == PRO_B_02 && proarr[2] == PRO_REPLY_BYTE_COUNT) {

        if ((calculateCRC(proarr, FRAMESIZE - 2)) == ((proarr[8] << 8) | proarr[7])) {  //calculate crc from first 7 bytes and compare with received crc (bytes 7 & 8)
          ((uint8_t*)&res)[3]= proarr[3];
          ((uint8_t*)&res)[2]= proarr[4];
          ((uint8_t*)&res)[1]= proarr[5];
          ((uint8_t*)&res)[0]= proarr[6];
        } else {
          readErr = PRO_ERR_CRC_ERROR;                                          //err debug (1)
        }

      } else {
        readErr = PRO_ERR_WRONG_BYTES;                                          //err debug (2)
      }

    } else {
      readErr = PRO_ERR_NOT_ENOUGHT_BYTES;                                      //err debug (3)
    }

  }

  flush(mstimeout);                                                             //read serial if any old data is available and wait for RESPONSE_TIMEOUT (in ms)
  
  if (proSer.available())                                                       //if serial rx buffer (after RESPONSE_TIMEOUT) still contains data then something spam rs485, check node(s) or increase RESPONSE_TIMEOUT
    readErr = PRO_ERR_TIMEOUT;                                                  //err debug (4) but returned value may be correct

  if (readErr != PRO_ERR_NO_ERROR) {                                            //if error then copy temp error value to global val and increment global error counter
    readingerrcode = readErr;
    readingerrcount++; 
  } else {
    ++readingsuccesscount;
  }

#if !defined ( USE_HARDWARESERIAL )
  proSer.stopListening();                                                       //disable softserial rx interrupt
#endif

  return (res);
}

uint16_t PRO03::getErrCode(bool _clear) {
  uint16_t _tmp = readingerrcode;
  if (_clear == true)
    clearErrCode();
  return (_tmp);
}

uint32_t PRO03::getErrCount(bool _clear) {
  uint32_t _tmp = readingerrcount;
  if (_clear == true)
    clearErrCount();
  return (_tmp);
}

uint32_t PRO03::getSuccCount(bool _clear) {
  uint32_t _tmp = readingsuccesscount;
  if (_clear == true)
    clearSuccCount();
  return (_tmp);
}

void PRO03::clearErrCode() {
  readingerrcode = PRO_ERR_NO_ERROR;
}

void PRO03::clearErrCount() {
  readingerrcount = 0;
}

void PRO03::clearSuccCount() {
  readingsuccesscount = 0;
}

void PRO03::setMsTurnaround(uint16_t _msturnaround) {
  if (_msturnaround < SDM_MIN_DELAY)
    msturnaround = SDM_MIN_DELAY;
  else if (_msturnaround > SDM_MAX_DELAY)
    msturnaround = SDM_MAX_DELAY;
  else
    msturnaround = _msturnaround; 
}

void PRO03::setMsTimeout(uint16_t _mstimeout) {
  if (_mstimeout < SDM_MIN_DELAY)
    mstimeout = SDM_MIN_DELAY;
  else if (_mstimeout > SDM_MAX_DELAY)
    mstimeout = SDM_MAX_DELAY;
  else
    mstimeout = _mstimeout; 
}

uint16_t PRO03::getMsTurnaround() {
  return (msturnaround);
}

uint16_t PRO03::getMsTimeout() {
  return (mstimeout);
}

uint16_t PRO03::calculateCRC(uint8_t *array, uint8_t len) {
  uint16_t _crc, _flag;
  _crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    _crc ^= (uint16_t)array[i];
    for (uint8_t j = 8; j; j--) {
      _flag = _crc & 0x0001;
      _crc >>= 1;
      if (_flag)
        _crc ^= 0xA001;
    }
  }
  return _crc;
}

void PRO03::flush(unsigned long _flushtime) {
  unsigned long flushtime = millis() + _flushtime;
  while (proSer.available() || flushtime >= millis()) {
    if (proSer.available())                                                     //read serial if any old data is available
      proSer.read();
    delay(1);
  }
}

void PRO03::dereSet(bool _state) {
  if (_dere_pin != NOT_A_PIN)
    digitalWrite(_dere_pin, _state);                                            //receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}

#include "PRO04.h"
//------------------------------------------------------------------------------
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
PRO04::PRO04(HardwareSerial& serial, long baud, int dere_pin, int config, bool swapuart) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_swapuart = swapuart;
}
#elif defined ( ESP32 )
PRO04::PRO04(HardwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO04::PRO04(HardwareSerial& serial, long baud, int dere_pin, int config) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
}
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
PRO04::PRO04(SoftwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
PRO04::PRO04(SoftwareSerial& serial, long baud, int dere_pin) : proSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
}
#endif
#endif

PRO04::~PRO04() {
}

void PRO04::begin(void) {
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
  proSer.begin(_baud, (SerialConfig)_config);
#elif defined ( ESP32 )
  proSer.begin(_baud, _config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud, _config);
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
  proSer.begin(_baud, (SoftwareSerialConfig)_config, _rx_pin, _tx_pin);
#else
  proSer.begin(_baud);
#endif
#endif

#if defined ( USE_HARDWARESERIAL ) && defined ( ESP8266 )
  if (_swapuart)
    proSer.swap();
#endif
  if (_dere_pin != NOT_A_PIN) {
    pinMode(_dere_pin, OUTPUT);                                                 //set output pin mode for DE/RE pin when used (for control MAX485)
  }
  dereSet(LOW);                                                                 //set init state to receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}

float PRO04::readVal(uint16_t reg, uint8_t node) {
  uint16_t temp;
  unsigned long resptime;
  uint8_t proarr[FRAMESIZE] = {node, PRO_B_02, 0, 0, PRO_B_05, PRO_B_06, 0, 0, 0};
  float res = NAN;
  uint16_t readErr = PRO_ERR_NO_ERROR;

  proarr[2] = highByte(reg);
  proarr[3] = lowByte(reg);

  temp = calculateCRC(proarr, FRAMESIZE - 3);                                   //calculate out crc only from first 6 bytes

  proarr[6] = lowByte(temp);
  proarr[7] = highByte(temp);

#if !defined ( USE_HARDWARESERIAL )
  proSer.listen();                                                              //enable softserial rx interrupt
#endif

  flush();                                                                      //read serial if any old data is available

  dereSet(HIGH);                                                                //transmit to SDM  -> DE Enable, /RE Disable (for control MAX485)

  delay(2);                                                                     //fix for issue (nan reading) by sjfaustino: https://github.com/reaper7/SDM_Energy_Meter/issues/7#issuecomment-272111524

  proSer.write(proarr, FRAMESIZE - 1);                                          //send 8 bytes

  proSer.flush();                                                               //clear out tx buffer

  dereSet(LOW);                                                                 //receive from SDM -> DE Disable, /RE Enable (for control MAX485)

  resptime = millis() + msturnaround;

  while (proSer.available() < FRAMESIZE) {
    if (resptime < millis()) {
      readErr = PRO_ERR_TIMEOUT;                                                //err debug (4)
      break;
    }
    yield();
  }

  if (readErr == PRO_ERR_NO_ERROR) {                                            //if no timeout...

    if (proSer.available() >= FRAMESIZE) {

      for(int n=0; n<FRAMESIZE; n++) {
        proarr[n] = proSer.read();
      }

      if (proarr[0] == node && proarr[1] == PRO_B_02 && proarr[2] == PRO_REPLY_BYTE_COUNT) {

        if ((calculateCRC(proarr, FRAMESIZE - 2)) == ((proarr[8] << 8) | proarr[7])) {  //calculate crc from first 7 bytes and compare with received crc (bytes 7 & 8)
          ((uint8_t*)&res)[3]= proarr[3];
          ((uint8_t*)&res)[2]= proarr[4];
          ((uint8_t*)&res)[1]= proarr[5];
          ((uint8_t*)&res)[0]= proarr[6];
        } else {
          readErr = PRO_ERR_CRC_ERROR;                                          //err debug (1)
        }

      } else {
        readErr = PRO_ERR_WRONG_BYTES;                                          //err debug (2)
      }

    } else {
      readErr = PRO_ERR_NOT_ENOUGHT_BYTES;                                      //err debug (3)
    }

  }

  flush(mstimeout);                                                             //read serial if any old data is available and wait for RESPONSE_TIMEOUT (in ms)
  
  if (proSer.available())                                                       //if serial rx buffer (after RESPONSE_TIMEOUT) still contains data then something spam rs485, check node(s) or increase RESPONSE_TIMEOUT
    readErr = PRO_ERR_TIMEOUT;                                                  //err debug (4) but returned value may be correct

  if (readErr != PRO_ERR_NO_ERROR) {                                            //if error then copy temp error value to global val and increment global error counter
    readingerrcode = readErr;
    readingerrcount++; 
  } else {
    ++readingsuccesscount;
  }

#if !defined ( USE_HARDWARESERIAL )
  proSer.stopListening();                                                       //disable softserial rx interrupt
#endif

  return (res);
}

uint16_t PRO04::getErrCode(bool _clear) {
  uint16_t _tmp = readingerrcode;
  if (_clear == true)
    clearErrCode();
  return (_tmp);
}

uint32_t PRO04::getErrCount(bool _clear) {
  uint32_t _tmp = readingerrcount;
  if (_clear == true)
    clearErrCount();
  return (_tmp);
}

uint32_t PRO04::getSuccCount(bool _clear) {
  uint32_t _tmp = readingsuccesscount;
  if (_clear == true)
    clearSuccCount();
  return (_tmp);
}

void PRO04::clearErrCode() {
  readingerrcode = PRO_ERR_NO_ERROR;
}

void PRO04::clearErrCount() {
  readingerrcount = 0;
}

void PRO04::clearSuccCount() {
  readingsuccesscount = 0;
}

void PRO04::setMsTurnaround(uint16_t _msturnaround) {
  if (_msturnaround < SDM_MIN_DELAY)
    msturnaround = SDM_MIN_DELAY;
  else if (_msturnaround > SDM_MAX_DELAY)
    msturnaround = SDM_MAX_DELAY;
  else
    msturnaround = _msturnaround; 
}

void PRO04::setMsTimeout(uint16_t _mstimeout) {
  if (_mstimeout < SDM_MIN_DELAY)
    mstimeout = SDM_MIN_DELAY;
  else if (_mstimeout > SDM_MAX_DELAY)
    mstimeout = SDM_MAX_DELAY;
  else
    mstimeout = _mstimeout; 
}

uint16_t PRO04::getMsTurnaround() {
  return (msturnaround);
}

uint16_t PRO04::getMsTimeout() {
  return (mstimeout);
}

uint16_t PRO04::calculateCRC(uint8_t *array, uint8_t len) {
  uint16_t _crc, _flag;
  _crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    _crc ^= (uint16_t)array[i];
    for (uint8_t j = 8; j; j--) {
      _flag = _crc & 0x0001;
      _crc >>= 1;
      if (_flag)
        _crc ^= 0xA001;
    }
  }
  return _crc;
}

void PRO04::flush(unsigned long _flushtime) {
  unsigned long flushtime = millis() + _flushtime;
  while (proSer.available() || flushtime >= millis()) {
    if (proSer.available())                                                     //read serial if any old data is available
      proSer.read();
    delay(1);
  }
}

void PRO04::dereSet(bool _state) {
  if (_dere_pin != NOT_A_PIN)
    digitalWrite(_dere_pin, _state);                                            //receive from SDM -> DE Disable, /RE Enable (for control MAX485)
}

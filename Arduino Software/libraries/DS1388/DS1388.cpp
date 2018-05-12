// A library for handling DS1388 Realtime clock

#include <Arduino.h>
#include <Wire.h>
#include <DS1388.h>

// DS1388 Control register bits
#define DS1388_ADDRESS          0x68
#define DS1388_EEPROM_0         0x01
#define DS1388_EEPROM_1         0x02




////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

static const uint8_t daysInMonth [] PROGMEM = {
  31,28,31,30,31,30,31,31,30,31,30,31
};

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (long t) {
    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
// DateTime now (__DATE__, __TIME__);
// NOTE: using PSTR would further reduce the RAM footprint
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

uint8_t DateTime::dayOfWeek() const {
    uint16_t day = get() / SECONDS_PER_DAY;
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

long DateTime::get() const {
    uint16_t days = date2days(yOff, m, d);
    return time2long(days, hh, mm, ss);
}




////////////////////////////////////////////////////////////////////////////////
// DS 1388 implementation
uint8_t rtcDS1388::WDSeconds = bin2bcd(60); //default to 60 seconds;
uint8_t rtcDS1388::WDTSeconds = bin2bcd(0); //default to 60.00 seconds;

void rtcDS1388::adjust(const DateTime& dt) {
	Wire.beginTransmission(DS1388_ADDRESS);
    Wire.write((byte) 0);
    Wire.write(bin2bcd(0)); // hundreds of seconds 0x00
    Wire.write(bin2bcd(dt.second())); // 0x01
    Wire.write(bin2bcd(dt.minute())); // 0x02
    Wire.write(bin2bcd(dt.hour())); // 0x03
    Wire.write(bin2bcd(0)); // 0x04
    Wire.write(bin2bcd(dt.day())); // 0x05
    Wire.write(bin2bcd(dt.month())); // 0x06
    Wire.write(bin2bcd(dt.year() - 2000)); // 0x07
	Wire.endTransmission();

	Wire.beginTransmission(DS1388_ADDRESS);
    Wire.write((byte) 0x0b);
    Wire.write((byte) 0x00);      //clear the 'time is invalid ' flag bit (OSF)
	Wire.endTransmission();
}

DateTime rtcDS1388::now(int timeOffset, boolean useDST) {
	long lv;
	uint16_t days;
	
	Wire.beginTransmission(DS1388_ADDRESS);
    Wire.write((byte) 0);
	Wire.endTransmission();

	Wire.requestFrom(DS1388_ADDRESS, 8);
	uint8_t hs = bcd2bin(Wire.read() & 0x7F);  // hundreds of seconds
	uint8_t ss = bcd2bin(Wire.read() & 0x7F);
	uint8_t mm = bcd2bin(Wire.read());
	uint8_t hh = bcd2bin(Wire.read());
	Wire.read();
	uint8_t d = bcd2bin(Wire.read());
	uint8_t m = bcd2bin(Wire.read());
	uint16_t y = bcd2bin(Wire.read()) + 2000;
	
	days = date2days(y-2000, m, d);
	lv = time2long(days, hh, mm, ss);
	lv = lv + timeOffset*3600;

	if ((useDST == true) && (summertime_EU(y, m, d, hh, 1) == true)) {
		lv = lv + 3600;
	}
	return DateTime(lv);
}

// Test if we are running summertime
// ---------------------------------
boolean rtcDS1388::summertime_EU(int year, byte month, byte day, byte hour, byte tzHours)
// European Daylight Savings Time calculation by "jurs" for German Arduino Forum
// input parameters: "normal time" for year, month, day, hour and tzHours (0=UTC, 1=MEZ)
// return value: returns true during Daylight Saving Time, false otherwise
{
	if (month<3 || month>10) return false; // keine Sommerzeit in Jan, Feb, Nov, Dez
	if (month>3 && month<10) return true; // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
	if (month==3 && (hour + 24 * day)>=(1 + tzHours + 24*(31 - (5 * year /4 + 4) % 7)) || month==10 && (hour + 24 * day)<(1 + tzHours + 24*(31 - (5 * year /4 + 1) % 7))) {
		return true;
	} else {
		return false;
	}
}


uint8_t rtcDS1388::isrunning() {
	Wire.beginTransmission(DS1388_ADDRESS);
    Wire.write((byte)0x0b);
	Wire.endTransmission();

	Wire.requestFrom(DS1388_ADDRESS, 1);
	uint8_t ss = Wire.read();
	return !(ss>>7); //OSF flag bit
}

uint8_t rtcDS1388::getEEPROMBank(uint16_t pos) {
  if(pos > 255){
    return DS1388_ADDRESS | DS1388_EEPROM_1;
  } else {
    return DS1388_ADDRESS | DS1388_EEPROM_0;
  }
}

/*
 * DS1388 has 512 bytes EEPROM in 2 banks of 256 bytes each
 */
void rtcDS1388::EEPROMWrite(uint16_t pos, uint8_t c) {
  if(pos >= 512){
    return;
  }
  uint8_t rel_pos = pos % 256;
  // Set address
  Wire.beginTransmission(getEEPROMBank(pos));
  Wire.write((byte)rel_pos);
  // Wite data
  Wire.write((byte)c);
  Wire.endTransmission();
}


uint8_t rtcDS1388::EEPROMRead(uint16_t pos) {
  if(pos >= 512){
    return 0;
  }
  uint8_t rel_pos = pos % 256;
  Wire.beginTransmission(getEEPROMBank(pos));
  // Set address
  Wire.write((byte)rel_pos);
  Wire.endTransmission(true); // Stay open
  // Request one byte
  Wire.requestFrom(getEEPROMBank(pos), (uint8_t)1);
  uint8_t c = Wire.read();
  return c;
}

/*
 * DS1388 has 512 bytes EEPROM in 2 banks of 256 bytes each.
 * EEPROM is arranged in 64 pages of 8 bytes each.
 * Page operations take a page number (0-63) and write/read 8 bytes
 */
void rtcDS1388::EEPROMWritePage(uint8_t page, uint8_t *data) {
  if(page >= 64){
    return;
  }
  Wire.beginTransmission(getEEPROMBank((uint16_t)page * 8));
  uint8_t rel_pos =((uint16_t)page * 8) % 256;
  Wire.write((byte)rel_pos);
  for(uint8_t i=0; i<8; i++){
    Wire.write((byte)data[i]);
  }
  Wire.endTransmission();
}

void rtcDS1388::EEPROMReadPage(uint8_t page, uint8_t *data) {
  if(page >= 64){
    return;
  }
  Wire.beginTransmission(getEEPROMBank((uint16_t)page * 8));
  uint8_t rel_pos =((uint16_t)page * 8) % 256;
  // Set address
  Wire.write((byte)rel_pos);
  Wire.endTransmission(true); // Stay open
  // Request 8 byte
  Wire.requestFrom(getEEPROMBank((uint16_t)page * 8), (uint8_t)8);
  for(uint8_t i=0; i<8; i++){
    data[i] = Wire.read();
  }
}

void rtcDS1388::startWatchdogTimer(uint8_t Seconds, uint8_t TSeconds) {
  WDSeconds = bin2bcd(Seconds);
  WDTSeconds = bin2bcd(TSeconds);
  resetWatchdogTimer();
}

void rtcDS1388::resetWatchdogTimer() {
  //Disable the RTC watchdog first.
  Wire.beginTransmission(DS1388_ADDRESS);
  Wire.write(0x0b);
  Wire.write(0x00); //clear WF bit
  Wire.write(0x00); //turn off WD
  Wire.endTransmission();

  //Set the watchdog timer to the desired time
  Wire.beginTransmission(DS1388_ADDRESS);
  Wire.write(0x08);
  Wire.write(WDTSeconds);  //08h    time 00-99 first nibble is Tenths of Seconds, second nibble is Hundredths of Seconds
  Wire.write(WDSeconds);  //09h - time 00-99 first nibble is Ten Seconds, second nibble is seconds
  Wire.endTransmission();

  //Enable the watchdog timer in the RTC.  0x0c -> 0x03 (WDE and WDE/RST)
  Wire.beginTransmission(DS1388_ADDRESS);
  Wire.write(0x0c);
  Wire.write(0x03);
  Wire.endTransmission();
}

void rtcDS1388::setTrickleCharger(uint8_t state) {
  Wire.beginTransmission(DS1388_ADDRESS);
  Wire.write(0x0a);
  Wire.write(0xa5);
  Wire.endTransmission();
}




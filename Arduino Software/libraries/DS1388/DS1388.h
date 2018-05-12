// A library for handling DS1388 Realtime clock

#ifndef DS1388_H
#define DS1388_H

// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
#define SECONDS_PER_DAY         86400L


class DateTime {
public:
    DateTime (long t =0);
    DateTime (uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
    DateTime (const char* date, const char* time);

    uint16_t year() const       { return 2000 + yOff; }
    uint8_t month() const       { return m; }
    uint8_t day() const         { return d; }
    uint8_t hour() const        { return hh; }
    uint8_t minute() const      { return mm; }
    uint8_t second() const      { return ss; }
    uint8_t dayOfWeek() const;

    // 32-bit times as seconds since 1/1/2000
    long get() const;

protected:
    uint8_t yOff, m, d, hh, mm, ss;
};

// DS1388 version
class rtcDS1388 {
protected:
    static uint8_t WDSeconds;
    static uint8_t WDTSeconds;
public:
    static void begin() {};
    static void adjust(const DateTime& dt);
    static DateTime now(int timeOffset, boolean useDST);
    static uint8_t isrunning();

    // EEPROM
    static uint8_t getEEPROMBank(uint16_t pos);
    static void EEPROMWrite(uint16_t pos, uint8_t c);
    static uint8_t EEPROMRead(uint16_t pos);
    static void EEPROMWritePage(uint8_t page, uint8_t* data);
    static void EEPROMReadPage(uint8_t page, uint8_t* buffer);

    //Watchdog
    static void startWatchdogTimer(uint8_t Seconds, uint8_t TSeconds);
    static void resetWatchdogTimer();

    //Trickle Charger
    static void setTrickleCharger(uint8_t state);

    // utility functions
    static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
private:
    static boolean summertime_EU(int year, byte month, byte day, byte hour, byte tzHours);
};

#endif
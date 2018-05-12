/*
  CATGenie library for MKR1000 
*/
	
#ifndef CATGenie_H
#define CATGenie_H

#include "Arduino.h"
#include "Wire.h"
#include <CATGenieHW.h>
#include <extEEPROM.h>

// EEPROM is 512kb
#define EEP_TASKSTART 1024          	// Start Addr. tasklists
#define MAXTASKS 400					// Max number of tasks in one tasklist (8 byte/task * 400 = 3200 Byte / tasklist)
#define EEP_TASKLISTMAX 5           	// Max number of tasklists sizeofStruct(process) = 3200 * 5 = 16000 Byte for 5 tasklists 
#define EEP_WEBPAGE 24576            	// Start Addr. mit WEB Page
#define EEP_MAXWEBPAGE 40000        	// Max WEB Page length

// Watersensor definitions for analoge values
#define SENSSERVICE		350				// Sensor needs cleaning (max = 1024)
#define SENSSERVTIME	20				// Number of minutes for sensor service activation 
#define SENSFREE		580				// Free if analog value <= this value (max = 1024)
#define SENSHARDWARE	850				// Hardware switch off if analog value >= this value (max = 1024)
#define SENSFILTER		0.8				// Daemping analog input water sensor (0-1) 1=no daemping

// Tasklists for programs
#define PROGM 1                    		// Tasklist to use for Manual (no LED) 
#define PROG1 1                    		// Tasklist to use for LED1
#define PROG2 1                    		// Tasklist to use for LED2
#define PROG3 1                    		// Tasklist to use for LED3
#define PROG4 1                    		// Tasklist to use for LED4
#define PROGC 1                    		// Tasklist to use for CAT

// CAT Sensor
#define CATINTIME 5              		// CAT must be in for at least this time to activate cleaning (Seconds)
#define CLEANDELAY 5              		// Start clean after delay (Minutes)
#define CATMONTIME 60             		// Monitor time for CAT must be in (Seconds)

// Force cleaning
#define FORCECLEAN 18              		// Force clean after n hours not started

// Force reset cleaning locked after this time
#define MAXCLEANLOCK	60				//  (60 Minutes)

// Define max running times for service mode
#define MAXBOWLTIME 20*(60*10)			// Max. Bowl running time (20 Minutes)
#define MAXDRYERTIME 15*(60*10)			// Max. Dryer running time (15 Minutes)
#define MAXWATERTIME 2*(60*10)			// Max. Water running time (2 Minute)
#define MAXDOSTIME 10*(1*10)			// Max. Dosage time (10 Seconds)
#define MAXSCOPTIME 20*(1*10)			// Max. Scopper time (20 Seconds)
#define MAXDRAINTIME 2*(60*10)			// Max. Drain time (2 Minutes)


class CATGenieClass{
protected:
	volatile uint16_t waitTime = 0;
	volatile uint8_t cleaningState = 0;
	volatile int cleaningPointer = -1;
	volatile uint8_t waitWater = 0;
	volatile int taskError = 0;
	uint16_t watSensorLast = 0;

public:
	CATGenieClass() {};
	void begin();
	bool runL();
	void runI();
	int startCleaning(uint8_t tasklist);
	bool stopCleaning();
	int isCleaning();
	int getCleaningPointer();
	int getRemainingTime();
	int getError();
	uint16_t getWaterSensor();
	bool getOutput(uint16_t output);
	void setOutput(uint16_t output);
	void setOutput(uint16_t output, uint16_t duration);
	void resetOutput(uint16_t output);
	void flashOutput(uint16_t output, uint16_t cycle);
	void resetOutputAll();
	void newProgram();
	int addProgTask(uint8_t type, uint16_t par1, uint16_t par2);
	int saveProgram(uint8_t id);

private:
	int _execCommand(uint16_t pointer);
	void _handleOutputs();
	void _SetResetOutput(uint16_t output, bool state, uint8_t type, uint16_t time);
	void _ChangeOutput(uint16_t output);
	uint16_t _getWaterSensor();
	void _fromArray(uint16_t output);
	bool _toArray(uint16_t output, uint16_t type, uint16_t duration);
};

extern CATGenieClass myCATGenie;
#endif

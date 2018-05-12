/*
  CATGenie library for MKR1000 
*/

#include "CATGenie.h"

CATGenieClass myCATGenie;

// EEPROM (if you change parameter do the same in the main project)
extEEPROM eepcat(kbits_256,1,64,EEP_ADDR);  // device size, number of devices, page size

/////////////////////////////////////////////////////
// Used for flush outputs, reset output after time
// 0=type, 1=output-id, 2=set time, 3=remaining time
/////////////////////////////////////////////////////
volatile uint16_t outputArray [24][4] = {
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0},	
	{0,0,0,0}	
};


/////////////////////////////////////////////////////
// Holds the actual cleaning tasklist
// Also used for saving cleaning tasklists to EEProm
// type: 1 = wait
// par1: time in 0,1s
//
// type: 2 = set/reset output
// par1: ouptut id
// par2: 0=off, 1=on
//
/////////////////////////////////////////////////////

typedef struct {
	uint8_t type;
	uint16_t par1;
	uint16_t par2;
} task;

task process[MAXTASKS];


/////////////////////////////////////////////////////////
// Called from Timer Interrupt 4
// waitTime = process wait timer
// outputArray[][x] x: 0=type, 1=output-id, 2=set time, 3=remaining time
/////////////////////////////////////////////////////////
void CATGenieClass::runI() {
	
	// Handle clean process wait timer
	if (waitTime != 0) {
		waitTime = waitTime - 1;
	}
	for (int i = 0; i < 24; i++) {
		if (outputArray[i][0] == 0) {
			break;
		} else {
			if (outputArray[i][3] > 0) {
				outputArray[i][3] = outputArray[i][3] - 1;
			}
		}
	}
}



/////////////////////////////////////////////////////////
// Handle CATGenie Object
// Must NOT be called from Interrupt !!!!!!!!!!!!!!!!
// Returns true if cleaning cycle finished (only once)
/////////////////////////////////////////////////////////
bool CATGenieClass::runL() {
		
	// Reset Outputs after ON-Time expired, flush output
	_handleOutputs();

	// Handle cleaning process
	if (cleaningState != 0) {
		if (waitWater != 0) {
			// Wait for water sensor covered or hardware switch off
			if ((waitWater == 1) && ((_getWaterSensor() > SENSHARDWARE) || (digitalRead(WATERVALVE) == HIGH))) {
				waitWater = 0;
				waitTime = 0;
				return false;
			// Test if water sensor free
			} else if ((waitWater == 2) && (_getWaterSensor() < SENSFREE)) {
				waitWater = 0;
				waitTime = 0;
				return false;
			// Set error after time
			} else {
				if (waitTime == 0) {
					if (waitWater == 1) {
						taskError = 1;
						cleaningState = 0;
						digitalWrite(WATERLED,LOW);
						return true;
					} else {
						taskError = 2;
						cleaningState = 0;
						digitalWrite(WATERLED,LOW);
						return true;
					}
					waitWater = 0;
					return false;
				}
			}
		} else {
			if (waitTime == 0) {
				cleaningPointer = cleaningPointer + 1;
				if ((process[cleaningPointer].type == 0) || (cleaningPointer >= MAXTASKS-1)) {
					cleaningState = 0;
					digitalWrite(WATERLED,LOW);
					return true;
				} else {
					waitWater = false;
					taskError = _execCommand(cleaningPointer);
					if (taskError != 0) {
						cleaningState = 0;
						digitalWrite(WATERLED,LOW);
						return true;
					}
				}
			}
		}		
	}		
	
	return false;
}

/////////////////////////////////////////////////////////
// Initialize the CATGenie object and in/outputs
// Timer 4 interrupt must not be enabled for this call
/////////////////////////////////////////////////////////
void CATGenieClass::begin() {	
	// Processor Inputs
	pinMode(BUTTON_AUTO,INPUT_PULLUP);
	pinMode(BUTTON_START,INPUT_PULLUP);

	pinMode(DRYERFUSE,INPUT_PULLUP);
	pinMode(CATSENSOR,INPUT_PULLUP);
	pinMode(WATERVALVE,INPUT_PULLUP);

	// Processor Outputs
	pinMode(WATERLED,OUTPUT);
	digitalWrite(WATERLED,LOW);

	pinMode(WATERON,OUTPUT);
	digitalWrite(WATERON,LOW);

	pinMode(CATLED,OUTPUT);
	digitalWrite(CATLED,LOW);

	// Init all ports as outputs (0x20) for LEDs
	Wire.beginTransmission(0x20); 	//begins talking to the slave device
	Wire.write(MCP23008_IODIR); 	//selects the IODIRA register
	Wire.write(0x00); 				//this sets all port A pins to outputs
	Wire.endTransmission(); 		//stops talking to device  
	Wire.beginTransmission(0x20); 	//begins talking to the slave device
	Wire.write(MCP23008_GPIO); 		//selects the GPIO pins
	Wire.write(00000000); 			// turns all pins off
	Wire.endTransmission(); 		//stops talking to device  
 
	// Init all ports as outputs (0x21) for Relais
	Wire.beginTransmission(0x21); 	//begins talking to the slave device
	Wire.write(MCP23008_IODIR); 	//selects the IODIRA register
	Wire.write(0x00); 				//this sets all port A pins to outputs
	Wire.endTransmission(); 		//stops talking to device  
	Wire.beginTransmission(0x21); 	//begins talking to the slave device
	Wire.write(MCP23008_GPIO); 		//selects the GPIO pins
	Wire.write(00000000); 			// turns all pins off
	Wire.endTransmission(); 		//stops talking to device  	

	// Init output array
	for(int i = 0; i < 24; i++){	 
		for(int j = 0; j < 4; j++){	 
		  outputArray[i][j] = 0;	 
		}	 
	}
    
	// Switch all LED's on for 0.5s
	byte b = 0x1;    
    for (int i=0; i<8; i++) {
      Wire.beginTransmission(0x20); //starts talking to slave device
      Wire.write(0x09); //selects the GPIO pins
      Wire.write(b); // turns on pin of GPIOA
      Wire.endTransmission(); //ends communication with the device
      delay(500);
      b = b << 1;
    }
    Wire.beginTransmission(0x20); //starts talking to slave device
    Wire.write(0x09); //selects the GPIO pins
    Wire.write(0x00); // turns off all on pins of GPIOA
    Wire.endTransmission(); //ends communication with the device

	// Set no cleaning programm loaded
	process[0].type = 0;
	
	taskError = 0;
	cleaningState = 0;
	cleaningPointer = 0;
	waitTime = 0;
	waitWater = false;	
}


//////////////////////////////////////////////////////
// Start cleaning process
// Return	0 = ok -> Start cleaning process
// 			1 = Water sensor covered (high)
// 			2 = No valid cleaning program available
//////////////////////////////////////////////////////
int CATGenieClass::startCleaning(uint8_t tasklist) {
	int i;
	
	if (tasklist > EEP_TASKLISTMAX) {
		return 1;
	}
	
	// Get desired cleaning programm from EEProm
    i = eepcat.read(EEP_TASKSTART + (tasklist-1)*sizeof(process), (byte*)process, sizeof(process));
	if (i != 0) {
		return 2;
	}
	
	// Task file available (is the first byte a valid command type ? ---- should be improved by checksum)
	if ((process[0].type < 1) || (process[0].type > 3)) {
		return 3;
	}
	
	taskError = 0;
	cleaningState = 1;
	cleaningPointer = 0;
	waitTime = 0;
	waitWater = false;
	digitalWrite(WATERLED,HIGH);
	_execCommand(cleaningPointer);

	return 0;
}


///////////////////////////
// Stop cleaning process
///////////////////////////
bool CATGenieClass::stopCleaning() {

	resetOutputAll();
	taskError = 0;
	cleaningState = 0;
	waitWater = 0;
	digitalWrite(WATERLED,LOW);

	return true;
}

//////////////////////////////
// Get cleaning process state
// 0 = idle
// 1 = running
// 2 = hold
//////////////////////////////
int CATGenieClass::isCleaning() {

	return cleaningState;
}

///////////////////////////////
// Get cleaning pointer
//////////////////////////////
int CATGenieClass::getCleaningPointer() {

	return cleaningPointer;
}

///////////////////////////////
// Get remaining cleaning time
//////////////////////////////
int CATGenieClass::getRemainingTime() {
	float f;
	int i = 0;
	
	i = cleaningPointer + 1;
	f = waitTime;

	while ((process[i].type != 0) && (i <= MAXTASKS-1)) {
		// Add all wait times in seconds from actual pointer to the end
		if (process[i].type == 1) {
			f = f + (process[i].par1);
		}
		i = i + 1;
	}	
	return (int)(f/10);
}


////////////////////////////////////////////////////
// Get cleaning process error
// 0 = no error
// 1 = water sensor not high after max. time filling
// 2 = water sensor not low after discharge
/////////////////////////////////////////////////////
int CATGenieClass::getError() {

	return taskError;
}

///////////////////////////////////////////////
// Get actual analog value of the water sensor
///////////////////////////////////////////////
uint16_t CATGenieClass::getWaterSensor() {

	return _getWaterSensor();
}

///////////////////////////////////////
// Get actual state of a certain output
///////////////////////////////////////
bool CATGenieClass::getOutput(uint16_t output) {	
	uint16_t i2cAddr;
	uint16_t p;
	uint8_t gpio;
	uint8_t i;
	bool state;

	if (output <= 8) {
		state = digitalRead(output);
	} else {
		i2cAddr = (output & 0xff00) >> 8;
		p = (output & 0x00ff);
		// Get output states of device
		Wire.beginTransmission(i2cAddr); 	//begins talking to the slave device
		Wire.write(MCP23008_GPIO);	
		Wire.endTransmission();
		Wire.requestFrom(i2cAddr, 1);
		gpio = Wire.read();
		// isolate this output
		i = 1 << p;
		if ((gpio & i) == 0) {
			state = LOW;
		} else {
			state = HIGH;
		}		
	}

	return state;
}

///////////////////////////////////////
// Set certain output
///////////////////////////////////////
void CATGenieClass::setOutput(uint16_t output) {	

	_SetResetOutput(output, HIGH, 0, 0);
}

/////////////////////////////////////////
// Set certain output for a defined time
/////////////////////////////////////////
void CATGenieClass::setOutput(uint16_t output, uint16_t duration) {

	_SetResetOutput(output, HIGH, 1, duration);
}

///////////////////////////////////////
// Reset certain output
///////////////////////////////////////
void CATGenieClass::resetOutput(uint16_t output) {	

	_SetResetOutput(output, LOW, 0, 0);		
}

///////////////////////////////////////
// Reset all relais outputs
///////////////////////////////////////
void CATGenieClass::resetOutputAll() {	

	_SetResetOutput(BOWL,LOW,0,0);
	_SetResetOutput(BOWL_CW,LOW,0,0);
	_SetResetOutput(SCOPPER,LOW,0,0);
	_SetResetOutput(SCOPPER_DOWN,LOW,0,0);
	_SetResetOutput(DRYER,LOW,0,0);
	_SetResetOutput(DOSAGE_PUMP,LOW,0,0);
	_SetResetOutput(DRAIN_PUMP,LOW,0,0);
	_SetResetOutput(WATERLED,LOW,0,0);
	_SetResetOutput(WATERON,LOW,0,0);
}

/////////////////////////////////////////
// Flash certain output with frequency
/////////////////////////////////////////
void CATGenieClass::flashOutput(uint16_t output, uint16_t cycle) {

	_SetResetOutput(output, HIGH, 2, cycle);
}


/////////////////////////////////////////////////
// Methods for saving process tasklist to EEPorom
/////////////////////////////////////////////////

// Initialize actual cleaning programm
void CATGenieClass::newProgram() {	
	cleaningPointer = 0;
	process[cleaningPointer].type = 0;
}

// Add task to actual cleaning program
// type = 2 for set/reset/stop output
int CATGenieClass::addProgTask(uint8_t type, uint16_t par1, uint16_t par2) {	
	if (cleaningPointer >= MAXTASKS-1) {
		return -1;
	}
	process[cleaningPointer].type = type;
	process[cleaningPointer].par1 = par1;
	process[cleaningPointer].par2 = par2;

	cleaningPointer = cleaningPointer + 1;
	process[cleaningPointer].type = 0;
}

// Save actual cleaning programm
int CATGenieClass::saveProgram(uint8_t id) {	
	if (id  > EEP_TASKLISTMAX) {
		return -1;
	} else {
		int i = eepcat.write(EEP_TASKSTART + (id-1)*sizeof(process), (byte*) process, sizeof(process));
		if (i == 0) {
			return sizeof(process);
		} else {
			return -2;
		}
	}
}

////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////

// 0=type, 1=output-id, 2=set time, 3=remaining time
void CATGenieClass::_handleOutputs() {
	for (int i = 0; i < 24; i++) {
		// no more to do
		if (outputArray[i][0] == 0) {
			break;
		} else {					
			// Handle reset output after ON-time elapsed
			if ((outputArray[i][0] == 1) && (outputArray[i][3] == 0)) {
				_SetResetOutput(outputArray[i][1], LOW, 0, 0);
				continue;

			// Handle flash output
			} else if ((outputArray[i][0] == 2) && (outputArray[i][3] == 0)) {
				outputArray[i][3] = outputArray[i][2];
				_ChangeOutput(outputArray[i][1]);
			}
		}
	}
}

// Get actual analog value of the water sensor
uint16_t CATGenieClass::_getWaterSensor() {
	uint16_t val;

	if (digitalRead(WATERLED) == LOW) {
		digitalWrite(WATERLED,HIGH);
		delay(50);
		for (int i=0;i<2;i++) {
			val = analogRead(WATERSENSA);
			watSensorLast = val;
			delay(5);
		}
		digitalWrite(WATERLED,LOW);
	} else {
		val = SENSFILTER * analogRead(WATERSENSA) + (1.0 - SENSFILTER) * watSensorLast;
	}
	watSensorLast = val;
	return val;
}


// Set/Reset output internal (0-8) or external (20xx, 21xx)
void CATGenieClass::_SetResetOutput(uint16_t output, bool state, uint8_t type, uint16_t time) {
	uint16_t i2cAddr;
	uint16_t p;
	uint8_t gpio;
	bool state1 = state;
	
	// The following Outputs are active LOW
	if (output == BUZZER) {
		state1 = !state;	
	}

	if (output <= 8) {
		digitalWrite(output,state1);
	} else {
		i2cAddr = (output & 0xff00) >> 8;
		p = (output & 0x00ff);
		// Get output states of device
		Wire.beginTransmission(i2cAddr); 	//begins talking to the slave device
		Wire.write(MCP23008_GPIO);	
		Wire.endTransmission();
		Wire.requestFrom(i2cAddr, 1);
		gpio = Wire.read();

		// set/reset this output
		if (state1 == HIGH) {
			gpio |= 1 << p; 
		} else {
			gpio &= ~(1 << p);
		}
		// Write output states for device
		Wire.beginTransmission(i2cAddr);
		Wire.write(MCP23008_GPIO);
		Wire.write(gpio);
		Wire.endTransmission();
	}

	// If reset output remove from list
	if (state == LOW) {
		_fromArray(output);
	// Set output for max time
	} else if ((state == HIGH) && (type == 1) && (time > 0)) {
		_toArray(output, type, time);
	// Flash output
	} else if ((state == HIGH) && (type == 2) && (time > 0)) {
		_toArray(output, type, time);
	}
}

// Change output state internal (0-8) or external (20xx, 21xx)
void CATGenieClass::_ChangeOutput(uint16_t output) {
	uint16_t i2cAddr;
	uint16_t p;
	uint8_t gpio;
	uint8_t i;
	boolean state;
	
	if (output <= 8) {
		digitalWrite(output,!digitalRead(output));
	} else {
		i2cAddr = (output & 0xff00) >> 8;
		p = (output & 0x00ff);
		// Get output states of device
		Wire.beginTransmission(i2cAddr); 	//begins talking to the slave device
		Wire.write(MCP23008_GPIO);	
		Wire.endTransmission();
		Wire.requestFrom(i2cAddr, 1);
		gpio = Wire.read();

		// invert output state
		i = 1 << p;
		if ((gpio & i) == 0) {
			state = HIGH;
		} else {
			state = LOW;
		}
		
		// set/reset this output
		if (state == HIGH) {
			gpio |= 1 << p; 
		} else {
			gpio &= ~(1 << p);
		}
				
		// Write output states for device
		Wire.beginTransmission(i2cAddr);
		Wire.write(MCP23008_GPIO);
		Wire.write(gpio);
		Wire.endTransmission();
	}	
}

bool CATGenieClass::_toArray(uint16_t output, uint16_t type, uint16_t duration) {
	bool done = false;
	
	if (type == 0) {
		return done;
	}
	
	// This output already exists ?
	NVIC_DisableIRQ(TC4_IRQn);
	for (int i = 0; i < 24; i++) {	 
		if (outputArray[i][0] == 0) {
			outputArray[i][0] = type;
			outputArray[i][1] = output;
			outputArray[i][2] = duration;
			outputArray[i][3] = outputArray[i][2];
			done = true;
			break;
		} else if (outputArray[i][1] == output) {
			outputArray[i][0] = type;
			outputArray[i][1] = output;
			outputArray[i][2] = duration;
			outputArray[i][3] = outputArray[i][2];
			done = true;
			break;
		}
	}	
	NVIC_EnableIRQ(TC4_IRQn);
	return done;
}

void CATGenieClass::_fromArray(uint16_t output) {
	int i;
	int i1;

	// delete this entry
	NVIC_DisableIRQ(TC4_IRQn);
	i = 0;
	while (i < 24) {	 
		if (outputArray[i][0] == 0) {
			NVIC_EnableIRQ(TC4_IRQn);
			return;
		} else if ((outputArray[i][0] > 0) && (outputArray[i][1] == output)) {
			outputArray[i][0] = 0;
			outputArray[i][1] = 0;
			outputArray[i][2] = 0;
			outputArray[i][3] = 0;
			break;
		}
		i = i + 1;
	}

	// Find last entry
	i1 = 23;
	while (i1 >= 0) {	 
		if (outputArray[i1][0] > 0) {
			break;
		}
		i1 = i1 - 1;
	}
	// Move last entry to this entry
	if (i1 > i) {
		outputArray[i][0] = outputArray[i1][0];
		outputArray[i][1] = outputArray[i1][1];
		outputArray[i][2] = outputArray[i1][2];
		outputArray[i][3] = outputArray[i1][3];
		
		outputArray[i1][0] = 0;
		outputArray[i1][1] = 0;
		outputArray[i1][2] = 0;
		outputArray[i1][3] = 0;
	}
	NVIC_EnableIRQ(TC4_IRQn);
}

// Execute next command
// Return: 0=ok, -1,-2=isoutput is not as expected
int CATGenieClass::_execCommand(uint16_t pointer) {
	
	// Wait Time type = 1
	if (process[cleaningPointer].type == 1) {
		waitTime = process[cleaningPointer].par1;
		Serial.print(F("Wait time: "));
		Serial.println(process[cleaningPointer].par1 / 10);
		
	// Set / Reset Output type = 2
	} else if (process[cleaningPointer].type == 2) {
		switch(process[cleaningPointer].par1) {
			case BOWL:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(BOWL, LOW, 0, 0);
					_SetResetOutput(BOWL_CW, LOW, 0, 0);
					Serial.println(F("Bowl Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(BOWL, LOW, 0, 0);
					_SetResetOutput(BOWL_CW, LOW, 0, 0);
					delay(10);
					_SetResetOutput(BOWL, HIGH, 0, 0);	
					Serial.println(F("Bowl CCW"));
				}
				break;
			case BOWL_CW:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(BOWL, LOW, 0, 0);
					_SetResetOutput(BOWL_CW, LOW, 0, 0);
					Serial.println(F("Bowl Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(BOWL, LOW, 0, 0);
					_SetResetOutput(BOWL_CW, LOW, 0, 0);
					delay(10);
					_SetResetOutput(BOWL_CW, HIGH, 0, 0);	
					_SetResetOutput(BOWL, HIGH, 0, 0);	
					Serial.println(F("Bowl CW"));
				}
				break;
			case SCOPPER:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(SCOPPER, LOW, 0, 0);
					_SetResetOutput(SCOPPER_DOWN, LOW, 0, 0);
					Serial.println(F("Scopper Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(SCOPPER, LOW, 0, 0);
					_SetResetOutput(SCOPPER_DOWN, LOW, 0, 0);
					delay(10);
					_SetResetOutput(SCOPPER, HIGH, 0, 0);	
					Serial.println(F("Scopper Up"));
				}
				break;
			case SCOPPER_DOWN:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(SCOPPER, LOW, 0, 0);
					_SetResetOutput(SCOPPER_DOWN, LOW, 0, 0);
					Serial.println(F("Scopper Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(SCOPPER, LOW, 0, 0);
					_SetResetOutput(SCOPPER_DOWN, LOW, 0, 0);
					delay(10);
					_SetResetOutput(SCOPPER_DOWN, HIGH, 0, 0);	
					_SetResetOutput(SCOPPER, HIGH, 0, 0);	
					Serial.println(F("Scopper Down"));
				}
				break;
			case DRAIN_PUMP:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(DRAIN_PUMP, LOW, 0, 0);
					Serial.println(F("Drain pump Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(DRAIN_PUMP, HIGH, 0, 0);	
					Serial.println(F("Drain pump On"));
				}
				break;
			case DOSAGE_PUMP:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(DOSAGE_PUMP, LOW, 0, 0);
					Serial.println(F("Dosage pump Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(DOSAGE_PUMP, HIGH, 0, 0);	
					Serial.println(F("Dosage pump On"));
				}
				break;
			case DRYER:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(DRYER, LOW, 0, 0);
					Serial.println(F("Dryer Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(DRYER, HIGH, 0, 0);	
					cleaningState = 3;
					Serial.println(F("Dryer On"));
				}
				break;
			case WATERON:
				if (process[cleaningPointer].par2 == 0) {
					_SetResetOutput(WATERON, LOW, 0, 0);
					Serial.println(F("Water Stop"));
				} else if (process[cleaningPointer].par2 == 1) {
					_SetResetOutput(WATERON, HIGH, 0, 0);	
					cleaningState = 2;
					Serial.println(F("Water On"));
				}
				break;
		}

	// Wait for Water Sensor type = 3
	} else if (process[cleaningPointer].type == 3) {
		if (process[cleaningPointer].par1 == 1) {
			waitWater = 1;
			Serial.println(F("Wait for watersensor covered"));
		} else if (process[cleaningPointer].par1 == 2) {
			waitWater = 2;
			Serial.println(F("Wait for watersensor free"));
		} else {
			waitWater = 0;
			Serial.println(F("Wait for watersensor ????"));
		}			
		waitTime = process[cleaningPointer].par2;

	// Test output is off/on type = 4
	} else if (process[cleaningPointer].type == 4) {
		switch(process[cleaningPointer].par1) {
			case WATERON:				
				if (process[cleaningPointer].par2 == 0) {
					Serial.println(F("Is water relais stoped"));
					if (digitalRead(WATERVALVE) == LOW) {
						Serial.println(F("(Water relais should be stop but is on"));
						return 1;
					}
				} else if (process[cleaningPointer].par2 == 1) {
					Serial.println(F("Is water relais on"));
					if (digitalRead(WATERVALVE) == HIGH) {
						Serial.println(F("Water relais should be on but is stop"));
						return 1;
					}
				}
				break;
		}
	}	
	return 0;
}

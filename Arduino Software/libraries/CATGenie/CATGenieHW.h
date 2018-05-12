/////////////////////////////////////////////
// Hardware Configuration
// MKR1000
#define BUTTON_AUTO     1
#define BUTTON_START    2
#define BUZZER          0               // Output BUZZER
#define WATERLED        5               // Output WATERLED
#define WATERON         8               // Output WATEON
#define CATLED          6               // Output CATLED
#define CATSENSOR       4               // Input Digital CATSENSOR
#define WATERSENSA      A1              // Input Analog Watersensor
#define WATERVALVE	    A2              // Input Digital Water valve activ
#define DRYERFUSE       3               // Input Digital DRYERFUSE

// Define Outputs (MSB = I2C-Address, LSB = Port)
#define LED_1           0x2004
#define LED_2           0x2005
#define LED_3           0x2006
#define LED_4           0x2007
#define LED_CAT         0x2003
#define LED_LOCKED      0x2000
#define LED_CARTRIDGE   0x2002
#define LED_ERROR       0x2001
#define DOSAGE_PUMP	    0x2100
#define SCOPPER      	0x2102
#define SCOPPER_DOWN    0x2101
#define BOWL	        0x2104
#define BOWL_CW        	0x2103
#define DRAIN_PUMP	    0x2105
#define DRYER		    0x2106

// Output for Send/Receive
#define RS485RW 7                       // Output for Read=0/Write=1 of RS485

#define EEP_ADDR 0x50               	// I2c Address of the EEPROM

#define MCP23008_GPIO 0x09
#define MCP23008_IODIR 0x00


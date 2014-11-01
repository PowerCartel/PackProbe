/************************************************************************************

PackProbe

by Erik Speckman

For documentation and the latest version see: http://powercartel.com/projects/packprobe/
Source Repository: git@github.com:PowerCartel/PackProbe.git

Depends on SoftI2CMaster http://playground.arduino.cc/Main/SoftwareI2CLibrary

PackProbe allows you to obtain detailed history and health information on laptop battery 
packs without access to the corresponding laptop.

This is is a proof of concept. It doesn't handle error conditions and spurious values

PackProbe is based on:
  http://linuxehacking.blogspot.com/2014/03/recharging-and-reusing-acer-laptop.html 
  and http://forum.arduino.cc/index.php?topic=62955.0

*************************************************************************************/


/************************************************************************************

Configure and then load the SoftI2CMaster library

The default is for data (SDA) on DPin6 and clock (SDC) on DPin8 on the Arduino Yun.

For more details on configuring the library, see:
  http://playground.arduino.cc/Main/SoftwareI2CLibrary

For background on Arduino Port and Pin Mapping, see: 
  http://www.arduino.cc/en/Reference/PortManipulation

To find the pin mappings for your board see:
  Atmega 168/328 MCU (Uno, Mini, Nano, Fio, Pro): 
    http://arduino.cc/en/Hacking/PinMapping168

  Atmega 32u4 MCU (Yun, Leonardo, Micro): 
    http://arduino.cc/en/Hacking/PinMapping32u4


*************************************************************************************/

//  Arduino Dpin6 = Atmega PD7
#define SDA_PORT PORTD
#define SDA_PIN 7

// Arduino DPin8 = Atmega PB4
#define SCL_PORT PORTB
#define SCL_PIN 4

#define I2C_SLOWMODE 1
#include <SoftI2CMaster.h>


// standard I2C address for Smart Battery packs
byte deviceAddress = 11;



/************************************************************************************

Output Selection

This sketch was made to run on the WiFi-enabled Arduino Yun.

The default programming and output connection of the Yun is over TCP/IP 
rather than the serial connection used by most other Arduino boards.

The default output on the Yun is available through the Console library.

If you are using this sketch on board with a serial port, you'll need to
#include <Serial.h> instead and change every instance of "Console." in the 
code to "Serial." In addition you need to comment/uncomment the necessary 
code in the setup function before compiling and installing on your board.

We are looking for a clean way to simplify this configuration in the future, 
or make it automatic. Suggestions welcome.

*************************************************************************************/

// Yun Bridge Console Access
#include <Console.h>

// Standard and common non-standard Smart Battery commands
#define BATTERY_MODE             0x03
#define TEMPERATURE              0x08
#define VOLTAGE                  0x09
#define CURRENT                  0x0A
#define RELATIVE_SOC             0x0D
#define ABSOLUTE_SOC             0x0E
#define REMAINING_CAPACITY       0x0F
#define FULL_CHARGE_CAPACITY     0x10
#define TIME_TO_FULL             0x13
#define CHARGING_CURRENT         0x14
#define CHARGING_VOLTAGE         0x15
#define BATTERY_STATUS           0x16
#define CYCLE_COUNT              0x17
#define DESIGN_CAPACITY          0x18
#define DESIGN_VOLTAGE           0x19
#define SPEC_INFO                0x1A
#define MFG_DATE                 0x1B
#define SERIAL_NUM               0x1C
#define MFG_NAME                 0x20   // String
#define DEV_NAME                 0x21   // String
#define CELL_CHEM                0x22   // String
#define MFG_DATA                 0x23   // String
#define CELL4_VOLTAGE            0x3C   // Indidual cell voltages don't work on Lenovo and Dell Packs
#define CELL3_VOLTAGE            0x3D
#define CELL2_VOLTAGE            0x3E
#define CELL1_VOLTAGE            0x3F
#define STATE_OF_HEALTH          0x4F

#define bufferLen 32
uint8_t i2cBuffer[bufferLen];

void setup()
{
  
  /**************************************************

  Comment out the next two lines following lines for use with Serial output
  
  **************************************************/
  Bridge.begin();
  Console.begin();

  /**************************************************

  Uncomment the following lines for use with Serial output
  
  **************************************************/
  //Serial.begin(115200);  // start serial for output
  //Serial.println(i2c_init());
  //pinMode(22,INPUT_PULLUP);
  //pinMode(23,INPUT_PULLUP);


  while (!Console) {    
    ; // wait for Console port to connect.
  }

  Console.println("Console Initialized");
  
  i2c_init();
  Console.println("I2C Inialized");
  scan();
}

int fetchWord(byte func)
{
  i2c_start(deviceAddress<<1 | I2C_WRITE);
  i2c_write(func);
  i2c_rep_start(deviceAddress<<1 | I2C_READ);
  byte b1 = i2c_read(false);
  byte b2 = i2c_read(true);
  i2c_stop();
  return (int)b1|((( int)b2)<<8);
}

uint8_t i2c_smbus_read_block ( uint8_t command, uint8_t* blockBuffer, uint8_t blockBufferLen ) 
{
  uint8_t x, num_bytes;
  i2c_start((deviceAddress<<1) + I2C_WRITE);
  i2c_write(command);
  i2c_rep_start((deviceAddress<<1) + I2C_READ);
  num_bytes = i2c_read(false); // num of bytes; 1 byte will be index 0
  num_bytes = constrain(num_bytes,0,blockBufferLen-2); // room for null at the end
  for (x=0; x<num_bytes-1; x++) { // -1 because x=num_bytes-1 if x<y; last byte needs to be "nack"'d, x<y-1
    blockBuffer[x] = i2c_read(false);
  }
  blockBuffer[x++] = i2c_read(true); // this will nack the last byte and store it in x's num_bytes-1 address.
  blockBuffer[x] = 0; // and null it at last_byte+1
  i2c_stop();
  return num_bytes;
}

void scan()
{
  byte i = 0;
  for ( i= 0; i < 127; i++  )
  {
    Console.print("Address: 0x");
    Console.print(i,HEX);
    bool ack = i2c_start(i<<1 | I2C_WRITE); 
    if ( ack ) {
      Console.println(": OK");
      Console.flush();
    }
    else {
      Console.println(": -");
      Console.flush();      
    }
    i2c_stop();
  }
}

void loop()
{
  uint8_t length_read = 0;

  Console.print("Manufacturer Name: ");
  length_read = i2c_smbus_read_block(MFG_NAME, i2cBuffer, bufferLen);
  Console.write(i2cBuffer, length_read);
  Console.println("");

  Console.print("Device Name: ");
  length_read = i2c_smbus_read_block(DEV_NAME, i2cBuffer, bufferLen);
  Console.write(i2cBuffer, length_read);
  Console.println("");

  Console.print("Chemistry ");
  length_read = i2c_smbus_read_block(CELL_CHEM, i2cBuffer, bufferLen);
  Console.write(i2cBuffer, length_read);
  Console.println("");

  // Console.print("Manufacturer Data ");
  // length_read = i2c_smbus_read_block(MFG_DATA, i2cBuffer, bufferLen);
  // Console.write(i2cBuffer, length_read);
  // Console.println("");
  
  Console.print("Design Capacity: " );
  Console.println(fetchWord(DESIGN_CAPACITY));
  
  Console.print("Design Voltage: " );
  Console.println(fetchWord(DESIGN_VOLTAGE));
  
  String formatted_date = "Manufacture Date (Y-M-D): ";
  int mdate = fetchWord(MFG_DATE);
  int mday = B00011111 & mdate;
  int mmonth = mdate>>5 & B00001111;
  int myear = 1980 + (mdate>>9 & B01111111);
  formatted_date += myear;
  formatted_date += "-";
  formatted_date += mmonth;
  formatted_date += "-";
  formatted_date += mday;
  Console.println(formatted_date);

  Console.print("Serial Number: ");
  Console.println(fetchWord(SERIAL_NUM));

  Console.print("Specification Info: ");
  Console.println(fetchWord(SPEC_INFO));
 
  Console.print("Cycle Count: " );
  Console.println(fetchWord(CYCLE_COUNT));
  
  Console.print("Voltage: ");
  Console.println((float)fetchWord(VOLTAGE)/1000);

  Console.print("Full Charge Capacity: " );
  Console.println(fetchWord(FULL_CHARGE_CAPACITY));
  
  Console.print("Remaining Capacity: " );
  Console.println(fetchWord(REMAINING_CAPACITY));

  Console.print("Relative Charge(%): ");
  Console.println(fetchWord(RELATIVE_SOC));
  
  Console.print("Absolute Charge(%): ");
  Console.println(fetchWord(ABSOLUTE_SOC));
  
  Console.print("Minutes remaining for full charge: ");
  Console.println(fetchWord(TIME_TO_FULL));

  // These aren't part of the standard, but work with some packs.
  // They don't work with the Lenovo and Dell packs we've tested
  Console.print("Cell 1 Voltage: ");
  Console.println(fetchWord(CELL1_VOLTAGE));
  Console.print("Cell 2 Voltage: ");
  Console.println(fetchWord(CELL2_VOLTAGE));
  Console.print("Cell 3 Voltage: ");
  Console.println(fetchWord(CELL3_VOLTAGE));
  Console.print("Cell 4 Voltage: ");
  Console.println(fetchWord(CELL4_VOLTAGE));
  
  Console.print("State of Health: ");
  Console.println(fetchWord(STATE_OF_HEALTH));

  Console.print("Battery Mode (BIN): 0b");
  Console.println(fetchWord(BATTERY_MODE),BIN);

  Console.print("Battery Status (BIN): 0b");
  Console.println(fetchWord(BATTERY_STATUS),BIN);
  
  Console.print("Charging Current: ");
  Console.println(fetchWord(CHARGING_CURRENT));
  
  Console.print("Charging Voltage: ");
  Console.println(fetchWord(CHARGING_VOLTAGE));

  Console.print("Temp: ");
  unsigned int tempk = fetchWord(TEMPERATURE);
  Console.println((float)tempk/10.0-273.15);

  Console.print("Current (mA): " );
  Console.println(fetchWord(CURRENT));
  
  Console.println(".");
  delay(1000);
}

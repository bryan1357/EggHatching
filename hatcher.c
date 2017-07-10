#include <wiringPi.h>
#include <mcp3004.h>				// Analog signal -> Digital signal
#include <wiringPiI2C.h>			// LCD Display
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <softPwm.h>
#include "hatcher.h"

////////////////// Start: MCP3004 /////////////////////////////////////////////////////
#define SPI_BASE 		200			// SPI interface
#define SPI_CHAN		0	    	// Using CH0
////////////////// End: MCP3004 /////////////////////////////////////////////////////

#define GPIO_ServoPin		1		// wiringPi No 1, BCM No 18
#define GPIO_HeaterPin 		4		// wiringPi No 4, BCM No 23
#define GPIO_HeaterPin2		5		// wiringPi No 5, BCM No 24
#define GPIO_CameraLight	6		// wiringPi No 6, BCM No 25
#define GPIO_DHTPin			7		// wiringPi No 7, BCM No  4 

#define GPIO_SwitchSetup 	26     	// wiringPi No 26, BCM No 12	
#define GPIO_SwitchItem 	27     	// wiringPi No 27, BCM No 16
#define GPIO_SwitchAdd 		28     	// wiringPi No 28, BCM No 20
#define GPIO_SwitchMinus 	29     	// wiringPi No 29, BCM No 21

/////////////////// Start of LCD Display /////////////////////////////////////////////
// LCD Display: Define some device parameters
#define I2C_ADDR   			0x27 	// I2C device address
#define LCD_CHR  			1 		// Mode - Sending data
#define LCD_CMD  			0 		// Mode - Sending command
#define LCD_LINE1  			0x80 	// 1st line
#define LCD_LINE2  			0xC0 	// 2nd line
#define LCD_BACKLIGHT   	0x08	// On
//#define LCD_BACKLIGHT    0x00 	// Off
#define LCD_ENABLE  		0b00000100 	// Enable bit
////////////////// End of LCD Display ///////////////////////////////////////////////

#define cIntervalDispTemp   0		// Timer index for Display Temperature
#define cIntervalSetup      1		// Timer index for Setup Temperature
#define cIntervalTimeLapse  2		// Timer index for Time Lapse
#define cIntervalTimeRdFile 3		// Timer index for Recording Hatching Data
#define cIntervalEggFlip    4       // Timer index for eggs' flipping
#define cIntervalTimeMax	5		// Max Item

#define cXLeft              -1		// Setup Temperature button
#define cXRight             1		// Setup Temperature button
#define cYUp                2		// Setup Temperature button
#define cYDown              -2		// Setup Temperature button
#define cZPress             3		// Setup Temperature button
#define cNoKeyPress         0		// Setup Temperature button
#define MAXTIMINGS			85		// for Humidity Sensor

#define cServoMotor0        7       // for servo motor : degree 0
#define cServoMotor180      26      // for servo motor : degree 180

#define FILE_HATCH_DATA		"/home/pi/my_projects/egg_hatcher/fHatchData.txt"

struHatcher		*gptrMapHatcher;
int				giFHandle;			// File Handle

///////////////////////// Start: Declare global variables ///////////////////////////
const int giSampleTimes = 5;      	// Get LM35 temperature sensor sample
unsigned int giSampleInterval[cIntervalTimeMax] = {1000, 300, 7200000, 600000, 6600000};      // sample Time 

float 	gfTempUpperLimit = 39.0, gfTempLowerLimit = 38.0, gfTopHeatTemp=1.0, gfTempGap = 1;

bool 	gbSetupMode = FALSE;  		// weather in setup mode or not
bool 	gbMenuFreshDisplay = FALSE; // Refresh Menu
int 	gMenuLayer = 0;
int 	dht11_dat[5] = { 0, 0, 0, 0, 0 };	// for digital temperature & humidity sensor

///////////////////////// End: Declare global variables /////////////////////////////

//////////////////// Start: Declare the definition of functions /////////////////////
bool 	Fun_Setup(void);
void 	Fun_Thread_Setup(void);
void 	Fun_ThreadLoop(void);
void 	Fun_ThreadClose(void);
void 	Fun_Close(void);
void 	Fun_TimeLapseProcess(void);
void	Fun_TakePicture(void);
void 	Fun_DealChangeFromMapping(void);

// LCD Display: Functions for LCD Display
void  	Fun_InitialLCD(void);
void 	Fun_lcd_init(void);
void 	Fun_lcd_byte(int bits, int mode);
void 	Fun_lcd_toggle_enable(int bits);
void 	Fun_lcd_lcdLoc(int line); 		//move cursor
void 	Fun_lcd_ClrLcd(void); 			// clr LCD return home
void 	Fun_lcd_typeln(const char *s);
int  	giLcdfd;  						// seen by all subroutines
////////////////// End of LCD Display /////////////////////////////////////////////

void 	Fun_HeaterTurnOnOff (bool bOnOff);
void 	Fun_Heater2TurnOnOff (bool bOnOff);
void 	Fun_CameraLightSwitch (bool bOnOff);
void 	Fun_ServoMotorMove(int iValue);
void    Fun_EggFliping(bool bReset);
bool 	Fun_TimeInterval(int iIntervalItem);
void  	Fun_DisplayEachTemp (void);
float 	Fun_GetTemperature (int iPin);
void 	Fun_ReadDHT11_dat(void);
void  	Fun_Heater(void);
bool 	Fun_CheckOverHeat(void);
bool	Fun_CheckTooLower(void);
void 	Fun_SetupTempMenu(void);
bool	Fun_SetupModeOnOff(void);
int 	Fun_SetupCheckJoyStickKeyPress(void);
void 	Fun_SetupDisplayMenu(bool bRefresh, int iKeyIn);
void 	Fun_SetupRefreshMenu(bool bRefreshAll, int iCurPosition);
void	Fun_RecordHatchData(void);

///////////////////////////////////////////////////////////
// Function: Deal with on line command
//////////////////////////////////////////////////////////
int 	lsh_launch(char **args);
int 	lsh_execute(char **args);
char 	**lsh_split_line(char *line);
//////////////////// End: Declare the definition of functions /////////////////////

////////////////////////////////////////////////////////////////////////////////////////
// Function: Raspberry Pi Thread uses to deal with analog signal via SPI
////////////////////////////////////////////////////////////////////////////////////////
PI_THREAD (Loop_Thread0)
{
	Fun_Thread_Setup();
	
	// System Loop
	while (1)
	{
		Fun_ThreadLoop();
		// delay(1000);
		delay(500);
	}
	Fun_ThreadClose ();
}

/////////////////////////////////////////////////////////////////////////////////////////
// main function
/////////////////////////////////////////////////////////////////////////////////////////
int main (void)
{
	// egg-hatchery setup
	if (!Fun_Setup())
		return EXIT_FAILURE;
	
	// Run system loop
	while (!gptrMapHatcher->bShutDown)
	{
		Fun_ReadDHT11_dat();
		
		// wait 1sec to refresh
		delay( 1000 );
	}
			
	Fun_Close();
	
	return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Function: System Setup
/////////////////////////////////////////////////////////////////////////////////////////
bool Fun_Setup(void)
{
	int iThreadReturn;	
		
	// dispaly message
	printf("This is a egg-hatching machine.\n");
	printf("System initialize begin...\n");
	
	// Call wiringPi Setup
	if (wiringPiSetup() == -1)
	{
		printf ("error:Can't Start wiringPi!\n");
		//return FALSE;
		return FALSE;
	}
	// dispaly message
	printf("wiringPi Setup: OK\n");
		
	// create memory mapping
	giFHandle = open(FILE_EXCHANGE, O_CREAT|O_RDWR, 00777 );
	if (giFHandle < 0)
	{
		printf("error: %s file open error! \n", FILE_EXCHANGE);
		return FALSE;
	}
	lseek(giFHandle, sizeof(struHatcher)-1, SEEK_SET);
	write(giFHandle,"",1);
	
	// create memory mapping inorder to translate data
	gptrMapHatcher = (struHatcher *)mmap(NULL, sizeof(struHatcher), 
		PROT_READ|PROT_WRITE, MAP_SHARED, giFHandle, 0);
		
	// import data to memory
	gptrMapHatcher->fTempLowerLimit = gfTempLowerLimit;
	gptrMapHatcher->fTempUpperLimit = gfTempUpperLimit;
	gptrMapHatcher->fTempGap = gfTempGap;
	gptrMapHatcher->uiTakePicInterval = giSampleInterval[cIntervalTimeLapse];
    gptrMapHatcher->uiEggFlipInterval = giSampleInterval[cIntervalEggFlip];
	gptrMapHatcher->bShutDown = FALSE;
	gptrMapHatcher->bLightOn = FALSE;
	
	// dispaly message
	printf("memory mapping: OK\n");
		
	// Create a thread to read temperatures
	iThreadReturn = piThreadCreate (Loop_Thread0) ;
	if (iThreadReturn != 0)
	{
		printf ("error:can't start a thread.\n");
		return FALSE;
	}
	// dispaly message
	printf("Pi thread create: OK\n");
	
	return TRUE;	
}

/////////////////////////////////////////////////////////////////////////////////////////
// Function: System Close
/////////////////////////////////////////////////////////////////////////////////////////
void 	Fun_Close(void)
{
	close (giFHandle);
	munmap(gptrMapHatcher, sizeof(struHatcher));
}

////////////////////////////////////////////////////////////////////////////////////////
// Function: Thread System Setup
////////////////////////////////////////////////////////////////////////////////////////
void Fun_Thread_Setup(void)
{
	// Setup GPIO Pin Mode : Relay control Heater
	pinMode(GPIO_HeaterPin, OUTPUT);
	pinMode(GPIO_HeaterPin2, OUTPUT);
	
	// Setup GPIO Pin Mode : Relay control Camera light0
	pinMode(GPIO_CameraLight, OUTPUT);
	
	// Setup GPIO Pin Mode : For Setting up Temperature
	//pinMode(GPIO_ServoPin, OUTPUT);
    softPwmCreate (GPIO_ServoPin, 0, 100);
	pinMode(GPIO_SwitchSetup, INPUT); 
	pinMode(GPIO_SwitchItem, INPUT);
	pinMode(GPIO_SwitchAdd, INPUT); 
	pinMode(GPIO_SwitchMinus, INPUT);
			
	// initial MCP3008 Chip (Analog->Digital)
	mcp3004Setup (SPI_BASE, SPI_CHAN);
	
	// initial LCD Display
	Fun_InitialLCD();
	// dispaly message
	printf("LCD Display: OK\n");
	
	// turn heater on
	Fun_HeaterTurnOnOff(TRUE);
	Fun_Heater2TurnOnOff(TRUE);
	// dispaly message
	printf("Heater turn on: OK\n");
	
	// trun on the servo motor
	Fun_ServoMotorMove(cServoMotor0);
    //Fun_EggFliping(TRUE);
    
	// dispaly message
	printf("Servo motor: OK\n");
	
	// dispaly message
	printf("System Start: OK\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Function: Thread loop function
/////////////////////////////////////////////////////////////////////////////////////////
void Fun_ThreadLoop(void)
{
	if (Fun_TimeInterval(cIntervalDispTemp))     // Display Temperature every 1 sec
	{
		if (!gbSetupMode)  // if system is not in Setup Mode
			Fun_DisplayEachTemp ();
	}
    
	// Process Heater
	Fun_Heater ();
	
	// Process Temperature setting up 
	Fun_SetupTempMenu ();
	
	// Process taking picture
	Fun_TimeLapseProcess();
	
    // Process eggs' slipping
    Fun_EggFliping(FALSE);
    
	// Process changes from memory mapping
	Fun_DealChangeFromMapping();
}

/////////////////////////////////////////////////////////////////////////////////////////
// Function: Thread Close function
/////////////////////////////////////////////////////////////////////////////////////////
void Fun_ThreadClose(void)
{
	// turn off the heater
	Fun_HeaterTurnOnOff(FALSE);
	Fun_Heater2TurnOnOff(FALSE);
	
	// turn off the Servo
	//Fun_ServoMotorMove(giServoMotor0);
    Fun_EggFliping(TRUE);
}

/////////////////////////////////////////////////////////////////////////////
// Function : Time interval
/////////////////////////////////////////////////////////////////////////////
bool Fun_TimeInterval(int iIntervalItem) 
{
	unsigned long lCurrentTime;
	unsigned long lTimePassed;
	static unsigned long lPreTime[cIntervalTimeMax] = {0, 0, 0, 0, 0};

	lCurrentTime = millis();
	if (lCurrentTime > lPreTime[iIntervalItem])
	{
		lTimePassed = lCurrentTime - lPreTime[iIntervalItem];
	}
	//else
	//{
	//  lTimePassed = ULONG_MAX - previous_time + current_time;
	//}

	if(lTimePassed >= giSampleInterval[iIntervalItem])
	{
		lPreTime[iIntervalItem] = lCurrentTime;
		return TRUE;
	}
	else
		return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions: Take pictures process
/////////////////////////////////////////////////////////////////////////////////////
void 	Fun_TimeLapseProcess(void)
{
	if (Fun_TimeInterval(cIntervalTimeLapse))     // Time is up for taking a picture
	{
		Fun_CameraLightSwitch (TRUE);
		delay (1000);
		Fun_TakePicture();
		delay (10000);
		Fun_CameraLightSwitch (FALSE);
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions: Deal with eggs' flipping
/////////////////////////////////////////////////////////////////////////////////////
void Fun_EggFliping(bool bReset)
{
    int i;
    static int iPreValue = cServoMotor0;
    
    if (bReset && iPreValue != cServoMotor0)    // reset to 0 degree
    {
        for (i = cServoMotor180; i>= cServoMotor0; i--)   // make it flipping slow
        {
            Fun_ServoMotorMove(i);
            delay(500);
        }
    
        //Fun_ServoMotorMove(giServoMotor0);
        iPreValue == cServoMotor0;
    }
    
    if (Fun_TimeInterval(cIntervalEggFlip))     // Time is up for flipping eggs
	{
        if (iPreValue == cServoMotor0)
        {
            for (i = cServoMotor0; i<= cServoMotor180; i++)   // make it flipping slow
            {
                Fun_ServoMotorMove(i);
                //printf("degree i = %d\n", i);
                
                delay(500);
            }
            
            //Fun_ServoMotorMove(giServoMotor180);
            iPreValue = cServoMotor180;
            //printf("iPreValue = %d\n", iPreValue);
        }
        else
        {
            for (i = cServoMotor180; i>= cServoMotor0; i--)   // make it flipping slow
            {
                Fun_ServoMotorMove(i);
                //printf("degree i = %d\n", i);
                delay(500);
            }

            //Fun_ServoMotorMove(giServoMotor0);
            iPreValue = cServoMotor0;
            //printf("iPreValue = %d\n", iPreValue);
        }   
            
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Function: Using system command -- raspistill to take a picture
//////////////////////////////////////////////////////////////////////////////////////////////// 
void	Fun_TakePicture(void)
{
	struct tm 	*ptrTm;
	time_t 		now;
	char		strCommand[128];
	char 		*line;
	char 		**pArgs;
	int 		status;
	
	now =time(NULL);
	ptrTm=localtime(&now);
	
	sprintf(strCommand, "raspistill -vf -hf -o /home/pi/my_projects/TimeLapse/%04d%02d%02d%02d%02d%02d.jpg", 
		ptrTm->tm_year+1900, ptrTm->tm_mon+1, ptrTm->tm_mday, ptrTm->tm_hour, ptrTm->tm_min, ptrTm->tm_sec); 
		
	pArgs = lsh_split_line(strCommand);
	status = lsh_execute(pArgs);
	free(pArgs);
}

// Process changes from memory mapping
void 	Fun_DealChangeFromMapping(void)
{
	if (gptrMapHatcher->bChangeFlag)
	{
		gfTempUpperLimit = gptrMapHatcher->fTempUpperLimit;
		gfTempLowerLimit = gptrMapHatcher->fTempLowerLimit;
		giSampleInterval[cIntervalTimeLapse] = gptrMapHatcher->uiTakePicInterval;
        giSampleInterval[cIntervalEggFlip]= gptrMapHatcher->uiEggFlipInterval;
		Fun_CameraLightSwitch (gptrMapHatcher->bLightOn);		// turn light on
		gptrMapHatcher->bChangeFlag = FALSE;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// LCD Display: Initialize
/////////////////////////////////////////////////////////////////////////////////////
void  Fun_InitialLCD(void)
{
  giLcdfd = wiringPiI2CSetup(I2C_ADDR);
  Fun_lcd_init(); // setup LCD
  
  Fun_lcd_lcdLoc(LCD_LINE1);
  Fun_lcd_typeln("Welcome..I am a");
  Fun_lcd_lcdLoc(LCD_LINE2);
  Fun_lcd_typeln("Egg-Hatcher");
  //delayMicroseconds(1000);
  delay (1000);
  
  
  // 輸出初始化文字
  Fun_lcd_ClrLcd();
  Fun_lcd_lcdLoc(LCD_LINE1);
  Fun_lcd_typeln("Initializing...");
  //delayMicroseconds(1000);
  delay (1000);
  Fun_lcd_lcdLoc(LCD_LINE2);
  Fun_lcd_typeln("OK!");
    
  // 告知使用者可以開始手動輸入訊息
  Fun_lcd_ClrLcd();
  Fun_lcd_lcdLoc(LCD_LINE1);
  Fun_lcd_typeln("Temp measure...");
  //delayMicroseconds(1000);
  delay (1000);
  Fun_lcd_ClrLcd();
}

/////////////////////////////////////////////////////////////////////////////////////
// LCD Display: 
/////////////////////////////////////////////////////////////////////////////////////
void Fun_lcd_init(void)
{
  // Initialise display
  Fun_lcd_byte(0x33, LCD_CMD); // Initialise
  Fun_lcd_byte(0x32, LCD_CMD); // Initialise
  Fun_lcd_byte(0x06, LCD_CMD); // Cursor move direction
  Fun_lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
  Fun_lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
  Fun_lcd_byte(0x01, LCD_CMD); // Clear display
  //delayMicroseconds(500);
  delay(1000);
}

/////////////////////////////////////////////////////////////////////////////////////
// LCD Display: clr lcd go home loc 0x80
/////////////////////////////////////////////////////////////////////////////////////
void Fun_lcd_ClrLcd(void)
{
  Fun_lcd_byte(0x01, LCD_CMD);
  Fun_lcd_byte(0x02, LCD_CMD);
}

/////////////////////////////////////////////////////////////////////////////////////
// LCD Display: go to location on LCD
/////////////////////////////////////////////////////////////////////////////////////
void Fun_lcd_lcdLoc(int line)
{
  Fun_lcd_byte(line, LCD_CMD);
}

/////////////////////////////////////////////////////////////////////////////////////
// LCD Display: Display a line of string
/////////////////////////////////////////////////////////////////////////////////////
void Fun_lcd_typeln(const char *s)
{
  while ( *s ) Fun_lcd_byte(*(s++), LCD_CHR);
}

/////////////////////////////////////////////////////////////////////////////////////
// LCD Display: Display a Byte
/////////////////////////////////////////////////////////////////////////////////////
void Fun_lcd_byte(int bits, int mode)
{
  int bits_high;
  int bits_low;
  
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT ;
  bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT ;

  // High bits
  wiringPiI2CReadReg8(giLcdfd, bits_high);
  Fun_lcd_toggle_enable(bits_high);

  // Low bits
  wiringPiI2CReadReg8(giLcdfd, bits_low);
  Fun_lcd_toggle_enable(bits_low);
}

/////////////////////////////////////////////////////////////////////////////////////
// LCD Display: 
/////////////////////////////////////////////////////////////////////////////////////
void Fun_lcd_toggle_enable(int bits)
{
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(giLcdfd, (bits | LCD_ENABLE));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(giLcdfd, (bits & ~LCD_ENABLE));
  delayMicroseconds(500);
}


/////////////////////////////////////////////////////////////////////////////////////
// Function: Turn on / off the Heater
/////////////////////////////////////////////////////////////////////////////////////
void Fun_HeaterTurnOnOff (bool bOnOff)
{
	static bool bPreOnOff = FALSE;
  
	if (bOnOff != bPreOnOff)
	{
		if (bOnOff)
			digitalWrite(GPIO_HeaterPin, HIGH); 	// turn off heater
		else
			digitalWrite(GPIO_HeaterPin, LOW); 	// turn off heater

		bPreOnOff = bOnOff;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Function: Turn on / off the Heater
/////////////////////////////////////////////////////////////////////////////////////
void Fun_Heater2TurnOnOff (bool bOnOff)
{
	static bool bPreOnOff = FALSE;
  
	if (bOnOff != bPreOnOff)
	{
		if (bOnOff)
			digitalWrite(GPIO_HeaterPin2, HIGH); 	// turn off heater
		else
			digitalWrite(GPIO_HeaterPin2, LOW); 	// turn off heater

		bPreOnOff = bOnOff;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Function: Turn on / off the Camera Light
/////////////////////////////////////////////////////////////////////////////////////
void Fun_CameraLightSwitch (bool bOnOff)
{
	static bool bPreOnOff = FALSE;
  
	if (bOnOff != bPreOnOff)
	{
		if (bOnOff)
			digitalWrite(GPIO_CameraLight, HIGH); 	// turn off Camera Light
		else
			digitalWrite(GPIO_CameraLight, LOW); 	// turn off Camera Light

		bPreOnOff = bOnOff;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Function: Move Servo Motor
/////////////////////////////////////////////////////////////////////////////////////
void Fun_ServoMotorMove(int iValue)
{
	//softPwmCreate (GPIO_ServoPin, 0, 1024);
    	
	//if (bOnOff)		// Turn on 
		softPwmWrite (GPIO_ServoPin, iValue);
	//else			// Turn Off
	//	softPwmWrite (GPIO_ServoPin, 0);
}
	
//////////////////////////////////////////////////////////////////////////////
// Get Temperature from sensor
//////////////////////////////////////////////////////////////////////////////
void  Fun_DisplayEachTemp (void)
{
	float 	fTemp;
	static float	fTempTotal[3] = {0.0, 0.0, 0.0}, fTempAvg[3];
	static int 		iTimes = 0;
	int 	i;
	char 	strBuffer[20];

	// Read each LM35 Sensor's temperature data
	//for (i = 0; i < 3; i++)
	for (i = 0; i < 2; i++)
	{
		fTemp = Fun_GetTemperature (i);   // reading analog pin =0~2

		// for debug
		//Serial.print("Sensor Read ");          // 輸出顯示Tep 字串代表溫度
		//Serial.print(i+1);
		//Serial.print("=");
		//Serial.print(fTemp);  // 輸出顯示val 的值
		// 
    
		fTempTotal[i] = fTempTotal[i] + fTemp;

		//
		//Serial.print(" Total=");        // 輸出顯示Tep 字串代表溫度
		//Serial.println(fTempTotal[i]);  // 輸出顯示val 的值
		//
	}
	
	iTimes ++;
  
	if(iTimes == giSampleTimes)
	{
		// Calculate the average temperature
		for(i = 0; i < 2; i++)
		{
			fTempAvg[i] = fTempTotal[i] / iTimes;
			
			// set to globle temperature
			gptrMapHatcher->fCurrTemp[i] = fTempAvg[i];
			fTempTotal[i] = 0.0;	// reset
		}
		
		// Read Temeparure form Humidity sensor
		//Fun_ReadDHT11_dat();
		
		// Display the temperaures
		sprintf(strBuffer, "T1:%2.1f T2:%2.1f", gptrMapHatcher->fCurrTemp[0], gptrMapHatcher->fCurrTemp[1]);
		Fun_lcd_ClrLcd();
		Fun_lcd_lcdLoc(LCD_LINE1);
		Fun_lcd_typeln(strBuffer);

		sprintf(strBuffer, "T3:%2.1f Hm:%2.1f%%", gptrMapHatcher->fCurrTemp[2], gptrMapHatcher->fHumidity);
		Fun_lcd_lcdLoc(LCD_LINE2);
		Fun_lcd_typeln(strBuffer);

		// recording hatching data
		Fun_RecordHatchData();
		
		// reset temperature data
		iTimes = 0;
	}
}

//////////////////////////////////////////////////////////////////////////////
// Get Temperature from sensor via MCP3004
//////////////////////////////////////////////////////////////////////////////
float Fun_GetTemperature (int iPin)
{
	int 	iVal, i;
	float 	fVolts, fTemp;

  	iVal = analogRead (SPI_BASE+iPin);
	fVolts = (iVal * 3.3)/1024.0;
	fTemp = fVolts/(10.0/1000.0);

	return fTemp;
}


//////////////////////////////////////////////////////////////////////////////
// Read Humidity Sensor (Temperature & humidity)
//////////////////////////////////////////////////////////////////////////////
void Fun_ReadDHT11_dat(void)
{
	uint8_t laststate = HIGH;
	uint8_t counter	= 0;
	uint8_t j = 0, i;
	float   h, c;
 
	dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;
	
 	// pull pin down for 18 milliseconds 
	pinMode( GPIO_DHTPin, OUTPUT );
	digitalWrite( GPIO_DHTPin, LOW );
	delay( 18 );
	// then pull it up for 40 microseconds
	digitalWrite( GPIO_DHTPin, HIGH );
	delayMicroseconds( 40 );
	// prepare to read the pin 
	pinMode( GPIO_DHTPin, INPUT );
 
	// detect change and read data 
	for ( i = 0; i < MAXTIMINGS; i++ )
	{
		counter = 0;
		while ( digitalRead( GPIO_DHTPin ) == laststate )
		{
			counter++;
			delayMicroseconds( 1 );
			if ( counter == 255 )
			{
				break;
			}
		}
		laststate = digitalRead( GPIO_DHTPin );
 
		if ( counter == 255 )
			break;
 
		// ignore first 3 transitions
		if ( (i >= 4) && (i % 2 == 0) )
		{
			// shove each bit into the storage bytes 
			dht11_dat[j / 8] <<= 1;
			if ( counter > 16 )
				dht11_dat[j / 8] |= 1;
			j++;
		}
	}
 
	// check we read 40 bits (8bit x 5 ) + verify checksum in the last byte
	// print it out if data is good
	
	if ( (j >= 40) &&
	     (dht11_dat[4] == ( (dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xFF) ) )
	{
		h = (float)((dht11_dat[0] << 8) + dht11_dat[1]) / 10;
		if ( h > 100 )
		{
			gptrMapHatcher->fHumidity = dht11_dat[0];	// for DHT11
		}
		
		c = (float)(((dht11_dat[2] & 0x7F) << 8) + dht11_dat[3]) / 10;
		if ( c > 125 )
		{
			gptrMapHatcher->fCurrTemp[2] = dht11_dat[2];	// for DHT11
		}
		if ( dht11_dat[2] & 0x80 )
		{
			gptrMapHatcher->fCurrTemp[2] = -c;
		}
		
		//gfTemperature[2] = (float)dht11_dat[2];
		//gfHumidity = (float)dht11_dat[0];
		
		// for debug
		//printf("Temperature = %d.%d *C, Humidity = %d.%d %% \n", dht11_dat[2], dht11_dat[3],
		//	dht11_dat[0], dht11_dat[1]);
	}
	// for debug
	//else
	//{
	//	printf( "Data not good, skip\n" );
	//}
}

//////////////////////////////////////////////////////////////////////////////
// Function: Deal with Heater on or off
//////////////////////////////////////////////////////////////////////////////
void  Fun_Heater(void)
{
  	//unsigned long lCurrentTime;
	//unsigned long lTimePassed = 0;
	//static unsigned long lPreTime = 0, lTimeInterval = 90000;
    //static bool bStartCountDown = FALSE;
    
	if (Fun_CheckOverHeat())
    {
        Fun_HeaterTurnOnOff (FALSE);    // turn off the first Heater
        Fun_Heater2TurnOnOff (FALSE);   // turn off the second heater

        /*
        if (!bStartCountDown)   
        {
            Fun_HeaterTurnOnOff (FALSE);    // turn off the first Heater
            bStartCountDown = TRUE;         // Start to countdown for turn off the second Heater
            lPreTime = millis();
        }
        else
        {
            lCurrentTime = millis();
            if (lCurrentTime > lPreTime)
                lTimePassed = lCurrentTime - lPreTime;

            if(lTimePassed >= lTimeInterval)
            {
                lPreTime = lCurrentTime;
                Fun_Heater2TurnOnOff (FALSE);
                bStartCountDown = FALSE;
            }
        }*/
    }
  
	if (Fun_CheckTooLower())
    {
        Fun_HeaterTurnOnOff (TRUE);
        Fun_Heater2TurnOnOff (TRUE);
        //bStartCountDown = FALSE;
    }
}

/////////////////////////////////////////////////////////////////////////////////////
// Function: Check weather the Temperature is  over the top limit 
/////////////////////////////////////////////////////////////////////////////////////
bool Fun_CheckOverHeat(void)
{
	int 	i;
	bool 	bOverHeat, bOneTooHigh;

	bOverHeat = TRUE;
	bOneTooHigh = FALSE;
  
	//for (i = 0; i < 3; i++)
	for (i = 0; i < 2; i++)
	{
		if (gptrMapHatcher->fCurrTemp[i] < gfTempUpperLimit)    // only one temperature sensor below
			bOverHeat = FALSE;

		if (gptrMapHatcher->fCurrTemp[i] > (gfTempUpperLimit+gfTopHeatTemp))
			bOneTooHigh = TRUE;
	}

	if (bOneTooHigh)
		return TRUE;
  
	return bOverHeat;
}

/////////////////////////////////////////////////////////////////////////////////////
// Function: Check weather the Temperature is lower than the bottom limit 
/////////////////////////////////////////////////////////////////////////////////////
bool	Fun_CheckTooLower(void)
{
 	int 	i;
	bool 	bTooLow;

	bTooLow = TRUE;
	//for (i = 0; i < 3; i++)
	for (i = 0; i < 2; i++)
	{
		if (gptrMapHatcher->fCurrTemp[i] > gfTempLowerLimit)    // only one temperature sensor still high
			bTooLow = FALSE;
	}

	return bTooLow;
}

//////////////////////////////////////////////////////////////////////////////
// Function: Setup Temperature range (Main Function)
//////////////////////////////////////////////////////////////////////////////
void Fun_SetupTempMenu(void)
{ 
	int   iKeyPress=0; 
	bool	bPreSetupMode = FALSE;

	if (!Fun_TimeInterval(cIntervalSetup))  // if sample time isn't reach->return
		return;
   
	bPreSetupMode = gbSetupMode;
	Fun_SetupModeOnOff();
  
	if (!bPreSetupMode && gbSetupMode)
	{
		Fun_SetupDisplayMenu (gbMenuFreshDisplay, 0);
		return;
	}
  
	if (!gbSetupMode)
		return;

	iKeyPress = Fun_SetupCheckJoyStickKeyPress();   // dealing with x,y button
	if (iKeyPress == cNoKeyPress)    // No any Key is Pressed
		return;  

  //if (gbMenuFreshDisplay)       // Refresh Menu
  //{
    Fun_SetupDisplayMenu (FALSE, iKeyPress);
    //gbMenuFreshDisplay = FALSE;
  //}  
  //else
  //  Fun_SetupDisplayMenu (FALSE, );
}

///////////////////////////////////////////////////////////
// Swith setup mode on/off (When user press Z button)
// dealing with Z button
//////////////////////////////////////////////////////////
bool	Fun_SetupModeOnOff(void)
{
	int iSwitch;
   
	// Check weather z button press or not
 	iSwitch = digitalRead(GPIO_SwitchSetup);
	
	//if (iSwitch == LOW)
	if (iSwitch == HIGH)
	{
		if (gbSetupMode)
		{
			gbSetupMode = FALSE;        // turn off setup mode
			Fun_lcd_ClrLcd();
			//gbMenuFreshDisplay = TRUE;  // refresh LCD screen
		}
		else
		{
			gbSetupMode = TRUE;         // turn on 
			gbMenuFreshDisplay = TRUE;  // refresh LCD screen
		}
	}
  
	return  gbSetupMode;
}

//////////////////////////////////////////////////////////////////////////////
// return: 1->x key right
//        -1->x key left
//         2->y key up
//        -2 -> y key down, 
//         3->z key, 
//         0->no any is pressed
//////////////////////////////////////////////////////////////////////////////
int Fun_SetupCheckJoyStickKeyPress(void)
{
	int ix, iy, iz;
    
    ix = digitalRead(GPIO_SwitchItem); 
    iy = digitalRead(GPIO_SwitchAdd);
	iz = digitalRead(GPIO_SwitchMinus);
    
    //if (ix == LOW)   // x axis change menu
	if (ix == HIGH)   // x axis change menu
	{
		// debug
		//Serial.print("Get Switch Item button:"); 
		//Serial.println(ix ,DEC); 
		//
		
		return  cXRight;
	}
	
    // Y Axis
    //if (iy==LOW)   // y axis change menu
	if (iy==HIGH)   // y axis change menu
	{
		// debug
		//Serial.print("Get Switch up button:"); 
		//Serial.println(iy ,DEC); 
		//

		return  cYUp;
	}
	
    //if (iz==LOW)
	if (iz==HIGH)
	{
		// debug
		//Serial.print("Get Switch down Button:"); 
		//Serial.println(iz ,DEC); 
		//
		return cYDown;
    }
      
    return cNoKeyPress;
}

///////////////////////////////////////////////////////////
// Function:
// Parameters: 
// Return:
//////////////////////////////////////////////////////////
void Fun_SetupDisplayMenu(bool bRefresh, int iKeyIn)
{
	static int iCursor = 0;     // 0->Cursor position at Top Temp , 1->Cursor position at Low Temp 
    
	if (iKeyIn == cYUp)
	{
		if (iCursor == 0)   // setting up High Temp
		{
			gfTempUpperLimit += 1;
			
			// Update to memory mapping
			gptrMapHatcher->fTempUpperLimit = gfTempUpperLimit;
		}
		else if (iCursor == 1)  // setting up Low Temp
		{
			if ((gfTempUpperLimit-gfTempLowerLimit) > gfTempGap)    // Low Temp can't over top Temp
			{
				gfTempLowerLimit += 1;
				
				// Update to memory mapping
				gptrMapHatcher->fTempLowerLimit = gfTempLowerLimit;
			}
		}
	}
	else if (iKeyIn == cYDown) 
	{
		if (iCursor == 0)   // setting up High Temp
		{
			if ((gfTempUpperLimit-gfTempLowerLimit) > gfTempGap)    // Low Temp can't over top Temp
			{
				gfTempUpperLimit -= 1;
				
				// Update to memory mapping
				gptrMapHatcher->fTempUpperLimit = gfTempUpperLimit;
			}
		}
		else if (iCursor == 1)  // setting up Low Temp 
		{
			gfTempLowerLimit -= 1;
			
			// Update to memory mapping
			gptrMapHatcher->fTempLowerLimit = gfTempLowerLimit;
		}
	}
  //else if (iKeyIn == cXLeft)    // Chnage Setting items
  //{
  //  iCursor -= 1;
  //  if (iCursor < 0)
  //    iCursor = 1;
        
  //  bRefresh = TRUE;
  //}
	else if (iKeyIn == cXRight)   // Chnage Setting items
	{
		iCursor += 1;
		if (iCursor > 1)
			iCursor = 0;
      
		bRefresh = TRUE;
	}
  
	Fun_SetupRefreshMenu(bRefresh, iCursor);
}

///////////////////////////////////////////////////////////
// Function: Setup Temperature Menu
//////////////////////////////////////////////////////////
void 	Fun_SetupRefreshMenu(bool bRefreshAll, int iCurPosition)
{
	bool bRefreshItem[2] = {FALSE, FALSE};
	char strBuffer[20];
  
	if (bRefreshAll)
	{ 
		bRefreshItem[0] = TRUE;
		bRefreshItem[1] = TRUE;
		Fun_lcd_ClrLcd();      // Clear Screen
	}
	else 
	{
		if (iCurPosition == 0)
			bRefreshItem[0] = TRUE;
		else if (iCurPosition == 1)
			bRefreshItem[1] = TRUE;
	}

	if (bRefreshItem[0])  // refresh item 0
	{
		if (iCurPosition == 0)
			sprintf(strBuffer, "* Top Temp %2.0f", gfTempUpperLimit);	
		else
			sprintf(strBuffer, "  Top Temp %2.0f", gfTempUpperLimit);
		
		Fun_lcd_lcdLoc(LCD_LINE1);
		Fun_lcd_typeln(strBuffer);
	}
  
	if (bRefreshItem[1])  // refresh item 1
	{
		if (iCurPosition == 1)
			sprintf(strBuffer, "* Low Temp %2.0f", gfTempLowerLimit);
		else
			sprintf(strBuffer, "  Low Temp %2.0f", gfTempLowerLimit);
		
		Fun_lcd_lcdLoc(LCD_LINE2);
		Fun_lcd_typeln(strBuffer);
	}
}

/////////////////////////////////////////////////////////////////////////////////
// Recording Hatching Data to a File
/////////////////////////////////////////////////////////////////////////////////
void	Fun_RecordHatchData(void)
{
	FILE 			*ptrFileHatch;		// File Handle recording hatching information
	struct tm 		*ptrTm;
	time_t 			now;
	
	if (!Fun_TimeInterval(cIntervalTimeRdFile))  // if sample time isn't reach->return
		return;
	
	// open File to record hatching information
	ptrFileHatch = fopen(FILE_HATCH_DATA, "a+" );
	if( ptrFileHatch == NULL )
	{
        printf("File %s open: failure!",  FILE_HATCH_DATA);
		return;
	}
	//else
	//	printf("File %s open: OK\n",  FILE_HATCH_DATA);	// dispaly message
	
	now =time(NULL);
	ptrTm=localtime(&now);
		
	fprintf(ptrFileHatch, "%02d%02d%02d%02d%02d %.1f %.1f %.1f %.1f\n", ptrTm->tm_mon+1, ptrTm->tm_mday, 
		ptrTm->tm_hour, ptrTm->tm_min, ptrTm->tm_sec, gptrMapHatcher->fCurrTemp[0], 
		gptrMapHatcher->fCurrTemp[1], gptrMapHatcher->fCurrTemp[2], gptrMapHatcher->fHumidity); 	
		
		// close File
	if (ptrFileHatch != NULL)
		fclose(ptrFileHatch);
}

///////////////////////////////////////////////////////////////////////////
// brief Split a line into tokens (very naively).
// param line The line.
// return Null-terminated array of tokens.
//////////////////////////////////////////////////////////////////////////
char **lsh_split_line(char *line)
{
	int bufsize = LSH_TOK_BUFSIZE, position = 0;
	char **tokens = malloc(bufsize * sizeof(char*));
	char *token, **tokens_backup;

	if (!tokens) 
	{
		fprintf(stderr, "lsh: allocation error\n");
		exit(EXIT_FAILURE);
	}

	token = strtok(line, LSH_TOK_DELIM);
	while (token != NULL) 
	{
		tokens[position] = token;
		position++;

		if (position >= bufsize) 
		{
			bufsize += LSH_TOK_BUFSIZE;
			tokens_backup = tokens;
			tokens = realloc(tokens, bufsize * sizeof(char*));
			if (!tokens) 
			{
				free(tokens_backup);
				fprintf(stderr, "lsh: allocation error\n");
				exit(EXIT_FAILURE);
			}
		}

		token = strtok(NULL, LSH_TOK_DELIM);
	}
	tokens[position] = NULL;
	return tokens;
}

/////////////////////////////////////////////////////////////////////////////////
// @brief Execute shell built-in or launch program.
// @param args Null terminated list of arguments.
// @return 1 if the shell should continue running, 0 if it should terminate
/////////////////////////////////////////////////////////////////////////////////
int lsh_execute(char **args)
{
	int i;

	if (args[0] == NULL)
	{
		// An empty command was entered.
		return 1;
	}

	return lsh_launch(args);
}

/////////////////////////////////////////////////////////////////////////////////
// @brief Launch a program and wait for it to terminate.
// @param args Null terminated list of arguments (including program).
// @return Always returns 1, to continue execution.
/////////////////////////////////////////////////////////////////////////////////
int lsh_launch(char **args)
{
	pid_t pid;
	int status;

	pid = fork();
	if (pid == 0)
	{
		// Child process
		if (execvp(args[0], args) == -1)
		{
			perror("lsh");
		}
		exit(EXIT_FAILURE);
	} else if (pid < 0)
	{
		// Error forking
		perror("lsh");
	} else
	{
		// Parent process
		do
		{
			waitpid(pid, &status, WUNTRACED);
		} while (!WIFEXITED(status) && !WIFSIGNALED(status));
	}

	return 1;
}
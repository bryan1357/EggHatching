#ifndef _HATCHER_H_
#define _HATCHER_H_

typedef unsigned int bool;

//////////////// for on line command ////////////////////////////////////////
#define LSH_RL_BUFSIZE 		1024
#define LSH_TOK_BUFSIZE 	64
#define LSH_TOK_DELIM 		" \t\r\n\a"
/////////////////////////////////////////////////////////////////////////////
#define FILE_EXCHANGE		"/home/pi/my_projects/egg_hatcher/fExchg"

typedef struct
{
	float   fCurrTemp[3];			// current Temperarue
	float   fHumidity;				// current Humidity
	bool	bChangeFlag;			// When Modify parameters, set this flag to true	
	float	fTempLowerLimit;		// Lower Limit of Temperature
	float	fTempUpperLimit;		// Upper limit of Temperature
	float   fTempGap;				// the Temperature between Lower and Upper Temp Limit
	unsigned int uiTakePicInterval;	// raspistill take picture's interval
    unsigned int uiEggFlipInterval; // Flipping eggs' interval time
	bool    bLightOn;				// Turn the box light on/off
	bool    bShutDown;				// Shut down the main application (the app :hatcher)
}	struHatcher;


#endif
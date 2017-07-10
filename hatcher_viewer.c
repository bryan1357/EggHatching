#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <wiringPi.h>
#include "hatcher.h"

struHatcher		*gptrMapHatcher;
int				giFHandle;		// File Handle
/* List of builtin commands, followed by their corresponding functions. */
char 	*builtin_str[] = {"help", "exit", "display", "setUpTemp", "setLowTemp", "timeLapse", "eggFlip", "shutDown", "light"};

///////////////////////// End: Declare global variables /////////////////////////////


bool 	Fun_Setup(void);
void 	Fun_Close(void);
///////////////////////////////////////////////////////////
// Function: Deal with on line command
//////////////////////////////////////////////////////////
int 	lsh_cd(char **args);
int 	lsh_help(char **args);
int 	lsh_exit(char **args);
int 	lsh_display(char **args);
int 	lsh_setUpTemp(char **args);
int 	lsh_setLowTemp(char **args);
int 	lsh_timeLapse(char **args);
int 	lsh_eggFlip(char **args);
int 	lsh_ShutDown(char **args);
int 	lsh_lightOnOff(char **args);
int 	(*builtin_func[]) (char **) = {&lsh_help, &lsh_exit, &lsh_display, &lsh_setUpTemp, 
			&lsh_setLowTemp, &lsh_timeLapse, &lsh_eggFlip, &lsh_ShutDown, &lsh_lightOnOff};
int 	lsh_num_builtins(void);
int 	lsh_launch(char **args);
int 	lsh_execute(char **args);
char 	*lsh_read_line(void);
char 	**lsh_split_line(char *line);
void 	lsh_loop(void);
//////////////////// End: Declare the definition of functions /////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// main function
/////////////////////////////////////////////////////////////////////////////////////////
int main (void)
{
	// initialize
	if (!Fun_Setup())
		return 0;

	// Run command loop.
	lsh_loop();
	
	Fun_Close();

	return EXIT_SUCCESS;
	//return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Function: System Setup (initialize) 
/////////////////////////////////////////////////////////////////////////////////////////
bool 	Fun_Setup(void)
{
	giFHandle = open(FILE_EXCHANGE, O_CREAT|O_RDWR, 00777 );
	if (giFHandle < 0)
	{
		printf("error: %s file open error! \n", FILE_EXCHANGE);
		return FALSE;
	}
		
	gptrMapHatcher = (struHatcher *)mmap(NULL, sizeof(struHatcher), 
		PROT_READ|PROT_WRITE, MAP_SHARED, giFHandle, 0);
	
	// debug
	printf ("T1=%2.1f, T2=%2.1f, T3=%2.1f, Humidity=%2.1f%% \n", gptrMapHatcher->fCurrTemp[0], 
	    gptrMapHatcher->fCurrTemp[1], gptrMapHatcher->fCurrTemp[2], gptrMapHatcher->fHumidity);
	printf("The temperature range is %2.1f to %2.1f \n", gptrMapHatcher->fTempLowerLimit, 
		gptrMapHatcher->fTempUpperLimit);
	printf("The timr lapse interval is %ld milli seconds \n", gptrMapHatcher->uiTakePicInterval);
    printf("The Flipping eggs Time interval is %ld milli seconds \n", gptrMapHatcher->uiEggFlipInterval);
	// debug
	
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

///////////////////////////////////////////////////////////
// Function: Deal with on line command
//////////////////////////////////////////////////////////
int lsh_num_builtins(void)
{
	return sizeof(builtin_str) / sizeof(char *);
}

/**
   @brief Bultin command: change directory.
   @param args List of args.  args[0] is "cd".  args[1] is the directory.
   @return Always returns 1, to continue executing.
 */
int lsh_display(char **args)
{
	printf("The temperatures range: %2.1f to %2.1f \n", gptrMapHatcher->fTempLowerLimit,
		gptrMapHatcher->fTempUpperLimit);
	printf("Temperaturs now are T1:%2.1f T2:%2.1f T3:%2.1f \n", gptrMapHatcher->fCurrTemp[0], 
	    gptrMapHatcher->fCurrTemp[1], gptrMapHatcher->fCurrTemp[2]);
	printf("Humidity is %2.1f%% \n",  gptrMapHatcher->fHumidity);	
	printf("The Time Lapse interval is %ld (millisecond)\n", gptrMapHatcher->uiTakePicInterval);
    printf("The Flipping eggs Time interval is %ld (millisecond)\n", gptrMapHatcher->uiEggFlipInterval);
	
	return 1;
}

/**
   @brief Builtin command: print help.
   @param args List of args.  Not examined.
   @return Always returns 1, to continue executing.
 */
int lsh_help(char **args)
{
	printf("          This is a egg-hatcher system. \n");
	//printf("Type on line command and arguments, then hit enter.\n");
	printf("The following are the commands that you can use.\n\n");
	printf("display : show the controlled temepature range and current temperatures\n");
	printf("setUpTemp <value>: set Upper Limit Temperature\n");
	printf("setLowTemp <value>: set Lower Limit Temperature\n");
	printf("timeLapse <value>: set the interval time (millsecond) of takeing picture\n");
    printf("eggFlip <value>: set the egg flipping time (millsecond)\n");
	printf("light: turn the light on/off(1=On, 0=Off)\n");
	printf("shutDown: shut dwon the main application (hatcher)\n");
	printf("exit: exit this application\n\n");
	return 1;
}

/**
   @brief Builtin command: exit.
   @param args List of args.  Not examined.
   @return Always returns 0, to terminate execution.
 */
int lsh_exit(char **args)
{
	return 0;
}

int lsh_setUpTemp(char **args)
{
	float fSetValue;
	
	printf("The temperatures range: %2.1f to %2.1f \n", gptrMapHatcher->fTempLowerLimit, 
		gptrMapHatcher->fTempUpperLimit);
	
	if (args[1] == NULL)
	{
		printf("error: Value expected \n"); 
		return 1;
	}
		
	fSetValue = atof(args[1]);
	if ((fSetValue - gptrMapHatcher->fTempLowerLimit) >= gptrMapHatcher->fTempGap)
	{
		gptrMapHatcher->fTempUpperLimit = fSetValue;
		gptrMapHatcher->bChangeFlag = TRUE;					// set the flage to TRUE
		printf("The new upper limit temperature is setting to %2.1f \n", gptrMapHatcher->fTempUpperLimit);
		printf("The new temperatures range: %2.1f to %2.1f \n", gptrMapHatcher->fTempLowerLimit, 
			gptrMapHatcher->fTempUpperLimit);
	}
	else
	{
		printf("Fail to set up the Up-temperatures: invalid value. \n");
	}
	
	return 1;
}

int lsh_setLowTemp(char **args)
{
	float fSetValue;
	
	printf("The temperatures range: %2.1f to %2.1f \n", gptrMapHatcher->fTempLowerLimit, 
		gptrMapHatcher->fTempUpperLimit);
	if (args[1] == NULL)
	{
		printf("error: Value expected \n"); 
		return 1;
	}
	
	fSetValue = atof(args[1]);
	if ((gptrMapHatcher->fTempUpperLimit-fSetValue) >= gptrMapHatcher->fTempGap)
	{
		gptrMapHatcher->fTempLowerLimit = fSetValue;
		gptrMapHatcher->bChangeFlag = TRUE;					// set the flage to TRUE
		printf("The new lower limit temperature is setting to %2.1f \n", gptrMapHatcher->fTempLowerLimit);
		printf("The new temperatures range: %2.1f to %2.1f \n", gptrMapHatcher->fTempLowerLimit, 
			gptrMapHatcher->fTempUpperLimit);
	}
	else
	{
		printf("Fail to set up the Down-temperatures: invalid value. \n");
	}
	
	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Function: Call raspistill to take a picture
//////////////////////////////////////////////////////////////////////////////////////////////// 
int 	lsh_timeLapse(char **args)
{
	unsigned int	uiValue;
	
	printf("The Time Lapse interval is %ld \n", gptrMapHatcher->uiTakePicInterval);
	
	if (args[1] == NULL)
	{
		printf("error: Value expected \n"); 
		return 1;
	}
	
	uiValue = atoi(args[1]);
	if (uiValue > 1000)
	{
		gptrMapHatcher->uiTakePicInterval = uiValue;
		gptrMapHatcher->bChangeFlag = TRUE;					// set the flage to TRUE
		printf("The New Time Lapse Interval is %ld \n", gptrMapHatcher->uiTakePicInterval);
	}
	else
	{
		printf("Fail to set up the Time Lapse interval: invalid value. \n");
	}
	
	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Function: setting eggs' flipping time
//////////////////////////////////////////////////////////////////////////////////////////////// 
int 	lsh_eggFlip(char **args)
{
	unsigned int	uiValue;
	
	printf("The Eggs Flipping interval is %ld \n", gptrMapHatcher->uiEggFlipInterval);
	
	if (args[1] == NULL)
	{
		printf("error: Value expected \n"); 
		return 1;
	}
	
	uiValue = atoi(args[1]);
	if (uiValue > 10000)
	{
		gptrMapHatcher->uiEggFlipInterval = uiValue;
		gptrMapHatcher->bChangeFlag = TRUE;					// set the flage to TRUE
		printf("The New Eggs' Flipping Interval is %ld \n", gptrMapHatcher->uiEggFlipInterval);
	}
	else
	{
		printf("Fail to set up the Eggs' Flipping interval: invalid value. \n");
	}
	
	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Function: Shut Down the main Application
//////////////////////////////////////////////////////////////////////////////////////////////// 
int 	lsh_ShutDown(char **args)
{
	unsigned int	uiValue;
	
	printf("The egg-hatching main application (hatch) is going to be shut downTime....\n");
	delay (1000);
	
	gptrMapHatcher->bShutDown = TRUE;
	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Function: Call raspistill to turn light On/Off
//////////////////////////////////////////////////////////////////////////////////////////////// 
int 	lsh_lightOnOff(char **args)
{
	int	iValue;
	
	if (args[1] == NULL)
	{
		printf("error: Value expected (0=Off, 1=On)\n"); 
		return 1;
	}
	
	iValue = atoi(args[1]);
	if (iValue)
	{
		gptrMapHatcher->bLightOn = TRUE;		// Turn light On
		gptrMapHatcher->bChangeFlag = TRUE;		// set the flage to TRUE
		printf("Turn the light ON \n");
	}
	else
	{
		gptrMapHatcher->bLightOn = FALSE;		// Turn light Off
		gptrMapHatcher->bChangeFlag = TRUE;		// set the flage to TRUE
		printf("Turn the light OFF \n");
	}
	
	return 1;
}

/**
  @brief Launch a program and wait for it to terminate.
  @param args Null terminated list of arguments (including program).
  @return Always returns 1, to continue execution.
 */
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

/**
   @brief Execute shell built-in or launch program.
   @param args Null terminated list of arguments.
   @return 1 if the shell should continue running, 0 if it should terminate
 */
int lsh_execute(char **args)
{
	int i;

	if (args[0] == NULL)
	{
		// An empty command was entered.
		return 1;
	}

	for (i = 0; i < lsh_num_builtins(); i++) 
	{
		if (strcmp(args[0], builtin_str[i]) == 0)
		{
			return (*builtin_func[i])(args);
		}
	}

	//return lsh_launch(args);
	return 1;
}

/**
   @brief Read a line of input from stdin.
   @return The line from stdin.
 */
char *lsh_read_line(void)
{
	int bufsize = LSH_RL_BUFSIZE;
	int position = 0;
	char *buffer = malloc(sizeof(char) * bufsize);
	int c;

	if (!buffer)
	{
		fprintf(stderr, "lsh: allocation error\n");
		exit(EXIT_FAILURE);
	}

	while (1)
	{
		// Read a character
		c = getchar();

		// If we hit EOF, replace it with a null character and return.
		if (c == EOF || c == '\n')
		{
			buffer[position] = '\0';
			return buffer;
		} else
		{
			buffer[position] = c;
		}
		position++;

		// If we have exceeded the buffer, reallocate.
		if (position >= bufsize) 
		{
			bufsize += LSH_RL_BUFSIZE;
			buffer = realloc(buffer, bufsize);
			if (!buffer)
			{
				fprintf(stderr, "lsh: allocation error\n");
				exit(EXIT_FAILURE);
			}
		}
	}
}

/**
   @brief Split a line into tokens (very naively).
   @param line The line.
   @return Null-terminated array of tokens.
 */
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

/**
   @brief Loop getting input and executing it.
 */
void lsh_loop(void)
{
	char *line;
	char **args;
	int status;

	do
	{
		printf("> ");
		line = lsh_read_line();   // read command that input by user
		args = lsh_split_line(line);	// split user's command
		status = lsh_execute(args);		// using command to find comparable function to execuate

		free(line);
		free(args);
	} while (status);
}

/*
 * Sensor.h
 *
 *  Created on: 19 déc. 2012
 *      Author: bruno
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <math.h>

//#define POLICY SCHED_RR
#define POLICY SCHED_FIFO
#define THREADSTACK  65536

#define INVALID_CHECKSUM -1
#define OLD_SAMPLE        0
#define NEW_SAMPLE        1

#define NUM_SENSOR 		5
#define DATABUFSIZE 	100
#define GYROCALMAX		1000
#define SONARCALMAX		100

enum { ACCELEROMETRE, GYROSCOPE, SONAR, BAROMETRE, MAGNETOMETRE };

#define ACCEL_RATE		1
#define GYRO_RATE		1
#define SONAR_RATE		8
#define BAROM_RATE		2
#define MAGNETO_RATE	2

typedef struct SensorRawData_struct {
	int16_t		status;
	uint32_t	timestamp_s;
	uint32_t	timestamp_n;
	uint32_t	ech_num;
	uint16_t	type;
	int16_t		data[3];
} SensorRawData;

typedef struct SensorData_struct {
	uint32_t	TimeDelay;
	double		Data[3];
} SensorData;

typedef struct SensorParam_struct {
	int16_t		AbsErrorMax;
	int16_t		RelErrorMax;
	double 		minVal, centerVal, maxVal, Conversion;
	double		alpha[3][3], beta[3];
} SensorParam;

typedef struct Sensor_struct {
	const char 			*DevName;
	const char 			*Name;
	int					File;
	uint16_t			type;
	pthread_spinlock_t 	DataLock;
	sem_t				DataSem;
	pthread_mutex_t 	DataSampleMutex;
	pthread_cond_t  	DataNewSampleCondVar;
	pthread_t 			SensorThread;
	pthread_t 			LogThread;
	uint16_t			DoLog;
	uint16_t			DataIdx;
	SensorParam			*Param;
	SensorRawData		*RawData;
	SensorData			*Data;
} SensorStruct;

///////////////////////////////////   Sans Moteurs   //////////////////////////////////////////////
//Pour le capteur : ACCELEROMETRE (m/s^2)                                                        //
//A = [   0.98557275   -0.00058222    0.08572299 ]   B = [  -0.02160798 ]   detA =   0.98861002  //
//    [  -0.00058136    0.99482663    0.03341127 ]       [  -0.01358815 ]                        //
//    [   0.08572485    0.03341410    1.01688016 ]       [   0.02055694 ]                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////   Avec Moteurs   //////////////////////////////////////////////
//Pour le capteur : ACCELEROMETRE (m/s^2)                                                        //
//A = [   0.98081557   -0.05644951   -0.02957259 ]   B = [  -0.03472795 ]   detA =   0.98893316  //
//    [  -0.05646295    0.98907009   -0.03478460 ]       [  -0.00820768 ]                        //
//    [  -0.02957224   -0.03477524    1.02502064 ]       [   0.01836154 ]                        //
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Pour le capteur : ACCELEROMETRE (m/s^2)                                                        //
//A = [   3.93450205    0.11586273    0.28655248 ]   B = [  -0.02479121 ]   detA =  63.57541051  //
//    [   0.11587201    3.98803974    0.02692368 ]       [  -0.01402387 ]                        //
//    [   0.28656791    0.02694574    4.07615105 ]       [   0.02083645 ]                        //
///////////////////////////////////////////////////////////////////////////////////////////////////

#define ACCEL_PARAM { .AbsErrorMax 	        = 25, \
		 	 	 	  .RelErrorMax   		= 25, \
		 	 	 	  .minVal 		 		= 0.0, \
		 	 	 	  .centerVal 	 		= 2048.0, \
		 	 	 	  .maxVal 		 		= 4095.0, \
					  .Conversion 	 		= 4.0/2048.0, \
					  .alpha 		 		= {{0.98081557, -0.05644951, -0.02957259},{-0.05646295, 0.98907009, -0.03478460},{-0.02957224, -0.03477524, 1.02502064}}, \
					  .beta 		 		= {-0.03472795, -0.00820768, 0.01836154} \
					}

#define GYRO_PARAM { .AbsErrorMax		   = 25, \
	 	 	 	 	 .RelErrorMax		   = 25, \
	 	 	 	 	 .minVal			   = -4096.0, \
	 	 	 	 	 .centerVal			   = 0.0, \
	 	 	 	 	 .maxVal			   = 4095.0, \
	 	 	 	 	 .Conversion		   = M_PI/(180.0*16.375), \
	 	 	 	 	 .alpha				   = {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
	 	 	 	 	 .beta				   = {0.0, 0.0, 0.0} \
				   }

#define SONAR_PARAM { .AbsErrorMax			= 25, \
					  .RelErrorMax			= 25, \
					  .minVal				= 0.0, \
					  .centerVal			= 0.0, \
					  .maxVal				= 0.0, \
					  .Conversion			= 0.00034454, \
					  .alpha				= {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
					  .beta					= {0.0, 0.0, 0.0} \
					}

#define BAROM_PARAM { .AbsErrorMax			= 25, \
					  .RelErrorMax			= 25, \
					  .minVal				= 0.0, \
					  .centerVal			= 0.0, \
					  .maxVal				= 0.0, \
					  .Conversion			= 1.0, \
					  .alpha				= {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
					  .beta					= {0.0, 0.0, 0.0} \
					}

///////////////////////////////////   Sans Moteurs   //////////////////////////////////////////////
//Pour le capteur : MAGNETOMETRE                                                                 //
//A = [   2.12245267   -0.01329366   -0.01210688 ]   B = [  -0.05651377 ]   detA =  10.39523362  //
//    [  -0.01327904    2.09459729   -0.01931713 ]       [  -0.17169723 ]                        //
//    [  -0.01176129   -0.01836221    2.33860626 ]       [  -1.34522884 ]                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////   Avec Moteurs   //////////////////////////////////////////////
//Pour le capteur : MAGNETOMETRE                                                                 //
//A = [   2.11841928    0.00351888    0.00397052 ]   B = [  -0.12157171 ]   detA =  10.19543999  //
//    [   0.00331451    2.07320710   -0.01264231 ]       [  -0.16832892 ]                        //
//    [   0.00518187   -0.00926185    2.32147984 ]       [  -1.31929740 ]                        //
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Pour le capteur : MAGNETOMETRE                                                                 //
//A = [  25.50081279   -0.15794066   -0.16042178 ]   B = [  -0.05447308 ]   detA = 17883.35484213//
//    [  -0.15779214   25.09669342   -0.27330934 ]       [  -0.16979382 ]                        //
//    [  -0.15683823   -0.26239220   27.94830314 ]       [  -1.35064453 ]                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
#define MAGNETO_PARAM { .AbsErrorMax  		  = 25, \
						.RelErrorMax  		  = 25, \
						.minVal		  		  = -300.0, \
						.centerVal	  		  = 0.0, \
						.maxVal		  		  = 300.0, \
						.Conversion	  		  = 12.0/4096.0, \
						.alpha		  		  = {{2.11841928, 0.00351888, 0.00397052},{0.00331451, 2.07320710, -0.01264231},{0.00518187, -0.00926185, 2.32147984}}, \
						.beta		  		  = {-0.12157171, -0.16832892, -1.31929740}, \
					  }


#define ACCEL_INIT { .DevName 	= "/dev/accel", \
					 .Name 		= "ACCELEROMETRE (m/s^2)", \
					 .File 		= -1, \
					 .type 		= ACCELEROMETRE, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[ACCELEROMETRE], \
					 .RawData	= &(RawData[ACCELEROMETRE][0]), \
					 .Data		= &(NavData[ACCELEROMETRE][0]) \
				   }

#define GYRO_INIT  { .DevName 	= "/dev/gyro", \
					 .Name 		= "GYROSCOPE", \
					 .File 		= -1, \
					 .type 		= GYROSCOPE, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[GYROSCOPE], \
					 .RawData	= &(RawData[GYROSCOPE][0]), \
					 .Data		= &(NavData[GYROSCOPE][0]) \
				   }

#define SONAR_INIT { .DevName 	= "/dev/sonar", \
					 .Name 		= "SONAR", \
					 .File 		= -1, \
					 .type 		= SONAR, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[SONAR], \
					 .RawData	= &(RawData[SONAR][0]), \
					 .Data		= &(NavData[SONAR][0]) \
				   }

#define BAROM_INIT { .DevName 	= "/dev/barom", \
					 .Name 		= "BAROMETRE", \
					 .File 		= -1, \
					 .type 		= BAROMETRE, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[BAROMETRE], \
					 .RawData	= &(RawData[BAROMETRE][0]), \
					 .Data		= &(NavData[BAROMETRE][0]) \
				   }

#define MAGNETO_INIT { .DevName  = "/dev/magneto", \
					   .Name 	 = "MAGNETOMETRE", \
					   .File 	 = -1, \
					   .type 	 = MAGNETOMETRE, \
					   .DoLog    = 0, \
					   .DataIdx  = 0, \
					   .Param	 = &ParamData[MAGNETOMETRE], \
					   .RawData	 = &(RawData[MAGNETOMETRE][0]), \
					   .Data	 = &(NavData[MAGNETOMETRE][0]) \
				     }


void *SensorTask(void *ptr);
void *SensorLogTask(void *ptr);

int   SensorsInit(SensorStruct SensorTab[]);
int   SensorsStart (void);
int   SensorsStop (SensorStruct SensorTab[]);
int   SensorsLogsInit (SensorStruct SensorTab[]);
int   SensorsLogsStart (void);
int   SensorsLogsStop (SensorStruct SensorTab[]);

#endif /* SENSOR_H_ */

/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Sensor.h"

#define ABS(x) (((x) < 0.0) ? -(x) : (x))

#define MAX_TOT_SAMPLE 1000

#define NANOSECS_IN_SEC 1000000000

extern SensorStruct SensorTab[NUM_SENSOR];

pthread_barrier_t SensorStartBarrier;
pthread_barrier_t LogStartBarrier;
pthread_mutex_t Log_Mutex;

uint8_t SensorsActivated = 0;
uint8_t LogActivated = 0;
uint8_t numLogOutput = 0;

void *SensorTask(void *ptr) {
	/* A faire! */
	/* Tache qui sera instancié pour chaque sensor. Elle s'occupe d'aller */
	/* chercher les donnees du sensor.                                    */
	SensorStruct *Sensor = (SensorStruct *) ptr;
	SensorData tmpData;
	SensorData *sensData;
	SensorRawData tmpRaw;
	SensorRawData *RawData;
	uint16_t Idx;
	uint32_t timeprev_s = 0;
	uint32_t timeprev_n = 1;	//On initialise a 1ns pour eviter une division par zero au demarrage

	pthread_barrier_wait(&(SensorStartBarrier));

	while (SensorsActivated) {
		//sem_wait(&(Sensor->DataSem));
		if (SensorsActivated == 0)
			break;

		//On lit les donnees des capteurs (bloquant)
		if (read(Sensor->File, &tmpRaw, sizeof(SensorRawData))
				== sizeof(SensorRawData)) {
			//On verifie la validite des donnees lues des pilotes
			if (tmpRaw.status != 0 && Sensor->type == tmpRaw.type) {

				//On convertit les donnees en valeurs reelles
				tmpData.Data[0] = (Sensor->Param->Conversion
						* (((double) (tmpRaw.data[0]))
								- Sensor->Param->centerVal));
				tmpData.Data[1] = (Sensor->Param->Conversion
						* (((double) (tmpRaw.data[1]))
								- Sensor->Param->centerVal));
				tmpData.Data[2] = (Sensor->Param->Conversion
						* (((double) (tmpRaw.data[2]))
								- Sensor->Param->centerVal));

				//if (tmpRaw.type == ACCELEROMETRE) {
				//printf("Raw[0] : %i\nRaw[1] : %i\nRaw[2] : %i\n", tmpRaw.data[0], tmpRaw.data[1], tmpRaw.data[2]);
				//printf("Data[0] : %f\nData[1] : %f\nData[2] : %f\n\n\n", tmpData.Data[0], tmpData.Data[1], tmpData.Data[2]);
				//}

				//On calcule le temps entre l'echatillon precedent et l'echantillon courant
				tmpData.TimeDelay = tmpRaw.timestamp_s - timeprev_s;
				if (tmpData.TimeDelay != 0) {
					tmpData.TimeDelay = (NANOSECS_IN_SEC - timeprev_n)
							+ tmpRaw.timestamp_n;
				} else {
					tmpData.TimeDelay = tmpRaw.timestamp_n - timeprev_n;
				}
				//On met a jour le temps de l'echantillon
				timeprev_s = tmpRaw.timestamp_s;
				timeprev_n = tmpRaw.timestamp_n;

				//printf("timeDelay in nanosecs: %i\n", tmpData.TimeDelay);

				//On va chercher l'index de l'echantillon present
				//pthread_spin_lock(&(Sensor->DataLock));
				Idx = Sensor->DataIdx;

				//On assigne les pointeurs vers l'echantillon present
				sensData = &(Sensor->Data[Idx]);
				RawData = &(Sensor->RawData[Idx]);

				//On copie les donnees des capteurs et converties dans leur tableau respectif
				memcpy((void *) RawData, (void *) &tmpRaw,
						sizeof(SensorRawData));
				memcpy((void *) sensData, (void *) &tmpData,
						sizeof(SensorData));
				Sensor->DataIdx = (Sensor->DataIdx + 1) % DATABUFSIZE;
				//pthread_spin_unlock(&(Sensor->DataLock));

				//if (tmpRaw.ech_num - tmpRawPrev.ech_num != 1) {
				// we missed a sample --- notify???
				//}

				//On signale qu'un nouvel echantillon est disponible pour etre traite
				pthread_mutex_lock(&(Sensor->DataSampleMutex));
				pthread_cond_broadcast(&(Sensor->DataNewSampleCondVar));
				pthread_mutex_unlock(&(Sensor->DataSampleMutex));

				//Les données ont été lues et placées dans "data"
			} else {
				//we had problems reading or no sample is available
			}
		} else {
			//La structure n'a pas été copiée en entier
		}

	}
	pthread_exit(0); /* exit thread */
}

int SensorsInit(SensorStruct SensorTab[NUM_SENSOR]) {
	/* A faire! */
	/* Ici, vous devriez faire l'initialisation de chacun des capteurs.  */
	/* C'est-à-dire de faire les initialisations requises, telles que    */
	/* ouvrir les fichiers des capteurs, et de créer les Tâches qui vont */
	/* s'occuper de réceptionner les échantillons des capteurs.          */
	int i;
	pthread_attr_t attr;
	struct sched_param param;
	int minprio, maxprio;
	int retval = 0;

	//open sensor ports
	for (i = 0; i < NUM_SENSOR; i++) {
		SensorTab[i].File = open(SensorTab[i].DevName, O_RDONLY);
		if (SensorTab[i].File < 0) {
			printf("Sensor open: Impossible d'ouvrir le %s\n",
					SensorTab[i].DevName);
			return SensorTab[i].File;
		}
	}

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);

	param.sched_priority = minprio + ((maxprio - minprio) / 5);

	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	pthread_barrier_init(&SensorStartBarrier, NULL, NUM_SENSOR + 1);//TODO: check how many threads are needed for startup
	for (i = 0; i < NUM_SENSOR; i++) {
		sem_init(&(SensorTab[i].DataSem), 0, 0);
		pthread_spin_init(&(SensorTab[i].DataLock), PTHREAD_PROCESS_SHARED);
		pthread_mutex_init(&(SensorTab[i].DataSampleMutex), NULL );
		pthread_cond_init(&(SensorTab[i].DataNewSampleCondVar), NULL );

		retval = pthread_create(&(SensorTab[i].SensorThread), &attr, SensorTask,
				(void *) &(SensorTab[i]));
		if (retval) {
			printf(
					"pthread_create : Impossible de créer le thread sensor_control_thread\n");
			return retval;

			pthread_attr_destroy(&attr);
		}
	}

	return retval;
}

int SensorsStart(void) {
	/* A faire! */
	/* Ici, vous devriez démarrer l'acquisition sur les capteurs.        */
	/* Les capteurs ainsi que tout le reste du système devrait être      */
	/* prêt à faire leur travail et il ne reste plus qu'à tout démarrer. */

	SensorsActivated = 1;
	pthread_barrier_wait(&SensorStartBarrier);
	pthread_barrier_destroy(&SensorStartBarrier);
	printf("%s Sensor démarré\n", __FUNCTION__);

	return 0;
}

int SensorsStop(SensorStruct SensorTab[NUM_SENSOR]) {
	/* A faire! */
	/* Ici, vous devriez défaire ce que vous avez fait comme travail dans */
	/* SensorsInit() (toujours verifier les retours de chaque call)...    */
	int i, err = 0;
	SensorStruct *Sensor;

	SensorsActivated = 0;
	//sem_post(&SensorTimerSem);

	for (i = 0; i < NUM_SENSOR; i++) {
		Sensor = &(SensorTab[i]);
		sem_post(&(Sensor->DataSem));
		pthread_cond_broadcast(&(Sensor->DataNewSampleCondVar));

		if (err = pthread_join(Sensor->SensorThread, NULL )) {
			printf("pthread_join(MotorThreadTask) : Erreur\n");
			return err;
		}
		sem_destroy(&(Sensor->DataSem));
		pthread_spin_destroy(&(Sensor->DataLock));
		pthread_mutex_destroy(&(Sensor->DataSampleMutex));
		pthread_cond_destroy(&(Sensor->DataNewSampleCondVar));
	}

	return err;
}

/* Le code ci-dessous est un CADEAU !!!	*/
/* Ce code permet d'afficher dans la console les valeurs reçues des capteurs.               */
/* Évidemment, celà suppose que les acquisitions sur les capteurs fonctionnent correctement. */
/* Donc, dans un premier temps, ce code peut vous servir d'exemple ou de source d'idées.     */
/* Et dans un deuxième temps, peut vous servir pour valider ou vérifier vos acquisitions.    */
/*                                                                                           */
/* NOTE : Ce code suppose que les échantillons des capteurs sont placés dans un tampon       */
/*        circulaire de taille DATABUFSIZE, tant pour les données brutes (RawData) que       */
/*        les données converties (NavData) (voir ci-dessous)                                 */
void *SensorLogTask(void *ptr) {
	SensorStruct *Sensor = (SensorStruct *) ptr;
	uint16_t *Idx = &(Sensor->DataIdx);
	uint16_t LocalIdx = DATABUFSIZE;
	SensorData *NavData = NULL;
	SensorRawData *RawData = NULL;
	SensorRawData tpRaw;
	SensorData tpNav;
	double norm;

	printf("%s : Log de %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&(LogStartBarrier));

	while (LogActivated) {
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		while (LocalIdx == *Idx) {
			pthread_cond_wait(&(Sensor->DataNewSampleCondVar),
					&(Sensor->DataSampleMutex));
		}
		pthread_mutex_unlock(&(Sensor->DataSampleMutex));

		//pthread_spin_lock(&(Sensor->DataLock));
		NavData = &(Sensor->Data[LocalIdx]);
		RawData = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
		//pthread_spin_unlock(&(Sensor->DataLock));

		pthread_mutex_lock(&Log_Mutex);
		if (numLogOutput == 0)
			printf(
					"Sensor  :     TimeStamp      SampleDelay  Status  SampleNum   Raw Sample Data  =>        Converted Sample Data               Norme\n");
		else
			switch (tpRaw.type) {
			case ACCELEROMETRE:
				norm = sqrt(
						tpNav.Data[0] * tpNav.Data[0]
								+ tpNav.Data[1] * tpNav.Data[1]
								+ tpNav.Data[2] * tpNav.Data[2]);
				printf(
						"Accel   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n",
						tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay,
						tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0],
						(uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2],
						tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
				break;
			case GYROSCOPE:
				norm = sqrt(
						tpNav.Data[0] * tpNav.Data[0]
								+ tpNav.Data[1] * tpNav.Data[1]
								+ tpNav.Data[2] * tpNav.Data[2]);
				printf(
						"Gyro    : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n",
						tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay,
						tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0],
						(uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2],
						tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
				break;
			case SONAR:
				printf(
						"Sonar   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n",
						tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay,
						tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0],
						tpNav.Data[0]);
				break;
			case BAROMETRE:
				printf(
						"Barom   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n",
						tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay,
						tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0],
						tpNav.Data[0]);
				break;
			case MAGNETOMETRE:
				norm = sqrt(
						tpNav.Data[0] * tpNav.Data[0]
								+ tpNav.Data[1] * tpNav.Data[1]
								+ tpNav.Data[2] * tpNav.Data[2]);
				printf(
						"Magneto : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n",
						tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay,
						tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0],
						(uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2],
						tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
				break;
			}
		LocalIdx = *Idx;
		numLogOutput++;
		if (numLogOutput > 20)
			numLogOutput = 0;
		pthread_mutex_unlock(&Log_Mutex);
	}

	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);

	pthread_exit(0); /* exit thread */
}

int InitSensorLog(SensorStruct *Sensor) {
	pthread_attr_t attr;
	struct sched_param param;
	int retval;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Creating Log thread : %s\n", Sensor->Name);
	if ((retval = pthread_create(&(Sensor->LogThread), &attr, SensorLogTask,
			(void *) Sensor)) != 0)
		printf("%s : Impossible de créer Tâche Log de %s => retval = %d\n",
				__FUNCTION__, Sensor->Name, retval);

	pthread_attr_destroy(&attr);

	return 0;
}

int SensorsLogsInit(SensorStruct SensorTab[]) {
	int16_t i, numLog = 0;
	int16_t retval = 0;

	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			if ((retval = InitSensorLog(&SensorTab[i])) < 0) {
				printf(
						"%s : Impossible d'initialiser log de %s => retval = %d\n",
						__FUNCTION__, SensorTab[i].Name, retval);
				return -1;
			}
			numLog++;
		}
	}
	pthread_barrier_init(&LogStartBarrier, NULL, numLog + 1);
	pthread_mutex_init(&Log_Mutex, NULL );

	return 0;
}
;

int SensorsLogsStart(void) {
	LogActivated = 1;
	pthread_barrier_wait(&(LogStartBarrier));
	pthread_barrier_destroy(&LogStartBarrier);
	printf("%s NavLog démarré\n", __FUNCTION__);

	return 0;
}
;

int SensorsLogsStop(SensorStruct SensorTab[]) {
	int16_t i;

	LogActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			pthread_mutex_lock(&(SensorTab[i].DataSampleMutex));
			//SensorTab[i].DataIdx++;
			pthread_cond_broadcast(&(SensorTab[i].DataNewSampleCondVar));
			pthread_mutex_unlock(&(SensorTab[i].DataSampleMutex));

			pthread_join(SensorTab[i].LogThread, NULL );
			SensorTab[i].DoLog = 0;
		}
	}
	pthread_mutex_destroy(&Log_Mutex);

	return 0;
}
;


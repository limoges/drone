/*
 * Motor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Motor.h"

#define TimeDelay 50000

extern sem_t MotorTimerSem;
extern int MotorActivated;

pthread_barrier_t MotorStartBarrier;

int gpio_set(int nr, int val) {
	char cmdline[200];

	if (val < 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d i", nr);
	else if (val > 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 1", nr);
	else
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 0", nr);

	return system(cmdline);
}

int motor_open(void) {
	struct termios config;
	int uart = open(MOTOR_UART, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart < 0) {
		printf("motor_open : impossible d'ouvrir le uart du moteur\n");
		return uart;
	}

	fcntl(uart, F_SETFL, 0); //read calls are non blocking

	//set port config
	tcgetattr(uart, &config);
	cfsetspeed(&config, B115200);
	config.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	config.c_iflag = 0; //clear input config
	config.c_lflag = 0; //clear local config
	config.c_oflag &= ~OPOST; //clear output config (raw output)
	cfmakeraw(&config);
	tcsetattr(uart, TCSANOW, &config);
	return uart;
}

int motor_cmd(int file, uint8_t cmd, uint8_t *reply, int replylen) {
	int size;

	write(file, &cmd, 1);
	fsync(file);
	usleep(TimeDelay);
	size = read(file, reply, replylen);

	return size;
}

int MotorPortInit(MotorStruct *Motor) {
	uint8_t reply[256];
	int i;

	//open motor port
	Motor->file = motor_open();
	if (Motor->file < 0) {
		printf("motor_open: Impossible d'ouvrir le UART\n");
		return Motor->file;
	}
	//reset IRQ flipflop - this code resets GPIO_ERROR_READ to 0
	gpio_set(GPIO_ERROR_RESET, 0);
	usleep(2 * TimeDelay);

	//all select lines inactive
	gpio_set(GPIO_M1, 0);
	gpio_set(GPIO_M2, 0);
	gpio_set(GPIO_M3, 0);
	gpio_set(GPIO_M4, 0);
	usleep(2 * TimeDelay);

	//configure motors
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_M1 + i, -1);
		usleep(2 * TimeDelay);
		motor_cmd(Motor->file, 0xE0, reply, 2);
		motor_cmd(Motor->file, 0x91, reply, 121);
		motor_cmd(Motor->file, 0xA1, reply, 2);
		motor_cmd(Motor->file, i + 1, reply, 1);
		motor_cmd(Motor->file, 0x40, reply, 2);
		gpio_set(GPIO_M1 + i, 0);
		usleep(2 * TimeDelay);
	}

	//all select lines active
	gpio_set(GPIO_M1, -1);
	gpio_set(GPIO_M2, -1);
	gpio_set(GPIO_M3, -1);
	gpio_set(GPIO_M4, -1);
	usleep(2 * TimeDelay);

	gpio_set(GPIO_ERROR_READ, -1);
	usleep(2 * TimeDelay);

	return 0;
}

void SetPWM(uint16_t mot_filedesc, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3,
		uint16_t pwm4) {
	unsigned char cmd[5];

	cmd[0] = 0x20 | ((pwm1 & 0x1ff) >> 4);
	cmd[1] = ((pwm1 & 0x1ff) << 4) | ((pwm2 & 0x1ff) >> 5);
	cmd[2] = ((pwm2 & 0x1ff) << 3) | ((pwm3 & 0x1ff) >> 6);
	cmd[3] = ((pwm3 & 0x1ff) << 2) | ((pwm4 & 0x1ff) >> 7);
	cmd[4] = ((pwm4 & 0x1ff) << 1);

	write(mot_filedesc, cmd, 5);
}

void setLEDs(uint16_t mot_filedesc, uint16_t led1, uint16_t led2, uint16_t led3,
		uint8_t led4) {
	uint8_t cmd[2];
	uint16_t leds;

	led1 &= MOTOR_LEDORANGE;
	led2 &= MOTOR_LEDORANGE;
	led3 &= MOTOR_LEDORANGE;
	led4 &= MOTOR_LEDORANGE;
	leds = (led1 << 1) | (led2 << 2) | (led3 << 3) | (led4 << 4);

	cmd[0] = 0x60;
	cmd[0] |= (leds & 0xFF00) >> 8;
	cmd[1] = leds & 0x00FF;

	write(mot_filedesc, cmd, 2);
}

void motor_send(MotorStruct *Motor, int SendMode) {
	/* Fonction utilitaire pour simplifier les transmissions aux moteurs */
	uint16_t motorpwm[4];
	uint16_t motorleds[4];

	/*pthread_spin_lock(&(Motor->MotorLock));
	 switch (SendMode) {
	 case MOTOR_NONE:
	 break;
	 case MOTOR_PWM_ONLY:
	 SetPWM(Motor->file, Motor->pwm[0], Motor->pwm[1], Motor->pwm[2],
	 Motor->pwm[3]);
	 break;
	 case MOTOR_LED_ONLY:
	 setLEDs(Motor->file, Motor->led[0], Motor->led[1], Motor->led[2],
	 Motor->led[3]);
	 break;
	 case MOTOR_PWM_LED:
	 SetPWM(Motor->file, Motor->pwm[0], Motor->pwm[1], Motor->pwm[2],
	 Motor->pwm[3]);
	 setLEDs(Motor->file, Motor->led[0], Motor->led[1], Motor->led[2],
	 Motor->led[3]);
	 break;
	 }
	 pthread_spin_unlock(&(Motor->MotorLock));*/

	pthread_spin_lock(&(Motor->MotorLock));
	motorpwm[0] = Motor->pwm[0];
	motorpwm[0] = Motor->pwm[1];
	motorpwm[0] = Motor->pwm[2];
	motorpwm[0] = Motor->pwm[3];

	motorpwm[0] = Motor->led[0];
	motorpwm[0] = Motor->led[1];
	motorpwm[0] = Motor->led[2];
	motorpwm[0] = Motor->led[3];
	pthread_spin_unlock(&(Motor->MotorLock));

	switch (SendMode) {
	case MOTOR_NONE:
		break;
	case MOTOR_PWM_ONLY:
		SetPWM(Motor->file, motorpwm[0], motorpwm[1], motorpwm[2], motorpwm[3]);
		break;
	case MOTOR_LED_ONLY:
		setLEDs(Motor->file, motorleds[0], motorleds[1], motorleds[2],
				motorleds[3]);
		break;
	case MOTOR_PWM_LED:
		SetPWM(Motor->file, motorpwm[0], motorpwm[1], motorpwm[2], motorpwm[3]);
		setLEDs(Motor->file, motorleds[0], motorleds[1], motorleds[2],
				motorleds[3]);
		break;
	}

}

void *MotorTask(void *ptr) {
	/* A faire! */
	/* Tache qui transmet les nouvelles valeurs de vitesse */
	/* à chaque moteur à interval régulier (5 ms).         */
	MotorStruct *Motor = (MotorStruct *) ptr;

	pthread_barrier_wait(&MotorStartBarrier);

	printf("%s : Moteur démarré\n", __FUNCTION__);

	while (MotorActivated) {
		sem_wait(&MotorTimerSem);
		if (MotorActivated == 0)
			break;

		motor_send(Motor, MOTOR_PWM_LED);
//		DOSOMETHING();

	}
	pthread_exit(0); /* exit thread */
}

int MotorInit(MotorStruct *Motor) {
	/* A faire! */
	/* Ici, vous devriez faire l'initialisation des moteurs.   */
	/* C'est-à-dire initialiser le Port des moteurs avec la    */
	/* fonction MotorPortInit() et créer la Tâche MotorTask()  */
	/* qui va s'occuper des mises à jours des moteurs en cours */
	/* d'exécution.                                            */
	pthread_attr_t attr;
	struct sched_param param;
	int minprio, maxprio;
	int retval = 0;

	if (MotorPortInit(Motor) < 0) {
		return -1;
	} else {
		pthread_attr_init(&attr);
		pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
		minprio = sched_get_priority_min(POLICY);
		maxprio = sched_get_priority_max(POLICY);
		pthread_attr_setschedpolicy(&attr, POLICY);

		param.sched_priority = minprio + ((maxprio - minprio) / 6);

		pthread_attr_setstacksize(&attr, THREADSTACK);
		pthread_attr_setschedparam(&attr, &param);

		pthread_barrier_init(&MotorStartBarrier, NULL, 2);
		sem_init(&MotorTimerSem, 0, 0);

		retval = pthread_create(&(Motor->MotorThread), &attr, MotorTask,
				(void *) Motor);
		if (retval) {
			printf(
					"pthread_create : Impossible de créer le thread motor_control_thread\n");
			return retval;
		}

		pthread_attr_destroy(&attr);

	}

	return retval;
}

int MotorStart(void) {
	/* A faire! */
	/* Ici, vous devriez démarrer la mise à jour des moteurs (MotorTask).    */
	/* Tout le système devrait être prêt à faire leur travail et il ne reste */
	/* plus qu'à tout démarrer.                                              */

	MotorActivated = 1;
	pthread_barrier_wait(&MotorStartBarrier);
	pthread_barrier_destroy(&MotorStartBarrier);
	printf("%s Motor démarré\n", __FUNCTION__);

	return 0; //retval;

}

int MotorStop(MotorStruct *Motor) {
	/* A faire! */
	/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */
	int err = 0;

	MotorActivated = 0;
	sem_post(&MotorTimerSem);

	err = pthread_join(Motor->MotorThread, NULL );
	if (err) {
		printf("pthread_join(MotorThreadTask) : Erreur\n");
		return err;
	}

	sem_destroy(&MotorTimerSem);

	return err;
}


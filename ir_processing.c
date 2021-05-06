#include "ch.h"
#include "hal.h"
#include <main.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <sensors/proximity.h>
#include <ir_processing.h>

#include "leds.h"
#include <audio_processing.h>
#include <ir_processing.h>

#include <camera.h>

#define ACTIVATE_LED 1
#define NUM_SENSORS 8
#define SEUIL_IR_STUCK 100
#define IR_STOP 40



///   a jouter
//    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//    proximity_msg_t prox_values;



void toggle_led_stuck(void){
	set_led(LED1,ACTIVATE_LED);
	set_led(LED3,ACTIVATE_LED);
	set_led(LED5,ACTIVATE_LED);
	set_led(LED7,ACTIVATE_LED);

	for(int i=0 ; i<10000;i++)
	{
		asm("nop");
	}
	clear_leds();
}

void epuck_move(int direction){
	/*
	 * Moves Epuck in the direction dictated by the sound
	 * if no obstacle detected by IR sensor
	 */
	clear_leds();
	if(direction == FORWARDS){
		left_motor_set_speed(600);
		right_motor_set_speed(600);
		set_led(LED1,ACTIVATE_LED);
		}

	//turn left
		else if(direction == LEFT){
			left_motor_set_speed(-600);
			right_motor_set_speed(600);
			set_led(LED7,ACTIVATE_LED);
		}

		//turn right
		else if(direction == RIGHT){
			left_motor_set_speed(600);
			right_motor_set_speed(-600);
			set_led(LED3,ACTIVATE_LED);
		}

	//go backward
		else if( direction == BACK){
			left_motor_set_speed(-600);
			right_motor_set_speed(-600);
			set_led(LED5,ACTIVATE_LED);
		}

		//help I'm stuck!
		else if(direction == STUCK) {
			clear_leds();
//			for(int i=0 ; i<100;i++)		//boucle d'attente pour se debloquer
//				{
				toggle_led_stuck();
//				}
				left_motor_set_speed(0);
				right_motor_set_speed(0);

			for(int i=0 ; i<100;i++)		//boucle d'attente pour se debloquer
			{
				toggle_led_stuck();
			}
		}
	//no direction --> stop
		else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			set_led(LED1,ACTIVATE_LED);
			set_led(LED3,ACTIVATE_LED);
			set_led(LED5,ACTIVATE_LED);
			set_led(LED7,ACTIVATE_LED);
		}

}

void ok_to_move(int direction){
	/*
	 * Checks if obstacle in direction we want to move
	 */
	int prox_values [NUM_SENSORS];

	for(int i=0; i<NUM_SENSORS; i++){
		prox_values[i]= get_calibrated_prox(i);
	}
	switch (direction){
		case FORWARDS:
			if(prox_values[0]>SEUIL_IR_STUCK || prox_values[1]>SEUIL_IR_STUCK || prox_values[6]>SEUIL_IR_STUCK ||prox_values[7]>SEUIL_IR_STUCK ){
				epuck_move (STUCK);
			}
			else{
				epuck_move (FORWARDS);
			}
			break;

		case LEFT:
			if(prox_values[5]>SEUIL_IR_STUCK|| prox_values[4]>SEUIL_IR_STUCK || prox_values[6]>SEUIL_IR_STUCK){
				epuck_move (STUCK);
			}
			else{
				epuck_move (LEFT);
			}
			break;

		case RIGHT:
			if(prox_values[2]>SEUIL_IR_STUCK || prox_values[1]>SEUIL_IR_STUCK ||prox_values[3]>SEUIL_IR_STUCK ){
				epuck_move (STUCK);
			}
			else{
				epuck_move (RIGHT);
			}
			break;

		case BACK:
			if(prox_values[3]>SEUIL_IR_STUCK || prox_values[4]>SEUIL_IR_STUCK){
				epuck_move (STUCK);
			}
			else{
				epuck_move (BACK);
			}
			break;

		case STOP:
			epuck_move(STOP);
			break;
}

}


void e_puck_follow(void){

	int prox_values [NUM_SENSORS];
	int speed_right =0;
	int speed_left=0;
	int max_value=5; 		//on limite le bruit
	int max_sensor=-1;

	for(int i=0; i<NUM_SENSORS; i++){
		prox_values[i]= get_calibrated_prox(i);

		if (prox_values[i] > max_value){
			max_value=prox_values[i];
			max_sensor=i;
		}
	}

	chprintf((BaseSequentialStream *)&SDU1, "max value =%d, index=%d \r\n\n", max_value, max_sensor);

	clear_leds();

	switch(max_sensor){
	case -1 :				//tous les sensors sont à zéros
		speed_right=0;
		speed_left=0;
		break;
	case 0:					//IR0 est la valeur max
		if(prox_values[0] < IR_STOP){	//mais il est aussi > 5 -->trop loin
			speed_right=300;
			speed_left=300;
		}
		else if(prox_values[0] > (IR_STOP +30)){
			speed_right=-300;
			speed_left=-300;
		}
		else{
			speed_right=0;
			speed_left=0;
		}
		break;

	case 1:						//un peu à droite --> tourne
		speed_right=-300;
		speed_left=300;
		break;

	case 2:
		if(prox_values[2]>20)
		{
			speed_right=-300;
			speed_left=300;
		}
		else{
			speed_right=0;
			speed_left=0;
		}
		break;

	case 5:
		if(prox_values[5]>20){
			speed_right=300;
			speed_left=-300;
		}
		else{
			speed_right=0;
			speed_left=0;
		}
		break;

	case 6:						//un peu à gauche --> tourne
		speed_right=300;
		speed_left=-300;
		break;

	case 7:					//IR7 est la valeur max - mm chose avant gauche
			if(prox_values[7] < IR_STOP){	//mais il est aussi > 5 -->trop loin
				speed_right=300;
				speed_left=300;
			}
			else if(prox_values[7] > (IR_STOP +30)){
				speed_right=-300;
				speed_left=-300;
			}
			else{
				speed_right=0;
				speed_left=0;
			}
			break;

	default:
		speed_right=0;
		speed_left=0;
		break;
	}






	//	set_rgb_led(LED2,10, 10, 0);
//	set_rgb_led(LED4,10,10,0);
//	set_rgb_led(LED6,10,10,0);
//	set_rgb_led(LED8,10,10,0);

//	if (prox_values[0] < IR_STOP && prox_values[0]>5 ){
//		speed_right= 600;
//		speed_left=600;
//		set_led(LED1,ACTIVATE_LED);
//	}
//	else if (prox_values[0] > (IR_STOP+30) )
//	{
//		speed_right=-600;
//		speed_left=-600;
//		set_led(LED5,ACTIVATE_LED);
//	}
//	else{
//		speed_right=0;
//		speed_left=0;
//		set_led(LED1,ACTIVATE_LED);
//		set_led(LED3,ACTIVATE_LED);
//		set_led(LED5,ACTIVATE_LED);
//		set_led(LED7,ACTIVATE_LED);
//	}

	left_motor_set_speed(speed_left);
	right_motor_set_speed(speed_right);

	for(int i=0 ; i<10000;i++)
	{
		asm("nop");
	}

}




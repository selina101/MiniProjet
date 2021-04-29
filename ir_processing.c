#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <sensors/proximity.h>
#include <ir_processing.h>

#include "leds.h"
#include <audio_processing.h>
#include <ir_processing.h>

#define ACTIVATE_LED 1
#define NUM_SENSORS 8

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
			left_motor_set_speed(300);
			right_motor_set_speed(-300);

			for(int i=0 ; i<100;i++)		//boucle d'attente pour se debloquer
				{
					toggle_led_stuck();
				}


		}
	//no direction --> stop
		else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
//			set_led(LED1,ACTIVATE_LED);
//			set_led(LED3,ACTIVATE_LED);
//			set_led(LED5,ACTIVATE_LED);
//			set_led(LED7,ACTIVATE_LED);
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
			if(prox_values[0]>100 || prox_values[1]>100 || prox_values[6]>100 ||prox_values[7]>100 ){
				epuck_move (STUCK);
			}
			else{
				epuck_move (FORWARDS);
			}
			break;

		case LEFT:
			if(prox_values[5]>100|| prox_values[4]>100 || prox_values[6]>100){
				epuck_move (STUCK);
			}
			else{
				epuck_move (LEFT);
			}
			break;

		case RIGHT:
			if(prox_values[2]>100 || prox_values[1]>100 ||prox_values[3]>100 ){
				epuck_move (STUCK);
			}
			else{
				epuck_move (RIGHT);
			}
			break;

		case BACK:
			if(prox_values[3]>100 || prox_values[4]>100){
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

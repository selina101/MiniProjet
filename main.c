#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>

#include "camera/dcmi_camera.h"
#include <camera.h>
#include "leds.h"

#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

messagebus_t bus;

MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){									//????????????? permet de mesurer tps execution
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

void delay(int n)
{
	int new_n = 4*n;
	while(new_n--)
	{
		asm("nop");
	}
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();
    //init ir
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(); // ce qui lance le thread du ir
    calibrate_ir();

    //starts the microphones processing thread. --> thread is defined in e-puck library
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);

    //start camera
	dcmi_start();
	po8030_start();

	int valeur_int=0;

    /* Infinite loop. */
    while (1) {
    	clear_leds();
    	valeur_int = detect_color();

    	if (valeur_int == RED){
    		set_led(LED1,1);
    		set_led(LED3,1);
    	}
    	else if (valeur_int == GREEN){
    	    set_led(LED5,1);
    	    set_led(LED7,1);
    	 }
    	else {
        	set_led(LED1,1);
    		set_led(LED3,1);
    		set_led(LED5,1);
    		set_led(LED7,1);
    	}


//
//    	chprintf((BaseSequentialStream *)&SDU1, "%R=%d \r\n", valeur_int);
//    	//SDU1 on met port 8 pour le port du stm32fxxx et ensuite cliquer sur open et non pas change
//    	delay(1000000);
//    	//les valeurs commencent à 8cm et à 3 cm on est à 100 à ~2cm on est vers 500?

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

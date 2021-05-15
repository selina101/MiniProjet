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

#include "leds.h"
#include "spi_comm.h"

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


int main(void)
{

    halInit();
    chSysInit();
	spi_comm_start();

    //starts the serial communication
    serial_start();

    //starts the USB communication
    usb_start();

    //inits the motors
    motors_init();

    //init ir
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
    calibrate_ir();


    //starts the microphones processing thread. --> thread is defined in e-puck library
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);


    /* Infinite loop. */
    while (1) {



    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

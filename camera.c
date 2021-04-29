#include "camera/po8030.h"
#include "camera/dcmi_camera.h"
#include <camera.h>


#include <usbcfg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include <chprintf.h>


#define IMAGE_BUFFER_SIZE		640
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

int detect_color(void){

	int color=0;
	uint8_t *img_buff_ptr;
	uint16_t r = 0, g = 0, b = 0;

// Init camera - just 2 lines
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();


// Read camera.
	dcmi_capture_start();
	wait_image_ready();
	img_buff_ptr = dcmi_get_last_image_ptr();


	r = (int)img_buff_ptr[0]&0xF8;
	g = (int)(img_buff_ptr[0]&0x07)<<5 | (img_buff_ptr[1]&0xE0)>>3;
	b = (int)(img_buff_ptr[1]&0x1F)<<3;

	if ( r>g){
		color=RED;
	}
	else if(g >= r){
		color=GREEN;
	}

	return color;

}

/*
 * usb_functions.c
 *
 *  Created on: Dec 10, 2023
 *      Author: mateuszwrobel
 */
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern char usb_out_buffer[];
extern char usb_in_buffer[]; //32 signs + CR, LF, /0
extern volatile uint8_t usb_receive_flag;

void usb_send(char *string){
	uint8_t len = strlen(string);
    CDC_Transmit_FS((uint8_t*) string, len);
}

void usb_buf_clear (void){
	for (int i = 0; i < USB_BUF_SIZE; i++){
		usb_out_buffer [i] = '\0';
	}
}

void tr_cursor_hide( uint8_t hide ) {
	if(hide) {
		sprintf(usb_out_buffer, "\x1b[25l");
		usb_send(usb_out_buffer);
	} else {
		sprintf(usb_out_buffer, "\x1b[25h");
		usb_send(usb_out_buffer);
	}
}


void tr_cls() {
	sprintf(usb_out_buffer, "\x1b[2J\x1b[H");
	usb_send(usb_out_buffer);
}


void fill_line( char ascii, uint8_t cnt ) {
	sprintf(usb_out_buffer, "%c", ascii);
	for(uint8_t i=0; i<cnt; i++) usb_send( usb_out_buffer );
}


void tr_attr( uint8_t atr, uint8_t fg, uint8_t bg ) {
	sprintf(usb_out_buffer, "\x01b[3%d;3%d;4%dm", atr, fg, bg);
	usb_send(usb_out_buffer);
}


void tr_pen_color( uint8_t cl ) {
	sprintf(usb_out_buffer, "\x01b[3%dm", (cl));
	usb_send(usb_out_buffer);
}

void tr_brush_color( uint8_t cl ) {
	sprintf(usb_out_buffer, "\x01b[4%dm", (cl));
	usb_send(usb_out_buffer);
}


void tr_locate( uint8_t y, uint8_t x ) {
	sprintf(usb_out_buffer, "\x01b[%d;%dH", y, x);
	usb_send(usb_out_buffer);
}






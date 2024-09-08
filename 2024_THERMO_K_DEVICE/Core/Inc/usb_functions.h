/*
 * usb_functions.h
 *
 *  Created on: Dec 10, 2023
 *      Author: mateuszwrobel
 */

#ifndef INC_USB_FUNCTIONS_H_
#define INC_USB_FUNCTIONS_H_

void usb_send(char *string);
void usb_buf_clear (void);

// atrybuty znaku
#define RESET		0
#define BOLD 		1
#define DIM			2
#define UNDERLINE 	3
#define BLINK		4
#define REVERSE		7
#define HIDDEN		8

// kolory czcionki lub t³a
#define USB_BLACK 		0
#define USB_RED			1
#define USB_GREEN		2
#define USB_YELLOW		3
#define USB_BLUE		4
#define USB_MAGENTA		5
#define USB_CYAN		6
#define	USB_WHITE		7


void tr_cls();											// clear screen
void tr_cursor_hide( uint8_t hide );					// cursor show = 0 / hide = 1
void tr_locate( uint8_t y, uint8_t x );					// set row and column
void tr_pen_color( uint8_t cl );						// set font color
void tr_brush_color( uint8_t cl );						// set background color
void tr_attr( uint8_t atr, uint8_t fg, uint8_t bg );	// set atribute: sign, font and background color

void fill_line( char ascii, uint8_t cnt );


#endif /* INC_USB_FUNCTIONS_H_ */

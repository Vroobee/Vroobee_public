/*
 * at_cmd.c
 *
 *  Created on: 16 wrz 2022
 *      Author: mateuszwrobel
 */

#include "stm32f1xx_hal.h"
#include "at_cmd.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_functions.h"
#include "eeprom.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define pgm_read_word(addr) (*(const unsigned long *)(addr))
#define AT_CNT 	6

const TATCMD command_at[AT_CNT] = {
	// { at_cmd } , { pointer for AT cmd service function },

		{"AT", 		at_service},
		{"ATI", 	ati_service},
		{"AT+SMP", 	ats_service},
		{"START", 	atstart_service},
		{"STOP", 	atstop_service},
		{"AT+RST", 	atrst_service},
};

extern char usb_out_buffer[128], usb_in_buffer[35];
extern uint32_t sampling;
extern uint8_t measurement_start, default_settings;

extern uint16_t VirtAddVarTab[NB_OF_VAR];
extern uint16_t VarDataTab[NB_OF_VAR];
extern uint16_t VarDataTabRead[NB_OF_VAR];
extern uint16_t VarIndex, VarDataTmp;

void parse_cdc_data( char * pBuf ) {

	int8_t (*_at_srv)(uint8_t inout, char * data);

	char * cmd_wsk;
	char * reszta;
	uint8_t i=0, len;

	if ( strpbrk(pBuf, "=?"))	{
		// AT cmd service in/out + params

		if ( strpbrk(pBuf, "?"))	{
			// service AT+CMD?

			cmd_wsk = strtok_r(pBuf, "?", &reszta);
			len = strlen(cmd_wsk);
			for(i=0;i<AT_CNT;i++) {
				if ( len && 0 == strncasecmp(cmd_wsk, command_at[i].command_at, len) ) {
					if( pgm_read_word(command_at[i].command_at) ) {
						_at_srv = (void *) pgm_read_word(&command_at[i].at_service) ;
						if( _at_srv) {
							if( _at_srv( 0, reszta ) < 0 ) usb_send("ERROR\r\n");
						}
					}
					usb_send("\r\n");
					break;
				}
			}

		} else {
			// service AT+CMD = params

			cmd_wsk = strtok_r(pBuf, "=", &reszta);
			len = strlen(cmd_wsk);
			for(i=0;i<AT_CNT;i++) {
				if ( len && 0 == strncasecmp(cmd_wsk, command_at[i].command_at, len) ) {
					if( pgm_read_word(command_at[i].command_at) ) {
						_at_srv = (void *) pgm_read_word(&command_at[i].at_service);
						if( _at_srv && ! _at_srv( 1, reszta ) ) usb_send("OK\r\n");
						else usb_send("ERROR\r\n");
					}
					break;
				}
			}
		}

	} else {
		// no params AT service

		if( 0 == pBuf[0] ) usb_send("\r\n");	// CR / CRLF action
		else {
			for(i=0;i<AT_CNT;i++) {
				if ( 0 == strncasecmp(pBuf, command_at[i].command_at, strlen(pBuf)) ) {
					if( pgm_read_word(command_at[i].command_at) ) {
						_at_srv = (void *)pgm_read_word(&command_at[i].at_service);
						if( _at_srv) _at_srv(2,0);
					}
					break;
				}
			}
		}
	}

	if( AT_CNT == i ){
		usb_send("ERROR\r\n");
	}

}


//----------------- AT service functions ----------------------------------
int8_t at_service(uint8_t inout, char * params) {
	cdc_send_str("OK\r\n");
	return 0;
}

int8_t ati_service(uint8_t inout, char * params) {
	cdc_send_str("Software v1.0\r\n");
	HAL_Delay(1);
	return 0;
}

int8_t ats_service(uint8_t inout, char * params) {
	char *wsk;

	if (0 == inout){
		sprintf(usb_out_buffer, "AT+SMP = %ld\r\n", sampling);
		usb_send(usb_out_buffer);
	} else if (1 == inout){
		if(!strlen(params)) return -1;
		wsk = strtok(params, "\r");
		uint8_t value = atoi(wsk);
		if (value >= 1 && value <= 600){
			sampling = value;
			sprintf(usb_out_buffer, "DATA SAVED! AT+SMP = %ld [s]\n\r", sampling);
			usb_send(usb_out_buffer);

			VarDataTab[0] = 55;
			VarDataTab[1] = sampling;
			VarDataTab[2] = default_settings;

			for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++){
				if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[VarIndex])) != HAL_OK){
			  		Error_Handler();
			  	}
			}
		} else {
			sprintf(usb_out_buffer, "Incorrect value (1 to 600 seconds)!\n\r");
			usb_send(usb_out_buffer);
			return -1;
		}
	} else if (2 == inout){
		sprintf(usb_out_buffer, "AT+SMP = (1 to 600) -> set sampling time [s]\n\r");
		usb_send(usb_out_buffer);
	} else {
		if( !inout ) return -1;
	}

	return 0;
}

int8_t atstart_service(uint8_t inout, char * params) {
	measurement_start |= (1<<0);
	HAL_Delay(1);
	return 0;
}

int8_t atstop_service(uint8_t inout, char * params) {
	measurement_start &=~ (1<<0);
	HAL_Delay(1);
	return 0;
}

int8_t atrst_service(uint8_t inout, char * params) {
	if (2 == inout){
		sprintf(usb_out_buffer, "FACTORY RESET IN 3 SECONDS\n\r");
		usb_send(usb_out_buffer);
		HAL_Delay(1000);
		sprintf(usb_out_buffer, "FACTORY RESET IN 2 SECONDS\n\r");
		usb_send(usb_out_buffer);
		HAL_Delay(1000);
		sprintf(usb_out_buffer, "FACTORY RESET IN 1 SECOND\n\r");
		usb_send(usb_out_buffer);
		HAL_Delay(900);

		default_settings = 0;
		VarDataTab[0] = 55;
		VarDataTab[1] = sampling;
		VarDataTab[2] = default_settings;

		for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++){
			if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[VarIndex])) != HAL_OK){
		  		Error_Handler();
		  	}
		}
		HAL_Delay(100);
		NVIC_SystemReset();
	} else {
		if( !inout ) return -1;
	}
	return 0;
}






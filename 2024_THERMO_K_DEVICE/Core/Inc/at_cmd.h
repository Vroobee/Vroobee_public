/*
 * at_cmd.h
 *
 *  Created on: 16 wrz 2022
 *      Author: mateuszwrobel
 */

#ifndef INC_AT_CMD_H_
#define INC_AT_CMD_H_

#include <stdint.h>

typedef struct {
	char command_at[8];
	int8_t (* at_service)(uint8_t inout, char * params);
} const TATCMD;

extern TATCMD command_at[];

void parse_cdc_data( char * pBuf);

int8_t at_service(uint8_t inout, char * params);
int8_t ati_service(uint8_t inout, char * params);
int8_t ats_service(uint8_t inout, char * params);
int8_t atstart_service(uint8_t inout, char * params);
int8_t atstop_service(uint8_t inout, char * params);
int8_t atrst_service(uint8_t inout, char * params);



#endif /* INC_AT_CMD_H_ */

/*
 * sms.c
 *
 *  Created on: Jan 16, 2021
 *      Author: tom_h
 */
#include "main.h"
#include "sms.h"
#include "msg.h"
#include <stdio.h>
#include <string.h>
#include "globals.h"

struct at_resp_struct at_resp[12] =
{
		{"UNDEF"        , 9, 9, {9, 9, 9, 9}},
		{"AT"           , 0, 1, {9, 9, 9, 9}},
		{"ATI"          , 0, 4, {1, 2, 3, 9}},
		{"AT+CPIN=1234" , 9, 1, {2, 9, 9, 9}},
		{"AT+CMGF=1"    , 0, 4, {1, 2, 3, 9}},
		{"AT+CSCS=GSM"  , 0, 4, {1, 2, 3, 9}},
		{"AT+CMGS="     , 9, 9, {0, 9, 9, 9}},
		{"AT+xxx"    , 0, 4, {1, 2, 3, 9}},
		{"ATI"      , 0, 4, {1, 2, 3, 9}},
		{"ATI"      , 0, 4, {1, 2, 3, 9}},
		{"ATI"      , 0, 4, {1, 2, 3, 9}},
		{"ATI"      , 0, 4, {1, 2, 3, 9}}

};

void sms_get_at_cmnd(at_cmnd_type cmd_idx, char *cmd){
	strcpy(cmd, at_resp[cmd_idx].command );
	strcat(cmd, &newline_str);
}
void sms_debug(at_cmnd_type at_cmnd_idx, struct msg_handle_struct *pmsg){
	uint8_t i;
	uint8_t content_idx;

    printf("Echo      :%s%s", pmsg->row_ptr[at_resp[at_cmnd_idx].echo_idx],&newline_str);
    printf("Ok        :%s%s", pmsg->row_ptr[at_resp[at_cmnd_idx].ok_idx],&newline_str);
    for (i= 0; i<MAX_CONTENT_ROWS;i++){
    	content_idx = at_resp[at_cmnd_idx].content[i];
    	if (content_idx < MAX_TOTAL_ROWS) {
            printf("Content %u :%s%s",i, pmsg->row_ptr[content_idx],&newline_str);
    	}
    }
};

Sms_StatusType sms_check(at_cmnd_type at_cmnd_idx, struct msg_handle_struct *pmsg){
	uint8_t i;
	uint8_t content_idx;
	Sms_StatusType result;

	result = SMS_OK;
    if (strcmp(pmsg->row_ptr[at_resp[at_cmnd_idx].echo_idx],at_resp[at_cmnd_idx].command) != 0){
    	result = SMS_ERROR_ECHO;
    } else
    {
        if (strlen(pmsg->row_ptr[at_resp[at_cmnd_idx].ok_idx]) == 0)
        {
        	result = SMS_ERROR_OK;
        } else
        {
            for (i= 0; i<MAX_CONTENT_ROWS;i++){
            	content_idx = at_resp[at_cmnd_idx].content[i];
            	if (content_idx < MAX_TOTAL_ROWS) {
                    if(strlen(pmsg->row_ptr[content_idx]) == 0)
                  	{
                    	result = SMS_ERROR_CONTENT;
                    }
            	}
            }
        }
    }
    return result;
};


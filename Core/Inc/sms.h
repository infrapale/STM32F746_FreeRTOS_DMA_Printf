/*
 * sms.h
 *
 *  Created on: Jan 11, 2021
 *      Author: tom_h
 */

#ifndef INC_SMS_H_
#define INC_SMS_H_
#include "msg.h"

#define SMS_EOL  '\r'
#define MAX_AT_CMND_LEN 16
#define MAX_CONTENT_ROWS 4
#define MAX_TOTAL_ROWS MAX_CONTENT_ROWS + 2

typedef enum
{
  SMS_UNDEF    = 0x00U,
  SMS_OK       ,
  SMS_ERROR    ,
  SMS_ERROR_ECHO,
  SMS_ERROR_OK,
  SMS_ERROR_CONTENT
} Sms_StatusType;

typedef enum  {
	AT_UNDEFINED = 0,
	AT_AT,
	AT_CHIP_INFO,
	AT_SET_PIN,
	AT_SMS_SET_TEXT_MODE,
	AT_SMS_SET_GSM_CHAR,
	AT_SMS_SEND,
	AT_x,
	AT_
} at_cmnd_type;



struct at_resp_struct{
	char      command[MAX_AT_CMND_LEN];
	uint16_t  echo_idx;
	uint16_t  ok_idx;
	uint16_t  content[MAX_CONTENT_ROWS];
};

void sms_get_at_cmnd(at_cmnd_type cmd_idx, char *cmd);
void sms_debug(at_cmnd_type at_cmnd_idx, struct msg_handle_struct *pmsg);
Sms_StatusType sms_check(at_cmnd_type at_cmnd_idx, struct msg_handle_struct *pmsg);


#endif /* INC_SMS_H_ */

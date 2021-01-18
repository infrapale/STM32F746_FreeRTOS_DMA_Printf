/*
 * msg.h
 *
 *  Created on: Jan 7, 2021
 *      Author: tom_h
 */

#ifndef INC_MSG_H_
#define INC_MSG_H_

#define MAX_MSG_LEN 128
#define MAX_MSG_ROW 16
#define MAX_MSG_SUB_ROW 8

typedef enum
{
  MSG_UNDEF    = 0x00U,
  MSG_OK       = 0x001,
  MSG_ERROR    = 0x02U,
  MSG_FULL     = 0x03U,
} Msg_StatusTypeDef;


typedef enum  {
	msg_type_undefined = 0,
	msg_type_gsm_rx,
	msg_type_gsm_tx,
	msg_type_received_sms,
	msg_type_send_sms,
	msg_type_console_rx,
	msg_type_console_tx,
	msg_type_radio_rx,
	msg_type_radio_tx
} msg_type;

enum msg_reserve_status {
	msg_undef  = 0,
	msg_ok     = 1,
	msg_error  = 2,
	msg_full   = 3
};





struct  msg_handle_struct {
	uint8_t index;
	uint8_t reserved;
	msg_type type;
	uint32_t reserved_at;
	uint16_t bytes;
	uint8_t  buf[MAX_MSG_LEN];
	char *row_ptr[MAX_MSG_SUB_ROW];
};

typedef struct {
    enum msg_reserve_status status;
    uint8_t row_indx;
} msg_status;


void msg_initialize(void);
uint8_t msg_free_rows(void);
Msg_StatusTypeDef msg_reserve(struct msg_handle_struct **msg_handle, msg_type m_type);
Msg_StatusTypeDef msg_release(struct msg_handle_struct *msg_handle);
Msg_StatusTypeDef msg_get_handle(struct msg_handle_struct **msg_handle, uint8_t row_indx);
uint16_t msg_row_split(struct msg_handle_struct *msg_handle);
#endif /* INC_MSG_H_ */



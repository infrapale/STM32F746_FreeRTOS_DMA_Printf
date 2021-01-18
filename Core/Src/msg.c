/*
 * msg.c
 *
 *  Created on: Jan 7, 2021
 *      Author: tom_h
 */

#include <main.h>
#include <msg.h>
#include <stdio.h>
#include <string.h>
#include "globals.h"


static struct msg_handle_struct msg_repo[MAX_MSG_ROW];
//static const char newline_str[] = "\r\n";

/**
 * @brief Initialize message repository
 * @note
 * @param - :
 * @retval None
 */
void msg_initialize(void){
	memset(msg_repo, 0x00, sizeof(msg_repo));
	for  (uint8_t i=0;i <MAX_MSG_ROW ;i++){
		msg_repo[i].index = i;

	}
}

/**
 * @brief  Get number of free rows in repository
 * @note
 * @param  :
 * @retval #free rows, 0 = repository is full, >0 room for a new message
 */
uint8_t msg_free_rows(void){
	uint8_t cnt = 0;
	for(uint8_t i=0; i < MAX_MSG_ROW; i++ ){
		if (msg_repo[i].reserved == 0){
			cnt++;
		}
	}
	return cnt;
}

/**
 * @brief
 * @note
 * @param  :
 * @retval None
 */
Msg_StatusTypeDef msg_reserve(struct msg_handle_struct **msg_handle, msg_type m_type)
{
    uint8_t indx = 0;
    Msg_StatusTypeDef resp;
    struct msg_handle_struct *msg_ptr;
    resp = MSG_UNDEF;

    while ((indx < MAX_MSG_ROW) && (resp == MSG_UNDEF)){
    	if (msg_repo[indx].reserved == 0){
    		resp = MSG_OK;
    		msg_ptr =  &msg_repo[indx];
    		msg_ptr->reserved = 1;
    		msg_ptr->index = indx;
    		msg_ptr->type = m_type;
    		msg_ptr->reserved_at = HAL_GetTick();
    		*msg_handle = msg_ptr;
    	} else {
    		if (++indx >= MAX_MSG_ROW){
    			resp = MSG_FULL;
    		}
    	}
    }
    return resp;
}


Msg_StatusTypeDef msg_release(struct msg_handle_struct *msg_handle)
{
    memset(msg_handle,0x00,sizeof(struct msg_handle_struct));
    return MSG_OK;
}



Msg_StatusTypeDef msg_get_handle(struct msg_handle_struct **msg_handle, uint8_t row_indx)
{
	if (row_indx < MAX_MSG_ROW){
	    *msg_handle = &msg_repo[row_indx];
	    return MSG_OK;
	} else
	{
	    *msg_handle = NULL;
	    return MSG_ERROR;
	}
}

uint16_t msg_row_split(struct msg_handle_struct *msg_handle)
{
	uint8_t row_cntr = 0;
	char    *c_ptr;
	uint16_t c_cntr;
	memset(msg_handle->row_ptr,0x00,sizeof(msg_handle->row_ptr));
	c_ptr  = (char *)msg_handle->buf;
	c_cntr = 0;

	msg_handle->row_ptr[row_cntr] = c_ptr;

	while(*c_ptr){
		switch(*c_ptr){
		case '\r':
		case '\n':
			if(c_cntr > 0 ){
				row_cntr++;
				c_cntr = 0;
			}
			*c_ptr = 0x00;
			break;
		case 0x00:
			break;
		default:
			if (c_cntr == 0 ){
				msg_handle->row_ptr[row_cntr] = c_ptr;
			}
			c_cntr++;
			break;
		}
		c_ptr++;
	}
	return row_cntr;

}


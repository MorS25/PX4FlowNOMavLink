/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "usbd_cdc_vcp.h"
#include "settings.h"
#include "usart.h"
#include "mt9v034.h"
#include "dcmi.h"
#include "gyro.h"
#include "debug.h"
#include "communication.h"

extern uint32_t get_boot_time_ms();
extern void buffer_reset();

static uint32_t m_parameter_i = 0;

/**
 * @brief Initialize MAVLINK system
 */
void communication_init(void)
{
	//mavlink_system.sysid = global_data.param[PARAM_SYSTEM_ID]; // System ID, 1-255
	//mavlink_system.compid = global_data.param[PARAM_COMPONENT_ID]; // Component/Subsystem ID, 1-255
}

/**
 * @brief Send System State
 */
void communication_system_state_send(void)
{
	/* send heartbeat to announce presence of this system */
	//mavlink_msg_heartbeat_send(MAVLINK_COMM_0, global_data.param[PARAM_SYSTEM_TYPE], global_data.param[PARAM_AUTOPILOT_TYPE], 0, 0, 0);
	//mavlink_msg_heartbeat_send(MAVLINK_COMM_2, global_data.param[PARAM_SYSTEM_TYPE], global_data.param[PARAM_AUTOPILOT_TYPE], 0, 0, 0);
}

/**
 * @brief Send one low-priority parameter message
 */
void communication_parameter_send(void)
{
	/* send parameters one by one */
	if (m_parameter_i < ONBOARD_PARAM_COUNT)
	{
		//mavlink_msg_param_value_send(MAVLINK_COMM_0,
		//		global_data.param_name[m_parameter_i],
		//		global_data.param[m_parameter_i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, m_parameter_i);
		//mavlink_msg_param_value_send(MAVLINK_COMM_2,
		//		global_data.param_name[m_parameter_i],
		//		global_data.param[m_parameter_i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, m_parameter_i);
		m_parameter_i++;
	}
}


/**
 * @brief Receive from usart3
 */
void communication_receive(void)
{
	//mavlink_message_t msg;
	//mavlink_status_t status = { 0 };

	while (usart3_char_available())
	{
		uint8_t c = usart3_rx_ringbuffer_pop();

		/* Try to get a new message */
		//if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
		//{
			/* Handle message */
		//	handle_mavlink_message(MAVLINK_COMM_0, &msg);
		//}
		/* And get the next one */
	}
}

/**
 * @brief Receive from usart2
 */
void communication_receive_forward(void)
{
	//mavlink_message_t msg;
	//mavlink_status_t status = { 0 };

	while (usart2_char_available())
	{
		uint8_t c = usart2_rx_ringbuffer_pop();

		/* Try to get a new message */
		//if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status))
		//{
		//	/* Handle message */
		//	handle_mavlink_message(MAVLINK_COMM_1, &msg);
		//}
		/* And get the next one */
	}
}

/**
 * @brief Receive from usb
 */
void communication_receive_usb(void)
{
	//mavlink_message_t msg;
	//mavlink_status_t status = { 0 };
	//uint8_t character;

	//while (VCP_get_char(&character))
	//{
	//	/* Try to get a new message */
	//	if (mavlink_parse_char(MAVLINK_COMM_2, character, &msg, &status))
	//	{
	//		/* Handle message */
	//		handle_mavlink_message(MAVLINK_COMM_2, &msg);
	//	}
	//	/* And get the next one */
	//}
}


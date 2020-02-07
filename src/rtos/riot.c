/***************************************************************************
 *   Copyright (C) 2015 by Daniel Krebs                                    *
 *   Daniel Krebs - github@daniel-krebs.net                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "target/armv7m.h"
#include "rtos_riot_stackings.h"

static bool riot_detect_rtos(struct target *target);
static int riot_create(struct target *target);
static int riot_update_threads(struct rtos *rtos);
static int riot_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,struct rtos_reg **reg_list, int *num_regs);
static int riot_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

#if 0
struct riot_thread_state {
	int value;
	const char *desc;
};

// ref.: <RIOT>/core/include/sched.h
static const struct riot_thread_state riot_thread_states[] = {
	{  0,"STOPPED"},                /**< has terminated                           */
	{  1,"ZOMBIE"},                 /**< has terminated & keeps thread's thread_t */
	{  2,"SLEEPING"},               /**< sleeping                                 */
	{  3,"MUTEX_BLOCKED"},          /**< waiting for a locked mutex               */
	{  4,"RECEIVE_BLOCKED"},        /**< waiting for a message                    */
	{  5,"SEND_BLOCKED"},           /**< waiting for message to be delivered      */
	{  6,"REPLY_BLOCKED"},          /**< waiting for a message response           */
	{  7,"FLAG_BLOCKED_ANY"},       /**< waiting for any flag from flag_mask      */
	{  8,"FLAG_BLOCKED_ALL"},       /**< waiting for all flags in flag_mask       */
	{  9,"MBOX_BLOCKED"},           /**< waiting for get/put on mbox              */
	{ 10,"COND_BLOCKED"},           /**< waiting for a condition variable         */
	{ 11,"RUNNING"},                /**< currently running                        */
	{ 12,"PENDING"}                 /**< waiting to be scheduled to run           */
};
#define RIOT_NUM_STATES (sizeof(riot_thread_states)/sizeof(struct riot_thread_state))

static const char * riot_state2str(uint8_t status){
        unsigned int k;
		for (k = 0; k < RIOT_NUM_STATES; k++) {
			if (riot_thread_states[k].value == status)
				break;
		}
		if (k == RIOT_NUM_STATES) 
			return "unknown state" ;
		else
			riot_thread_states[k].desc;
}
#else
// ref.: <RIOT>/core/include/sched.h
typedef enum {
    STATUS_STOPPED=0,                 /**< has terminated                           */
    STATUS_ZOMBIE,                  /**< has terminated & keeps thread's thread_t */
    STATUS_SLEEPING,                /**< sleeping                                 */
    STATUS_MUTEX_BLOCKED,           /**< waiting for a locked mutex               */
    STATUS_RECEIVE_BLOCKED,         /**< waiting for a message                    */
    STATUS_SEND_BLOCKED,            /**< waiting for message to be delivered      */
    STATUS_REPLY_BLOCKED,           /**< waiting for a message response           */
    STATUS_FLAG_BLOCKED_ANY,        /**< waiting for any flag from flag_mask      */
    STATUS_FLAG_BLOCKED_ALL,        /**< waiting for all flags in flag_mask       */
    STATUS_MBOX_BLOCKED,            /**< waiting for get/put on mbox              */
    STATUS_COND_BLOCKED,            /**< waiting for a condition variable         */
    STATUS_RUNNING,                 /**< currently running                        */
    STATUS_PENDING,                 /**< waiting to be scheduled to run           */
    STATUS_NUMOF                    /**< number of supported thread states        */
} thread_status_t;

// ref.: <RIOT>/sys/ps/ps.c
static const char * state_names[] = {
    [STATUS_RUNNING] = "running",
    [STATUS_PENDING] = "pending",
    [STATUS_STOPPED] = "stopped",
    [STATUS_SLEEPING] = "sleeping",
    [STATUS_ZOMBIE] = "zombie",
    [STATUS_MUTEX_BLOCKED] = "bl mutex",
    [STATUS_RECEIVE_BLOCKED] = "bl rx",
    [STATUS_SEND_BLOCKED] = "bl send",
    [STATUS_REPLY_BLOCKED] = "bl reply",
    [STATUS_FLAG_BLOCKED_ANY] = "bl anyfl",
    [STATUS_FLAG_BLOCKED_ALL] = "bl allfl",
    [STATUS_MBOX_BLOCKED] = "bl mbox",
    [STATUS_COND_BLOCKED] = "bl cond",
};

static const char * riot_state2str(uint8_t status){
	static const char * unknown="unknown";
	if( status < STATUS_NUMOF )
		return state_names[status];
	else 
		return unknown;
}

#endif


enum riot_architecture {
	ARMv6M = 0,
	ARMv7M = 1,
};

struct riot_params {
	unsigned char thread_sp_offset;
	unsigned char thread_status_offset;
	const struct rtos_register_stacking *stacking_info;
};

static const struct riot_params riot_params_list[] = {
	{	/* ARMv6M */
		0x00,							/* thread_sp_offset */
		0x04,							/* thread_status_offset */
		&rtos_riot_Cortex_M0_stacking,	/* stacking_info */
	},
	{	/* ARMv7M */
		0x00,							/* thread_sp_offset */
		0x04,							/* thread_status_offset */
		&rtos_riot_Cortex_M34_stacking,	/* stacking_info */
	},
};
#define RIOT_NUM_PARAMS ((int)(sizeof(riot_params_list)/sizeof(struct riot_params)))


enum riot_symbol_values {
	RIOT_THREADS_BASE = 0,
	RIOT_NUM_THREADS,
	RIOT_ACTIVE_PID,
	RIOT_MAX_THREADS,
	RIOT_NAME_OFFSET,
};

/* refer core/sched.c */
static const char * const riot_symbol_list[] = {
	"sched_threads",
	"sched_num_threads",
	"sched_active_pid",
	"max_threads",
	"_tcb_name_offset",
	NULL
};

const enum riot_symbol_values riot_optional_symbols[] = {
	RIOT_NAME_OFFSET,
};


const struct rtos_type riot_rtos = {
	.name = "RIOT",

	.detect_rtos = riot_detect_rtos,
	.create = riot_create,
	.update_threads = riot_update_threads,
	.get_thread_reg_list = riot_get_thread_reg_list,
	.get_symbol_list_to_lookup = riot_get_symbol_list_to_lookup,

};

static int riot_update_threads(struct rtos *rtos)
{
	int retval;
	int tasks_found = 0;
	const struct riot_params *param;

	if (rtos == NULL)
		return -1;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (const struct riot_params *) rtos->rtos_specific_params;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for RIOT");
		return -4;
	}

	if (rtos->symbols[RIOT_THREADS_BASE].address == 0) {
		LOG_ERROR("Can't find symbol `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
		return -2;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* Reset values */
	rtos->current_thread = 0;
	rtos->thread_count = 0;

	/* read the current thread id */
	int16_t active_pid = 0;
	retval = target_read_buffer(rtos->target,
								rtos->symbols[RIOT_ACTIVE_PID].address,
								sizeof(active_pid),
								(uint8_t *)&active_pid);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't read symbol `%s`", riot_symbol_list[RIOT_ACTIVE_PID]);
		return retval;
	}
	rtos->current_thread = active_pid;

	/* read the current thread count */
	/* It's `int` in RIOT, but this is Cortex M* only anyway */
	int32_t thread_count = 0;
	retval = target_read_buffer(rtos->target,
								rtos->symbols[RIOT_NUM_THREADS].address,
								sizeof(thread_count),
								(uint8_t *)&thread_count);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't read symbol `%s`", riot_symbol_list[RIOT_NUM_THREADS]);
		return retval;
	}
	rtos->thread_count = thread_count;

	/* read the maximum number of threads */
	uint8_t max_threads = 0;
	retval = target_read_buffer(rtos->target,
								rtos->symbols[RIOT_MAX_THREADS].address,
								sizeof(max_threads),
								(uint8_t *)&max_threads);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't read symbol `%s`", riot_symbol_list[RIOT_MAX_THREADS]);
		return retval;
	}

	/* Base address of thread array */
	uint32_t threads_base = rtos->symbols[RIOT_THREADS_BASE].address;

	/* Try to get the offset of tcb_t::name, if absent RIOT wasn't compiled with
	 * DEVELHELP, so there are no thread names */
	uint8_t name_offset = 0;
	if (rtos->symbols[RIOT_NAME_OFFSET].address != 0) {
		retval = target_read_buffer(rtos->target,
									rtos->symbols[RIOT_NAME_OFFSET].address,
									sizeof(name_offset),
									(uint8_t *)&name_offset);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't read symbol `%s`", riot_symbol_list[RIOT_NAME_OFFSET]);
			return retval;
		}
	}

	/* Allocate memory for thread description */
	rtos->thread_details = malloc(sizeof(struct thread_detail) * thread_count);

	/* Buffer for thread names, maximum to display is 32 */
	char buffer[32];

	for (int i = 0; i < max_threads; i++) {

		/* get pointer to tcb_t */
		uint32_t tcb_pointer = 0;
		retval = target_read_buffer(rtos->target,
									threads_base + (i * 4),
									sizeof(tcb_pointer),
									(uint8_t *)&tcb_pointer);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
			return retval;
		}

		if (tcb_pointer == 0) {
			/* PID unused */
			continue;
		}

		/* Index is PID */
		rtos->thread_details[tasks_found].threadid = i;

		/* read thread state */
		uint8_t status = 0;
		retval = target_read_buffer(rtos->target,
									tcb_pointer + param->thread_status_offset,
									sizeof(status),
									(uint8_t *)&status);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
			return retval;
		}

		/* Search for state */
		const char *state_str = riot_state2str(status);

		/* Copy state string */
		rtos->thread_details[tasks_found].extra_info_str = malloc(strlen(state_str) + 1);
		strcpy(rtos->thread_details[tasks_found].extra_info_str, state_str);

		/* Thread names are only available if compiled with DEVELHELP */
		if (name_offset != 0) {
			uint32_t name_pointer = 0;
			retval = target_read_buffer(rtos->target,
										tcb_pointer + name_offset,
										sizeof(name_pointer),
										(uint8_t *)&name_pointer);
			if (retval != ERROR_OK) {
				LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
				return retval;
			}

			/* read thread name */
			retval = target_read_buffer(rtos->target,
										name_pointer,
										sizeof(buffer),
										(uint8_t *)&buffer);
			if (retval != ERROR_OK) {
				LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
				return retval;
			}

			/* Make sure the string inside the buffer terminates */
			if (buffer[sizeof(buffer) - 1] != 0)
				buffer[sizeof(buffer) - 1] = 0;

			/* Copy thread name */
			rtos->thread_details[tasks_found].thread_name_str = malloc(strlen(buffer) + 1);
			strcpy(rtos->thread_details[tasks_found].thread_name_str, buffer);

		} else {
			const char no_name[] = "Need DEVELHELP";
			rtos->thread_details[tasks_found].thread_name_str = malloc(strlen(no_name) + 1);
			strcpy(rtos->thread_details[tasks_found].thread_name_str, no_name);
		}

		rtos->thread_details[tasks_found].exists = true;
// 		rtos->thread_details[tasks_found].thread_name_str = NULL;
//         rtos->thread_details[tasks_found].extra_info_str = NULL;

		tasks_found++;
	}

	return 0;
}

static int riot_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct riot_params *param;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (const struct riot_params *) rtos->rtos_specific_params;

	/* find the thread with given thread id */
	uint32_t threads_base = rtos->symbols[RIOT_THREADS_BASE].address;
	uint32_t tcb_pointer = 0;
	retval = target_read_buffer(rtos->target,
								threads_base + (thread_id * 4),
								sizeof(tcb_pointer),
								(uint8_t *)&tcb_pointer);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
		return retval;
	}

	/* read stack pointer for that thread */
	uint32_t stackptr = 0;
	retval = target_read_buffer(rtos->target,
								tcb_pointer + param->thread_sp_offset,
								sizeof(stackptr),
								(uint8_t *)&stackptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
		return retval;
	}

	return rtos_generic_stack_read(rtos->target,
								   param->stacking_info,
								   stackptr,
								   reg_list, num_regs);
}

static int riot_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(riot_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(riot_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = riot_symbol_list[i];
		(*symbol_list)[i].optional = false;

		/* Lookup if symbol is optional */
		for (unsigned k = 0; k < sizeof(riot_optional_symbols); k++) {
			if (i == riot_optional_symbols[k]) {
				(*symbol_list)[i].optional = true;
				break;
			}
		}
	}

	return 0;
}

static bool riot_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
	     (target->rtos->symbols[RIOT_THREADS_BASE].address != 0)) {
		/* looks like RIOT */
		return true;
	}
	return false;
}

static int riot_create(struct target *target)
{
	struct armv7m_common *armv7m_target = target_to_armv7m(target);

	if (armv7m_target->arm.is_armv6m) {
		target->rtos->rtos_specific_params = (void *) &riot_params_list[ARMv6M];
	} else if (is_armv7m(armv7m_target)) {
		target->rtos->rtos_specific_params = (void *) &riot_params_list[ARMv7M];
	} else {
		LOG_ERROR("Unsupported target type '%s'", target->type->name);
		return -1;
	}

	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;

	return 0;
}

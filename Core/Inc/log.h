/*
 * log.h
 *
 *  Created on: Mar 24, 2022
 *      Author: evanl
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

int __io_putchar(int ch);

typedef enum{
	log_i2c = 0,
	log_lps22hh = 1,
}Log_Subsystem_t;

void log_init();
void _log(Log_Subsystem_t subsystem, const char* msg, ...);

#endif /* INC_LOG_H_ */

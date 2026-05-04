/**********************************************************************
 * File : QMI8658A_PORTS.h
 * Brief: Small platform binding layer for the QMI8658A driver.
 *
 * The original MC3416 driver keeps allocation and delay primitives behind
 * port macros.  This file follows the same style so the sensor driver stays
 * independent from the RTOS/MCU target.
 **********************************************************************/
#ifndef QMI8658A_PORTS_H
#define QMI8658A_PORTS_H

#include <target_port.h>

#define qmi8658a_malloc(x)    t_malloc(x)
#define qmi8658a_free(x)      t_free(x)
#define qmi8658a_Delay(x)     t_sleep(x)

#endif /* QMI8658A_PORTS_H */

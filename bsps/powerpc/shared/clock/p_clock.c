/*
 *  Clock Tick interrupt conexion code.
 */

/*
 *  COPYRIGHT (c) 1989-1997.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may in
 *  the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 *
 *  Modified to support the MPC750.
 *  Modifications Copyright (c) 1999 Eric Valette valette@crf.canon.fr
 */

#include <bsp.h>
#include <bsp/irq.h>
#include <libcpu/c_clock.h>
#include <libcpu/cpuIdent.h>

int BSP_disconnect_clock_handler(void)
{
  return 1;
}

int BSP_connect_clock_handler(void)
{
  rtems_status_code sc;
  
  if ( ppc_cpu_is_bookE() )
      sc = rtems_interrupt_handler_install(
        BSP_DECREMENTER,
        "Clock",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) clockIsrBookE,
        NULL
      );
  else
      sc = rtems_interrupt_handler_install(
        BSP_DECREMENTER,
        "Clock",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) clockIsr,
        NULL
      );
  return 1;
}

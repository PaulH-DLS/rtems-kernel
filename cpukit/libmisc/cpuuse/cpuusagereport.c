/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSImplCPUUsageReporting
 *
 * @brief This source file contains the definition of
 *   rtems_cpu_usage_report() and rtems_cpu_usage_report_with_plugin().
 */

/*
 *  COPYRIGHT (c) 1989-2010.
 *  On-Line Applications Research Corporation (OAR).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>

#include <rtems/cpuuse.h>
#include <rtems/printer.h>


#include "cpuuseimpl.h"



typedef struct  {
  usage_function_pointer  stats_callback; 
   cpuuse_info  * cpuuse_data;
   void * arg;
} cpu_usage_context;


static bool  print_usage_stats( Thread_Control *the_thread,  void * arg, cpuuse_info  * cpuuse_data ){
  cpu_usage_context *ctx;
  ctx = arg;

  rtems_printf(
    ctx->arg,
    " 0x%08" PRIx32 " | %-38s |"
      "%7" PRIu32 ".%06" PRIu32 " |%4" PRIu32 ".%03" PRIu32 "\n",
    the_thread->Object.id,
    cpuuse_data->name,
    cpuuse_data->seconds, cpuuse_data->nanoseconds,
    cpuuse_data->ival, cpuuse_data->fval
  );
  return false;
}


bool cpu_usage_visitor( Thread_Control *the_thread, void *arg )
{
  cpu_usage_context *ctx;
  ctx = arg;
  cpuuse_info  * cpuuse_data;
  cpuuse_data =   ctx-> cpuuse_data;
  _Thread_Get_name( the_thread, cpuuse_data->name, sizeof( cpuuse_data->name ) );
  cpuuse_data->time_used_after_last_reset = 
  _Thread_Get_CPU_time_used_after_last_reset(the_thread );
  _TOD_Get_uptime( &cpuuse_data->uptime );
  _Timestamp_Subtract( &cpuuse_data->time_used_after_last_reset, 
    &cpuuse_data->uptime, &cpuuse_data->total );
  _Timestamp_Divide( &cpuuse_data->time_used_after_last_reset, 
    &cpuuse_data->total, &cpuuse_data->ival, &cpuuse_data->fval );
  cpuuse_data->seconds = _Timestamp_Get_seconds( 
    &cpuuse_data->time_used_after_last_reset );
  cpuuse_data->nanoseconds = _Timestamp_Get_nanoseconds(
    &cpuuse_data->time_used_after_last_reset ) /  TOD_NANOSECONDS_PER_MICROSECOND;
  ctx-> cpuuse_data = cpuuse_data;
  ctx->stats_callback(the_thread, arg , ctx-> cpuuse_data );
  return false;
}




void rtems_cpu_usage_report_with_callback(  usage_function_pointer callback  , cpuuse_info  * info ,void * arg)
{
  cpu_usage_context  ctx;
  ctx.arg = arg;
  Timestamp_Set_to_zero( &info->total );
  info->time_used_after_last_reset = CPU_usage_Uptime_at_last_reset;
  ctx.cpuuse_data = info;
  ctx.stats_callback = print_usage_stats;
  rtems_task_iterate( cpu_usage_visitor, &ctx );
}

/*
 *  rtems_cpu_usage_report
 */
void rtems_cpu_usage_report_with_plugin(
  const rtems_printer *printer
)
{
  cpu_usage_context  ctx;
  uint32_t           seconds;
  uint32_t           nanoseconds;
  cpuuse_info        info;

  ctx.arg = printer;

  /*
   *  When not using nanosecond CPU usage resolution, we have to count
   *  the number of "ticks" we gave credit for to give the user a rough
   *  guideline as to what each number means proportionally.
   */
  
  _Timestamp_Set_to_zero( &info.total );
  info.time_used_after_last_reset = CPU_usage_Uptime_at_last_reset;

  rtems_printf(
     printer,
     "-------------------------------------------------------------------------------\n"
     "                              CPU USAGE BY THREAD\n"
     "------------+----------------------------------------+---------------+---------\n"
     " ID         | NAME                                   | SECONDS       | PERCENT\n"
     "------------+----------------------------------------+---------------+---------\n"
  );

  rtems_cpu_usage_report_with_callback(  usage_function_pointer callback  , cpuuse_info  * info ,  ctx.arg );

  seconds = _Timestamp_Get_seconds( &info.total );
  nanoseconds = _Timestamp_Get_nanoseconds( &info.total ) /
    TOD_NANOSECONDS_PER_MICROSECOND;
  rtems_printf(
     printer,
     "------------+----------------------------------------+---------------+---------\n"
     " TIME SINCE LAST CPU USAGE RESET IN SECONDS:                    %7" PRIu32 ".%06" PRIu32 "\n"
     "-------------------------------------------------------------------------------\n",
     seconds, nanoseconds
  );
}

void rtems_cpu_usage_report( void )
{
  rtems_printer printer;
  rtems_print_printer_printk( &printer );
  rtems_cpu_usage_report_with_plugin( &printer );
}

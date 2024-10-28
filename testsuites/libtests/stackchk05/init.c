/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @brief  Stack Smash Protection Test File
 */

/*
 *  COPYRIGHT (c) 2024 Mohamed Hassan <muhammad.hamdy.hassan@gmail.com>
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

#include "system.h"

#include <rtems/bspIo.h>

extern const char rtems_test_name[];

const char* long_string = "This string is definitely too long for the buffer";

void invoke_stack_overflows(void)
{
  char buffer_short[20];
  
  strcpy(buffer_short, long_string);
  
  printk("Stack Overflow invoked\n");
}

rtems_task Init( rtems_task_argument argument )
{
  TEST_BEGIN();

  invoke_stack_overflows();

  rtems_task_exit();
}

void Fatal_extension(
  rtems_fatal_source source,
  bool               always_set_to_false,
  rtems_fatal_code   error
)
{
  if ( source != RTEMS_FATAL_SOURCE_STACK_CHECKER ) 
  {
    printk( "unexpected fatal source\n" );
  } 
  else 
  {
    TEST_END();
    rtems_test_exit(0);
  }
}

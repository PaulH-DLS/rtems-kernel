/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @brief Stack Smash Protector Configuration File
 *
 * This file configure a custom stack protector.
 */

/*
 * COPYRIGHT (c) 2024 Mohamed Hassan <muhammad.hamdy.hassan@gmail.com>
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

#include <rtems.h>
#include <rtems/score/thread.h>
#include <rtems/bspIo.h>

const char rtems_test_name[] = "STACKCHK06";

RTEMS_NO_RETURN void __stack_chk_fail(void);

/* configuration information */
#define CONFIGURE_STACK_CHECKER_ENABLED

RTEMS_NO_RETURN void __stack_chk_fail(void)
{
  // Custom handling for stack overflow
  printk("Custom stack overflow handler invoked\n");
  rtems_fatal(RTEMS_FATAL_SOURCE_STACK_CHECKER, true);
}

#define CONFIGURE_INIT
#include "../stackchk05/system.h"

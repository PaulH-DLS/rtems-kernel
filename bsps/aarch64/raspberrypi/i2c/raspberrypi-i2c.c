/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup raspberrypi_4_i2c
 *
 * @brief I2C Driver
 */

/*
 * Copyright (C) 2025 Shaunak Datar
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


#include <bsp/irq.h>
#include <bsp/raspberrypi-i2c.h>
#include <bsp/raspberrypi.h>
#include <bsp/rpi-gpio.h>
#include <dev/i2c/i2c.h>

typedef struct {
  i2c_bus                 base;
  uintptr_t               input_clock;
  rtems_id                task_id;
  uintptr_t               base_address;
  raspberrypi_bsc_masters device;
  uintptr_t               remaining_bytes;
  uintptr_t               remaining_transfers;
  uint8_t                *current_buffer;
  uintptr_t               current_buffer_size;
  bool                    read_transfer;
} raspberrypi_i2c_bus;

static void i2c_polling_read( raspberrypi_i2c_bus *bus )
{
  while ( !( S_REG( bus ) & S_DONE ) && ( bus->remaining_bytes > 0 ) ) {
    while ( ( S_REG( bus ) & S_RXD ) && ( bus->remaining_bytes > 0 ) ) {
      *bus->current_buffer = BCM2835_REG(
                               bus->base_address + BCM2711_I2C_FIFO
                             ) &
                             BCM2711_I2C_FIFO_MASK;
      bus->current_buffer++;
      bus->remaining_bytes--;

      // Check for errors
      if ( S_REG( bus ) & ( S_CLKT | S_ERR ) ) {
        return;
      }
    }
  }
}

static void i2c_polling_write( raspberrypi_i2c_bus *bus )
{
  while ( !( S_REG( bus ) & S_DONE ) && ( bus->remaining_bytes > 0 ) ) {
    while ( bus->remaining_bytes > 0 && ( S_REG( bus ) & S_TXD ) ) {
      BCM2835_REG( bus->base_address + BCM2711_I2C_FIFO ) = *(
        bus->current_buffer
      );
      bus->current_buffer++;
      bus->remaining_bytes--;
    }
  }
}

static int rpi_i2c_bus_transfer( raspberrypi_i2c_bus *bus )
{
  if ( bus->read_transfer ) {
    i2c_polling_read( bus );
  } else {
    i2c_polling_write( bus );
  }
  if ( ( S_REG( bus ) & S_ERR ) || ( S_REG( bus ) & S_CLKT ) ||
       ( bus->remaining_bytes != 0 ) ) {
    return -EIO;
  }

  S_REG( bus ) = S_DONE;
  return 0;
}

static void rpi_i2c_destroy( i2c_bus *base )
{
  raspberrypi_i2c_bus *bus = (raspberrypi_i2c_bus *) base;

  i2c_bus_destroy_and_free( &bus->base );
}

static int rpi_i2c_set_clock( i2c_bus *base, unsigned long clock )
{
  raspberrypi_i2c_bus *bus = (raspberrypi_i2c_bus *) base;
  uint32_t             clock_rate;
  uint16_t             divider;

  divider = BSC_CORE_CLK_HZ / clock;

  clock_rate = BSC_CORE_CLK_HZ / divider;

  while ( clock_rate > clock ) {
    ++divider;
    clock_rate = BSC_CORE_CLK_HZ / divider;
  }

  BCM2835_REG( bus->base_address + BCM2711_I2C_DIV ) = divider;

  bus->input_clock = clock_rate;

  return 0;
}

static int rpi_i2c_setup_transfer( raspberrypi_i2c_bus *bus )
{
  int rv;
  while ( bus->remaining_transfers > 0 ) {
    bus->remaining_bytes = bus->remaining_transfers > 1 ?
                             BCM2711_I2C_DLEN_MASK :
                             ( bus->current_buffer_size & BCM2711_I2C_DLEN_MASK
                             );
    BCM2835_REG( bus->base_address + BCM2711_I2C_DLEN ) = bus->remaining_bytes;
    S_REG( bus ) = S_ERR | S_CLKT | S_DONE;

    rv = rpi_i2c_bus_transfer( bus );

    if ( rv < 0 ) {
      return rv;
    }

    --bus->remaining_transfers;
  }
  return 0;
}

static int rpi_i2c_transfer( i2c_bus *base, i2c_msg *msgs, uint32_t msg_count )
{
  raspberrypi_i2c_bus *bus = (raspberrypi_i2c_bus *) base;
  int                  rv  = 0;
  uint32_t             i;
  uint8_t              msbs;
  int supported_flags = I2C_M_TEN | I2C_M_RD;

  for(i = 0; i < msg_count; i++ ) {
    
    if( msgs[ i ].len == 0 || msgs[ i ].buf == NULL) {
      return -EINVAL;
    }

    if ( ( msgs[ i ].flags & ~supported_flags ) != 0 ) {
      return -EINVAL;
    }
  }
  
  for ( i = 0; i < msg_count; i++ ) {
    bus->current_buffer      = msgs[ i ].buf;
    bus->current_buffer_size = msgs[ i ].len;
    bus->remaining_transfers = ( bus->current_buffer_size +
                                 ( BCM2711_I2C_DLEN_MASK - 1 ) ) /
                               BCM2711_I2C_DLEN_MASK;


    if ( msgs[ i ].flags & I2C_M_TEN ) // 10-bit slave address
    {
      /* Add the 8 lsbs of the 10-bit slave address to the fifo register*/
      BCM2835_REG(
        bus->base_address + BCM2711_I2C_FIFO
      )    = msgs[ i ].addr & BCM2711_I2C_FIFO_MASK;

      msbs = msgs[ i ].addr >> 8;
      BCM2835_REG(
        bus->base_address + BCM2711_I2C_SLAVE_ADDRESS
      ) = BCM2711_10_BIT_ADDR_MASK | msbs;
      
    } else {
      BCM2835_REG(
        bus->base_address + BCM2711_I2C_SLAVE_ADDRESS
      ) = msgs[ i ].addr;
    }

    if ( msgs[ i ].flags & I2C_M_RD ) {
      C_REG( bus )       |= C_CLEAR | C_READ | C_ST; // Read packet transfer
      bus->read_transfer  = true;
    } else {
      C_REG( bus )       |= C_CLEAR | C_ST; // Write packet transfer
      bus->read_transfer  = false;
    }

    rv = rpi_i2c_setup_transfer( bus );
    if ( rv < 0 ) {
      return rv;
    }
  }

  return rv;
}

static rtems_status_code i2c_gpio_init(
  raspberrypi_bsc_masters device,
  raspberrypi_i2c_bus    *bus
)
{
  switch ( device ) {
    case raspberrypi_bscm0:
      raspberrypi_gpio_set_function( 0, GPIO_AF0 );
      raspberrypi_gpio_set_function( 1, GPIO_AF0 );
      bus->base_address = BCM2711_I2C0_BASE;
      break;
    case raspberrypi_bscm1:
      raspberrypi_gpio_set_function( 2, GPIO_AF0 );
      raspberrypi_gpio_set_function( 3, GPIO_AF0 );
      bus->base_address = BCM2711_I2C1_BASE;
      break;
    case raspberrypi_bscm3:
      raspberrypi_gpio_set_function( 4, GPIO_AF5 );
      raspberrypi_gpio_set_function( 5, GPIO_AF5 );
      bus->base_address = BCM2711_I2C3_BASE;
      break;
    case raspberrypi_bscm4:
      raspberrypi_gpio_set_function( 6, GPIO_AF5 );
      raspberrypi_gpio_set_function( 7, GPIO_AF5 );
      bus->base_address = BCM2711_I2C4_BASE;
      break;
    case raspberrypi_bscm5:
      raspberrypi_gpio_set_function( 10, GPIO_AF5 );
      raspberrypi_gpio_set_function( 11, GPIO_AF5 );
      bus->base_address = BCM2711_I2C5_BASE;
      break;
    case raspberrypi_bscm6:
      raspberrypi_gpio_set_function( 22, GPIO_AF5 );
      raspberrypi_gpio_set_function( 23, GPIO_AF5 );
      bus->base_address = BCM2711_I2C6_BASE;
      break;
    default:
      return RTEMS_INVALID_ADDRESS;
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code rpi_i2c_init(
  raspberrypi_bsc_masters device,
  uint32_t                bus_clock
)
{
  raspberrypi_i2c_bus *bus;
  rtems_status_code    sc;
  const char          *bus_path;

  bus = (raspberrypi_i2c_bus *) i2c_bus_alloc_and_init( sizeof( *bus ) );
  if ( bus == NULL ) {
    return RTEMS_NO_MEMORY;
  }

  switch ( device ) {
    case raspberrypi_bscm0:
      bus_path = "/dev/i2c-0";
      break;
    case raspberrypi_bscm1:
      bus_path = "/dev/i2c-1";
      break;
    case raspberrypi_bscm3:
      bus_path = "/dev/i2c-3";
      break;
    case raspberrypi_bscm4:
      bus_path = "/dev/i2c-4";
      break;
    case raspberrypi_bscm5:
      bus_path = "/dev/i2c-5";
      break;
    case raspberrypi_bscm6:
      bus_path = "/dev/i2c-6";
      break;
    default:
      i2c_bus_destroy_and_free( &bus->base );
      return RTEMS_INVALID_NUMBER;
  }

  sc = i2c_gpio_init( device, bus );
  if ( sc != RTEMS_SUCCESSFUL ) {
    i2c_bus_destroy_and_free( &bus->base );
    return sc;
  }

  /* Enable I2C */
  C_REG( bus ) = C_CLEAR;
  C_REG( bus ) = C_I2CEN;

  sc = rpi_i2c_set_clock( &bus->base, bus_clock );
  if ( sc != RTEMS_SUCCESSFUL ) {
    i2c_bus_destroy_and_free( &bus->base );
    return sc;
  }

  bus->base.transfer      = rpi_i2c_transfer;
  bus->base.set_clock     = rpi_i2c_set_clock;
  bus->base.destroy       = rpi_i2c_destroy;
  bus->base.functionality = I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR;

  return i2c_bus_register( &bus->base, bus_path );
}

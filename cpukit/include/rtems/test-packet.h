/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSTestFrameworkPacket
 *
 * @brief This header file provides the interfaces of the
 *   @ref RTEMSTestFrameworkPacket.
 */

/*
 * Copyright (C) 2024 embedded brains GmbH & Co. KG
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

#ifndef _RTEMS_TEST_PACKET_H
#define _RTEMS_TEST_PACKET_H

#include <rtems/base64.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup RTEMSTestFrameworkPacket Packet Processor
 *
 * @ingroup RTEMSTestFramework
 *
 * @brief The RTEMS Test Framework packet processor provides a simple mechanism
 *   to exchange reliable and in-order data through transmitting and receiving
 *   one character at a time.
 *
 * The packet processor does not buffer data.  The processor uses a
 * stop-and-wait automatic repeat request method. There is at most one packet
 * in transmission.  The data transfer is done using a single character input
 * and output method.  The protocol uses 12-bit sequence numbers, so a host
 * could use a sliding window method to increase throughput.  All integers and
 * data are base64url encoded.  A 24-bit CRC is used to ensure the data
 * integrity.  The '{' character starts a packet.  The '}' character terminates
 * a packet.  The '#' character prefixes a 24-bit CRC value.  The ':' character
 * separates fields.  The '+' character prefixes data fields.  The following
 * packets are defined:
 *
 * * hello: {<12-bit seq><12-bit ack>:H#<24-bit CRC>}
 *
 * * acknowledge: {<12-bit seq><12-bit ack>:A#<24-bit CRC>}
 *
 * * reject: {<12-bit seq><12-bit ack>:R
 *   :<12-bit seq of rejected packet>
 *   #<24-bit CRC>}
 *
 * * signal: {<12-bit seq><12-bit ack>:S
 *   :<64-bit signal number>
 *   :<64-bit signal value>
 *   <optional list of colon separated 64-bit values>
 *   #<24-bit CRC>}
 *
 * * channel: {<12-bit seq><12-bit ack>:C
 *   :<64-bit channel number>
 *   :<64-bit channel data size>
 *   <optional list of colon separated 64-bit values>
 *   #<24-bit CRC>
 *   +<channel data>
 *   #<24-bit CRC>}
 *
 * The intended use case are boot loaders and test runners.  For example, test
 * runners may interface with an external test server performing equipment
 * handling on request using the packet processor.
 *
 * Use T_packet_initialize() to initialize the packet processor.  Use
 * T_packet_process() to drive the packet processing.  You can enqueue
 * packets for transmission with T_packet_enqueue().  You can reliably send
 * signals with T_packet_send().  You can reliably, in-order transmit and
 * receive channel data with T_packet_channel_push() and
 * T_packet_channel_exchange().
 *
 * A simple boot loader for test runs could be implemented like this:
 *
 * @code
 * #include <bsp.h>
 * #include <rtems/bspIo.h>
 * #include <rtems/counter.h>
 * #include <rtems/test-packet.h>
 *
 * typedef struct {
 *   T_packet_event_control base;
 *   uint8_t *load_address;
 * } boot_control;
 *
 * static void output_char(T_packet_control* self, uint8_t ch) {
 *   (void)self;
 *   rtems_putc(ch);
 * }
 *
 * static T_packet_status event_handler(T_packet_control* self,
 *                                      T_packet_event_control* base,
 *                                      T_packet_event evt) {
 *   boot_control* evt_ctrl = (boot_control*)base;
 *
 *   switch (evt) {
 *     case T_PACKET_EVENT_SIGNAL:
 *       if (T_packet_get_signal_number(self) == 0) {
 *         T_packet_output_acknowledge(self);
 *         bsp_restart(T_packet_get_signal_value_as_address(self));
 *       }
 *
 *       break;
 *     case T_PACKET_EVENT_HELLO:
 *       T_packet_output_acknowledge(self);
 *       break;
 *     case T_PACKET_EVENT_CHANNEL_BEGIN:
 *       if (T_packet_get_channel_number(self) == 0) {
 *         void *address;
 *
 *         if (T_packet_get_extra_count(self) >= 1) {
 *           address = T_packet_get_extra_as_address(self, 0);
 *           evt_ctrl->load_address = address;
 *         } else {
 *           address = evt_ctrl->load_address;
 *         }
 *
 *         T_packet_set_channel_target(self, address);
 *       }
 *
 *       break;
 *     case T_PACKET_EVENT_CHANNEL_END:
 *       if (T_packet_get_channel_number(self) == 0) {
 *         evt_ctrl->load_address += T_packet_get_channel_size(self);
 *         T_packet_output_acknowledge(self);
 *       }
 *
 *       break;
 *     case T_PACKET_EVENT_OUTPUT_END:
 *       rtems_putc('\n');
 *       break;
 *     default:
 *       break;
 *   }
 *
 *   return T_PACKET_SUCCESSFUL;
 * }
 *
 * static uint32_t clock_monotonic(T_packet_control* self) {
 *   (void)self;
 *   return rtems_counter_read();
 * }
 *
 * static void Init(rtems_task_argument arg) {
 *   (void)arg;
 *
 *   T_packet_control self;
 *   T_packet_initialize(&self, 0, NULL, output_char, clock_monotonic);
 *
 *   boot_control event;
 *   event.load_address = NULL;
 *   T_packet_prepend_event_handler(&self, &event.base, event_handler);
 *
 *   while (true) {
 *     T_packet_process(&self, getchark());
 *   }
 * }
 * @endcode
 *
 * @{
 */

/**
 * @brief These enumerators represent packet processing states.
 */
typedef enum {
  T_PACKET_STATE_START,
  T_PACKET_STATE_COLON,
  T_PACKET_STATE_CRC,
  T_PACKET_STATE_DECODE_DATA,
  T_PACKET_STATE_HASH,
  T_PACKET_STATE_REJECT,
  T_PACKET_STATE_SEQ_ACK,
  T_PACKET_STATE_TYPE,
  T_PACKET_STATE_VALUE
} T_packet_state;

/**
 * @brief These enumerators represent packet events.
 */
typedef enum {
  /**
   * @brief This event happens when an acknowledge package was received.
   */
  T_PACKET_EVENT_ACKNOWLEDGE,

  /**
   * @brief This event happens when channel data may get received.
   *
   * Call T_packet_set_channel_target() to set a target for the received
   * channel data, otherwise the packet is rejected.
   *
   * @see T_packet_get_channel_number() and T_packet_get_channel_size().
   */
  T_PACKET_EVENT_CHANNEL_BEGIN,

  /**
   * @brief This event happens when channel data was received successfully.
   *
   * Call T_packet_output_acknowledge() to acknowledge the channel data,
   * otherwise the packet is rejected.
   *
   * @see T_packet_get_channel_number() and T_packet_get_channel_size().
   */
  T_PACKET_EVENT_CHANNEL_END,

  /**
   * @brief This event happens when an duplicate packet was received.
   */
  T_PACKET_EVENT_DUPLICATE,

  /**
   * @brief This event happens when a garbage character was received.
   */
  T_PACKET_EVENT_GARBAGE,

  /**
   * @brief This event happens when a hello package was received.
   *
   * Call T_packet_output_acknowledge() to acknowledge the hello, otherwise
   * the packet is rejected.
   */
  T_PACKET_EVENT_HELLO,

  /**
   * @brief This event happens when a not acknowledge packet is transmitted.
   */
  T_PACKET_EVENT_NACK,

  /**
   * @brief This event happens when idle processing is performed.
   */
  T_PACKET_EVENT_NOTHING,

  /**
   * @brief This event happens when a packet output starts.
   */
  T_PACKET_EVENT_OUTPUT_BEGIN,

  /**
   * @brief This event happens when a packet output ends.
   */
  T_PACKET_EVENT_OUTPUT_END,

  /**
   * @brief This event happens when a packet is rejected.
   */
  T_PACKET_EVENT_REJECT,

  /**
   * @brief This event happens when a packet is transmitted again.
   */
  T_PACKET_EVENT_SEND_AGAIN,

  /**
   * @brief This event happens when a packet is dequeued for transmission.
   */
  T_PACKET_EVENT_SEND_DEQUEUE,

  /**
   * @brief This event happens when a transmitted packet was acknowledged.
   */
  T_PACKET_EVENT_SEND_DONE,

  /**
   * @brief This event happens when a signal was received.
   *
   * Call T_packet_output_acknowledge() to acknowledge the signal, otherwise
   * the packet is rejected.
   *
   * @see T_packet_get_signal_number(), T_packet_get_signal_value(), and
   *   T_packet_get_signal_value_as_address().
   */
  T_PACKET_EVENT_SIGNAL
} T_packet_event;

/**
 * @brief These enumerators represent the status of packet requests.
 */
typedef enum {
  /**
   * @brief Indicates a successful operation.
   */
  T_PACKET_SUCCESSFUL,

  /**
   * @brief Indicates that the event handling should continue.
   */
  T_PACKET_CONTINUE,

  /**
   * @brief Indicates that the request timed out.
   */
  T_PACKET_TIMEOUT,

  /**
   * @brief Indicates that a data buffer is not large enough.
   */
  T_PACKET_OVERFLOW,

  /**
   * @brief Indicates that no input character handler was available.
   */
  T_PACKET_NO_INPUT_CHAR_HANDLER
} T_packet_status;

typedef struct T_packet_packet T_packet_packet;

typedef struct T_packet_control T_packet_control;

typedef struct T_packet_event_control T_packet_event_control;

typedef T_packet_status (*T_packet_event_handler)(T_packet_control*,
                                                  T_packet_event_control*,
                                                  T_packet_event);

/**
 * @brief This structure represents a packet event handler.
 */
struct T_packet_event_control {
  T_packet_event_control* next;
  T_packet_event_handler handler;
};

/**
 * @brief This structure contains a packet to send.
 */
struct T_packet_packet {
  T_packet_packet* next;
  void (*output)(T_packet_control*, T_packet_packet*);
  void (*done)(T_packet_control*, T_packet_packet*);
};

/**
 * @brief This structure contains a signal packet to send.
 */
typedef struct {
  T_packet_packet base;
  uint64_t signal_number;
  uint64_t signal_value;
} T_packet_signal_packet;

/**
 * @brief This structure contains a channel packet to send.
 */
typedef struct {
  T_packet_packet base;
  uint64_t channel_number;
  const void* data_begin;
  size_t data_size;
} T_packet_channel_packet;

/**
 * @brief This structure represents a packet transfer.
 */
typedef struct {
  union {
    T_packet_packet base;
    T_packet_channel_packet channel;
    T_packet_signal_packet signal;
  };
  T_packet_status status;
  T_packet_event_control event;
} T_packet_packet_transfer;

/**
 * @brief This structure represents a packet processor.
 */
struct T_packet_control {
  T_packet_state state;
  T_packet_event_control* event_head;
  T_packet_event packet_done_event;
  uint32_t my_seq;
  uint32_t other_seq;
  size_t seq_ack_idx;
  uint32_t seq_ack;
  uint8_t packet_type;
  uint32_t crc_calculated;
  uint32_t crc_received;
  size_t crc_idx;
  size_t value_idx;
  uint64_t values[11];
  bool target_was_set;
  Base64_Decode_control b64_decode;
  uint32_t snd_timeout;
  T_packet_packet* snd_pending;
  T_packet_packet* snd_head;
  T_packet_packet** snd_tail;
  T_packet_packet hello;
  void (*output_char)(T_packet_control*, uint8_t);
  uint32_t (*clock_monotonic)(T_packet_control*);
  int (*input_char)(T_packet_control*);
};

/**
 * @brief Initializes the packet processor.
 *
 * @param[out] self is the packet processor to initialize.
 *
 * @param seq is the initial sequence and acknowledge number.
 *
 * @param input_char_handler is the handler to get characters.  This handler is
 *   optional and may be NULL.
 *
 * @param output_char_handler is the handler to output characters.
 *
 * @param clock_monotonic_handler is the handler to get the current value of a
 *   monotonic clock.
 */
void T_packet_initialize(
    T_packet_control* self,
    uint32_t seq,
    int (*input_char_handler)(T_packet_control*),
    void (*output_char_handler)(T_packet_control*, uint8_t),
    uint32_t (*clock_monotonic_handler)(T_packet_control*));

/**
 * @brief Appends the event handler to the event handler list of the packet
 *   processor.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[out] ctrl is the event control for the handler.
 *
 * @param handler is the event handler.
 */
void T_packet_append_event_handler(T_packet_control* self,
                                   T_packet_event_control* ctrl,
                                   T_packet_event_handler handler);

/**
 * @brief Prepends the event handler to the event handler list of the packet
 *   processor.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[out] ctrl is the event control for the handler.
 *
 * @param handler is the event handler.
 */
void T_packet_prepend_event_handler(T_packet_control* self,
                                    T_packet_event_control* ctrl,
                                    T_packet_event_handler handler);

/**
 * @brief Removes the event handler from the event handler list of the I/O
 *   packet processor.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[in] ctrl is the event control to remove.
 */
void T_packet_remove_event_handler(T_packet_control* self,
                                   T_packet_event_control* ctrl);

/**
 * @brief Processes the character or performs the idle processing.
 *
 * It may output at most one packet per call using the output character
 * handler.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param ch is the character to process or -1 to perform the idle processing.
 */
void T_packet_process(T_packet_control* self, int ch);

/**
 * @brief Sends the packet and waits for a non-continuation status.
 *
 * The status change shall be done by the provided packet send done handler or
 * the event handler.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[in, out] transfer is the packet transfer.
 *
 * @param timeout is the optional request timeout.  When the value is zero, the
 *   function waits forever for the non-continuation status.  Otherwise, the
 * function returns with a timeout status if the non-continuation status was not
 *   established within the specified time frame.  The timeout value is with
 *   respect to the used monotonic clock handler.
 */
T_packet_status T_packet_send(T_packet_control* self,
                              T_packet_packet_transfer* transfer,
                              uint32_t timeout);

/**
 * @brief Sends the signal and waits for the acknowledge.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param signal_number is the signal number.
 *
 * @param signal_value is the signal value.
 *
 * @param timeout is the optional request timeout.  When the value is zero, the
 *   function waits forever for the acknowledge.  Otherwise, the function
 *   returns with a timeout status if no acknowledge was received within the
 *   specified time frame.  The timeout value is with respect to the used
 *   monotonic clock handler.
 */
T_packet_status T_packet_signal(T_packet_control* self,
                                uint64_t signal_number,
                                uint64_t signal_value,
                                uint32_t timeout);

/**
 * @brief Pushes the data to the channel and waits for the acknowledge.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param channel_number is the channel number.
 *
 * @param data_begin is the begin address of the data area to push.
 *
 * @param data_size is the size in bytes of the data area to push.
 *
 * @param timeout is the optional request timeout.  When the value is zero, the
 *   function waits forever for the acknowledge.  Otherwise, the function
 *   returns with a timeout status if no acknowledge was received within the
 *   specified time frame.  The timeout value is with respect to the used
 *   monotonic clock handler.
 */
T_packet_status T_packet_channel_push(T_packet_control* self,
                                      uint64_t channel_number,
                                      const void* data_begin,
                                      size_t data_size,
                                      uint32_t timeout);

/**
 * @brief Pushes the data to the channel and waits for an
 *   acknowledge with response data.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param channel_number is the channel number.
 *
 * @param transmit_begin is the begin address of the data area to transmit.
 *
 * @param transmit_size is the size in bytes of the data area to transmit.
 *
 * @param[out] receive_begin is the begin address of the data area to receive
 * data.
 *
 * @param[in, out] receive_size is the pointer to a buffer size object.  On
 *   entry, the value of this object is the size in bytes of the data area to
 *   receive data.  On return, the value of this object is the size in bytes of
 *   the data received.
 *
 * @param timeout is the optional request timeout.  When the value is zero, the
 *   function waits forever for the acknowledge and received data.  Otherwise,
 *   the function returns with a timeout status if no acknowledge or data was
 *   received within the specified time frame.  The timeout value is with
 *   respect to the used monotonic clock handler.
 */
T_packet_status T_packet_channel_exchange(T_packet_control* self,
                                          uint64_t channel_number,
                                          const void* transmit_begin,
                                          size_t transmit_size,
                                          void* receive_begin,
                                          size_t* receive_size,
                                          uint32_t timeout);
/**
 * @brief Gets the extra value count.
 *
 * It may be used in the ::T_PACKET_EVENT_CHANNEL_BEGIN and
 * ::T_PACKET_EVENT_SIGNAL events.
 *
 * @param[in] self is the packet processor.
 *
 * @return Returns the extra value count.
 */
static inline uint64_t T_packet_get_extra_count(const T_packet_control* self) {
  return self->value_idx - 2;
}

/**
 * @brief Gets the extra value associated with the index.
 *
 * @param[in] self is the packet processor.
 *
 * @param index is the index of the extra value.
 *
 * @return Returns the extra value associated with the index.
 */
static inline uint64_t T_packet_get_extra(const T_packet_control* self,
                                          size_t index) {
  return self->values[index + 2];
}

/**
 * @brief Gets the extra value associated with the index as an address.
 *
 * @param[in] self is the packet processor.
 *
 * @param index is the index of the extra value.
 *
 * @return Returns the extra value associated with the index as an address.
 */
static inline void* T_packet_get_extra_as_address(const T_packet_control* self,
                                                  size_t index) {
  return (void*)(uintptr_t)self->values[index + 2];
}

/**
 * @brief Gets the extra value associated with the index as a size.
 *
 * @param[in] self is the packet processor.
 *
 * @param index is the index of the extra value.
 *
 * @return Returns the extra value associated with the index as a size.
 */
static inline size_t T_packet_get_extra_as_size(const T_packet_control* self,
                                                size_t index) {
  return (size_t)self->values[index + 2];
}

/**
 * @brief Gets the signal number.
 *
 * It may be used in ::T_PACKET_EVENT_SIGNAL events.
 *
 * @param[in] self is the packet processor.
 *
 * @return Returns the signal number.
 */
static inline uint64_t T_packet_get_signal_number(
    const T_packet_control* self) {
  return self->values[0];
}

/**
 * @brief Gets the signal value.
 *
 * It may be used in ::T_PACKET_EVENT_SIGNAL events.
 *
 * @param[in] self is the packet processor.
 *
 * @return Returns the signal value.
 */
static inline uint64_t T_packet_get_signal_value(const T_packet_control* self) {
  return self->values[1];
}

/**
 * @brief Gets the signal value as an address.
 *
 * It may be used in ::T_PACKET_EVENT_SIGNAL events.
 *
 * @param[in] self is the packet processor.
 *
 * @return Returns the signal value as an address.
 */
static inline void* T_packet_get_signal_value_as_address(
    const T_packet_control* self) {
  return (void*)(uintptr_t)self->values[1];
}

/**
 * @brief Gets the channel number.
 *
 * It may be used in ::T_PACKET_EVENT_CHANNEL_BEGIN and
 * ::T_PACKET_EVENT_CHANNEL_END events.
 *
 * @param[in] self is the packet processor.
 *
 * @return Returns the channel number.
 */
static inline uint64_t T_packet_get_channel_number(
    const T_packet_control* self) {
  return self->values[0];
}

/**
 * @brief Gets the channel data size in bytes.
 *
 * It may be used in ::T_PACKET_EVENT_CHANNEL_BEGIN and
 * ::T_PACKET_EVENT_CHANNEL_END events.
 *
 * @param[in] self is the packet processor.
 *
 * @return Returns the channel data size in bytes.
 */
static inline size_t T_packet_get_channel_size(const T_packet_control* self) {
  return (size_t)self->values[1];
}

/**
 * @brief Sets the channel data target address.
 *
 * It may be used in ::T_PACKET_EVENT_CHANNEL_BEGIN events.
 *
 * @param[in] self is the packet processor.
 *
 * @param[out] target is the channel data target address.  The target area
 *   shall be large enough to receive T_packet_get_channel_size() bytes.
 */
static inline void T_packet_set_channel_target(T_packet_control* self,
                                               void* target) {
  self->target_was_set = true;
  _Base64_Decode_initialize(&self->b64_decode, target,
                            T_packet_get_channel_size(self));
}

/**
 * @brief Outputs an acknowledge packet.
 *
 * @param[in, out] self is the packet processor.
 */
void T_packet_output_acknowledge(T_packet_control* self);

/**
 * @brief Outputs a reject packet.
 *
 * @param[in, out] self is the packet processor.
 */
void T_packet_output_reject(T_packet_control* self);

/**
 * @brief Does nothing.
 *
 * @param[in] self is the packet processor.
 *
 * @param[in] pkt is the successfully transmitted packet.
 */
void T_packet_done_default(T_packet_control* self, T_packet_packet* pkt);

/**
 * @brief Outputs a hello packet.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[in, out] pkt is the packet to output.
 */
void T_packet_output_hello(T_packet_control* self, T_packet_packet* pkt);

/**
 * @brief Outputs a signal packet.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[in, out] pkt is the packet to output.
 */
void T_packet_output_signal(T_packet_control* self, T_packet_packet* pkt);

/**
 * @brief Outputs a channel packet.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[in, out] pkt is the packet to output.
 */
void T_packet_output_channel(T_packet_control* self, T_packet_packet* pkt);

/**
 * @brief Enqueues an initialized packet for transmission.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[in, out] pkt is the packet to enqueue.
 */
static inline void T_packet_enqueue(T_packet_control* self,
                                    T_packet_packet* pkt) {
  pkt->next = NULL;
  *self->snd_tail = pkt;
  self->snd_tail = &pkt->next;
}

/**
 * @brief Removes the packet from the send queue.
 *
 * @param[in, out] self is the packet processor.
 *
 * @param[in, out] pkt is the packet to cancel.
 */
void T_packet_cancel(T_packet_control* self, T_packet_packet* pkt);

/**
 * @brief Initializes the signal packet.
 *
 * @param[out] pkt is the packet to enqueue.
 *
 * @param signal_number is the signal number.
 *
 * @param signal_value is the signal value.
 *
 * @param done is the transfer done handler.
 */
static inline void T_packet_initialize_signal(T_packet_signal_packet* pkt,
                                              uint64_t signal_number,
                                              uint64_t signal_value,
                                              void (*done)(T_packet_control*,
                                                           T_packet_packet*)) {
  pkt->base.output = T_packet_output_signal;
  pkt->base.done = done;
  pkt->signal_number = signal_number;
  pkt->signal_value = signal_value;
}

/**
 * @brief Initializes the channel packet.
 *
 * @param[out] pkt is the packet to enqueue.
 *
 * @param channel_number is the channel number.
 *
 * @param data_begin is the begin address of the data area.
 *
 * @param data_size is the size in bytes of the data area.
 *
 * @param done is the transfer done handler.
 */
static inline void T_packet_initialize_channel(T_packet_channel_packet* pkt,
                                               uint64_t channel_number,
                                               const void* data_begin,
                                               size_t data_size,
                                               void (*done)(T_packet_control*,
                                                            T_packet_packet*)) {
  pkt->base.output = T_packet_output_channel;
  pkt->base.done = done;
  pkt->channel_number = channel_number;
  pkt->data_begin = data_begin;
  pkt->data_size = data_size;
}

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _RTEMS_TEST_PACKET_H */

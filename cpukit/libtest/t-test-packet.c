/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (C) 2023 embedded brains GmbH & Co. KG
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

#include <rtems/test-packet.h>

#include <limits.h>
#include <string.h>

#include <rtems/crc.h>

#define SEQ_SHIFT 12
#define SEQ_MASK UINT32_C(0xfff)
#define ACK_VALID UINT32_C(0x1000000)
#define SEQ_VALID UINT32_C(0x2000000)

void T_packet_append_event_handler(T_packet_control* self,
                                   T_packet_event_control* ctrl,
                                   T_packet_event_handler handler) {
  ctrl->handler = handler;
  ctrl->next = NULL;
  T_packet_event_control* tail = self->event_head;
  T_packet_event_control** next = &self->event_head;

  while (true) {
    if (tail == NULL) {
      *next = ctrl;
      return;
    }

    next = &tail->next;
    tail = tail->next;
  }
}

void T_packet_prepend_event_handler(T_packet_control* self,
                                    T_packet_event_control* ctrl,
                                    T_packet_event_handler handler) {
  ctrl->handler = handler;
  ctrl->next = self->event_head;
  self->event_head = ctrl;
}

void T_packet_remove_event_handler(T_packet_control* self,
                                   T_packet_event_control* pkt) {
  T_packet_event_control* other = self->event_head;
  T_packet_event_control** prev = &self->event_head;

  while (other != NULL) {
    if (pkt == other) {
      *prev = pkt->next;
      return;
    }

    prev = &other->next;
    other = other->next;
  }
}

static void event(T_packet_control* self, T_packet_event event) {
  T_packet_event_control* ctrl = self->event_head;

  while (ctrl != NULL) {
    T_packet_event_control* next = ctrl->next;
    T_packet_status status = (*ctrl->handler)(self, ctrl, event);

    if (status != T_PACKET_CONTINUE) {
      return;
    }

    ctrl = next;
  }
}

static void output_char(T_packet_control* self, uint8_t ch) {
  (*self->output_char)(self, ch);
}

static uint32_t output_start(T_packet_control* self, uint8_t packet_type) {
  event(self, T_PACKET_EVENT_OUTPUT_BEGIN);
  output_char(self, '{');
  uint32_t crc = CRC24Q_SEED;
  uint32_t seq_ack = (self->my_seq << SEQ_SHIFT) | self->other_seq;

  for (int i = 18; i >= 0; i -= 6) {
    uint8_t ch = _Base64url_Encoding[(seq_ack >> i) & 0x3f];
    output_char(self, ch);
    crc = _CRC24Q_Update(crc, ch);
  }

  output_char(self, ':');
  crc = _CRC24Q_Update(crc, ':');
  output_char(self, packet_type);
  crc = _CRC24Q_Update(crc, packet_type);

  return crc;
}

static uint32_t output_value(T_packet_control* self,
                             uint32_t crc,
                             uint64_t value) {
  output_char(self, ':');
  crc = _CRC24Q_Update(crc, ':');

  int i = 60;

  /* Skip leading zeros */
  while (i >= 6) {
    if (((value >> i) & 0x3f) != 0) {
      break;
    }

    i -= 6;
  }

  while (i >= 0) {
    uint8_t ch = _Base64url_Encoding[(value >> i) & 0x3f];
    output_char(self, ch);
    crc = _CRC24Q_Update(crc, ch);
    i -= 6;
  }

  return crc;
}

static void output_crc(T_packet_control* self, uint32_t crc) {
  output_char(self, '#');

  for (int i = 18; i >= 0; i -= 6) {
    uint8_t ch = _Base64url_Encoding[(crc >> i) & 0x3f];
    output_char(self, ch);
  }
}

static void output_end(T_packet_control* self) {
  output_char(self, '}');
  event(self, T_PACKET_EVENT_OUTPUT_END);
}

static void inc_my_seq(T_packet_control* self) {
  self->my_seq = (self->my_seq + UINT32_C(1)) & SEQ_MASK;
}

static void make_pending(T_packet_control* self, T_packet_packet* pkt) {
  inc_my_seq(self);
  self->snd_pending = pkt;
  event(self, T_PACKET_EVENT_SEND_DEQUEUE);
}

static void output_packet(T_packet_control* self,
                          T_packet_packet* pkt,
                          uint32_t t0) {
  (*pkt->output)(self, pkt);
  uint32_t t1 = (*self->clock_monotonic)(self);
  self->snd_timeout = t0 + 4 * (t1 - t0);
}

static void output_simple_packet(T_packet_control* self, uint8_t packet_type) {
  uint32_t crc = output_start(self, packet_type);
  output_crc(self, crc);
  output_end(self);
}

static void send_done(T_packet_control* self) {
  T_packet_packet* pending = self->snd_pending;

  if (pending != NULL) {
    T_packet_packet* next = pending->next;
    self->snd_pending = NULL;
    self->snd_head = next;

    if (next == NULL) {
      self->snd_tail = &self->snd_head;
    }

    event(self, T_PACKET_EVENT_SEND_DONE);
    (*pending->done)(self, pending);
  }
}

static void output_response(T_packet_control* self, uint8_t packet_type) {
  uint32_t seq_ack = self->seq_ack;

  if ((seq_ack & ACK_VALID) != 0 && (seq_ack & SEQ_MASK) == self->my_seq) {
    send_done(self);
  }

  if ((seq_ack & SEQ_VALID) != 0) {
    self->other_seq = (seq_ack >> SEQ_SHIFT) & SEQ_MASK;
  }

  self->state = T_PACKET_STATE_START;

  if (packet_type == 'R' && self->snd_pending == NULL) {
    inc_my_seq(self);
    uint32_t crc = output_start(self, packet_type);
    crc = output_value(self, crc, (seq_ack >> SEQ_SHIFT) & SEQ_MASK);
    output_crc(self, crc);
    output_end(self);
    return;
  }

  T_packet_packet* head = self->snd_head;

  if (head == NULL) {
    if (self->packet_type != 'A') {
      inc_my_seq(self);
      output_simple_packet(self, packet_type);
    }

    return;
  }

  if (self->snd_pending != NULL) {
    event(self, T_PACKET_EVENT_SEND_AGAIN);
  } else {
    make_pending(self, head);
  }

  output_packet(self, head, (*self->clock_monotonic)(self));
}

static void output_nack(T_packet_control* self) {
  event(self, T_PACKET_EVENT_NACK);
  output_response(self, 'A');
}

void T_packet_output_acknowledge(T_packet_control* self) {
  output_response(self, 'A');
}

void T_packet_output_reject(T_packet_control* self) {
  output_response(self, 'R');
}

void T_packet_initialize(
    T_packet_control* self,
    uint32_t seq,
    int (*input_char_handler)(T_packet_control*),
    void (*output_char_handler)(T_packet_control*, uint8_t),
    uint32_t (*clock_monotonic_handler)(T_packet_control*)) {
  self = memset(self, 0, sizeof(*self));
  seq &= SEQ_MASK;
  self->my_seq = seq;
  self->other_seq = seq;
  self->hello.output = T_packet_output_hello;
  self->hello.done = T_packet_done_default;
  self->snd_head = &self->hello;
  self->snd_tail = &self->hello.next;
  self->input_char = input_char_handler;
  self->output_char = output_char_handler;
  self->clock_monotonic = clock_monotonic_handler;
}

static void begin(T_packet_control* self) {
  self->state = T_PACKET_STATE_SEQ_ACK;
  self->crc_calculated = CRC24Q_SEED;
  self->seq_ack_idx = 0;
  self->seq_ack = 0;
}

static void receive_crc(T_packet_control* self) {
  self->crc_received = 0;
  self->crc_idx = 0;
  self->state = T_PACKET_STATE_CRC;
}

static void update_crc(T_packet_control* self, uint8_t ch) {
  self->crc_calculated = _CRC24Q_Update(self->crc_calculated, ch);
}

static void do_seq_ack(T_packet_control* self, uint8_t ch) {
  update_crc(self, ch);
  size_t seq_ack_idx = self->seq_ack_idx;

  if (seq_ack_idx < 4) {
    if (ch < RTEMS_ARRAY_SIZE(_Base64_Decoding)) {
      uint8_t decoded_ch = _Base64_Decoding[ch];

      if (decoded_ch <= 63) {
        self->seq_ack = (self->seq_ack << 6) | decoded_ch;
        self->seq_ack_idx = seq_ack_idx + 1;
      } else {
        output_nack(self);
      }
    } else {
      output_nack(self);
    }
  } else {
    switch (ch) {
      case ':':
        self->state = T_PACKET_STATE_TYPE;
        break;
      default:
        output_nack(self);
        break;
    }
  }
}

static void do_type(T_packet_control* self, uint8_t ch) {
  update_crc(self, ch);
  self->packet_type = ch;

  switch (ch) {
    case 'C':
      self->packet_done_event = T_PACKET_EVENT_CHANNEL_END;
      self->state = T_PACKET_STATE_COLON;
      break;
    case 'S':
      self->packet_done_event = T_PACKET_EVENT_SIGNAL;
      self->state = T_PACKET_STATE_COLON;
      break;
    case 'H':
      self->packet_done_event = T_PACKET_EVENT_HELLO;
      self->state = T_PACKET_STATE_HASH;
      break;
    case 'A':
      self->packet_done_event = T_PACKET_EVENT_ACKNOWLEDGE;
      self->state = T_PACKET_STATE_HASH;
      break;
    default:
      self->packet_done_event = T_PACKET_EVENT_REJECT;
      self->state = T_PACKET_STATE_REJECT;
      break;
  }
}

static void do_colon(T_packet_control* self, uint8_t ch) {
  switch (ch) {
    case ':':
      update_crc(self, ch);
      self->value_idx = 0;
      self->values[0] = 0;
      self->state = T_PACKET_STATE_VALUE;
      break;
    default:
      output_nack(self);
      break;
  }
}

static void update_value(T_packet_control* self, uint8_t ch, size_t value_idx) {
  if (ch >= RTEMS_ARRAY_SIZE(_Base64_Decoding)) {
    output_nack(self);
    return;
  }

  uint8_t decoded_ch = _Base64_Decoding[ch];

  if (decoded_ch > 63) {
    output_nack(self);
    return;
  }

  uint64_t value = self->values[value_idx];

  if ((value & UINT64_C(0xfc00000000000000)) != 0) {
    output_nack(self);
    return;
  }

  self->values[value_idx] = (value << 6) | decoded_ch;
  update_crc(self, ch);
}

static void do_value(T_packet_control* self, uint8_t ch) {
  size_t value_idx = self->value_idx;

  switch (ch) {
    case '#':
      self->value_idx = value_idx + 1;
      receive_crc(self);
      break;
    case ':':
      if (self->value_idx <= RTEMS_ARRAY_SIZE(self->values) - 2) {
        ++value_idx;
        self->value_idx = value_idx;
        self->values[value_idx] = 0;
        update_crc(self, ch);
      } else {
        output_nack(self);
      }

      break;
    default:
      update_value(self, ch, value_idx);
      break;
  }
}

static void do_hash(T_packet_control* self, uint8_t ch) {
  switch (ch) {
    case '#':
      receive_crc(self);
      break;
    default:
      output_nack(self);
      break;
  }
}

static void do_reject(T_packet_control* self, uint8_t ch) {
  switch (ch) {
    case '#':
      receive_crc(self);
      break;
    default:
      update_crc(self, ch);
      break;
  }
}

static void next_component(T_packet_control* self) {
  if (self->value_idx < 2) {
    output_nack(self);
    return;
  }

  self->crc_calculated = CRC24Q_SEED;

  switch (self->packet_type) {
    case 'C': {
      self->target_was_set = false;
      self->state = T_PACKET_STATE_DECODE_DATA;
      event(self, T_PACKET_EVENT_CHANNEL_BEGIN);

      if (!self->target_was_set) {
        T_packet_output_reject(self);
      }

      break;
    }
    default:
      output_nack(self);
      break;
  }
}

static void check_crc(T_packet_control* self, uint8_t ch) {
  if ((self->crc_calculated & CRC24Q_MASK) != self->crc_received) {
    output_nack(self);
    return;
  }

  uint32_t seq_ack = self->seq_ack | ACK_VALID;
  uint32_t other_seq = (seq_ack >> SEQ_SHIFT) & SEQ_MASK;

  if (((self->other_seq - other_seq) & SEQ_MASK) < SEQ_MASK / 2) {
    event(self, T_PACKET_EVENT_DUPLICATE);
    T_packet_output_acknowledge(self);
    return;
  }

  if (self->packet_done_event == T_PACKET_EVENT_REJECT) {
    self->seq_ack = seq_ack;
    event(self, T_PACKET_EVENT_REJECT);
    T_packet_output_reject(self);
    return;
  }

  switch (ch) {
    case '}':
      self->seq_ack = seq_ack | SEQ_VALID;

      if (self->packet_type == 'S' && self->value_idx < 2) {
        output_nack(self);
        return;
      }

      event(self, self->packet_done_event);

      if (self->state != T_PACKET_STATE_START) {
        if (self->packet_type == 'A') {
          output_response(self, 'A');
        } else {
          self->seq_ack = seq_ack;
          T_packet_output_reject(self);
        }
      }

      break;
    case '+':
      self->seq_ack = seq_ack;
      next_component(self);
      break;
    default:
      output_nack(self);
      break;
  }
}

static void do_crc(T_packet_control* self, uint8_t ch) {
  size_t crc_idx = self->crc_idx;

  if (crc_idx < 4) {
    if (ch < RTEMS_ARRAY_SIZE(_Base64_Decoding)) {
      uint8_t decoded_ch = _Base64_Decoding[ch];

      if (decoded_ch <= 63) {
        self->crc_received = (self->crc_received << 6) | decoded_ch;
        self->crc_idx = crc_idx + 1;
      } else {
        output_nack(self);
      }
    } else {
      output_nack(self);
    }
  } else {
    check_crc(self, ch);
  }
}

static void do_decode_data(T_packet_control* self, uint8_t ch) {
  switch (ch) {
    case '#':
      if (self->b64_decode.target == self->b64_decode.target_end) {
        receive_crc(self);
      } else {
        output_nack(self);
      }
      break;
    default: {
      Base64_Decode_status status = _Base64_Decode(&self->b64_decode, ch);

      if (status == BASE64_DECODE_SUCCESS) {
        update_crc(self, ch);
      } else {
        output_nack(self);
      }

      break;
    }
  }
}

static void idle_processing(T_packet_control* self) {
  event(self, T_PACKET_EVENT_NOTHING);

  if (self->state != T_PACKET_STATE_START) {
    return;
  }

  T_packet_packet* head = self->snd_head;

  if (head == NULL) {
    return;
  }

  uint32_t now = (*self->clock_monotonic)(self);

  if (self->snd_pending != NULL) {
    if ((self->snd_timeout - now) <= UINT32_MAX / 2) {
      return;
    }

    event(self, T_PACKET_EVENT_SEND_AGAIN);
  } else {
    make_pending(self, head);
  }

  output_packet(self, head, now);
}

void T_packet_process(T_packet_control* self, int ch_or_nothing) {
  if (ch_or_nothing == -1) {
    idle_processing(self);
    return;
  }

  uint8_t ch = (uint8_t)ch_or_nothing;

  if (ch == '{') {
    begin(self);
    return;
  }

  switch (self->state) {
    case T_PACKET_STATE_SEQ_ACK:
      do_seq_ack(self, ch);
      break;
    case T_PACKET_STATE_TYPE:
      do_type(self, ch);
      break;
    case T_PACKET_STATE_COLON:
      do_colon(self, ch);
      break;
    case T_PACKET_STATE_VALUE:
      do_value(self, ch);
      break;
    case T_PACKET_STATE_DECODE_DATA:
      do_decode_data(self, ch);
      break;
    case T_PACKET_STATE_HASH:
      do_hash(self, ch);
      break;
    case T_PACKET_STATE_CRC:
      do_crc(self, ch);
      break;
    case T_PACKET_STATE_REJECT:
      do_reject(self, ch);
      break;
    default:
      /* Wait for packet start */
      event(self, T_PACKET_EVENT_GARBAGE);
      break;
  }
}

void T_packet_done_default(T_packet_control* self, T_packet_packet* pkt) {
  (void)self;
  (void)pkt;
}

void T_packet_output_hello(T_packet_control* self, T_packet_packet* pkt) {
  (void)pkt;
  output_simple_packet(self, 'H');
}

void T_packet_output_signal(T_packet_control* self, T_packet_packet* pkt) {
  uint32_t crc = output_start(self, 'S');
  T_packet_signal_packet* signal_pkt = (T_packet_signal_packet*)pkt;
  crc = output_value(self, crc, signal_pkt->signal_number);
  crc = output_value(self, crc, signal_pkt->signal_value);
  output_crc(self, crc);
  output_end(self);
}

static void b64_output_char(int c, void* arg) {
  T_packet_control* self = arg;
  output_char(self, (uint8_t)c);
  self->crc_calculated = _CRC24Q_Update(self->crc_calculated, (uint8_t)c);
}

void T_packet_output_channel(T_packet_control* self, T_packet_packet* pkt) {
  uint32_t crc = output_start(self, 'C');
  T_packet_channel_packet* channel_pkt = (T_packet_channel_packet*)pkt;
  crc = output_value(self, crc, channel_pkt->channel_number);
  crc = output_value(self, crc, channel_pkt->data_size);
  output_crc(self, crc);
  output_char(self, '+');
  self->crc_calculated = CRC24Q_SEED;
  _Base64url_Encode(b64_output_char, self, channel_pkt->data_begin,
                    channel_pkt->data_size, NULL, INT_MAX);
  output_crc(self, self->crc_calculated);
  output_end(self);
}

void T_packet_cancel(T_packet_control* self, T_packet_packet* pkt) {
  T_packet_packet* enq_pkt = self->snd_head;
  T_packet_packet** prev = &self->snd_head;

  if (self->snd_pending == pkt) {
    self->snd_pending = NULL;
  }

  while (enq_pkt != NULL) {
    if (pkt == enq_pkt) {
      *prev = pkt->next;

      if (self->snd_tail == &pkt->next) {
        self->snd_tail = prev;
      }

      return;
    }

    prev = &enq_pkt->next;
    enq_pkt = enq_pkt->next;
  }
}

static void done_success(T_packet_control* self, T_packet_packet* pkt) {
  (void)self;
  T_packet_packet_transfer* transfer = (T_packet_packet_transfer*)pkt;
  transfer->status = T_PACKET_SUCCESSFUL;
}

T_packet_status T_packet_send(T_packet_control* self,
                              T_packet_packet_transfer* transfer,
                              uint32_t timeout) {
  if (self->input_char == NULL) {
    T_packet_remove_event_handler(self, &transfer->event);
    return T_PACKET_NO_INPUT_CHAR_HANDLER;
  }

  transfer->status = T_PACKET_CONTINUE;
  T_packet_enqueue(self, &transfer->base);
  T_packet_process(self, -1);

  bool check_for_timeout = (timeout != 0);
  uint32_t t0 = 0;

  if (check_for_timeout) {
    t0 = (*self->clock_monotonic)(self);
  }

  while (true) {
    T_packet_process(self, (*self->input_char)(self));

    if (transfer->status != T_PACKET_CONTINUE) {
      return transfer->status;
    }

    if (check_for_timeout) {
      uint32_t t1 = (*self->clock_monotonic)(self);
      uint32_t new_timeout = timeout - (t1 - t0);

      if (new_timeout > timeout) {
        T_packet_cancel(self, &transfer->base);
        T_packet_remove_event_handler(self, &transfer->event);
        return T_PACKET_TIMEOUT;
      }

      timeout = new_timeout;
      t0 = t1;
    }
  }
}

T_packet_status T_packet_signal(T_packet_control* self,
                                uint64_t signal_number,
                                uint64_t signal_value,
                                uint32_t timeout) {
  T_packet_packet_transfer transfer;
  T_packet_initialize_signal(&transfer.signal, signal_number, signal_value,
                             done_success);
  return T_packet_send(self, &transfer, timeout);
}

T_packet_status T_packet_channel_push(T_packet_control* self,
                                      uint64_t channel_number,
                                      const void* data_begin,
                                      size_t data_size,
                                      uint32_t timeout) {
  T_packet_packet_transfer transfer;
  T_packet_initialize_channel(&transfer.channel, channel_number, data_begin,
                              data_size, done_success);
  return T_packet_send(self, &transfer, timeout);
}

typedef struct {
  T_packet_packet_transfer base;
  void* receive_begin;
  size_t receive_size_max;
  size_t receive_size_return;
} channel_transfer;

static T_packet_status channel_exchange_event(T_packet_control* self,
                                              T_packet_event_control* ctrl,
                                              T_packet_event evt) {
  channel_transfer* transfer =
      RTEMS_CONTAINER_OF(ctrl, channel_transfer, base.event);

  if (evt == T_PACKET_EVENT_CHANNEL_BEGIN) {
    if (T_packet_get_channel_number(self) !=
        transfer->base.channel.channel_number) {
      return T_PACKET_CONTINUE;
    }

    size_t size = T_packet_get_channel_size(self);

    if (size <= transfer->receive_size_max) {
      transfer->receive_size_return = size;
      T_packet_set_channel_target(self, transfer->receive_begin);
    } else {
      transfer->base.status = T_PACKET_OVERFLOW;
      T_packet_remove_event_handler(self, ctrl);
    }

    return T_PACKET_SUCCESSFUL;
  }

  if (evt == T_PACKET_EVENT_CHANNEL_END) {
    if (T_packet_get_channel_number(self) !=
        transfer->base.channel.channel_number) {
      return T_PACKET_CONTINUE;
    }

    transfer->base.status = T_PACKET_SUCCESSFUL;
    T_packet_remove_event_handler(self, ctrl);
    T_packet_output_acknowledge(self);
    return T_PACKET_SUCCESSFUL;
  }

  return T_PACKET_CONTINUE;
}

T_packet_status T_packet_channel_exchange(T_packet_control* self,
                                          uint64_t channel_number,
                                          const void* transmit_begin,
                                          size_t transmit_size,
                                          void* receive_begin,
                                          size_t* receive_size,
                                          uint32_t timeout) {
  channel_transfer transfer;
  T_packet_initialize_channel(&transfer.base.channel, channel_number,
                              transmit_begin, transmit_size,
                              T_packet_done_default);
  transfer.receive_begin = receive_begin;
  transfer.receive_size_max = *receive_size;
  transfer.receive_size_return = 0;
  T_packet_prepend_event_handler(self, &transfer.base.event,
                                 channel_exchange_event);
  T_packet_status status = T_packet_send(self, &transfer.base, timeout);
  *receive_size = transfer.receive_size_return;
  return status;
}

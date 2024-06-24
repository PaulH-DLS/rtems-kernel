/* SPDX-License-Identifier: BSD-2-Clause */

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

#include <rtems/test-packet.h>

#include <inttypes.h>
#include <limits.h>
#include <string.h>
#include <sys/endian.h>

#include <rtems/crc.h>

#include <rtems/test.h>

typedef struct {
  T_packet_control base;
  T_packet_event_control event;
  T_packet_event_control event_success;
  T_packet_event_control event_continue;
  size_t response_idx;
  char response_buf[256];
  char load_buf[32];
  uint32_t counter;
  T_packet_signal_packet signal_pkt;
  T_packet_channel_packet channel_pkt;
  const char* input;
  uint32_t crc;
} test_control;

typedef struct {
  const char* input;
  const char* response;
  const char* load;
} test_case;

static const test_case test_cases[] = {
    {"@X@X@X@X@X", "@X@Q@R{1211:H#Mos3}@r@X@X@X@X@T@R{1211:H#Mos3}@r", ""},
    {"{a{B@XBCC:H#h4zx}x{{BBCC:H#h4zx}",
     "@X@H@Q@R{12BB:H#EmCT}@r@G@D@T@R{12BB:H#EmCT}@r", ""},
    {"{123456789", "@N@Q@R{1211:H#Mos3}@r@G@G@G@G", ""},
    {"{1234:HX", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:H#\xe2\x82\xac", "@N@Q@R{1211:H#Mos3}@r@G@G", ""},
    {"{1234:H#|", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:H#abcd|", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{12345", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{|{\xe2\x82\xac", "@N@Q@R{1211:H#Mos3}@r@N@T@R{1211:H#Mos3}@r@G@G", ""},
    {"{1234:X:#gOys}{1312:X:#N4Ge}",
     "@E@R{1211:R:12#TQxe}@r@E@R{1311:R:13#PksK}@r", ""},
    {"@X{1211:Y#cYlF}{1312:Y#qyFL}",
     "@X@Q@R{1211:H#Mos3}@r@E@T@R{1211:H#Mos3}@r@E@o@R{1311:R:13#PksK}@r", ""},
    {"{1234:SX", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:\xe2\x82\xac", "@N@Q@R{1211:H#Mos3}@r@G@G", ""},
    {"{1234:S:|", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:123456789a_", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S::::::::::::", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:0#noof|", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:0#noof+", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:0#abcd}", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:AAAAAAAAAAAAAAAAAAAAADerb7v#hu_C}", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:Derb7v#r2GB}", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:Derb7v#Awyf}", "@N@Q@R{1212:H#Md21}@r", ""},
    {"{1234:S:BI0VniHZUMh:A#8aeE}", "@S@Q@R{1212:H#Md21}@r", ""},
    {"{1234:S:BI0VniHZUMh:A#8aeE+", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:S:BI0VniHZUMh:AKvN76vN76vN:A:B:C#slsO}",
     "@S[0=0,0,0][1=1,0x1,1][2=2,0x2,2]@R{1211:R:12#TQxe}@r", ""},
    {"{1234:C:BI0VniHZUMh#Q5lB+", "@N@Q@R{1211:H#Mos3}@r", ""},
    {"{1234:C:BI0VniHZUMh:A#gl_H+", "@C@R{1211:R:12#TQxe}@r", ""},
    {"{1234:C:BI0VniHZUMh:N#vKHp+SGVsbG8sIHdvcmxkIQ==#b1BU}",
     "@C@c@Q@R{1212:H#Md21}@r", "Hello, world!"},
    {"{1234:C:BI0VniHZUMh:N#vKHp+SGVsbG8sIHdvcmxkI#J8VH}",
     "@C@N@Q@R{1211:H#Mos3}@r@G@G@G@G@G", "Hello, world "},
    {"{1234:C:BI0VniHZUMh:N#vKHp+SGVsbG8sIHdvcmxkI?", "@C@N@Q@R{1211:H#Mos3}@r",
     "Hello, world "},
    {"@X{121@S@X2:A#FhB3}@X@X@X@X@X{1313:A#TOcs}@X{1413:A#y8uF}",
     "@X@Q@R{1211:H#Mos3}@r@X@A@o@Q@R{1312:S:A:Ej#5Yfj}@r@X@X@X@X@T@R{1312:S:"
     "A:"
     "Ej#5Yfj}@r@X@A@o@s@X@A",
     ""},
    {"@X{1212:A@C#FhB3}@X",
     "@X@Q@R{1211:H#Mos3}@r@A@o@Q@R{1312:C:RW:G#sdj6+BlubBlub#1V-z}@r@X", ""}};

static void process(test_control* self, const char* input) {
  uint32_t crc = self->crc;

  while (*input != '\0') {
    crc = _CRC24Q_Update(crc, (uint8_t)*input);
    T_packet_process(&self->base, (uint8_t)*input);
    ++input;
  }

  self->crc = crc;
}

static void output_char(T_packet_control* base, uint8_t ch) {
  test_control* self = (test_control*)base;
  size_t idx = self->response_idx;
  self->response_idx = idx + 1;

  if (idx < sizeof(self->response_buf) - 1) {
    self->response_buf[idx] = (char)ch;
  }
}

static void print_extra(T_packet_control* base) {
  test_control* self = (test_control*)base;
  size_t n = T_packet_get_extra_count(base);

  for (size_t i = 0; i < n; ++i) {
    size_t idx = self->response_idx;

    if (idx < sizeof(self->response_buf)) {
      self->response_idx =
          idx + (size_t)T_snprintf(
                    &self->response_buf[idx], sizeof(self->response_buf) - idx,
                    "[%zu=%" PRIu64 ",%p,%zu]", i, T_packet_get_extra(base, i),
                    T_packet_get_extra_as_address(base, i),
                    T_packet_get_extra_as_size(base, i));
    }
  }
}

static int input_char(T_packet_control* base) {
  test_control* self = (test_control*)base;
  const char* input = self->input;
  uint8_t ch = (uint8_t)*input;

  if (ch == '\0') {
    return -1;
  }

  self->input = input + 1;
  return ch;
}

static T_packet_status event(T_packet_control* base,
                             T_packet_event_control* ctrl,
                             T_packet_event evt) {
  test_control* self = (test_control*)base;
  T_eq_ptr(ctrl, &self->event);
  output_char(base, '@');

  switch (evt) {
    case T_PACKET_EVENT_ACKNOWLEDGE:
      output_char(base, 'A');
      break;
    case T_PACKET_EVENT_CHANNEL_BEGIN: {
      output_char(base, 'C');
      T_eq_u64(T_packet_get_channel_number(base), UINT64_C(0x1234567887654321));
      size_t size = T_packet_get_channel_size(base);

      if (size != 0) {
        T_packet_set_channel_target(base, &self->load_buf[0]);
        T_eq_sz(size, 0xd);
      }

      print_extra(base);
      break;
    }
    case T_PACKET_EVENT_CHANNEL_END:
      output_char(base, 'c');
      T_packet_output_acknowledge(base);
      break;
    case T_PACKET_EVENT_DUPLICATE:
      output_char(base, 'D');
      break;
    case T_PACKET_EVENT_GARBAGE:
      output_char(base, 'G');
      break;
    case T_PACKET_EVENT_HELLO:
      output_char(base, 'H');
      T_packet_output_acknowledge(base);
      break;
    case T_PACKET_EVENT_NOTHING:
      output_char(base, 'X');
      break;
    case T_PACKET_EVENT_OUTPUT_BEGIN:
      output_char(base, 'R');
      break;
    case T_PACKET_EVENT_OUTPUT_END:
      output_char(base, 'r');
      break;
    case T_PACKET_EVENT_SEND_DONE:
      output_char(base, 'o');
      break;
    case T_PACKET_EVENT_REJECT:
      output_char(base, 'E');
      break;
    case T_PACKET_EVENT_NACK:
      output_char(base, 'N');
      break;
    case T_PACKET_EVENT_SEND_DEQUEUE:
      output_char(base, 'Q');
      break;
    case T_PACKET_EVENT_SEND_AGAIN:
      output_char(base, 'T');
      break;
    case T_PACKET_EVENT_SIGNAL: {
      output_char(base, 'S');
      T_eq_u64(T_packet_get_signal_number(base), UINT64_C(0x1234567887654321));
      uint64_t value = T_packet_get_signal_value(base);

      if (value == 0) {
        T_packet_output_acknowledge(base);
      } else {
        T_eq_u64(value, UINT64_C(0xabcdefabcdefabcd));
      }

      print_extra(base);
      break;
    }
    default:
      output_char(base, 'U');
      break;
  }

  return T_PACKET_SUCCESSFUL;
}

static T_packet_status event_success(T_packet_control* base,
                                     T_packet_event_control* ctrl,
                                     T_packet_event evt) {
  (void)evt;
  test_control* self = (test_control*)base;
  T_eq_ptr(ctrl, &self->event_success);
  output_char(base, '@');
  output_char(base, 'Z');
  return T_PACKET_SUCCESSFUL;
}

static T_packet_status event_continue(T_packet_control* base,
                                      T_packet_event_control* ctrl,
                                      T_packet_event evt) {
  (void)evt;
  test_control* self = (test_control*)base;
  T_eq_ptr(ctrl, &self->event_continue);
  output_char(base, '@');
  output_char(base, 'Y');
  return T_PACKET_CONTINUE;
}

static T_packet_status event_channel_load(T_packet_control* base,
                                          T_packet_event_control* ctrl,
                                          T_packet_event evt) {
  test_control* self = (test_control*)base;
  T_eq_ptr(ctrl, &self->event);

  if (evt == T_PACKET_EVENT_CHANNEL_BEGIN) {
    if (T_packet_get_channel_number(base) != 3) {
      return T_PACKET_CONTINUE;
    }

    T_lt_sz(T_packet_get_channel_size(base), sizeof(self->load_buf));
    T_packet_set_channel_target(base, &self->load_buf[0]);
    return T_PACKET_SUCCESSFUL;
  }

  if (evt == T_PACKET_EVENT_CHANNEL_END) {
    if (T_packet_get_channel_number(base) != 3) {
      return T_PACKET_CONTINUE;
    }

    T_packet_output_acknowledge(base);
    return T_PACKET_SUCCESSFUL;
  }

  return T_PACKET_CONTINUE;
}

static void signal_done(T_packet_control* base, T_packet_packet* pkt) {
  test_control* self = (test_control*)base;
  output_char(base, '@');
  output_char(base, 's');
  T_eq_ptr(pkt, &self->signal_pkt);
}

static void channel_done(T_packet_control* base, T_packet_packet* pkt) {
  test_control* self = (test_control*)base;
  output_char(base, '@');
  output_char(base, 'e');
  T_eq_ptr(pkt, &self->channel_pkt);
}

static uint32_t now(T_packet_control* base) {
  test_control* self = (test_control*)base;
  uint32_t counter = self->counter;
  self->counter = counter + 1;
  return self->counter;
}

static const char channel_data[] = {0x06, 0x5b, 0x9b, 0x06, 0x5b, 0x9b};

static void clear_response(test_control* self) {
  self->response_idx = 0;
  memset(&self->response_buf[0], 0, sizeof(self->response_buf));
}

static void initialize_test_control(test_control* self) {
  self->input = "";
  memset(&self->base, 0xff, sizeof(self->base));
  memset(&self->load_buf[0], 0, sizeof(self->load_buf));
  clear_response(self);
  T_packet_initialize(&self->base, 3445, NULL, output_char, now);
  T_packet_prepend_event_handler(&self->base, &self->event, event);
}

T_TEST_CASE(TPacket) {
  test_control self;

  for (size_t i = 0; i < RTEMS_ARRAY_SIZE(test_cases); ++i) {
    initialize_test_control(&self);
    const test_case* tc = &test_cases[i];
    const char* ch = tc->input;

    while (*ch != '\0') {
      if (*ch == '@') {
        ++ch;
        switch (*ch) {
          case 'X':
            T_packet_process(&self.base, -1);
            break;
          case 'S':
            T_packet_initialize_signal(&self.signal_pkt, 0, 0x123, signal_done);
            T_packet_enqueue(&self.base, &self.signal_pkt.base);
            break;
          case 'C':
            T_packet_initialize_channel(&self.channel_pkt, 0x456,
                                        &channel_data[0], sizeof(channel_data),
                                        channel_done);
            T_packet_enqueue(&self.base, &self.channel_pkt.base);
            break;
          default:
            T_unreachable();
            break;
        }
      } else {
        T_packet_process(&self.base, (uint8_t)*ch);
      }

      ++ch;
    }

    T_eq_str(&self.response_buf[0], tc->response);
    T_eq_str(&self.load_buf[0], tc->load);
  }
}

T_TEST_CASE(TPacketCancel) {
  test_control self;
  initialize_test_control(&self);
  T_null(self.base.snd_pending);
  T_eq_ptr(self.base.snd_head, &self.base.hello);
  T_eq_ptr(self.base.snd_tail, &self.base.hello.next);

  T_packet_process(&self.base, -1);
  T_eq_ptr(self.base.snd_pending, &self.base.hello);
  T_eq_ptr(self.base.snd_head, &self.base.hello);
  T_eq_ptr(self.base.snd_tail, &self.base.hello.next);

  T_packet_packet pkt;
  memset(&pkt, 0xff, sizeof(pkt));
  T_packet_enqueue(&self.base, &pkt);
  T_eq_ptr(self.base.snd_head, &self.base.hello);
  T_eq_ptr(self.base.snd_tail, &pkt.next);

  T_packet_packet pkt_2;
  memset(&pkt_2, 0xff, sizeof(pkt_2));
  T_packet_enqueue(&self.base, &pkt_2);
  T_eq_ptr(self.base.snd_head, &self.base.hello);
  T_eq_ptr(self.base.snd_tail, &pkt_2.next);

  T_packet_cancel(&self.base, &self.base.hello);
  T_null(self.base.snd_pending);
  T_eq_ptr(self.base.snd_head, &pkt);
  T_eq_ptr(self.base.snd_tail, &pkt_2.next);

  T_packet_cancel(&self.base, &pkt_2);
  T_null(self.base.snd_pending);
  T_eq_ptr(self.base.snd_head, &pkt);
  T_eq_ptr(self.base.snd_tail, &pkt.next);

  /* Double cancel has no effects */
  T_packet_cancel(&self.base, &pkt_2);
  T_null(self.base.snd_pending);
  T_eq_ptr(self.base.snd_head, &pkt);
  T_eq_ptr(self.base.snd_tail, &pkt.next);

  T_packet_cancel(&self.base, &pkt);
  T_null(self.base.snd_pending);
  T_null(self.base.snd_head);
  T_eq_ptr(self.base.snd_tail, &self.base.snd_head);
}

T_TEST_CASE(TPacketSignal) {
  test_control self;
  initialize_test_control(&self);
  T_packet_status status = T_packet_signal(&self.base, 1, 2, 0);
  T_eq_int(status, T_PACKET_NO_INPUT_CHAR_HANDLER);
  T_eq_str(&self.response_buf[0], "");

  self.base.input_char = input_char;
  self.input = "{1212:A#FhB3}{1313:A#TOcs}";
  status = T_packet_signal(&self.base, 3, 4, 0);
  T_eq_int(status, T_PACKET_SUCCESSFUL);
  T_null(self.base.snd_pending);
  T_null(self.base.snd_head);
  T_eq_str(&self.response_buf[0],
           "@X@Q@R{1211:H#Mos3}@r@A@o@Q@R{1312:S:D:E#AzkU}@r@A@o");

  clear_response(&self);
  status = T_packet_signal(&self.base, 5, 6, 2);
  T_eq_int(status, T_PACKET_TIMEOUT);
  T_null(self.base.snd_pending);
  T_null(self.base.snd_head);
  T_eq_str(&self.response_buf[0],
           "@X@Q@R{1413:S:F:G#So3p}@r@X@X@T@R{1413:S:F:G#So3p}@r");
}

T_TEST_CASE(TPacketChannelPush) {
  test_control self;
  initialize_test_control(&self);
  T_packet_status status = T_packet_channel_push(
      &self.base, 1, &channel_data[0], sizeof(channel_data), 0);
  T_eq_int(status, T_PACKET_NO_INPUT_CHAR_HANDLER);
  T_eq_str(&self.response_buf[0], "");

  self.base.input_char = input_char;
  self.input = "{1212:A#FhB3}{1313:A#TOcs}";
  status = T_packet_channel_push(&self.base, 2, &channel_data[0],
                                 sizeof(channel_data), 0);
  T_eq_int(status, T_PACKET_SUCCESSFUL);
  T_null(self.base.snd_pending);
  T_null(self.base.snd_head);
  T_eq_str(
      &self.response_buf[0],
      "@X@Q@R{1211:H#Mos3}@r@A@o@Q@R{1312:C:C:G#J4sp+BlubBlub#1V-z}@r@A@o");

  clear_response(&self);
  status = T_packet_channel_push(&self.base, 3, &channel_data[0],
                                 sizeof(channel_data), 2);
  T_eq_int(status, T_PACKET_TIMEOUT);
  T_null(self.base.snd_pending);
  T_null(self.base.snd_head);
  T_eq_str(&self.response_buf[0],
           "@X@Q@R{1413:C:D:G#4RFf+BlubBlub#1V-z}@r@X@X@T@R{1413:C:D:G#4RFf+"
           "BlubBlub#1V-z}@r");
}

T_TEST_CASE(TPacketChannelExchange) {
  test_control self;
  initialize_test_control(&self);
  T_packet_remove_event_handler(&self.base, &self.event);

  char receive_buf[32];
  size_t receive_size = sizeof(receive_buf);
  memset(&receive_buf[0], 0, sizeof(receive_buf));
  T_packet_status status = T_packet_channel_exchange(
      &self.base, 1, &channel_data[0], sizeof(channel_data), &receive_buf[0],
      &receive_size, 0);
  T_eq_int(status, T_PACKET_NO_INPUT_CHAR_HANDLER);
  T_eq_str(&self.response_buf[0], "");
  T_eq_sz(receive_size, 0);

  self.base.input_char = input_char;
  self.input = "{1212:A#FhB3}{1313:C:B:I#J6Gn+Zm9vIGJhcgA=#oHjZ}";
  receive_size = sizeof(receive_buf);
  memset(&receive_buf[0], 0, sizeof(receive_buf));
  status = T_packet_channel_exchange(&self.base, 1, &channel_data[0],
                                     sizeof(channel_data), &receive_buf[0],
                                     &receive_size, 0);
  T_eq_int(status, T_PACKET_SUCCESSFUL);
  T_null(self.base.snd_pending);
  T_null(self.base.snd_head);
  T_eq_str(&self.response_buf[0],
           "{1211:H#Mos3}{1312:C:B:G#pIL-+BlubBlub#1V-z}{1413:A#y8uF}");
  T_eq_sz(receive_size, 8);
  T_eq_str(&receive_buf[0], "foo bar");

  clear_response(&self);
  T_packet_append_event_handler(&self.base, &self.event, event_channel_load);
  self.input =
      "{1415:C:D:F#_jiM+b29wcwA=#xfkE}{1515:C:C:I#uBx9+Zm9vIGJhcgA=#oHjZ}";
  receive_size = 7;
  memset(&receive_buf[0], 0, sizeof(receive_buf));
  status = T_packet_channel_exchange(&self.base, 2, &channel_data[0],
                                     sizeof(channel_data), &receive_buf[0],
                                     &receive_size, 0);
  T_eq_int(status, T_PACKET_OVERFLOW);
  T_null(self.base.snd_pending);
  T_null(self.base.snd_head);
  T_eq_str(&self.response_buf[0],
           "{1513:C:C:G#mcuA+BlubBlub#1V-z}{1614:A#e961}{1714:R:15#Eohl}");
  T_eq_sz(receive_size, 0);
  T_eq_str(&receive_buf[0], "");
  T_eq_str(&self.load_buf[0], "oops");
  T_packet_remove_event_handler(&self.base, &self.event);
}

T_TEST_CASE(TPacketEventHandler) {
  test_control self;
  initialize_test_control(&self);
  T_eq_ptr(self.base.event_head, &self.event);

  T_packet_process(&self.base, -1);
  process(&self, "{1212:A#FhB3}{1313:A#TOcs}");
  T_eq_str(&self.response_buf[0], "@X@Q@R{1211:H#Mos3}@r@A@o@A");

  clear_response(&self);
  T_packet_remove_event_handler(&self.base, &self.event);
  T_null(self.base.event_head);
  T_packet_process(&self.base, -1);
  T_eq_str(&self.response_buf[0], "");

  clear_response(&self);
  T_packet_append_event_handler(&self.base, &self.event_success, event_success);
  T_packet_append_event_handler(&self.base, &self.event_continue,
                                event_continue);
  T_eq_ptr(self.base.event_head, &self.event_success);
  T_packet_process(&self.base, -1);
  T_eq_str(&self.response_buf[0], "@Z");

  T_packet_remove_event_handler(&self.base, &self.event_success);
  T_eq_ptr(self.base.event_head, &self.event_continue);

  T_packet_remove_event_handler(&self.base, &self.event_continue);
  T_null(self.base.event_head);

  clear_response(&self);
  T_packet_prepend_event_handler(&self.base, &self.event_success,
                                 event_success);
  T_packet_prepend_event_handler(&self.base, &self.event_continue,
                                 event_continue);
  T_eq_ptr(self.base.event_head, &self.event_continue);
  T_packet_process(&self.base, -1);
  T_eq_str(&self.response_buf[0], "@Y@Z");

  T_packet_remove_event_handler(&self.base, &self.event_success);
  T_eq_ptr(self.base.event_head, &self.event_continue);

  /* Double remove has no effects */
  T_packet_remove_event_handler(&self.base, &self.event_success);
  T_eq_ptr(self.base.event_head, &self.event_continue);

  T_packet_remove_event_handler(&self.base, &self.event_continue);
  T_null(self.base.event_head);
}

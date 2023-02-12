#include "Print.h"
/*
** Based on Dobot-Communication-Protocol-V1.1.5.pdf
**
** Copyright (c) 2022, Magnus Weidemann
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#include <Arduino.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "Dobot.h"

#if defined(DOBOT_DEBUG_SERIAL) && (DOBOT_DEBUG_LEVEL >= DOBOT_DEBUG_LEVEL_NONE)
static size_t _max_debug_len = 0;
static char debug_str[256];

static int debug_fmt(int lvl, char *fmt, ...)
{
  va_list ap;
  int res;
  char *lvl_str;

  DOBOT_DEBUG_SERIAL.print("Dobot.");
  switch (lvl)
  {
  case DOBOT_DEBUG_LEVEL_INFO:
    lvl_str = "INFO   ";
    break;
  case DOBOT_DEBUG_LEVEL_ERRORS:
    lvl_str = "ERROR  ";
    break;
  case DOBOT_DEBUG_LEVEL_DEBUG:
    lvl_str = "DEBUG  ";
    break;
  case DOBOT_DEBUG_LEVEL_WARNINGS:
    lvl_str = "WARNING";
    break;
  default:
    lvl_str = "??????";
    break;
  }
  DOBOT_DEBUG_SERIAL.print(lvl_str);
  DOBOT_DEBUG_SERIAL.print("# ");

  va_start(ap, fmt);
  res = vsprintf(debug_str, fmt, ap);
  DOBOT_DEBUG_SERIAL.print(debug_str);
  va_end(ap);

  if (res > _max_debug_len)
    _max_debug_len = res;

  return res;
}
#endif

#if defined(DOBOT_DEBUG_SERIAL) && (DOBOT_DEBUG_LEVEL >= DOBOT_DEBUG_LEVEL_INFO)
#define print_info(fmt, ...) debug_fmt(DOBOT_DEBUG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#else
#define print_info(fmt, ...) \
  do                         \
  {                          \
  } while (0)
#endif

#if defined(DOBOT_DEBUG_SERIAL) && (DOBOT_DEBUG_LEVEL >= DOBOT_DEBUG_LEVEL_DEBUG)
#define print_debug(fmt, ...) debug_fmt(DOBOT_DEBUG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#else
#define print_debug(fmt, ...) \
  do                          \
  {                           \
  } while (0)
#endif

#if defined(DOBOT_DEBUG_SERIAL) && (DOBOT_DEBUG_LEVEL >= DOBOT_DEBUG_LEVEL_ERRORS)
#define print_error(fmt, ...) debug_fmt(DOBOT_DEBUG_LEVEL_ERRORS, fmt, ##__VA_ARGS__)
#else
#define print_error(fmt, ...) \
  do                          \
  {                           \
  } while (0)
#endif

#if defined(DOBOT_DEBUG_SERIAL) && (DOBOT_DEBUG_LEVEL >= DOBOT_DEBUG_LEVEL_WARNINGS)
#define print_warning(fmt, ...) debug_fmt(DOBOT_DEBUG_LEVEL_WARNINGS, fmt, ##__VA_ARGS__)
#else
#define print_warning(fmt, ...) \
  do                            \
  {                             \
  } while (0)
#endif

Dobot::Dobot(HardwareSerial *serial)
{
  _serial = serial;
}

Dobot::~Dobot()
{
}

void Dobot::begin()
{
  _serial->begin(115200, SERIAL_8N1);
  _serial->flush(); // remove garbage received doing opening of port.
  print_debug("Dobot communication initialized.\n");
}

void Dobot::end()
{
  print_debug("Dobot communication deinitialize.\n");
  _serial->end();
}

Dobot::RETURN_CODE Dobot::getSerialNumber(char *sn, size_t max_length)
{
  Dobot::RETURN_CODE rc;
  print_debug("Requesting serial number ...\n");
  rc = encode_packet(&tx, ProtocolId::DeviceSN, false, false, 0, NULL);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500); // 110ms with debug message, <1ms raw
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != tx.Payload.ID)
  {
    print_error("Failed to receive response (ID %d != %d).\n", rx.Payload.ID, tx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  int sn_len = strnlen(rx.Payload.Params.SerialNumberParms.serialNumber, (rx.Len - 2));
  print_debug("length of received serial number = %d.\n", sn_len);

  // it has been seen that request has been echoed back due to bad connection
  // therefor we must check if the responce does actual has a serial number.
  if (!sn_len) {
    print_error("Missing serial number in responce.\n");
    return RETURN_CODE::GENERAL_ERROR;
  }

  int length = min(max_length, sn_len + 1);
  if (length < sn_len)
  {
    print_warning("Serial number longer that supplied buffer (%d > %d).\n", sn_len, max_length);
  }
  strlcpy(sn, rx.Payload.Params.SerialNumberParms.serialNumber, length);

  // print actual serial number received
  rx.Payload.Params.SerialNumberParms.serialNumber[sn_len] = '\0';
  print_info("Serial number received '%s'.\n", rx.Payload.Params.SerialNumberParms.serialNumber);
  return rc;
}

Dobot::RETURN_CODE Dobot::getAlarmState(int (*alarms)[8])
{
  Dobot::RETURN_CODE rc;
  print_info("Requesting alarm states ...\n");
  rc = encode_packet(&tx, ProtocolId::AlarmsState, false, false, 0, NULL);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != tx.Payload.ID)
  {
    print_error("Failed to receive response (ID %d != %d).\n", rx.Payload.ID, tx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  if (rx.Len != (2 + sizeof(Alarms_t)))
  {
    print_error("Failed to receive response (parm size %d != %d).\n", rx.Len, (2 + sizeof(Alarms_t)));
    return RETURN_CODE::PARM_SIZE;
  }

  // return alarm values
  for (int i = 0; i < 8; i++)
  {
    (*alarms)[i] = rx.Payload.Params.Alarms.alarm[i];
  }

  print_debug("Alarm states received.\n");
  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::clearAlarmState()
{
  Dobot::RETURN_CODE rc;
  print_info("Requesting clear all alarm states ...\n");
  rc = encode_packet(&tx, ProtocolId::ClearAllAlarmsState, true, false, 0, NULL);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != tx.Payload.ID)
  {
    print_error("Failed to receive response (ID %d != %d).\n", rx.Payload.ID, tx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  print_debug("Alarm states cleared.\n");
  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::setHomeParams(bool queued, float x, float y, float z, float r, uint64_t *queuedCmdIndex)
{
  Dobot::RETURN_CODE rc;
  print_info("Requesting update of home coordinates ...\n");

  HomeParams_t params;
  memset(&params, 0x00, sizeof(params));

  rc = encode_packet(&tx, ProtocolId::HOMEParams, true, queued, sizeof(params), &params);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != tx.Payload.ID)
  {
    print_error("Failed to receive response (ID %d != %d).\n", rx.Payload.ID, tx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  if (queued)
  {
    if (rx.Len != (2 + sizeof(QueuedCmdIndex_t)))
    {
      print_error("Missing QueuedCmdIndex in response (Len %d != %d).\n", rx.Len, (2 + sizeof(QueuedCmdIndex_t)));
      return RETURN_CODE::GENERAL_ERROR;
    }
    if (queuedCmdIndex != NULL)
    {
      *queuedCmdIndex = rx.Payload.Params.QueuedCmdIndex;
    }
  }

  print_debug("HOME parameters set.\n");
  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::getHomeParams(float *x, float *y, float *z, float *r)
{
  Dobot::RETURN_CODE rc;
  print_info("Requesting home parameters ...\n");
  rc = encode_packet(&tx, ProtocolId::HOMEParams, false, false, 0, NULL);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != tx.Payload.ID)
  {
    print_error("Failed to receive response (ID %d != %d).\n", rx.Payload.ID, tx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  if (rx.Len != (2 + sizeof(HomeParams_t)))
  {
    print_error("Wrong size of paramets in responce (Len %d != %d).\n", rx.Len, (2 + sizeof(HomeParams_t)));
    return RETURN_CODE::GENERAL_ERROR;
  }

  *x = rx.Payload.Params.HomeParams.x;
  *y = rx.Payload.Params.HomeParams.y;
  *z = rx.Payload.Params.HomeParams.z;
  *r = rx.Payload.Params.HomeParams.r;

  print_debug("Home parameters received.\n");
  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::setHomeCmd(bool queued, uint64_t *queuedCmdIndex)
{
  Dobot::RETURN_CODE rc;
  print_info("Sending homing request ...\n");

  HomeCmdParms_t parms;
  memset(&parms, 0x00, sizeof(parms));

  rc = encode_packet(&tx, ProtocolId::HOMECmd, true, queued, sizeof(parms), &parms);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != tx.Payload.ID)
  {
    print_error("Failed to receive response (ID %d != %d).\n", rx.Payload.ID, tx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  if (queued)
  {
    if (rx.Len != (2 + sizeof(QueuedCmdIndex_t)))
    {
      print_error("Missing QueuedCmdIndex in response (Len %d != %d).\n", rx.Len, (2 + sizeof(QueuedCmdIndex_t)));
      return RETURN_CODE::GENERAL_ERROR;
    }
    if (queuedCmdIndex != NULL)
    {
      *queuedCmdIndex = rx.Payload.Params.QueuedCmdIndex;
    }
  }

  print_debug("Homeing request send.\n");
  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::setPTPCmd(bool queued, uint8_t PTPMode, float x, float y, float z, float r, uint64_t *queuedCmdIndex)
{
  Dobot::RETURN_CODE rc;
  print_info("Requesting setPTPCmd ...\n");

  PTPCmd_t parms = {
      .ptpMode = PTPMode,
      .x = x,
      .y = y,
      .z = z,
      .r = r,
  };

  rc = encode_packet(&tx, ProtocolId::PTPCmd, true, queued, sizeof(parms), (uint8_t *)&parms);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");
  if (rx.Payload.ID != ProtocolId::PTPCmd)
  {
    print_error("Unexpected response to request %d (got %d).\n", ProtocolId::EndEffectorSuctionCup, rx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  // TODO: Return any data received in response
  print_info("request successful.\n");
  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::setEndEffectorSuctionCup(bool queued, bool isCtrlEnabled, bool isSucked, uint64_t *queuedCmdIndex)
{
  Dobot::RETURN_CODE rc;
  print_info("Requesting setEndEffectorSuctionCup ...\n");

  SuctionCupParms_t parms = {
      .isCtrlEnable = (isCtrlEnabled ? 1 : 0),
      .isSucked = (isSucked ? 1 : 0)};

  rc = encode_packet(&tx, ProtocolId::EndEffectorSuctionCup, true, queued, sizeof(parms), (uint8_t *)&parms);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }

  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != ProtocolId::EndEffectorSuctionCup)
  {
    print_error("Unexpected response to request %d (got %d).\n", ProtocolId::EndEffectorSuctionCup, rx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::getEndEffectorSuctionCup(bool *isCtrlEnabled, bool *isSucked)
{
  Dobot::RETURN_CODE rc;
  print_debug("Requesting getEndEffectorSuctionCup ...\n");
  rc = encode_packet(&tx, ProtocolId::EndEffectorSuctionCup, false, false, 0, NULL);
  if ((int)rc)
  {
    print_error("Failed to generate package (%d).\n", rc);
    return rc;
  }

  print_packet(&tx, "TX -> ");
  rc = transmit_packet(&tx);
  if ((int)rc)
  {
    print_error("Failed to transmit request (%d).\n", rc);
    return rc;
  }

  rc = receive_packet(&rx, 500);
  if ((int)rc)
  {
    print_error("Failed to receive response (%d).\n", rc);
    return rc;
  }
  print_packet(&rx, "RX <- ");

  if (rx.Payload.ID != ProtocolId::EndEffectorSuctionCup)
  {
    print_error("Unexpected response to request %d (got %d).\n", ProtocolId::EndEffectorSuctionCup, rx.Payload.ID);
    return RETURN_CODE::GENERAL_ERROR;
  }

  *isCtrlEnabled = rx.Payload.Params.SuctionCupParms.isCtrlEnable;
  *isSucked = rx.Payload.Params.SuctionCupParms.isSucked;

  print_info("isCtrlEnabled=%d, isSucked=%d.\n", *isCtrlEnabled, *isSucked);

  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::debugInfo()
{
  print_info("Used debug buffer %d of %d.\n", _max_debug_len, sizeof(debug_str));
  return RETURN_CODE::SUCCESS;
}

// ----------------------------------------------------------------------------
// Private
// ----------------------------------------------------------------------------
void Dobot::print_packet(Packet *packet, const char *prefix)
{
#if defined(DOBOT_DEBUG_SERIAL) && (DOBOT_DEBUG_LEVEL >= DOBOT_DEBUG_LEVEL_INFO)
  print_info(prefix);
  DOBOT_DEBUG_SERIAL.print("|0x");
  for (int idx = 0; idx < sizeof(packet->Header); idx++)
  {
    if (((uint8_t *)(&packet->Header))[idx] < 0x10)
      DOBOT_DEBUG_SERIAL.print('0');
    DOBOT_DEBUG_SERIAL.print(((uint8_t *)(&packet->Header))[idx], HEX);
  }
  DOBOT_DEBUG_SERIAL.print("|");
  DOBOT_DEBUG_SERIAL.print(packet->Len);
  DOBOT_DEBUG_SERIAL.print("|");
  DOBOT_DEBUG_SERIAL.print(packet->Payload.ID);
  DOBOT_DEBUG_SERIAL.print("|");
  DOBOT_DEBUG_SERIAL.print((packet->Payload.Ctrl.rw ? "write" : "read"));
  DOBOT_DEBUG_SERIAL.print("|");
  DOBOT_DEBUG_SERIAL.print((packet->Payload.Ctrl.isQueued ? "Queued" : "Unqueued"));
  DOBOT_DEBUG_SERIAL.print("|");
  if (packet->Len > 2)
  {
    for (int idx = 0; idx < (packet->Len - 2); idx++)
    {
      if (((uint8_t *)(&packet->Payload.Params))[idx] < 0x10)
        DOBOT_DEBUG_SERIAL.print('0');
      DOBOT_DEBUG_SERIAL.print(((uint8_t *)(&packet->Payload.Params))[idx], HEX);
    }
  }
  else
  {
    DOBOT_DEBUG_SERIAL.print("EMPTY");
  }
  DOBOT_DEBUG_SERIAL.print("|");
  if (((uint8_t *)(&packet->Payload))[packet->Len] < 0x10)
    DOBOT_DEBUG_SERIAL.print('0');
  DOBOT_DEBUG_SERIAL.print(((uint8_t *)(&packet->Payload))[packet->Len], HEX);
  DOBOT_DEBUG_SERIAL.println("|");
#endif
}

// At the receiving end, the method of verifying whether a frame of data is correct as follows:
// Step 1) Add all the content of the Payload byte by byte (8 bits) to get result A.
// Step 2) Add result A and the check byte. If the result is 0, then the checksum is correct.
Dobot::RETURN_CODE Dobot::encode_packet(Packet *packet, ProtocolId id, bool rw, bool isQueued, uint8_t parms_length, void *parms)
{
  if (parms_length > sizeof(packet->Payload.Params))
  {
    print_error("Packet to large to transmit (%d > %d).\n", parms_length, sizeof(packet->Payload.Params));
    return RETURN_CODE::SIZE_ERROR;
  }
  memset(packet, 0, sizeof(Packet));

  uint8_t *start = (uint8_t *)packet;
  uint8_t *pos = start;

  // Header
  *pos++ = DOBOT_SYNC_BYTE;
  *pos++ = DOBOT_SYNC_BYTE;
  // Len
  *pos++ = sizeof(packet->Payload.ID) + sizeof(packet->Payload.Ctrl) + parms_length;
  // Payload

  uint8_t R = 0x00; // used to calculate checksum

  R += *pos++ = id;
  R += *pos++ = (rw ? 1 : 0) << 1 | (isQueued ? 1 : 0);

  // Payload - parms
  for (int i = 0; i < parms_length; i++)
  {
    R += *pos++ = ((uint8_t*) parms)[i];
  }

  // Add checksum
  // Dobot-Communication-Protocol-V1.1.5.pdf
  // 1.2.2 Checksum Calculation:
  //
  // In Dobot Magician communication protocol, the checksum at the ending side is calculated as follows.
  // Step 1) Add all the content of the Payload byte by byte (8 bits) to get result R (8 bits).
  // Step 2) Get the 2's complements of the result R (8 bits), and put it into check byte.
  //
  // NOTE
  //  2's complement. For an N-bit number, the 2’s complement is equal to 2 ^ N minus the number.
  //  In this protocol, assuming the result R is 0x0A, and the 2’s complement,
  //  i.e., the result of the above checking is equal to (2 ^ 8 - 0x0A) = (256 - 10) = 246 = 0xF6.
  //
  // this is equal to "crc = 0 - R" as "crc + R" must give 0 to be valid at reception.

  uint8_t crc = (0x00 - R) & 0xFF;
  *pos++ = crc;

  return RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::transmit_packet(Packet *packet)
{
  int length = sizeof(packet->Header) + sizeof(packet->Len) + packet->Len + sizeof(packet->Checksum);
  print_debug("TX size %d.\n", length);
  _serial->flush();
  _serial->write((uint8_t *)packet, length);
  return Dobot::RETURN_CODE::SUCCESS;
}

Dobot::RETURN_CODE Dobot::receive_packet(Packet *packet, long timeout)
{
  unsigned long start_time = millis();
  uint8_t received = 0;
  uint8_t remaining = 0xFF;
  uint8_t *buffer = (uint8_t *)packet;
  uint8_t A = 0; // crc check: Add result A and the check byte, If the result is 0, then checksum is correct.
  unsigned long alive = start_time + 1000;
  unsigned long processing_start = 0;

  print_debug("rx: timeout = %d.\n", timeout);
  while ((timeout == -1) || ((millis() - start_time) < timeout))
  {
    // add time used on processing and printing debug to timeout
    if (processing_start)
    {
      timeout += millis() - processing_start;
      processing_start = 0;
    }

    if (alive < millis())
    {
      print_debug("time left %lu ms.\n", (start_time + timeout) - millis());
      print_debug("alive %lu, now %lu.\n", alive, millis());
      alive += 1000;
    }

    if (_serial->available())
    {
      processing_start = millis(); // remember when receive processing started

      uint8_t b = _serial->read();
      buffer[received++] = b;

      // print_debug("rx[%02d] = 0x%02X\n", received, b);

      // Wait for sync bytes
      if (received <= sizeof(packet->Header))
      {
        if (b != DOBOT_SYNC_BYTE)
        {
          print_error("Invalid sync byte (0x%X) at %d, resetting receive index.\n", received, b);
          received = 0;
          continue;
        }
        // print_debug("sync = %X\n", b);
      }
      else if (received <= (sizeof(packet->Header) + sizeof(packet->Len)))
      {
        if (b == DOBOT_SYNC_BYTE)
        {
          print_error("Additional sync byte (0x%X) instead of length, ignoring.\n", b);
          received--;
          continue;
        }
        else if (b < (sizeof(packet->Payload.ID) + sizeof(packet->Payload.Ctrl)))
        {
          print_error("Invalid packet: length (%d) less than minimum size (%d).", b, (sizeof(packet->Payload.ID) + sizeof(packet->Payload.Ctrl)));
          received = 0;
          continue;
        }
        else if (b > sizeof(packet->Payload))
        {
          print_error("Invalid packet: length (%d) more than maximum size (%d).", b, sizeof(packet->Payload));
          received = 0;
          continue;
        }

        // valid length received
        print_debug("received length: %d\n", b);
        A = 0; // reset crc check value
        remaining = b;
      }
      // receiving payload
      else if (remaining > 0)
      {
        A += b;
        remaining--;
        // print_debug("payload (remaining %d).\n", remaining);
      }
      // payload received
      else
      {
        print_debug("packet received.\n");
        if (((b + A) & 0xFF) != 0)
        {
          print_error("Checksum error (%X + %X => %d != 0).\n", b, A, ((b + A) & 0xFF));
          received = 0;
          continue;
        }
        buffer[received] = b;
        print_debug("packet successfully received in %lu ms size %d bytes.\n", (millis() - start_time), received);
        return RETURN_CODE::SUCCESS;
      }
    }
  }
  return Dobot::RETURN_CODE::TIMEOUT;
}

/*
 * Fadecandy Firmware
 *
 * Copyright (c) 2013 Micah Elizabeth Scott
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <math.h>
#include "arm_math.h"
#include "fc_defs.h"
#include "HardwareSerial.h"
#include "wiring.h"

#include "FlexCAN.h"

#include "rio/talon_rio.h"

FlexCAN can(1000000);
static CAN_message_t rxmsg;

// Reserved RAM area for signalling entry to bootloader
extern uint32_t boot_token;

static void dfu_reboot()
{
    // Reboot to the Fadecandy Bootloader
    boot_token = 0x74624346;

    // Short delay to allow the host to receive the response to DFU_DETACH.
    uint32_t deadline = millis() + 10;
    while (millis() < deadline) {
        watchdog_refresh();
    }

    // Detach from USB, and use the watchdog to time out a 10ms USB disconnect.
    __disable_irq();
    USB0_CONTROL = 0;
    while (1);
}



extern "C" int main()
{
    pinMode(LED_BUILTIN, OUTPUT);

    // Announce firmware version
    serial_begin(BAUD2DIV(115200));
    serial_print("Fadecandy v" DEVICE_VER_STRING "\r\n");

    // Application main loop
    while (true) {
        watchdog_refresh();
        if (can.read(rxmsg)) {
            if (rxmsg.id == PACKET_DISABLE)
                serial_print("disabled");
            else if (rxmsg.id == PACKET_ENABLE)
                serial_print("enabled");
            else {
                int id = rxmsg.id & PART_DEVID;
                if (id == 0 || id == 1 || id == 2 || id == 3) {
                    int type = rxmsg.id & PART_PACKTYPE;
                    switch (type) {
                        case PACKET_SPEEDCHG: {
                            float v = packetSpeedchgSpeed(rxmsg.buf);
                            float f, b;

                            if (v == 0)
                                f = b = 1; // do braking when neutral
                            else {
                                f = max(v, 0);
                                b = -min(v, 0);
                            }

                            analogWrite(motorNumToPin(id,true), f);
                            analogWrite(motorNumToPin(id,false), b);
                            break;
                        }
                    }
                }
            }
        }
    }

    // Reboot into DFU bootloader
    dfu_reboot();
}

static int motorNumToPin(int num, bool forward){
    switch (num) {
        case 0:
            return forward?64:61;
        case 1:
            return forward?62:63;
        case 2:
            return forward?44:45;
        case 3:
            return forward?41:26;
    }
}

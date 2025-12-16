/* PC serial tool matched to your AVR code:
   - send 0 -> AVR replies 1 byte: current rpm
   - send 1..120 -> set rpm_ref, PI runs; AVR streams rpm periodically
*/

#include "serialport.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>

static void drain_stream(int sp, int ms_total)
{
    // Read and print any streamed RPM bytes for ms_total milliseconds
    unsigned char b;
    int loops = ms_total / 10;
    for (int i = 0; i < loops; i++) {
        // Try to read a byte (non-blocking depends on serialport.c settings)
        if (read(sp, &b, 1) == 1) {
            printf("RPM: %u\n", (unsigned)b);
            fflush(stdout);
        }
        usleep(10000); // 10 ms
    }
}

int main(void)
{
    int sp = serial_init("/dev/ttyS0", 0); // change to /dev/ttyS0 if needed
    if (sp == 0) {
        printf("Error! Serial port could not be opened.\n");
        return 1;
    }
    printf("Serial port open with identifier %d\n", sp);

    while (1) {
        unsigned int tmp;
        unsigned char cmd;
        unsigned char rpm;

        printf("\nEnter command (0=request RPM, 1..120=set RPM, 255=quit): ");
        if (scanf("%u", &tmp) != 1) {
            printf("Bad input, exiting.\n");
            break;
        }
        if (tmp == 255) break;

        // Match AVR clamp behavior for setpoints
        if (tmp > 255) tmp = 255;
        if (tmp > 120 && tmp != 0) tmp = 120;

        cmd = (unsigned char)tmp;

        // Send command byte
        if (write(sp, &cmd, 1) != 1) {
            printf("Write failed.\n");
            continue;
        }

        // If cmd==0, AVR replies immediately with 1 byte rpm_acc
        if (cmd == 0) {
            // Give AVR a moment to compute & respond
            usleep(20000);

            if (read(sp, &rpm, 1) == 1) {
                printf("AVR RPM reply: %u\n", (unsigned)rpm);
            } else {
                printf("No reply received (cmd=0)\n");
            }
        } else {
            printf("Setpoint sent: %u RPM\n", (unsigned)cmd);
            printf("Reading stream for ~2 seconds (Ctrl+C to stop program)...\n");

            // Your AVR sends rpm every ~15 PI ticks.
            // With Ts ~ 0.0655 s and "every 15 ticks" => ~1 second between bytes.
            // Read for 2 seconds so you likely see at least 1-2 samples.
            drain_stream(sp, 2000);
        }
    }

    serial_cleanup(sp);
    return 0;
}
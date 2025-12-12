#include "serialport.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

void loopBackTest(void);
void increaseSpeed(void);
void decreaseSpeed(void);

/*Declaration of variables*/
int sp;
uint8_t cin;
uint8_t cout;

int main(void) {
  /*Initialise serial port */
  sp = serial_init("/dev/ttyS0", 0);
  if (sp == 0) {
    printf("Error! Serial port could not be opened.\n");
    return -1;
  } else {
    printf("Serial port open with identifier %d \n", sp);
  }

  int selection;

  while (1) {
    printf("-----------------------------------\n");
    printf("AVR Control Menu (v7.c Logic):\n");
    printf(" 1 - Increase Speed (+5 PWM)\n");
    printf(" 2 - Decrease Speed (-5 PWM)\n");
    printf(" 0 - Loopback Test (Send 0-255)\n");
    printf("-----------------------------------\n");
    printf("Enter selection: ");

    if (scanf("%d", &selection) != 1) {
      printf("Invalid input. Please enter a number.\n");
      while (getchar() != '\n')
        ; // Clear buffer
      continue;
    }

    switch (selection) {
    case 0:
      loopBackTest();
      break;
    case 1:
      increaseSpeed();
      break;
    case 2:
      decreaseSpeed();
      break;
    default:
      printf("Invalid selection.\n");
      break;
    }
  }
  return 0;
}

void increaseSpeed(void) {
  uint8_t cmd = 1;
  uint8_t reply;

  printf("Sending '1' to Increase Speed...\n");
  write(sp, &cmd, 1);

  usleep(100000); // Wait 100ms

  int n = read(sp, &reply, 1);
  if (n > 0) {
    printf("AVR Reply (Debug Counter/RPM): %d\n", reply);
  } else {
    printf("Timeout: No reply from AVR.\n");
  }
}

void decreaseSpeed(void) {
  uint8_t cmd = 0;
  uint8_t reply;

  printf("Sending '0' to Decrease Speed...\n");
  write(sp, &cmd, 1);

  usleep(100000); // Wait 100ms

  int n = read(sp, &reply, 1);
  if (n > 0) {
    printf("AVR Reply (Debug Counter/RPM): %d\n", reply);
  } else {
    printf("Timeout: No reply from AVR.\n");
  }
}

void loopBackTest(void) {
  int val;
  printf("Enter byte to send (0-255): ");
  if (scanf("%d", &val) == 1) {
    cout = (uint8_t)val;
    write(sp, &cout, 1);

    usleep(100000);

    int n = read(sp, &cin, 1);
    if (n > 0) {
      printf("Sent: %d, Received: %d \n", cout, cin);
    } else {
      printf("Timeout.\n");
    }
  }
}

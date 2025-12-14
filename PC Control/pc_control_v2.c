#include "serialport.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

void loopBackTest(void);
void setRPM(void);
void setPWM(void);
void getRPM(void);

/*Declaration of variables*/
int sp;
uint8_t cin;
uint8_t cout;
int temp_in;

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
    printf("Enter your function: \n 0-Loop back test \n 1-Set RPM \n 2-Set PWM "
           "\n 3-Get RPM \n");
    printf("-----------------------------------\n");

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
      setRPM();
      break;
    case 2:
      setPWM();
      break;
    case 3:
      getRPM();
      break;
    default:
      printf("Invalid selection.\n");
      break;
    }
  }
  return 0;
}

void setRPM(void) {
  int rpm_val;
  uint8_t rpmout, rpmin;
  printf("Set your RPM: \n");
  if (scanf("%d", &rpm_val) == 1) {
    rpmout = (uint8_t)rpm_val;
    write(sp, &rpmout, 1);

    usleep(100000); // Wait 100ms

    int n = read(sp, &rpmin, 1);
    if (n > 0) {
      printf("Output: %d, Input: %d \n", rpmout, rpmin);
    } else {
      printf("Timeout: No reply from AVR.\n");
    }
  }
}

void setPWM(void) {
  int pwm_val;
  uint8_t pwmout, pwmin;
  printf("Set your PWM: \n");
  if (scanf("%d", &pwm_val) == 1) {
    pwmout = (uint8_t)pwm_val;
    write(sp, &pwmout, 1);

    usleep(100000); // Wait 100ms

    int n = read(sp, &pwmin, 1);
    if (n > 0) {
      printf("Output: %d, Input: %d \n", pwmout, pwmin);
    } else {
      printf("Timeout: No reply from AVR.\n");
    }
  }
}

void getRPM(void) {
  // This function seems to expect the AVR to send 2 bytes?
  // Based on dennis code, it reads twice.
  // But our v7.c only replies once per command.
  // We will send a dummy command (like 0?) to trigger a reply if needed,
  // OR just read if the AVR streams data.
  // Assuming we need to send a command to get RPM.
  // For now, let's just send '0' (which might mean "get status" in some
  // protocols) or maybe the user just wants to read what's there.

  // In v7.c, AVR replies ONLY when it receives a byte.
  // So we must send something. Let's send 0.

  uint8_t cmd = 0;
  uint8_t val;
  write(sp, &cmd, 1);

  usleep(100000);

  int n = read(sp, &val, 1);
  if (n > 0) {
    printf("Current Value: %d \n", val);
  } else {
    printf("Timeout.\n");
  }
}

void loopBackTest(void) {
  int val;
  printf("Please enter your desired byte (0-255):");
  if (scanf("%d", &val) == 1) {
    cout = (uint8_t)val;
    write(sp, &cout, 1);

    usleep(100000);

    int n = read(sp, &cin, 1);
    if (n > 0) {
      printf("Output: %d, Input: %d \n", cout, cin);
    } else {
      printf("Timeout.\n");
    }
  }
}

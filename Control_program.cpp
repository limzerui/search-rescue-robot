#include <errno.h>
#include <ncurses.h>
#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int main(int argc, char *argv[]) {
  int ch = 2;
  initscr();
  cbreak();
  noecho();

  int fd;
  if ((fd = serialOpen("/dev/ttyS0", 115200)) < 0) {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    return 1;
  }

  if (wiringPiSetup() == -1) {
    fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
    return 1;
  }

  timeout(100);

  while (true) {
    char c = getch();

    switch (c) {
    // Coarse movement
    case 'w':
      ch = 1;
      break; // forward (100ms)
    case 'a':
      ch = 3;
      break; // left (100ms)
    case 's':
      ch = 5;
      break; // back (100ms)
    case 'd':
      ch = 4;
      break; // right (100ms)

    // Claw and medpack
    case 'q':
      ch = 6;
      break; // open claw
    case 'e':
      ch = 7;
      break; // close claw
    case 'z':
      ch = 0;
      break; // hold medpack
    case 'c':
      ch = 8;
      break; // deposit medpack

    // Fine movement
    case 'j':
      ch = 9;
      break; // left (25ms)
    case 'l':
      ch = 10;
      break; // right (25ms)
    case 'i':
      ch = 11;
      break; // forward (25ms)
    case 'k':
      ch = 12;
      break; // back (25ms)

    case '=': // quit
      endwin();
      return 0;

    default:
      ch = 2; // stop
      break;
    }

    serialPutchar(fd, (uint8_t)ch);

    while (serialDataAvail(fd)) {
      char received = serialGetchar(fd);
      if (received == 0)
        printw("Red  \r");
      else if (received == 1)
        printw("Green\r");
      else
        printw("Black\r");
    }

    refresh();
  }

  endwin();
  return 0;
}
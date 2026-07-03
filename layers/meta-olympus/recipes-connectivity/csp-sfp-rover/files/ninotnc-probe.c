#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>

int main(int argc, char *argv[]) {
    const char *device = "/dev/ttyAMA0";
    if (argc > 1)
        device = argv[1];

    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "ninotnc-probe: cannot open %s: %s\n", device, strerror(errno));
        return 1;
    }

    struct termios tio;
    tcgetattr(fd, &tio);
    cfsetospeed(&tio, B57600);
    cfsetispeed(&tio, B57600);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tio.c_cflag |= CS8;
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &tio);

    /* Flush stale data */
    tcflush(fd, TCIOFLUSH);

    /* Send KISS GETALL command (0xC0 FEND, 0xFF broadcast, 0x0B cmd, 0xC0 FEND) */
    unsigned char probe[] = {0xC0, 0xFF, 0x0B, 0xC0};
    write(fd, probe, sizeof(probe));

    /* Wait for response up to 3 s */
    struct pollfd pfd = {.fd = fd, .events = POLLIN};
    int ret = poll(&pfd, 1, 3000);

    close(fd);

    if (ret > 0) {
        return 0;  /* TNC responded */
    }

    return 1;      /* no response – TNC not present */
}

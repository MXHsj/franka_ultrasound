//============================================================================
// Name        : read flight stick
// Author      : Xihan Ma
// Version     :
// Copyright   :
// Description :
//============================================================================

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <inttypes.h>

int main(int argc, char **argv)
{
    struct termios tio;
    struct termios stdio;
    struct termios old_stdio;
    int tty_fd;

    unsigned char c = 'D';
    tcgetattr(STDOUT_FILENO, &old_stdio);

    memset(&stdio, 0, sizeof(stdio));

    stdio.c_lflag |= ECHO;
    stdio.c_iflag |= ICRNL;
    stdio.c_oflag |= (OPOST | ONLCR);
    stdio.c_cc[VMIN] = 1;
    stdio.c_cc[VTIME] = 0;

    tcsetattr(STDOUT_FILENO, TCSANOW, &stdio);
    tcsetattr(STDOUT_FILENO, TCSAFLUSH, &stdio);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // make the reads non-blocking

    tty_fd = open(argv[1], O_RDWR | O_NONBLOCK);
    tcgetattr(tty_fd, &tio);

    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
    tio.c_cflag &= ~PARENB;              // No Parity
    tio.c_cflag &= ~CSTOPB;              // 1 stop bit
    tio.c_lflag = 0;

    tio.c_cc[VMIN] = 19;
    tio.c_cc[VTIME] = 5;

    cfsetospeed(&tio, B9600);
    cfsetispeed(&tio, B9600);

    tcsetattr(tty_fd, TCSAFLUSH, &tio);

    //read two analog inputs from Arduino
    char rxString[40];
    int ind, atd1, atd2;
    float voltage1, voltage2;
    char inchar = 'b';

    while (c != 'q')
    {
        if (read(tty_fd, &inchar, 1) > 0)
        {
            if (inchar == '1') // check start of the first stream
            {
                rxString[0] = inchar;
                ind = 1;
                // store array stream to rxString
                while (inchar != '\n') // check end of the stream
                {
                    if (read(tty_fd, &inchar, 1) > 0)
                    {
                        if (ind < 40) // if exceeding array size
                        {
                            rxString[ind] = inchar;
                            ind++;
                        }
                        else
                        {
                            inchar = '\n';
                        }
                    }
                }
                // filter rxStream
                if (ind < 40)
                {
                    sscanf(rxString, "%*s %d\t%f", &atd1, &voltage1); // sscanf(packet,"format of the packet",wanted_data1,wanted_data2);
                    printf("P1_ATD: %d\t P1_VOLTAGE: %.2f\t", atd1, voltage1);
                }
            }

            if (inchar == '2') // check start of the second stream
            {
                rxString[0] = inchar;
                ind = 1;
                // store array stream to rxString
                while (inchar != '\n') // check end of the stream
                {
                    if (read(tty_fd, &inchar, 1) > 0)
                    {
                        if (ind < 40) // if exceeding array size
                        {
                            rxString[ind] = inchar;
                            ind++;
                        }
                        else
                        {
                            inchar = '\n';
                        }
                    }
                }
                // filter rxStream
                if (ind < 40)
                {
                    sscanf(rxString, "%*s %d\t%f", &atd2, &voltage2);
                    printf("P2_ATD: %d\t P2_VOLTAGE: %.2f\n", atd2, voltage2);
                }
            }
        }
        read(STDIN_FILENO, &c, 1); // read from keyboard
    }

    write(STDOUT_FILENO, "\n", 1);
    close(tty_fd);
    tcsetattr(STDOUT_FILENO, TCSANOW, &old_stdio);

    return EXIT_SUCCESS;
}

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

#define BNOPORT 1153
#define BUFSIZE 2048


int main(int argc, char **argv)
{
		struct sockaddr_in myaddrBNO;      /* our address */
        struct sockaddr_in remaddrBNO;     /* remote address */
        socklen_t addrlenBNO = sizeof(remaddrBNO);            /* length of addresses */
        int recvlenBNO;                    /* # bytes received */
        int fdBNO;                         /* our socket */
        unsigned char bufBNO[BUFSIZE];     /* receive buffer */

        /* create a UDP socket */

        if ((fdBNO = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
                perror("cannot create socket\n");
                return 0;
        }

        /* bind the socket to any valid IP address and a specific port */

        memset((char *)&myaddrBNO, 0, sizeof(myaddrBNO));
        myaddrBNO.sin_family = AF_INET;
        myaddrBNO.sin_addr.s_addr = htonl(INADDR_ANY);
        myaddrBNO.sin_port = htons(BNOPORT);

        if (bind(fdBNO, (struct sockaddr *)&myaddrBNO, sizeof(myaddrBNO)) < 0) {
                perror("bind failed");
                return 0;
        }
        
        int heading, roll;

        /* now loop, receiving data and printing what we received */
        for (;;) {
                printf("waiting on port %d\n", BNOPORT);
                recvlenBNO = recvfrom(fdBNO, bufBNO, BUFSIZE, 0, (struct sockaddr *)&remaddrBNO, &addrlenBNO);
                printf("received %d bytes\n", recvlenBNO);
                if (recvlenBNO > 0) {
                        //buf[recvlen] = 0;
                        heading = (bufBNO[0] << 8) + bufBNO[1];
                        roll = bufBNO[2];
                        printf("received message: \"%d %d\"\n", heading, roll);
                }
        }
        /* never exits */
}

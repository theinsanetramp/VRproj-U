#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

#define SERVICE_PORT  1153
#define RECEIVEBUFLEN 1024

using namespace std;

struct sockaddr_in myaddr, remaddr;
int fd, recvlen;
socklen_t slen=sizeof(remaddr);
char receiveBuf[RECEIVEBUFLEN];

int main()
{
	/* create a socket */
  if ((fd=socket(AF_INET, SOCK_DGRAM, 0))==-1)
    printf("socket created\n");

  /* bind it to all local addresses and pick any port number */

  memset((char *)&myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(0);

  if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
    perror("bind failed");
    return 0;
  }       
	printf("waiting on port %d\n", SERVICE_PORT);
	recvlen = recvfrom(fd, receiveBuf, RECEIVEBUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
	cout << receiveBuf << endl;
	return 0;
}

#include "gps_definition.h"

int openSocket(void)
{	
	int sockfd;
	struct sockaddr_in serv_addr;

   	sockfd = socket(AF_INET, SOCK_STREAM, 0);
     
     	if (sockfd < 0)
	{ 
        	printf("ERROR opening socket\n");
		return -1;
     	}
	memset((char *) &serv_addr, 0, sizeof(serv_addr));
     
     	serv_addr.sin_family = AF_INET;
     	serv_addr.sin_addr.s_addr = INADDR_ANY;
     	serv_addr.sin_port = htons(HOSTNUM);

     	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{ 
              	printf("ERROR on binding\n");
		return -1;
	}
	return sockfd;
}

int listenSocket(int sockfd)
{
	int n;
	n = listen(sockfd, 5);
	if(n < 0)
	{
		printf("ERROR on listenning\n");
		return -1;
	}
}

int acceptNconnectClient(int sockfd)
{
	int conn_sock;
	socklen_t clilen;
	struct sockaddr_in cli_addr;
	memset((char *) &cli_addr, 0, sizeof(cli_addr));
	clilen = sizeof(cli_addr);
	printf("Connect to client...\n");
	conn_sock = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if(conn_sock < 0)
	{
		printf("ERROR on connecting. Cannot accept\n");
		return -1;
	}	
	return conn_sock;
}

void closeSocket(int sockfd)
{
	close(sockfd);
}

void setTimeout(unsigned int second, unsigned int microsecond)
{	
	struct timeval _timeout;
	_timeout.tv_sec = second;
	_timeout.tv_usec = microsecond;

}


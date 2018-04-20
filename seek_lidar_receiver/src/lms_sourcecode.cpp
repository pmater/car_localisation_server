#include "lms_definition.h"

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
	conn_sock = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if(conn_sock < 0)
	{
		printf("ERROR on connecting. Cannot accept\n");
		return -1;
	}	
	return conn_sock;
}

uint8_t * readSocket(int sockfd)
{
	uint8_t searchbuffer[2];
	uint8_t byte_length[2];
	uint8_t readbuffer[726];
	int n = 0;
	
	uint8_t *dataPointer;
	static uint8_t storebuffer[730];
	uint16_t length;

	memset(searchbuffer, 0, sizeof(readbuffer));
	memset(byte_length, 0, sizeof(byte_length));
	memset(readbuffer, 0, sizeof(readbuffer));
	memset(storebuffer, 0, sizeof(storebuffer));
	
	dataPointer = &storebuffer[0];
	printf("Finding start byte...\n");

	while((searchbuffer[0] != 0x02) || (searchbuffer[1] != 0x80))
	{
		searchbuffer[0] = searchbuffer[1];
		n = read(sockfd, &searchbuffer[1], 1);
	}
	if(n < 0)
	{
		printf("\nError. Cannot finding start byte\n");
	}
	n = read(sockfd, byte_length, 2);
	length = byte_length[0] | byte_length[1] << 8;
	n = read(sockfd, readbuffer, (int)length);
	memcpy(dataPointer, searchbuffer, sizeof(searchbuffer));
	memcpy(dataPointer+2, byte_length, sizeof(byte_length));
	memcpy(dataPointer+4, readbuffer, sizeof(readbuffer));
	if(n < 0)
	{
		printf("\nError on reading data\n");
	}
	return storebuffer;
}

uint8_t * readTimetag(int sockfd)
{
	static uint8_t _timetag[SIZE_OF_TIMETAG];
	int n = 0;
	memset(_timetag, 0, sizeof(_timetag));
	n = read(sockfd, _timetag, SIZE_OF_TIMETAG);

	if(n < 0) printf("Failed to read time tag\n");

	return _timetag;			
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


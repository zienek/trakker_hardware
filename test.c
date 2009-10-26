#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <linux/socket.h>
#include <netinet/in.h>

#define BUF_LEN (sizeof(unsigned short) * 512 * 4)

unsigned short buffer[512 * 4];

int main()
{
	int s;
	unsigned long opt;
	struct sockaddr_in addr;
	int client;

	s = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP);

	if (s==-1)
	{
		printf("socket FAILED...\n");
		return 0;
	}

	opt=1;
	if (setsockopt(s,6,1,&opt,sizeof(unsigned long)))
	{
		printf("setsockopt FAILED...\n");
		return 0;
	}

	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(40000);

	if (bind(s,(struct sockaddr *)&addr,sizeof(struct sockaddr_in)))
	{
		printf("bind FAILED...\n");
		return 0;
	}
	
	if (listen(s,5))
	{
		printf("listen FAILED...\n");
		return 0;
	}

	while ((client = accept(s,NULL,NULL)) >= 0)
	{
		unsigned long cmd;
		int bytes_received;
		int adc;

		printf("Client connected...\n");

		adc = open("/dev/at91adc0",O_RDONLY);

		if (adc >= 0)
		{

		}
		else
		{
			printf("Error opening /dev/at91adc0 - Client disconnected...\n");
			close(client);
			continue;
		}

		while ((bytes_received = recv(client,&cmd,sizeof(unsigned long),0)) > 0)
		{
			int read_len;
			
			printf("Capture command...\n");

			ioctl(adc,0); // startuje przetwarzanie w driverze
			// ioctl(adc, 1 , 1 ) ; // TODO  TODO
			while ((read_len = read(adc,buffer,BUF_LEN))==0);

			if (read_len == BUF_LEN)
			{
				int bytes_to_send = BUF_LEN;

				while (bytes_to_send)
				{
					int sent = send(client,buffer,bytes_to_send,0);

					if (sent > 0)
					{
						bytes_to_send -= sent;
					}
					else
					{
						printf("send FAILED...\n");
						break;
					}
				}

				if (bytes_to_send)
				{
					break;
				}
			}
			else
			{
				printf("readlen FAILED (read_len==%d)...\n",read_len);
				break;
			}
		}

		printf("Client disconnected...\n");

		close(client);
		close(adc);
	}

	printf("Ending program...\n");

	return 0;
}

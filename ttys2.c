#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <pthread.h>

int rs422_fd_s1;
int k;
static struct termios option_old;
//���ڳ�ʼ��
int uart_init(char *devname)
{
	int uart_fd;
	struct termios option_new;
	
	uart_fd = open(devname, O_RDWR);
	if(uart_fd < 0)
	{
		perror("open_dev");
		_exit(-1);
	}
	tcgetattr(uart_fd, &option_old);		//���洮������
	tcgetattr(uart_fd, &option_new);

	cfsetispeed(&option_new, B115200);		//������Ϊ9600
	cfsetospeed(&option_new, B115200);		//������Ϊ9600
	option_new.c_cflag &= ~CSIZE;			//��������λʱ�ȹر���ʷ����
	option_new.c_cflag |= CS8;				//����λΪ8λ
	option_new.c_cflag &= ~CSTOPB;			//1λֹͣλ
	option_new.c_cflag &= ~PARENB;			//����żУ��λ
	option_new.c_lflag &= ~(ICANON);		//�Ǳ�׼ģʽ
	option_new.c_lflag &= ~ECHO;			//�ػ��ԣ���ʹ��GPRSģ��ʱ��ػ���
	option_new.c_iflag &= ~(BRKINT | ICRNL|INPCK | ISTRIP | IXON);
      	option_new.c_oflag &= ~OPOST;
      	option_new.c_cflag |= CLOCAL | CREAD;
      	option_new.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	// option_new.c_lflag |= ECHO;				//������
	tcsetattr(uart_fd, TCSANOW, &option_new);
	return uart_fd;
}

//���ڷ���ʼ��
void uart_uninit(int uart_fd)
{
	/*��ԭ��������*/
	tcsetattr(uart_fd, TCSANOW, &option_old);
	
	/*�رմ���*/
	close(uart_fd);
}
//���߳̽�������
void *recv_fun(void *arg)
{
	char recv_buf[1024];
	memset(recv_buf,0,sizeof(recv_buf));
	while(read(rs422_fd_s1,recv_buf,sizeof(recv_buf))>0)
		{
			printf("recv:%s\n",recv_buf);
			memset(recv_buf,0,sizeof(recv_buf));
		}
}

int main()
{
	char send_buf[1024];
	int i;
	memset(send_buf,0x5a,sizeof(send_buf));
	pthread_t pth_recv;
	rs422_fd_s1=uart_init("/dev/ttyTHS1");
	pthread_create(&pth_recv,NULL,recv_fun,NULL);
	pthread_detach(pth_recv);
	while(1)
		{
			gets(send_buf);
			printf("send:%s\n",send_buf);

			write(rs422_fd_s1, send_buf, strlen(send_buf));
			memset(send_buf,0,sizeof(send_buf));
			
			sleep(1);
		}
	uart_uninit(rs422_fd_s1);
	return 0;
		
}


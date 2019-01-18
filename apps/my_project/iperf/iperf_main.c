/**
* iperf-liked network performance tool
*
*/


#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/fs.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>


#include <nuttx/timers/drv_hrt.h>
#include <nuttx/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include "netutils/netlib.h"
// #include "netdb.h"

#define IPERF_PORT          5001
#define IPERF_BUFSZ         (1 *1024)

#define IPERF_MODE_STOP     0
#define IPERF_MODE_SERVER   1
#define IPERF_MODE_CLIENT   2

#define RT_TICK_PER_SECOND  1000

#define CLIENT_TYPE_UDP     1
#define CLIENT_TYPE_TCP     2  
#define CLIENT_TYPE         CLIENT_TYPE_UDP

typedef struct 
{
    int mode;
    char *host;
    int port;
}IPERF_PARAM;

static IPERF_PARAM param = {IPERF_MODE_STOP, NULL, IPERF_PORT};

static uint8_t buff[IPERF_BUFSZ];

#if (CLIENT_TYPE == CLIENT_TYPE_TCP )

static int iperf_client(int argc, FAR char *argv[])
{
    int i;
    int sockfd;
    int ret;

    hrt_abstime tick1, tick2;
    static struct sockaddr_in server;

    ssize_t cnt_tick = 0;
    ssize_t total_bytes = 0;
    socklen_t addrlen;
    uint8_t connect_cnt = 0;


    for (i = 0; i < IPERF_BUFSZ; i ++)
        buff[i] = i & 0xff;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd < 0) 
    {
        printf("create socket failed!\n");
        exit(1);
    }

    memset(&server,0,sizeof(server));

    server.sin_family = AF_INET;
    server.sin_port = htons(param.port);
    server.sin_addr.s_addr = inet_addr((char*)param.host);

    addrlen  = sizeof(struct sockaddr_in); 

 CON: 
    ret = connect(sockfd, (const struct sockaddr*)&server, addrlen);
    if (ret < 0) 
    {
        printf("Connect failed ERROR=%d--%d\n",ret,connect_cnt);

        if(connect_cnt < 5)
        {
            sleep(2);connect_cnt++; 
            goto CON;                    
        }
        else
        {
            connect_cnt = 0;
            close(sockfd);
            exit(1);
        }

    }

    printf("Connect to iperf server successful!\n");
    {
        int flag = 1;
        setsockopt( sockfd,
                    IPPROTO_TCP,     /* set option at TCP level */
                    TCP_NODELAY,     /* name of option */
                    (void *) &flag,  /* the cast is historical cruft */
                    sizeof(int));    /* length of option value */
    }

    tick1 = hrt_absolute_time();
    for(;;)
    {
        cnt_tick++;
        ret = send(sockfd, buff, sizeof(buff), 0);
        if (ret < 0) 
        {
            printf("the TCP sendto data is failed ret= %d! \n",ret);
        }
        else
        {
            total_bytes += ret;
            ////////////////////////////////////
        	tick2 = hrt_absolute_time();
            if ((tick2 - tick1) >= RT_TICK_PER_SECOND*1000)
            {
                float f,time;
                time = (float)(tick2 - tick1)/(RT_TICK_PER_SECOND*1000.0);

                f = (float)((float)total_bytes / time);
                f *= 8;

                printf("%.2f \n", f/(1024.0*1024.0));
                tick1 = tick2;
                total_bytes = 0;
            }
            ////////////////////////////////////
        }
    }
    close(sockfd);
    for(;;)
    {
        sleep(1);
        printf("disconnected!\n");
    }
    return 0;
}

#elif  (CLIENT_TYPE == CLIENT_TYPE_UDP )

#define UDP_CLIENT_PORT       55555
#define UDP_CLIENT_IP         "192.168.10.200"

static int iperf_client(int argc, FAR char *argv[])
{
    int sockfd; 
    struct sockaddr_in server;
    socklen_t addrlen;
    ssize_t nbytes;

    ssize_t cnt_tick = 0;
    ssize_t total_bytes = 0;

    int addr_len = sizeof(struct sockaddr_in); 

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);//IPPROTO_UDP
    if (sockfd < 0) 
    {
        printf("create socket failed!\n");
        exit(1);
    }

    memset(&server, 0, sizeof(server));

    server.sin_family       = AF_INET;
    //server.sin_port         = htons(UDP_CLIENT_PORT);//大小端转换
    //server.sin_addr.s_addr  = inet_addr(UDP_CLIENT_IP);
    server.sin_port         = htons(param.port);
    server.sin_addr.s_addr  = inet_addr((char*)param.host);
    addrlen                 = sizeof(struct sockaddr_in);

    hrt_abstime tick1, tick2;

    tick1 = hrt_absolute_time();
    for(;;)
    {
        cnt_tick++;
        nbytes = sendto(sockfd,buff,sizeof(buff),0,(struct sockaddr *)&server,addrlen);
        if (nbytes < 0) 
        {
            printf("the udp sendto data is failed ret= %d! \n",nbytes);
        }
        else
        {
            total_bytes+=nbytes;
            // if(cnt_tick >= 1000)
            // {
            //     printf("the bytes bps= %d Mbps! \n",total_bytes/125000);
            //     cnt_tick = 0;
            //     total_bytes = 0;
            // }

        ////////////////////////////////////
        	tick2 = hrt_absolute_time();
            if ((tick2 - tick1) >= RT_TICK_PER_SECOND*1000)
            {
                float f,time;
                time = (float)(tick2 - tick1)/(RT_TICK_PER_SECOND*1000.0);

                f = (float)((float)total_bytes / time);
                f *= 8;

                printf("%.2f \n", f/(1024.0*1024.0));
                tick1 = tick2;
                total_bytes = 0;
            }
        ////////////////////////////////////



            //printf("the udp sendto data length = %d! \n",nbytes);
        }
        //usleep(RT_TICK_PER_SECOND);
    }
    close(sockfd);
    for(;;)
    {
        sleep(1);
        printf("disconnected!\n");
    }

    return 0;
}

#endif

#if (CLIENT_TYPE == CLIENT_TYPE_TCP )

static int iperf_server(int argc, FAR char *argv[])
{
    hrt_abstime tick1, tick2;
    struct sockaddr_in server_addr, client_addr;

    ssize_t sockfd = -1, connected, bytes_received, total_bytes=0;
    socklen_t sin_size;

    printf("server ip addr = %s : port = %d\n",(char*)param.host,param.port);    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        printf("Socket error\n");
        goto __exit;
    }

    memset(&server_addr, 0x0, sizeof(server_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(param.port);
    server_addr.sin_addr.s_addr = inet_addr((char*)param.host);
   
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
        printf("Unable to bind\n");
        goto __exit;
    }

    if (listen(sockfd, 5) == -1)
    {
        printf("Listen error\n");
        goto __exit;
    }

    sin_size = sizeof(struct sockaddr_in);
    connected = accept(sockfd, (struct sockaddr *)&client_addr, &sin_size);
    printf("new client connected from (%s, %d)\n",inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
    {
        int flag = 1;
        setsockopt( connected,
                    IPPROTO_TCP,     /* set option at TCP level */
                    TCP_NODELAY,     /* name of option */
                    (void *) &flag,  /* the cast is historical cruft */
                    sizeof(int));    /* length of option value */
    }

    total_bytes = 0;
    tick1 = hrt_absolute_time();
    for(;;)
    {
        bytes_received = recv(connected, buff, sizeof(buff), 0);        
        if (bytes_received <= 0) {
            printf("the tcp recv data is failed rev_ret=%d \n",bytes_received);
            break;           
        }else{
            total_bytes += bytes_received;
        	tick2 = hrt_absolute_time();
            if ((tick2 - tick1) >= RT_TICK_PER_SECOND*1000)
            {
                float f,time;
                time = (float)(tick2 - tick1)/(RT_TICK_PER_SECOND*1000.0);

                f = (float)((float)total_bytes / time);
                f *= 8;

                printf("%.2f \n", f/(1024.0*1024.0));
                tick1 = tick2;
                total_bytes = 0;
            }         
            send(connected, buff, bytes_received, 0);
        }
    }

    if (connected >= 0) close(connected);
    connected = -1;

__exit:
    if (sockfd >= 0) 
        close(sockfd);
    return 0;
}

#elif (CLIENT_TYPE == CLIENT_TYPE_UDP )

#define UDP_SERVER_PORT         55555
#define UDP_SERVER_IP           "192.168.10.30"

static int iperf_server(int argc, FAR char *argv[])
{
    int sockfd; 
    char * ip_addr = NULL;
    struct sockaddr_in server;
    struct sockaddr_in client;

    socklen_t addrlen;
    ssize_t ret,nbytes;

    int addr_len = sizeof(struct sockaddr_in); 

    printf("server ip addr = %s : port = %d\n",(char*)param.host,param.port);    

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);//IPPROTO_UDP
    if (sockfd < 0) 
    {
        printf("create socket failed!\n");
        exit(1);
    }

    memset(&server, 0, sizeof(server));

    server.sin_family       = AF_INET;
    server.sin_port         = htons(param.port); 
    server.sin_addr.s_addr  = inet_addr((char*)param.host);
    addrlen                 = sizeof(struct sockaddr_in);

    if (bind(sockfd, (struct sockaddr*)&server, addrlen) < 0)
    {
        printf("server: bind failure: %d\n", errno);
        exit(1);
    }   

    for(;;)
    {

        nbytes= recvfrom(sockfd, buff, sizeof(buff), 0, (struct sockaddr*)&client, &addrlen);  //接收来自server的信
        if (nbytes < 0)
        {          
            printf("the udp recvfrom data is failed rev_ret=%d \n",nbytes);
        }
        else
        {           
            printf("Received a string from client port=%d client=%s length=%d\n",
                htons(client.sin_port),inet_ntoa(client.sin_addr), nbytes);
            ret = sendto(sockfd,buff,nbytes,0,(struct sockaddr *)&client,addrlen);
            if (ret < 0) 
            {
                printf("the udp sendto data is failed ret= %d! \n",ret);
            }
            else
            {
                printf("the udp sendto data length = %d! \n",ret);
            }
        }
        usleep(RT_TICK_PER_SECOND);
    }
    close(sockfd);
    for(;;)
    {
        sleep(1);
        printf("disconnected!\n");
    }

    return 0;
}

#endif

void iperf_usage(void)
{
    printf("Usage: iperf [-s|-c host] [options]\n");
    printf("       iperf [-h|--stop]\n");
    printf("\n");
    printf("Client/Server:\n");
    printf("  -p #         server port to listen on/connect to\n");
    printf("\n");
    printf("Server specific:\n");
    printf("  -s           run in server mode\n");
    printf("\n");
    printf("Client specific:\n");
    printf("  -c <host>    run in client mode, connecting to <host>\n");
    printf("\n");
    printf("Miscellaneous:\n");
    printf("  -h           print this message and quit\n");
    printf("  --stop       stop iperf program\n");

    return ;
}

int iperf_main(int argc, char** argv)
{
    int mode 	= 0; 			/* server mode */
    char *host 	= NULL;
    int port 	= IPERF_PORT;
    struct in_addr local_addr;

    if (argc == 1) 
        goto __usage;
    else 
    {
        if (strcmp(argv[1], "-h") ==0) {
        	goto __usage;
        }

        /*
         * Stop iperf
         */
        else if (strcmp(argv[1], "--stop") ==0)
        {
            /* stop iperf */
            param.mode = IPERF_MODE_STOP;
            return 0;
        }
        /*
         * Set iperf as server mode
         */
        else if (strcmp(argv[1], "-s") ==0)
        {
            mode = IPERF_MODE_SERVER;

            local_addr.s_addr = 0;
            netlib_get_ipv4addr("eth0",(struct in_addr *)&local_addr);
            host = inet_ntoa(local_addr);//将地址转换成数据

            /* iperf -s -p 5000 */
            if (argc == 4)
            {
                if (strcmp(argv[2], "-p") == 0)
                {
                    port = atoi(argv[3]);
                }
                else 
                {
                    goto __usage;     
                }
            }
        }
        /*
         * Set iperf as client mode
         */
        else if (strcmp(argv[1], "-c") ==0)
        {
            mode = IPERF_MODE_CLIENT;

            if (argc < 3) {
            	goto __usage;
            }

            host = argv[2];//服务器IP地址
            if (argc == 5)
            {
                /* iperf -c host -p port */
                if (strcmp(argv[3], "-p") == 0)
                {
                    port = atoi(argv[4]);
                }
                else 
                {
                    goto __usage;
                }
            }
        }

        /*
         * call help information
         */
        else if (strcmp(argv[1], "-h") ==0)
        {
            goto __usage;
        }

        else goto __usage;
    }

    /* start iperf in */
    if (param.mode == IPERF_MODE_STOP)
    {
    	pid_t tid = 0;

        param.mode = mode;
        param.port = port;

        if (param.host){
            param.host = NULL;
            free(host);
        }

        if (host) {
        	param.host = strdup(host);
        }

        if (mode == IPERF_MODE_CLIENT)
        {
        	tid = task_create("iperfc", 100,2048, iperf_client,NULL);
            if (tid < 0)
            {
                int errcode = errno;
                fprintf(stderr, "ERROR: Failed to start iperfc: %d\n",errcode);
                return -errcode;
            }
        }
        else if (mode == IPERF_MODE_SERVER)
        {
        	tid = task_create("iperfs", 100,2048, iperf_server,NULL);
            if (tid < 0)
            {
                int errcode = errno;
                fprintf(stderr, "ERROR: Failed to start iperfc: %d\n",errcode);
                return -errcode;
            }
        }
    }
    else
    {
        printf("Please stop iperf firstly, by:\n");
        printf("iperf --stop\n");
    }

    return 0;

__usage:
    iperf_usage();
    return 0;
}

//#endif /* PKG_NETUTILS_IPERF */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <athomerobot/tcpconnector.h>
#include <athomerobot/tcpconnector.hpp>
#include <athomerobot/tcpacceptor.hpp>
#include <athomerobot/tcpstream.hpp>

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>   
#include <sys/ioctl.h>
#include <unistd.h>  
#include <iostream>
#include <fstream>
#include <errno.h>

#include <ros/ros.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
bool App_exit = false;

int receive_image(int socket)
{ // Start function 

int buffersize = 0, recv_size = 0,size = 0, read_size, write_size, packet_index =1,stat;

char imagearray[10241],verify = '1';
FILE *image;

//Find the size of the image
do{
stat = read(socket, &size, sizeof(int));
}while(stat<0);

printf("Packet received.\n");
printf("Packet size: %i\n",stat);
printf("Image size: %i\n",size);
printf(" \n");

char buffer[] = "Got it";

//Send our verification signal
do{
stat = write(socket, &buffer, sizeof(int));
}while(stat<0);

printf("Reply sent\n");
printf(" \n");

image = fopen("Capture2.png", "w");

if( image == NULL) {
printf("Error has occurred. Image file could not be opened\n");
return -1; }

//Loop while we have not received the entire file yet


int need_exit = 0;
struct timeval timeout = {10,0};

fd_set fds;
int buffer_fd, buffer_out;

while(recv_size < size) {
//while(packet_index < 2){

    FD_ZERO(&fds);
    FD_SET(socket,&fds);

    buffer_fd = select(FD_SETSIZE,&fds,NULL,NULL,&timeout);

    if (buffer_fd < 0)
       printf("error: bad file descriptor set.\n");

    if (buffer_fd == 0)
       printf("error: buffer read timeout expired.\n");

    if (buffer_fd > 0)
    {
        do{
               read_size = read(socket,imagearray, 10241);
            }while(read_size <0);

            printf("Packet number received: %i\n",packet_index);
        printf("Packet size: %i\n",read_size);


        //Write the currently read data into our image file
         write_size = fwrite(imagearray,1,read_size, image);
         printf("Written image size: %i\n",write_size); 

             if(read_size !=write_size) {
                 printf("error in read write\n");    }


             //Increment the total number of bytes read
             recv_size += read_size;
             packet_index++;
             printf("Total received image size: %i\n",recv_size);
             printf(" \n");
             printf(" \n");
    }

}


  fclose(image);
  printf("Image successfully Received!\n");
  return 1;
  }

int receive_image_main(int port){
	int socket_desc;
    struct sockaddr_in server;
    char *parray;


  //Create socket
  socket_desc = socket(AF_INET , SOCK_STREAM , 0);

  if (socket_desc == -1) {
  printf("Could not create socket");
  }

  memset(&server,0,sizeof(server));
  server.sin_addr.s_addr = inet_addr("127.0.0.1");
  server.sin_family = AF_INET;
  server.sin_port = htons(port);

  //Connect to remote server
  if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0) {
  cout<<strerror(errno);
  close(socket_desc);
  puts("Connect Error");
  return 1;
  }

  puts("Connected\n");

  receive_image(socket_desc);

  close(socket_desc);

  return 0;
}

int receive_message_main(int argc,char** argv)  
{


    int len;
    string message;
    char line[256];
    TCPConnector* connector = new TCPConnector();
    TCPStream* stream = connector->connect("127.0.0.1", 3300);
    if (stream) {
        message = "Is there life on Mars?";
        stream->send(message.c_str(), message.size());
        printf("sent - %s\n", message.c_str());
        len = stream->receive(line, sizeof(line));
        line[len] = 0;
        printf("received - %s\n", line);
        delete stream;
    }

    stream = connector->connect("127.0.0.1", 3300);
    if (stream) {
        message = "Why is there air?";
        stream->send(message.c_str(), message.size());
        printf("sent - %s\n", message.c_str());
        len = stream->receive(line, sizeof(line));
        line[len] = 0;
        printf("received - %s\n", line);
        delete stream;
    }
    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tcp_server");
  ros::Time::init();

  cout << "SEPANTA TCP/IP CLIENT STARTED DONE (93/04/02)" << endl;

   receive_message_main(argc,argv);

  ros::Rate ros_rate(20);

  while(ros::ok())
  {
    ros::spinOnce();
    ros_rate.sleep();
  }

  App_exit = true;

  return 0;
}  

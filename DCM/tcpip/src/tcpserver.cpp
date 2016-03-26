#include <ros/ros.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <athomerobot/tcpacceptor.h>
#include <athomerobot/tcpacceptor.hpp>
#include <athomerobot/tcpstream.hpp>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>   
#include <unistd.h>  
#include <iostream>
#include <fstream>
#include <errno.h>

using namespace std;
bool App_exit = false;

int send_image(int socket,string pathImage){

   FILE *picture;
   int size, read_size, stat, packet_index;
   char send_buffer[10240], read_buffer[256];
   packet_index = 1;

   picture = fopen(pathImage.c_str(), "r");
   printf("Getting Picture Size\n");   

   if(picture == NULL) {
        printf("Error Opening Image File"); } 

   fseek(picture, 0, SEEK_END);
   size = ftell(picture);
   fseek(picture, 0, SEEK_SET);
   printf("Total Picture size: %i\n",size);

   //Send Picture Size
   printf("Sending Picture Size\n");
   write(socket, (void *)&size, sizeof(int));

   //Send Picture as Byte Array
   printf("Sending Picture as Byte Array\n");

   do { //Read while we get errors that are due to signals.
      stat=read(socket, &read_buffer , 255);
      printf("Bytes read: %i\n",stat);
   } while (stat < 0);

   printf("Received data in socket\n");
   printf("Socket data: %c\n", read_buffer);

   while(!feof(picture)) {
   //while(packet_index = 1){
      //Read from the file into our send buffer
      read_size = fread(send_buffer, 1, sizeof(send_buffer)-1, picture);

      //Send data through our socket 
      do{
        stat = write(socket, send_buffer, read_size);  
      }while (stat < 0);

      printf("Packet Number: %i\n",packet_index);
      printf("Packet Size Sent: %i\n",read_size);     
      printf(" \n");
      printf(" \n");


      packet_index++;  

      //Zero out our send buffer
      bzero(send_buffer, sizeof(send_buffer));
     }
    }

int send_image_main(int port,string pathImage)
{
      int socket_desc , new_socket , c, read_size,buffer = 0;
          struct sockaddr_in server , client;
          char *readin;

          //Create socket
          socket_desc = socket(AF_INET , SOCK_STREAM , 0);
          if (socket_desc == -1)
          {
             printf("Could not create socket");
          }

          //Prepare the sockaddr_in structure
          server.sin_family = AF_INET;
          server.sin_addr.s_addr = INADDR_ANY;
          server.sin_port = htons( port );

          //Bind
         if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
         {
           puts("bind failed");
           return 1;
         }

         puts("bind done");

         //Listen
         listen(socket_desc , 3);

          //Accept and incoming connection
          puts("Waiting for incoming connections...");
          c = sizeof(struct sockaddr_in);

         if((new_socket = accept(socket_desc, (struct sockaddr *)&client,(socklen_t*)&c))){
    puts("Connection accepted");
             }

        fflush(stdout);

        if (new_socket<0)
        {
          perror("Accept Failed");
          return 1;
        }
        
        send_image(new_socket,pathImage);
        close(socket_desc);
        fflush(stdout);
        return 0;
}
       
int send_message_main(int argc,char** argv)
{
  

    TCPStream* stream = NULL;
    TCPAcceptor* acceptor = NULL;
   
    acceptor = new TCPAcceptor(3300);
    
    if (acceptor->start() == 0) {
        while (1) {
            stream = acceptor->accept();
            if (stream != NULL) {
                ssize_t len;
                char line[256];
                while ((len = stream->receive(line, sizeof(line))) > 0) {
                    line[len] = 0;
                    printf("received - %s\n", line);
                    stream->send(line, len);
                }
                delete stream;
            }
        }
    }
   
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tcp_server");
  ros::Time::init();

  cout << "SEPANTA TCP/IP STARTED DONE (93/04/02)" << endl;

  send_message_main(argc,argv);

  ros::Rate ros_rate(20);

  while(ros::ok())
  {
    ros::spinOnce();
    ros_rate.sleep();
  }

  App_exit = true;

  return 0;
} 

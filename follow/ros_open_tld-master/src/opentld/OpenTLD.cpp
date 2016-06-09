/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/**
  * @author Georg Nebehay
  */
#include "ros/ros.h"
#include "Main.h"
#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"

#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Image.h>

using tld::Config;
using tld::Gui;
using tld::Settings;

int main(int argc, char **argv)
{
    
	//Algo stuff
	Main *main = new Main();
	Config config;
	ImAcq *imAcq = imAcqAlloc();
	Gui *gui = new Gui();
	
	//Ros stuff
	ros::init(argc, argv, "TobotDriver");
	ros::NodeHandle my_node;
	ros::Rate loop_rate(10);
	ros::Publisher poete=my_node.advertise<geometry_msgs::PolygonStamped>("/tracking", 1000);
	ros::Subscriber scribe=my_node.subscribe<sensor_msgs::Image>( "/camera/rgb/image_color",1000, &Main::doWork, main);

	main->gui = gui;
	main->imAcq = imAcq;
	main->poete = &poete;
	if(config.init(argc, argv) == PROGRAM_EXIT)
	{
		return EXIT_FAILURE;
	}

	config.configure(main);
	
	//loading ros parameters
	main->loadRosparam();

	//ros::param::get("/OpenTLD/Graphical_interface", main->showOutput);
	srand(main->seed);
	//imAcqInit(imAcq);

	if(main->showOutput){
		gui->init();
	}
	while(ros::ok()){
		ros::spinOnce();
	}

	delete main;
	return EXIT_SUCCESS;
}

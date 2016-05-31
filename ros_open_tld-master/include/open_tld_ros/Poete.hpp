#ifndef POETE_MOTOR_DRIVER_H
#define POETE_MOTOR_DRIVER_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/PolygonStamped.h>

class Poete{

	protected : 
	bool _verbose;
	ros::Publisher _poete;
	std::string _topic_name;
	nav_msgs::Odometry _odom;

	public : 
	
	/****************µCONSTRUCtEUR*************************/
	
	Poete(ros::NodeHandle ao_nh, std::string topic) : _verbose(false), _topic_name(topic), _odom(){
		_poete=ao_nh.advertise<geometry_msgs/PolygonStamped>(_topic_name, 1000);
	};
	
	Poete(bool verb, ros::NodeHandle ao_nh, std::string topic) : _verbose(verb), _topic_name(topic), _odom(){
		_poete=ao_nh.advertise<geometry_msgs/PolygonStamped>(_topic_name, 1000);
	};
	/***************FUNTIONS******************************/
	
	void publish(const nav_msgs::Odometry& msg);
	void publish();
	geometry_msgs/PolygonStamped& getMes() {return _odom;}
	void setMsg(nav_msgs::Odometry& newer){_odom=newer;}
	void setVerbose(){_verbose=true;}
	void affiche();
	
};


inline void Poete::publish(geometry_msgs/PolygonStamped& msg){
	if(_verbose==true){
		std::cout << "Command send by the poete to " <<_topic_name<< " my general"<<std::endl;
		//std::cout << msg <<std::endl;
	}
	_odom=msg;	
	_poete.publish(msg);
}

inline void Poete::publish(){
	if(_verbose==true){
		std::cout << "Command send by the poete to " <<_topic_name<< " my general"<<std::endl;
		//std::cout << _odom <<std::endl;
	}
	_poete.publish(_odom);
}

inline void Poete::affiche(){
	std::cout<< "I'm a poete check on "<<_topic_name << " my lastest poem was "<<_odom<<std::endl;
}

#endif

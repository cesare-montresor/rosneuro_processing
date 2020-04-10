#include <ros/ros.h>
#include "rosneuro_processing/SmrBci.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "smrbci");

	rosneuro::SmrBci smrbci;
	
	if(smrbci.configure()== false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(10);
	while(true)
	{

	 if(smrbci.Classify() == true) {
	   std::cout <<"Classification" << std::endl; 
	 } 
	
         ros::spinOnce();
	 r.sleep();

	}

	ros::shutdown();
	return 0;
}

#include <ros/ros.h>
#include "rosneuro_processing/EogBci.hpp"
#include "rosneuro_processing/SmrBci.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "processing");


	rosneuro::SmrBci smrbci;
	rosneuro::EogBci eogbci;

	if(smrbci.configure()== false) {
		std::cerr<<"SMR BCI SETUP ERROR"<<std::endl;
		return -1;
	}
	
	if(eogbci.configure()== false) {
		std::cerr<<"EOG SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(256);
	while(ros::ok())
	{

		// SMR BCI
		if(smrbci.Classify() == true) {
			ROS_INFO_ONCE("Classification started"); 
		 } 


		// EOG DETECTION
		 if(eogbci.Apply() == true) {
			ROS_INFO_ONCE("Eog detection started"); 
		 }

		if(eogbci.HasArtifacts(1) == true) {
		       ROS_INFO_ONCE("Eog detected"); 
	
		}

	

         ros::spinOnce();
		 r.sleep();
	}

	ros::shutdown();
	return 0;
}

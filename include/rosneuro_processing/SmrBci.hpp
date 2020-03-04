#ifndef SMR_BCI_HPP
#define SMR_BCI_HPP

#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_data/NeuroDataTools.hpp"

#include <wtkprocessing/RingBuffer.hpp>
#include <wtkprocessing/Laplacian.hpp>
#include <wtkprocessing/Pwelch.hpp>
#include <wtkprocessing/Gaussian.hpp>
#include <wtkprocessing/Exponential.hpp>


namespace rosneuro {

class SmrBci {
	public:
		SmrBci(void);
		virtual ~SmrBci(void);

		bool configure(void);

		float GetFrameRate(void);

		void SetRejection(float);
		
		void SetIntegration(float);

		void Reset(void);

		bool Classify(void);

       	private:
		void on_received_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);

		bool on_request_classify(std_srvs::Empty::Request& req,
							   std_srvs::Empty::Response& res);
		bool on_request_reset(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);

	private:
		ros::NodeHandle		   nh_;
		ros::NodeHandle		   p_nh_;
		ros::Subscriber		   sub_data_;
		ros::Publisher		   pub_data_;
		ros::Publisher		   pub_idata_; //integrated
		std::string                sub_topic_data_;
		std::string	           pub_topic_data_;
		std::string	           pub_topic_idata_;
		bool 			   new_neuro_frame_;
		rosneuro_msgs::NeuroOutput msg_;
		rosneuro_msgs::NeuroOutput imsg_; //integrated

		ros::ServiceServer	srv_classify_;
		ros::ServiceServer	srv_reset_;


		unsigned int 	buffer_size_;
		unsigned int 	n_channels_;
		unsigned int 	n_samples_;
		std::string	lap_path_;
		unsigned int 	psd_wlength_;
		unsigned int 	psd_novl_;
		unsigned int 	sampling_freq_;
		unsigned int 	psd_dolog_;
		std::string 	decoder_type_;
		std::string	decoder_path_;
		float 		rejection_thr_;
		float 		integration_thr_;
		unsigned int	n_classes_;
		unsigned int    predicted_class_;
		std::vector<int> hard_prediction_;

		std::vector<std::string> class_labels_;	

		std::vector<float> data_;
		Eigen::MatrixXf dmap_;
		Eigen::MatrixXd dbuf_;
		Eigen::MatrixXd dlap_;
		Eigen::VectorXd dfet_;
	
		Eigen::VectorXd rawpp_;
		Eigen::VectorXd intpp_;

		wtk::proc::RingBuffer* 	buffer_;
		wtk::proc::Laplacian*	laplacian_;
		wtk::proc::Pwelch* 	psd_;
		wtk::proc::Gaussian* 	decoder_; //TO DO generalize with other kinds of decoder
		wtk::proc::Exponential*	integrator_;




		
		
		





};

}


#endif

#ifndef EOG_BCI_HPP
#define EOG_BCI_HPP

#include <string>


#include <ros/ros.h>

#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_data/NeuroDataTools.hpp"
#include <wtkprocessing/RingBuffer.hpp>
#include <wtkprocessing/ButterFilter.hpp>
#include <wtkprocessing/Padding.hpp>
#include <wtkprocessing/Envelope.hpp>

#define EOG_CHANNELS 2

namespace rosneuro {

class EogBci {


	public:

		EogBci(void);
		~EogBci(void);
		
		bool configure(void);
		float GetFrameRate(void);
		void SetThreshold(double value);
		//unsigned int GetFrameIdx(void);
		bool Apply(void);
		bool HasArtifacts(void);
		bool HasArtifacts(unsigned int count);


	private: 
		void on_received_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);


	private: 

		ros::NodeHandle		   nh_;
		ros::NodeHandle		   p_nh_;
		std::string                sub_topic_data_;
		bool 			   new_neuro_frame_;
		ros::Subscriber		   sub_data_;
		rosneuro_msgs::NeuroOutput msg_;

		unsigned int 	buffer_size_;
		unsigned int 	n_channels_;
		unsigned int 	n_samples_;
		unsigned int 	sampling_freq_;
		
		unsigned int	eog_channels_;
		unsigned int 	eog_type_;
		unsigned int    eog_order_;
		double          eog_fcoff1_;
		double 		eog_fcoff2_;
		double          eog_threshold_;
		unsigned int    eog_lchannel_;
		unsigned int    eog_rchannel_;
		unsigned int	chleft_;
		unsigned int	chright_;
		unsigned int    eog_padding_;


		
		float 		framerate_;

		std::vector<float> data_;
		Eigen::MatrixXf dmap_;
		Eigen::MatrixXd dbuf_;
		Eigen::MatrixXd dflt_;
		Eigen::MatrixXd denv_;
		Eigen::MatrixXd dfet_;
		Eigen::VectorXd heog_;
		Eigen::VectorXd veog_;
		Eigen::VectorXd heogpad_;
		Eigen::VectorXd veogpad_;
		Eigen::VectorXd heogfilt_;
		Eigen::VectorXd veogfilt_;
		Eigen::MatrixXd dpad_;
		Eigen::MatrixXd dpflt_;
		Eigen::VectorXd hvalue_;
		Eigen::VectorXd vvalue_;

		
		wtk::proc::RingBuffer* 	buffer_;
		wtk::proc::ButterFilter* butterfilter_;
		wtk::proc::Padding* padding_;
		wtk::proc::Envelope*   envelope_;

	




};

	
}


#endif

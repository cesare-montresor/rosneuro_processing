#ifndef EOG_BCI_CPP
#define EOG_BCI_CPP

#include "rosneuro_processing/EogBci.hpp"

namespace rosneuro {

EogBci::EogBci(void): p_nh_("~") {


	this->eog_channels_ = EOG_CHANNELS;

        this->buffer_ = new wtk::proc::RingBuffer();
	this->butterfilter_ = new wtk::proc::ButterFilter();
	this->envelope_ = new wtk::proc::Envelope();
	
	this->sub_topic_data_	 = "/neurodata";

}

EogBci::~EogBci(void) {

}



bool EogBci::configure(void) {
	
	ros::param::param("~buffer_size", (int&) this->buffer_size_, 512);
	ros::param::param("~n_channels", (int&) this->n_channels_, 16);
	ros::param::param("~sampling_freq", (int&) this->sampling_freq_, 512);
	ros::param::param("~n_samples", (int&) this->n_samples_, 32);

	this->framerate_ = 1000.0f*this->n_samples_/this->sampling_freq_;

	// Setup Ring buffer
	this->buffer_->Setup(this->buffer_size_, this->n_channels_);

	ros::param::param("~eog_filter_type", (int&) this->eog_type_, 3);  
	ros::param::param("~eog_filter_order", (int&) this->eog_order_, 2); 
	ros::param::param("~eog_filter_fcoff1", (double&) this->eog_fcoff1_, 1.0); 
	ros::param::param("~eog_filter_fcoff2", (double&) this->eog_fcoff2_, 10.0); 
	ros::param::param("~eog_threshold", (double&) this->eog_threshold_, 30.0); 



	// Setup Butterworth method
   try {
      
      this->butterfilter_->Setup(this->eog_type_, this->eog_order_, this->sampling_freq_, this->eog_fcoff1_, this->eog_fcoff2_);
      							
    } catch (std::runtime_error& e) {
      throw std::runtime_error("Can't setup Butterworth method because: '" + std::string(e.what()) + "'");
    }

    // Setup Envelope method
    try {
      
      this->envelope_->Setup(this->buffer_size_);
    } catch (std::runtime_error& e) {
      throw std::runtime_error("Can't setup Envelope method because: '" + std::string(e.what()) + "'");
    }


    ros::param::param("~eog_left_channel", (int&) this->eog_lchannel_, 12); 
    ros::param::param("~eog_right_channel", (int&) this->eog_rchannel_, 16);   


	// Setup EOG channels to channel vector indeces
	this->chleft_ = this->eog_lchannel_ - 1; // Gloria we can avoid to define two variables
	this->chright_ = this->eog_rchannel_ - 1; // Gloria we can avoid to define two variables
	
	
	// Setup temporary data matrices
	this->dmap_ = Eigen::MatrixXf::Zero(this->n_channels_, this->n_samples_);
	this->dbuf_ = Eigen::MatrixXd::Zero(this->buffer_size_, this->n_channels_);
	this->dflt_ = Eigen::MatrixXd::Zero(this->buffer_size_, this->eog_channels_);
	this->denv_ = Eigen::MatrixXd::Zero(this->buffer_size_, this->eog_channels_);
	this->dfet_ = Eigen::MatrixXd::Zero(this->n_samples_, this->eog_channels_);
	this->heog_     = Eigen::VectorXd::Zero(this->n_samples_);
	this->veog_     = Eigen::VectorXd::Zero(this->n_samples_);
	this->heogpad_  = Eigen::VectorXd::Zero(this->n_samples_);
	this->veogpad_  = Eigen::VectorXd::Zero(this->n_samples_);
	this->heogfilt_ = Eigen::VectorXd::Zero(this->n_samples_);
	this->veogfilt_ = Eigen::VectorXd::Zero(this->n_samples_);


	ros::param::param("~eog_filter_padding", (int&) this->eog_padding_, 2); 


	if(this->eog_padding_ != 0) {
		this->dpad_  = Eigen::MatrixXd::Zero(3*this->buffer_size_-2, this->eog_channels_);  // Gloria magic number
		this->dpflt_ = Eigen::MatrixXd::Zero(3*this->buffer_size_-2, this->eog_channels_);  // Gloria magic number

		// Setup Padding method
   		try {
      		this->padding_ = new wtk::proc::Padding();
      		this->padding_->Setup(this->eog_padding_, this->buffer_size_);
    	} catch (std::runtime_error& e) {
      		throw std::runtime_error("Can't setup Padding method because: '" + std::string(e.what()) + "'");
    	}
	}

	this->hvalue_ = Eigen::VectorXd::Zero(this->buffer_size_);
	this->vvalue_ = Eigen::VectorXd::Zero(this->buffer_size_);

	
	this->sub_data_ = this->p_nh_.subscribe(this->sub_topic_data_, 1000, &EogBci::on_received_data, this);
	this->new_neuro_frame_= false;


}



void EogBci::SetThreshold(double value) {
	ROS_INFO("Eog Threshold value set at %f", value);
	this->eog_threshold_ = value;
}


float EogBci::GetFrameRate(void) {
	
	return this->framerate_;
}


void EogBci::on_received_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {

	if(msg->eeg.info.nsamples == this->n_samples_ && msg->eeg.info.nchannels == this->n_channels_) {
		this->new_neuro_frame_ = true;
		this->data_ = msg->eeg.data;
	}

}

/*
unsigned int EogBci::GetFrameIdx(void) {
	return this->fidx_;
}


*/


bool EogBci::Apply(void) {

	// Copy data in eigen structure
	if(this->new_neuro_frame_== false)
	{
		//ROS_WARN("Not available data to classify");
		return false;
	}

	this->dmap_ = Eigen::Map<Eigen::MatrixXf>(this->data_.data(), this->n_channels_, this->n_samples_);

	this->dmap_.transposeInPlace();

	// Extract channels from buffer
	this->dfet_.col(0) = this->dmap_.col(this->chleft_).cast<double>();
	this->dfet_.col(1) = this->dmap_.col(this->chright_).cast<double>();

	// Compute HEOG and VEOG
	this->heog_ = this->dfet_.col(0) - this->dfet_.col(1);	
	this->veog_ = (this->dfet_.col(0) + this->dfet_.col(1)) / 2.0f;	

	// Apply filtering
	this->butterfilter_->Apply(this->heog_, this->heogfilt_);
	this->butterfilter_->Apply(this->veog_, this->veogfilt_);

	// Rectify
	this->heogfilt_ = this->heogfilt_.cwiseAbs();
	this->veogfilt_ = this->veogfilt_.cwiseAbs();


	// Temp
	this->hvalue_ = this->heogfilt_;
	this->vvalue_ = this->veogfilt_;

		
	this->new_neuro_frame_= false;

	return true;
}


bool EogBci::HasArtifacts(unsigned int count) {

	unsigned int hcount = 0;
	unsigned int vcount = 0;

	for(auto i=0; i<this->hvalue_.rows(); i++) {
		if (this->hvalue_(i) >= this->eog_threshold_)
			hcount++;
	}

	for(auto i=0; i<this->vvalue_.rows(); i++) {
		if (this->vvalue_(i) >= this->eog_threshold_)
			vcount++;
	}

	if( (hcount >= count) || (vcount >= count) ) {
		printf("Found %u samples with artifacts\n", std::max(hcount, vcount));
		return true;
	} else {
		return false;
	}
}

bool EogBci::HasArtifacts(void) {
	// Edited by L.Tonin  <luca.tonin@epfl.ch> on 23/07/19 12:39:16
	// I think this is wrong
	//if(this->hvalue_.maxCoeff() < this->eog_threshold_ || this->vvalue_.maxCoeff() < this->eog_threshold_)
	
	if(this->hvalue_.maxCoeff() >= this->eog_threshold_ || this->vvalue_.maxCoeff() >= this->eog_threshold_)
		return true;
	else
		return false;
}


}

#endif

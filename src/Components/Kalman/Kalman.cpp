/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk
 */

#include <memory>
#include <string>

#include "Kalman.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace Kalman {

Kalman::Kalman(const std::string & name) :
		Base::Component(name),
		cov_process("cov_process", 100, "range"),
		cov_measurement("cov_measurement", 10, "range")  {

	tracking = false;
	lost_counter = 0;

	cov_process.addConstraint("1");
	cov_process.addConstraint("1000");
	registerProperty(cov_process);
	
	cov_measurement.addConstraint("1");
	cov_measurement.addConstraint("1000");
	registerProperty(cov_measurement);

}

Kalman::~Kalman() {
}

void Kalman::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_meas", &in_meas);
	registerStream("out_pred", &out_pred);
	// Register handlers
	registerHandler("update", boost::bind(&Kalman::update, this));
	addDependency("update", NULL);

}

bool Kalman::onInit() {

	int stateSize = 5;
	int measSize = 3;
	int contrSize = 0;

	unsigned int type = CV_32F;
	kf = new cv::KalmanFilter(stateSize, measSize, contrSize, type);
	
	state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,r]
	meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_r]
	
	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 ]
	// [ 0 1 0  dT 0 ]
	// [ 0 0 1  0  0 ]
	// [ 0 0 0  1  0 ]
	// [ 0 0 0  0  1 ]
	kf->transitionMatrix = cv::Mat::eye(stateSize, stateSize, type);
	cv::setIdentity(kf->transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 ]
	// [ 0 1 0 0 0 ]
	// [ 0 0 0 0 1 ]
	kf->measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf->measurementMatrix.at<float>(0) = 1.0f;
	kf->measurementMatrix.at<float>(6) = 1.0f;
	kf->measurementMatrix.at<float>(14) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex 0  0    0    0  ]
	// [ 0  Ey 0    0    0  ]
	// [ 0  0  Ev_x 0    0  ]
	// [ 0  0  0    Ev_y 0  ]
	// [ 0  0  0    0    Er ]
	kf->processNoiseCov.at<float>(0) = 1e-2;
	kf->processNoiseCov.at<float>(6) = 1e-2;
	kf->processNoiseCov.at<float>(12) = 1e-2;
	kf->processNoiseCov.at<float>(18) = 1e-2;
	kf->processNoiseCov.at<float>(24) = 1e-2;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1e-1));
	
	
	return true;
}

bool Kalman::onFinish() {
	return true;
}

bool Kalman::onStop() {
	return true;
}

bool Kalman::onStart() {
	return true;
}

void Kalman::update() {	
	double ticks = (double) cv::getTickCount();
	double dT = (ticks - prev_ticks) / cv::getTickFrequency(); //seconds

	// Process Noise Covariance Matrix Q
	// [ Ex 0  0    0    0  ]
	// [ 0  Ey 0    0    0  ]
	// [ 0  0  Ev_x 0    0  ]
	// [ 0  0  0    Ev_y 0  ]
	// [ 0  0  0    0    Er ]
	cv::setIdentity(kf->processNoiseCov, cv::Scalar(1e-3 * cov_process));

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1e-3 * cov_measurement));

	// Kalman PREDICT
	if (tracking)
	{
		// >>>> Matrix A
		kf->transitionMatrix.at<float>(2) = dT;
		kf->transitionMatrix.at<float>(8) = dT;
		// <<<< Matrix A

		CLOG(LINFO) << "dT: " << dT;

		state = kf->predict();
		CLOG(LINFO) << "State post: " << std::endl << state;
		 
		std::vector<float> out;
		out.push_back(state.at<float>(0, 0));
		out.push_back(state.at<float>(0, 1));
		out.push_back(state.at<float>(0, 4));
		out_pred.write(out);
	}       
	
	if (in_meas.empty())
	{
		lost_counter++;
		CLOG(LDEBUG) << "lost_counter: " << lost_counter;
		
		if( lost_counter >= 50 )
		{
			tracking = false;
		} else {
			kf->statePost = state;
			kf->errorCovPost = kf->errorCovPre;
		}
	}
	else
	{
		std::vector<float> meas_v;
		while(!in_meas.empty())
			meas_v = in_meas.read();
		lost_counter = 0;

		meas.at<float>(0) = meas_v[0];
		meas.at<float>(1) = meas_v[1];
		meas.at<float>(2) = meas_v[2];

		CLOG(LDEBUG) << "Measure matrix: " << std::endl << meas;
		
		if (!tracking) // First detection!
		{
			CLOG(LDEBUG) << "Initialize: errorCovPre.size() = " << kf->errorCovPre.size();
			// >>>> Initialization
			kf->errorCovPre.at<float>(0) = 1; // px
			kf->errorCovPre.at<float>(6) = 1; // px
			kf->errorCovPre.at<float>(12) = 1;
			kf->errorCovPre.at<float>(18) = 1;
			kf->errorCovPre.at<float>(24) = 1; // px

			CLOG(LDEBUG) << "state.size()=" << state.size();
			state.at<float>(0) = meas.at<float>(0);
			state.at<float>(1) = meas.at<float>(1);
			state.at<float>(2) = 0;
			state.at<float>(3) = 0;
			state.at<float>(4) = meas.at<float>(2);
			// <<<< Initialization
			
			kf->statePost = state;
			kf->errorCovPost = kf->errorCovPre;

			tracking = true;
		} else {
			kf->correct(meas); // Kalman CORRECTION
		}

	}
	prev_ticks = ticks;
	// <<<<< Kalman Update
}



} //: namespace Kalman
} //: namespace Processors

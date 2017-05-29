/*!
 * \file
 * \brief
 * \author Maciej
 */

#include <memory>
#include <string>

#include "BackgroundEstimator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include "opencv2/bgsegm.hpp"


namespace Processors {
namespace BackgroundEstimator {

BackgroundEstimator::BackgroundEstimator(const std::string & name) :
		Base::Component(name) , 
		method("method", std::string("MOG"), "combo"), 
		rate("rate", 20, "range"),
		automatic("automatic", false) {
	
	method.addConstraint("MOG");
	method.addConstraint("MOG2");
	method.addConstraint("GMG");
	registerProperty(method);
	registerProperty(rate);
	registerProperty(automatic);
	
	
	
	reset_flag = false;
}

BackgroundEstimator::~BackgroundEstimator() {
}

void BackgroundEstimator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&BackgroundEstimator::onNewImage, this));
	addDependency("onNewImage", &in_img);
	registerHandler("reset", boost::bind(&BackgroundEstimator::reset, this));

}

bool BackgroundEstimator::onInit() {
	//create Background Subtractor objects
	pMOG = cv::bgsegm::createBackgroundSubtractorMOG(); //MOG approach
	pMOG2 = cv::createBackgroundSubtractorMOG2(); //MOG2 approach
	pGMG = cv::bgsegm::createBackgroundSubtractorGMG(); //GMG approach
	return true;
}

bool BackgroundEstimator::onFinish() {
	return true;
}

bool BackgroundEstimator::onStop() {
	return true;
}

bool BackgroundEstimator::onStart() {
	return true;
}

void BackgroundEstimator::onNewImage() {
	cv::Mat frame = in_img.read();
	float r = 0.01f * rate;
	if (automatic) {
		r = -1;
	}
	if (reset_flag) {
		r = 1;
		reset_flag = false;
	}
	
	if (method == "MOG")
		pSub = pMOG;
	if (method == "MOG2")
		pSub = pMOG2;
	if (method == "GMG")
		pSub = pGMG;
	
	pSub->apply(frame, fgMaskMOG2, r);
	out_img.write(fgMaskMOG2.clone());
}

void BackgroundEstimator::reset() {
	reset_flag = true;
}



} //: namespace BackgroundEstimator
} //: namespace Processors

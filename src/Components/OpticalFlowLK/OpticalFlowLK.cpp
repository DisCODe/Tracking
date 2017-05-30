/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk
 */

#include <memory>
#include <string>

#include "OpticalFlowLK.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace OpticalFlowLK {

OpticalFlowLK::OpticalFlowLK(const std::string & name) :
		Base::Component(name) , 
		tracker_window("tracker.window", 10, "range"), 
		tracker_pyramids("tracker.pyramids", 3, "range"), 
		tracker_min_eigen("tracker.min_eigen", 10, "range"), 
		tracker_term_count("tracker.term_count", 30, "range"), 
		tracker_term_eps("tracker.term_eps", 30, "range"), 
		detector_count("detector.count", 100, "range"), 
		detector_quality("detector.quality", 10, "range"), 
		detector_min_dist("detector.min_dist", 10, "range"), 
		detector_block_size("detector.block_size", 1, "range"),
		detector_subpix("detector.subpix", 5, "range") {
	
	registerProperty(tracker_window);
	registerProperty(tracker_pyramids);
	registerProperty(tracker_min_eigen);
	registerProperty(tracker_term_count);
	registerProperty(tracker_term_eps);
	
	detector_count.addConstraint("10");
	detector_count.addConstraint("1000");
	registerProperty(detector_count);
	registerProperty(detector_quality);
	registerProperty(detector_min_dist);
	registerProperty(detector_block_size);
	registerProperty(detector_subpix);

	flag_reinit = false;
	flag_clear = false;
}

OpticalFlowLK::~OpticalFlowLK() {
}

void OpticalFlowLK::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("in_point", &in_point);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&OpticalFlowLK::onNewImage, this));
	addDependency("onNewImage", &in_img);
	registerHandler("Clear", boost::bind(&OpticalFlowLK::Clear, this));
	registerHandler("Reinitialize", boost::bind(&OpticalFlowLK::Reinitialize, this));

}

bool OpticalFlowLK::onInit() {

	return true;
}

bool OpticalFlowLK::onFinish() {
	return true;
}

bool OpticalFlowLK::onStop() {
	return true;
}

bool OpticalFlowLK::onStart() {
	return true;
}

void OpticalFlowLK::onNewImage() {
	
	cv::Mat img = in_img.read();
	cv::Mat out = img.clone();
	
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, tracker_term_count, 1e-3 * tracker_term_eps);
	
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
	
	if (prev_img.empty()) {
		prev_img = img.clone();
		return;
	}
	
	if (flag_clear) {
		flag_clear = false;
		points[0].clear();
		points[1].clear();
	}
	
	if (flag_reinit) {
		cv::Size subPixWinSize(detector_subpix * 2 + 1, detector_subpix * 2 + 1);
		cv::goodFeaturesToTrack(img, points[0], detector_count, 1e-3 * detector_quality, detector_min_dist, cv::Mat(), detector_block_size * 2 + 1, 0, 0.04);
		cornerSubPix(img, points[0], subPixWinSize, cv::Size(-1,-1), termcrit);
		flag_reinit = false;
	}
	
	
	cv::Size winSize(tracker_window*2+1, tracker_window*2+1);
	
	while (!in_point.empty()) {
		std::vector<cv::Point2f> tmp;
		tmp.push_back(in_point.read());
		std::cout << tmp[0];
		cv::cornerSubPix( img, tmp, winSize, cv::Size(-1,-1), termcrit);
		points[0].push_back(tmp[0]);
		std::cout << " | " << tmp[0] << std::endl;
	}
	
	if (points[0].empty()) {
		out_img.write(img);
		return;
	}
	
	std::vector<uchar> status;
	std::vector<float> err;
	cv::calcOpticalFlowPyrLK(prev_img, img, points[0], points[1], status, err, winSize, tracker_pyramids, termcrit, 0, 1e-4 * tracker_min_eigen);
	
	
	
	size_t i, k;
	for( i = k = 0; i < points[1].size(); i++ )
	{
			if( !status[i] )
					continue;

			points[1][k++] = points[1][i];
			cv::circle( out, points[1][i], 3, cv::Scalar(0,255,0), -1, 8);
			cv::line(out, points[0][i], points[1][i], cv::Scalar(0, 0, 255), 1, 8);
	}
	points[1].resize(k);
		
	points[0] = points[1];
		
	out_img.write(out);
	
	prev_img = img.clone();
}

void OpticalFlowLK::Clear() {
	flag_clear = true;
}

void OpticalFlowLK::Reinitialize() {
	flag_reinit = true;
}



} //: namespace OpticalFlowLK
} //: namespace Processors

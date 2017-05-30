/*!
 * \file
 * \brief
 * \author Maciej
 */

#include <memory>
#include <string>

#include "OpticalFlowFarneback.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace OpticalFlowFarneback {

OpticalFlowFarneback::OpticalFlowFarneback(const std::string & name) :
		Base::Component(name),
		pyr_scale("pyr_scale", 5, "range"),
		levels("levels", 3, "range"),
		window("window", 15, "range"),
		iterations("iterations", 3, "range"),
		poly_n("poly_n", std::string("5"), "combo"),
		poly_sigma("poly_sigma", 11, "range"),
		show_vectors("show_vectors", true) {

	pyr_scale.addConstraint("1");
	pyr_scale.addConstraint("9");
	registerProperty(pyr_scale);
	levels.addConstraint("1");
	levels.addConstraint("5");
	registerProperty(levels);
	registerProperty(window);
	iterations.addConstraint("1");
	iterations.addConstraint("9");
	registerProperty(iterations);
	poly_n.addConstraint("5");
	poly_n.addConstraint("7");
	registerProperty(poly_n);
	registerProperty(poly_sigma);

	registerProperty(show_vectors);
}

OpticalFlowFarneback::~OpticalFlowFarneback() {
}

void OpticalFlowFarneback::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_flow", &out_flow);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&OpticalFlowFarneback::onNewImage, this));
	addDependency("onNewImage", &in_img);

}

bool OpticalFlowFarneback::onInit() {

	return true;
}

bool OpticalFlowFarneback::onFinish() {
	return true;
}

bool OpticalFlowFarneback::onStop() {
	return true;
}

bool OpticalFlowFarneback::onStart() {
	return true;
}

void OpticalFlowFarneback::onNewImage() {
	cv::Mat img = in_img.read();
	cv::Mat out = img.clone();
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
	cv::Mat flow;
	
	if (prev_img.empty()) {
		prev_img = img.clone();
		return;
	}
	
	int poly_n_int = 5;
	if (poly_n == "5")
		poly_n_int = 5;
	if (poly_n == "7")
		poly_n_int = 7;
		
	cv::calcOpticalFlowFarneback(prev_img, img, flow, 1e-1 * pyr_scale, levels, window, iterations, poly_n_int, 1e-1 * poly_sigma, 0 /* flags */);
	
	out_flow.write(flow);
	
	if (show_vectors) {
		int step = 20;
		for (int y = step; y < out.size().height; y+=step) {
			for (int x = step; x < out.size().width; x+=step) {
				cv::Point2f v(flow.at<cv::Point2f>(y,x));
				cv::Point2f cur(x,y);
				cv::circle( out, cur, 1, cv::Scalar(0,255,0), -1, 8);
				cv::line(out, cur, cur-v, cv::Scalar(0, 0, 255), 1, 8);
			}
		}
	} else {
		std::vector<cv::Mat> cart;
		std::vector<cv::Mat> polar; /* magnitude, angle */
		polar.push_back(cv::Mat());
		polar.push_back(cv::Mat());
		
		cv::split(flow, cart);
		
		cv::cartToPolar(cart[0], cart[1], polar[0], polar[1], true /* angleInDegrees */);
		polar[0] = polar[0] * 10;
		polar[1] = polar[1] * 0.5;
		//cv::normalize(polar[0], polar[0], 0, 255, cv::NORM_MINMAX);
		std::vector<cv::Mat> hsv;
		hsv.push_back(polar[1]);
		hsv.push_back(cv::Mat::ones(polar[0].size(), polar[0].type()) * 255);
		hsv.push_back(polar[0]);
		
		cv::merge(hsv, out);
		out.convertTo(out, CV_8UC3);
		cv::cvtColor(out, out, cv::COLOR_HSV2BGR);
	}
	
	prev_img = img.clone();
	
	out_img.write(out);
}



} //: namespace OpticalFlowFarneback
} //: namespace Processors

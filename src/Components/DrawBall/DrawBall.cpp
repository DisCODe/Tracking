/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk
 */

#include <memory>
#include <string>

#include "DrawBall.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace DrawBall {

DrawBall::DrawBall(const std::string & name) :
		Base::Component(name) , 
		decay("decay", 100, "range"), 
		color("color", 0, "range") {
	registerProperty(decay);
	
	color.addConstraint("0");
	color.addConstraint("360");
	registerProperty(color);
	registerProperty(decay);
}

DrawBall::~DrawBall() {
}

void DrawBall::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_ball", &in_ball);
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&DrawBall::onNewImage, this));
	addDependency("onNewImage", &in_img);

}

bool DrawBall::onInit() {

	return true;
}

bool DrawBall::onFinish() {
	return true;
}

bool DrawBall::onStop() {
	return true;
}

bool DrawBall::onStart() {
	return true;
}

void DrawBall::onNewImage() {
	cv::Mat img = in_img.read().clone();
	
	cv::Mat tmp(1, 1, CV_32FC3);
	cv::Point3f hsv(color, 1, 1);
	tmp.at<cv::Point3f>(0,0) = hsv;
	cv::cvtColor(tmp, tmp, cv::COLOR_HSV2BGR);
	cv::Point3f rgb = tmp.at<cv::Point3f>(0,0);
	cv::Scalar col(255*rgb.x, 255*rgb.y, 255*rgb.z);
	
	if (trace_img.empty()) {
		trace_img = cv::Mat::zeros(img.size(), img.type());
	} else {
		trace_img *= (1.0 - 1.0/decay);
	}
	
	if (!in_ball.empty()) {
		std::vector<float> ball = in_ball.read();
		cv::Point2f c(ball[0], ball[1]);
		
		if (!last_ball.empty()) {
			cv::Point2f last_c(last_ball[0], last_ball[1]);
			cv::line(trace_img, c, last_c, col, 2);
		}
		
		img += trace_img;
		cv::circle(img, c, ball[2], col, 2);
		last_ball = ball;
	} else {
		img += trace_img;
		last_ball.clear();
	}
	
	out_img.write(img);
}



} //: namespace DrawBall
} //: namespace Processors

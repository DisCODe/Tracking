/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk
 */

#ifndef DRAWBALL_HPP_
#define DRAWBALL_HPP_

#include "Base/Component_Aux.hpp"
#include "Base/Component.hpp"
#include "Base/DataStream.hpp"
#include "Base/Property.hpp"
#include "Base/EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace DrawBall {

/*!
 * \class DrawBall
 * \brief DrawBall processor class.
 *
 * 
 */
class DrawBall: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	DrawBall(const std::string & name = "DrawBall");

	/*!
	 * Destructor
	 */
	virtual ~DrawBall();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn<std::vector<float> > in_ball;
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<int> decay;
	Base::Property<int> color;

	
	// Handlers
	void onNewImage();
	
	cv::Mat trace_img;
	std::vector<float> last_ball;
};

} //: namespace DrawBall
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DrawBall", Processors::DrawBall::DrawBall)

#endif /* DRAWBALL_HPP_ */

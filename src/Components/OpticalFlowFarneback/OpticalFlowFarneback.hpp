/*!
 * \file
 * \brief 
 * \author Maciej
 */

#ifndef OPTICALFLOWFARNEBACK_HPP_
#define OPTICALFLOWFARNEBACK_HPP_

#include "Base/Component_Aux.hpp"
#include "Base/Component.hpp"
#include "Base/DataStream.hpp"
#include "Base/Property.hpp"
#include "Base/EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace OpticalFlowFarneback {

/*!
 * \class OpticalFlowFarneback
 * \brief OpticalFlowFarneback processor class.
 *
 * 
 */
class OpticalFlowFarneback: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	OpticalFlowFarneback(const std::string & name = "OpticalFlowFarneback");

	/*!
	 * Destructor
	 */
	virtual ~OpticalFlowFarneback();

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
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_flow;
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<int> pyr_scale;
	Base::Property<int> levels;
	Base::Property<int> window;
	Base::Property<int> iterations;
	Base::Property<std::string> poly_n;
	Base::Property<int> poly_sigma;
	Base::Property<bool> show_vectors;
	
	// Handlers
	void onNewImage();
	
	cv::Mat prev_img;

};

} //: namespace OpticalFlowFarneback
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("OpticalFlowFarneback", Processors::OpticalFlowFarneback::OpticalFlowFarneback)

#endif /* OPTICALFLOWFARNEBACK_HPP_ */

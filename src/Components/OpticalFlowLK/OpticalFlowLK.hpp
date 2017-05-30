/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk
 */

#ifndef OPTICALFLOWLK_HPP_
#define OPTICALFLOWLK_HPP_

#include "Base/Component_Aux.hpp"
#include "Base/Component.hpp"
#include "Base/DataStream.hpp"
#include "Base/Property.hpp"
#include "Base/EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace OpticalFlowLK {

/*!
 * \class OpticalFlowLK
 * \brief OpticalFlowLK processor class.
 *
 * 
 */
class OpticalFlowLK: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	OpticalFlowLK(const std::string & name = "OpticalFlowLK");

	/*!
	 * Destructor
	 */
	virtual ~OpticalFlowLK();

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
	Base::DataStreamIn<cv::Point2f> in_point;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<int> tracker_window;
	Base::Property<int> tracker_pyramids;
	Base::Property<int> tracker_min_eigen;
	Base::Property<int> tracker_term_count;
	Base::Property<int> tracker_term_eps;
	Base::Property<int> detector_count;
	Base::Property<int> detector_quality;
	Base::Property<int> detector_min_dist;
	Base::Property<int> detector_block_size;
	Base::Property<int> detector_subpix;

	
	// Handlers
	void onNewImage();
	void Clear();
	void Reinitialize();
	
	bool flag_reinit;
	bool flag_clear;
	
	cv::Mat prev_img;
	
	std::vector<cv::Point2f> points[2];

};

} //: namespace OpticalFlowLK
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("OpticalFlowLK", Processors::OpticalFlowLK::OpticalFlowLK)

#endif /* OPTICALFLOWLK_HPP_ */

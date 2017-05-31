/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk
 */

#ifndef KALMAN_HPP_
#define KALMAN_HPP_

#include "Base/Component_Aux.hpp"
#include "Base/Component.hpp"
#include "Base/DataStream.hpp"
#include "Base/Property.hpp"
#include "Base/EventHandler2.hpp"

#include <opencv2/opencv.hpp>

namespace Processors {
namespace Kalman {

/*!
 * \class Kalman
 * \brief Kalman processor class.
 *
 * 
 */
class Kalman: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Kalman(const std::string & name = "Kalman");

	/*!
	 * Destructor
	 */
	virtual ~Kalman();

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
	Base::DataStreamIn<std::vector<float> > in_meas;

	// Output data streams
	Base::DataStreamOut<std::vector<float> > out_pred;

	// Handlers

	// Properties
	Base::Property<int> cov_process;
	Base::Property<int> cov_measurement;

	
	// Handlers
	void update();

	cv::KalmanFilter * kf;
	cv::Mat state, meas;
	
	bool tracking;
	int lost_counter;
	double prev_ticks;
};

} //: namespace Kalman
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Kalman", Processors::Kalman::Kalman)

#endif /* KALMAN_HPP_ */

/*
 * controller.h
 *
 *  Created on: Oct 26, 2018
 *      Author: giovani
 */

#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif
		double controller(
							double Th, // Theta reference
							double X,  // X point reference
							double Y,  // Y point reference
							double et, // theta error
							double ex, //  X error
							double lamda, // lambda paramenter of the controller
							double pho, // pho: tilt angle of the camera
							double ty, // y axis translation to camera reference
							double tz, // z axis translation to camera reference
							double v // speed...
						  );

#ifdef __cplusplus
}
#endif

#endif /* SRC_CONTROLLER_H_ */

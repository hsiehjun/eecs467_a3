#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP
#include <math.h>
#include <string>

///////////////////////////////
// PHYSICAL CONSTANTS
///////////////////////////////

// arm lengths are in meters
const float ARM_LENGTH_0 = 0.12;
const float ARM_LENGTH_1 = 0.103;
const float ARM_LENGTH_2 = 0.1;
const float ARM_LENGTH_3 = 0.175; // including claw
const float CLAW_LENGTH = 0.08; // from claw joint to tip
const float ARM_ANGLE_MIN[] = {-3.14,-2.094,-2.094,-2.094,-2.618,-0.9 };
const float ARM_ANGLE_MAX[] = { 3.14, 2.094, 2.094, 2.094, 2.618, 2.3 };

const float ARM_SPEED = 0.2;
const float ARM_MAX_TORQUE = 0.7;
const std::string ARM_CHANNEL_NAME = "ARM_COMMAND";
const float ARM_ERROR_THRESH = 0.1;

const std::string colorCalibrationFileName = "color_calib.txt";

//////////////////////////////
// BLOB DETECTION CONSTANTS
//////////////////////////////
const float noObjectThresholdVal = 0.4;
const float noObjectThresholdSat = 0.4;
const double objectHueRedHalfRange = 15.0;
const double objectHueGreenHalfRange = 15.0;
const double objectHueBlueHalfRange = 15.0;
const double RAD = M_PI / 180;
const float blobMinPixels = 400;


// (x, y) global coordinates of blue corners in meters
const float centerToCenterDistance = 0.24;
const float cornerTopLeftLoc[] = {0, centerToCenterDistance/2};
const float cornerTopRightLoc[] = {centerToCenterDistance, centerToCenterDistance/2};
const float cornerBottomLeftLoc[] = {0, -centerToCenterDistance/2};
const float cornerBottomRightLoc[] = {centerToCenterDistance, -centerToCenterDistance/2};
 
// the length of one side of the board in meters.
const float boardSideLength = 0.18;

#endif /* CONSTANTS_HPP */

#ifndef KP_UTILITIES_H_INCLUDED
#define KP_UTILITIES_H_INCLUDED

#include <stdlib.h>
#include <iostream>
#include <fstream> //file io - of course
#include <time.h>
#include <pthread.h> //for threading

#include <string.h>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include "Aria.h"
#include "CoordData.h"
#ifndef _startKinect_H_
#define _startKinect_H_
#include "CSystemData.h"
#endif
#include "ArActionCBB.h"
#include "ArActionMatlab.h"
#include "CRobotInfo.h"
#include "CRuntimeSettings.h"

#define PI 3.14159265
#define D2R ((double)PI/180)
#define R2D ((double)180/PI)

//=====================================================
// General Description:
//
// This file exists to make the initialization process for
// the robots easier, making the main() a happier place to be.
// It includes an import function for runtime settings,
// as well as handling most robot startup tasks.
//=====================================================

namespace kp
{

//=====================================================
// configureRobots()
//
// Configures robots with actions, key handlers, and subscribes them
// to vicon.  This is a very important step in the initialization process,
// because here and only here we tell our robots what actions we'd like them
// to do.
//=====================================================
void configureRobots(CSystemData &mySysData, CRuntimeSettings &importedSettings);



}
#endif // KP_UTILITIES_H_INCLUDED

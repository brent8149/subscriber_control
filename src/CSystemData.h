#ifndef CSYSTEMDATA_H
#define CSYSTEMDATA_H

#include <fstream>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include <cstdint>
#include <thread>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include "Aria.h"
#include "CoordData.h"
#include "CVector.h"
#include "CRobotInfo.h"
#include "CRuntimeSettings.h"
#include <k2_client/Audio.h>
#include <k2_client/Body.h>
#include <k2_client/BodyArray.h>
#include <k2_client/JointPositionAndState.h>
#include <k2_client/JointOrientationAndType.h>
#include <k2_client/Lean.h>


//define globals
#define PI 3.14159265
#define D2R ((double)PI/180)
#define R2D ((double)180/PI)

//=====================================================
// General Description:
//
// Think of this class as the system "overseer".  It knows
// where all the relevant system objects are (positions, etc)
// and has handles to the actual robots we're controlling.
// Note that this class doesn't do any direct controlling of robot
// actions, those are handled through Aria "actions".
// This class also maintains a system log and signals the main thread
// to exit.
//=====================================================
class CSystemData
{
protected:
    std::string logFileStr;
    std::ofstream logFile;
    std::string filePath;
    long int updateTime, startTime;
    double tstep;
    int numRobots;
    float cycleFreq;

    CRuntimeSettings *myRuntimeSettings;

private:

public:
    ArRobot *robot;
    CRobotInfo *robotData;
    CRobotInfo beaconData;

    ArTcpConnection *TCPCon;
    ArSonarDevice *sonarDev;

    // Functors, intended to be bound to function pointers then
    // attached to key handlers (when you press the button, the
    // function happens)
    ArFunctorC<CSystemData> systemExitFunct;
    ArFunctorC<CSystemData> increaseVelocityFunct;
    ArFunctorC<CSystemData> decreaseVelocityFunct;
    ArFunctorC<CSystemData> stopVelocityFunct;

    // Those keyhandlers we've heard so much about.  There's only one
    // per program!
    ArKeyHandler keyHandler;

    // Subscriber objects
    ros::Subscriber coordinateSub;
    ros::Subscriber matlabSub;

    float u_control;

    // Loop Rate object
    //ros::Rate loop_rate;

    bool exitLoop;

    virtual ~CSystemData();


    //
    // Constructors
    //

    CSystemData(); //default constructor, doesn't do anything

    //=====================================================
    // CSystemData(robotNum, logpath)
    //
    // This is the preferred constructor because it initializes
    // the dynamic variables that belong to this class.  Specifically
    // it creates enough robots, robot data, and other objects
    // that are dependant on the total number of robots
    //=====================================================
    CSystemData(CRuntimeSettings &runtimeSettings);


    //
    // Member functions
    //


    //=====================================================
    // connectRobots()
    //
    // Connect to the robots acording to our runtimeSettings
    // object.  It also adds sonar.  For some reason, we
    // initialize the robotData class here, before we were
    // getting some memory corruption when connecting to the
    // robots when it was done in the CSystemData constructor.
    //=====================================================
    void connectRobots();


    //=====================================================
    // startRobots()
    //
    // Actually start robot movement.
    //=====================================================
    void startRobots();


    //=====================================================
    // updateSystemData()
    //
    // This should be tied to the nodehandle subscription.
    // It runs when ros spins and proceeds to update the data
    // in our robotData and beaconData objects.
    // Note that this takes a pointer to the vicon data,
    // which it parses.
    //=====================================================
    void updateSystemData(const vicon_reader::CoordData::ConstPtr &msg);
    void updateMatlabData(const std_msgs::Float32::ConstPtr& msg);


    //=====================================================
    // checkLogName
    //
    // Check a file name, then assign a proper name if necessary, outputs the corrected name
    // will add a timestamp if not ending in "txt".
    // This was probably needlessly complicated.
    //=====================================================
    std::string checkLogName(std::string fileName);


    //=====================================================
    // openLog()
    //
    // Opens log file safely.
    //=====================================================
    void openLog();


    //=====================================================
    // closeLog()
    //
    // Close log file safely.
    //=====================================================
    void closeLog();


    //=====================================================
    // systemExit()
    //
    // Signals the system to exit and closes the system log
    //=====================================================
    void systemExit();


    //=====================================================
    // writeSysLog()
    //
    // Write system data to the log.
    // Currently: beacon position, robot position, heading.
    //=====================================================
    void writeSysLog();


    //=====================================================
    // increaseVelocity()
    //
    // Intended to be a "functor" and added as a key binding
    // to an arrow key.  Increases velocity by 10.00.
    //=====================================================
    void increaseVelocity();


    //=====================================================
    // decreaseVelocity()
    //
    // Intended to be a "functor" and added as a key binding
    // to an arrow key.  Decreases velocity by 10.00.
    //=====================================================
    void decreaseVelocity();


    //=====================================================
    // decreaseVelocity()
    //
    // Stops the robots (sets velocity to 0 in the robotInfo
    // data structure.
    //=====================================================
    void stopVelocity();

};

#endif // CSYSTEMDATA_H

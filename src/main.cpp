

//utility header has all of our useful libraries included
#include "kp_utilities.h"

//=====================================================
// cbl_beacon_1_3
//
// This is the main function.  It is intended to be as
// abstract and high level as possible, with the smaller
// components being handled by either "kp_utilities" or
// some of the system data objects.
//=====================================================


int main(int argc, char** argv)
{
    // Intitialize ros/aria
    ros::init(argc, argv, "kp");
    ros::NodeHandle nodeHandle;
    Aria::init();

    /// Import initilization from file and store them in mySettings, true is for verbose
    CRuntimeSettings mySettings(true);

    // Set the loop rate (Hz)
    ros::Rate loop_rate(floor(mySettings.cycleFreq));

    // Create our system data based on the settings we've just imported.  The system data
    // pointer we give it will be saved inside the class for further use.
    CSystemData mySystemData(mySettings);

    // Now connect to the robots
    mySystemData.connectRobots();


    /// Configure the key handler, bind the handler to the FUNCTOR, not the function we'd like to excecute
    mySystemData.keyHandler.addKeyHandler(ArKeyHandler::UP, &mySystemData.systemExitFunct);
    mySystemData.keyHandler.addKeyHandler(ArKeyHandler::LEFT, &mySystemData.decreaseVelocityFunct);
    mySystemData.keyHandler.addKeyHandler(ArKeyHandler::RIGHT, &mySystemData.increaseVelocityFunct);
    mySystemData.keyHandler.addKeyHandler(ArKeyHandler::DOWN, &mySystemData.stopVelocityFunct);

    // Every time ros "spins", the system update function runs, update system data
    // We only have ONE subscriber - the computer, not each individual robot.  I.E.
    // our system object is just one node.
   // mySystemData.coordinateSub = nodeHandle.subscribe("coordData", 10, &CSystemData::updateSystemData, &mySystemData);
    //ros::Subscriber sub = nodeHandle.subscribe("/head/kinect2/audio", 10, chatterCallback);
    mySystemData.matlabSub = nodeHandle.subscribe("/head/control/scan", 10,  &CSystemData::updateMatlabData, &mySystemData);

    /// Add actions to our robot.
    std::cout<<"Adding actions to robots..."<<std::endl;
    kp::configureRobots(mySystemData, mySettings);

    ROS_INFO("Ready!");

    // Open the system log
    //mySystemData.openLog();

    // Get data flowing to the appropriate structures before we move the robots
    ros::spinOnce();
    ArUtil::sleep(2000);

    // Start the robots
    mySystemData.startRobots();

    std::cout<<"Robots have been started!"<<std::endl;

    // Start our master "loop" so to speak.  We do a loop rate sleep
    // for syncing purposes.  The actual processing and robot actions
    // take place in other threads you can't see!  The actions you added
    // run automatically, and the "resolver" decides what action to do.
    // Now you just sit back and watch.
    while ((nodeHandle.ok()))// && (!mySystemData.exitLoop))
    {
        // Get vicon data
        ros::spinOnce();

        // Log data to our text file
        //mySystemData.writeSysLog();

        // Sync up
        loop_rate.sleep();
    }

    // Clean things up.
    ROS_INFO("Exiting!!!\n");
    Aria::exit(0);
    ros::shutdown();
    return 0;
}

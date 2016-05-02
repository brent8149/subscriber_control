#include "kp_utilities.h"



//define the kp namespace for our utilities
namespace kp
{


void configureRobots(CSystemData &mySysData, CRuntimeSettings &importedSettings)
{

    for (int i = 0; i< importedSettings.numRobots; i++)
    {

        // Add actions for each robot
        mySysData.robot[i].addAction(new ArActionStallRecover(), 100);

        if(importedSettings.withSonar == true)
        {
            //mySysData.robot[i].addAction(new ArActionAvoidFront(), 50);
            mySysData.robot[i].addAction(new ArActionAvoidFront("Avoid Front Near", 250, 100), 52);
        }

        mySysData.robot[i].addAction( new ArActionMatlab(mySysData.robotData[i].name.c_str(),
                                  mySysData.robotData[(i+(importedSettings.numRobots-1))%importedSettings.numRobots],
                                    &mySysData.u_control),
                                    25);

        //   mySysData.robot[i].addAction( new ArActionKinect(mySysData.robotData[i].name.c_str(), &mySysData.beam_angle, &mySysData.beam_confidence), 25);
        //   mySysData.robot[i].addAction( new ArActionBody(mySysData.robotData[i].name.c_str(), &mySysData.pos_x, &mySysData.pos_y, &mySysData.pos_z), 25);
        //mySysData.robot[i].addAction( new ArActionBody(mySysData.robotData[i].name.c_str(), &mySysData.pos_x, &mySysData.pos_y,  &mySysData.pos_z), 25);

        // Add the actual control laws, this is currently configured for CBL
        // so both are running CBB, but only one has a lambda weight
        /*{
        mySysData.robot[i].addAction( new ArActionCBB(
                                          mySysData.robotData[i].name.c_str(),
                                          mySysData.robotData[(i+(importedSettings.numRobots-1))%importedSettings.numRobots],
                                          mySysData.robotData[i],
                                          mySysData.robotData[(i+1)%importedSettings.numRobots],
                                          mySysData.beaconData,
                                          importedSettings.alphaNeighbor,
                                          importedSettings.alphaBeacon,
                                          importedSettings.leaderLambda,
                                          importedSettings.mu,
                                          importedSettings.cycleFreq), 25); ///need seperate beacon name for data

        }
        else
        {
        mySysData.robot[i].addAction( new ArActionCBB(
                                          mySysData.robotData[i].name.c_str(),
                                          mySysData.robotData[(i+(importedSettings.numRobots-1))%importedSettings.numRobots],
                                          mySysData.robotData[i],
                                          mySysData.robotData[(i+1)%importedSettings.numRobots],
                                          mySysData.beaconData,
                                          importedSettings.alphaNeighbor,
                                          importedSettings.alphaBeacon,
                                          0.00,
                                          importedSettings.mu,
                                          importedSettings.cycleFreq), 25);
        }*/

        // Add key handler to each robot
        mySysData.robot[i].attachKeyHandler(&mySysData.keyHandler);
        mySysData.robot[i].setCycleTime(floor(1000.0/importedSettings.cycleFreq));
        mySysData.robot[i].setCycleChained(false);

        // Print information about the robot configuration.
        ROS_INFO("Loop Rate: %f", importedSettings.cycleFreq);
        ROS_INFO("Aria Cycle Time: %u", mySysData.robot[i].getCycleTime());
        ROS_INFO("Aria Cycle Warning Time: %u", mySysData.robot[i].getCycleWarningTime());
        ROS_INFO("Aria Connection Timeout Time: %d", mySysData.robot[i].getConnectionTimeoutTime());

        if(mySysData.robot[i].areSonarsEnabled())
        {
            ROS_INFO("Sonar enabled!");
        }
    }
}


}


#include "CSystemData.h"


//
// Constructors
//


CSystemData::CSystemData()
{
// Nothing to see here
}


CSystemData::CSystemData(CRuntimeSettings &runtimeSettings)
{
    numRobots = runtimeSettings.numRobots;

    // Create the correct amount of robots
    robot = new ArRobot[numRobots];
    robotData = new CRobotInfo[numRobots];

    // Create corresponding connection and sonar objects
    TCPCon = new ArTcpConnection[numRobots];
    sonarDev = new ArSonarDevice[numRobots];

    // Create functors - handles to functions for key bindings
    systemExitFunct = ArFunctorC<CSystemData>(this, &CSystemData::systemExit);
    increaseVelocityFunct = ArFunctorC<CSystemData>(this, &CSystemData::increaseVelocity);
    decreaseVelocityFunct = ArFunctorC<CSystemData>(this, &CSystemData::decreaseVelocity);
    stopVelocityFunct = ArFunctorC<CSystemData>(this, &CSystemData::stopVelocity);

    // Make sure we don't exit right away
    exitLoop = false;

    // Assign the proper name to the file for opening later
   // logFileStr = checkLogName(runtimeSettings.logpath);


    // Finally make the runtime settings local to the CSystem class
    myRuntimeSettings = &runtimeSettings;
    u_control = 0;

}


CSystemData::~CSystemData()
{
// Nothing to see here
}



//
// Member functions
//

void CSystemData::connectRobots()
{
    int ret;
    int i; //counter
    std::string str;
    std::cout<<"Connecting to robots!"<<std::endl;

    // Open connections to each of our robot friends
    //for (i = 0; i < numRobots; i++)
    //{
    i = 0;
            std::cout<<"Connecting to robot 1!"<<std::endl;
        if ((ret = TCPCon[i].open(myRuntimeSettings->host[i].c_str(), myRuntimeSettings->port[i])) != 0)
        {
            str = TCPCon[i].getOpenMessage(ret);
            std::cout << "Open to robot failed: " << str.c_str() << std::endl;
            Aria::exit(1);
        }

        robot[i].setDeviceConnection(&TCPCon[i]);
        if (!robot[i].blockingConnect())
        {
            std::cout << "Connect to robot failed" << std::endl;
            Aria::exit(1);
        }

        // Add sonar.
        if(myRuntimeSettings->withSonar == true)
        {
            robot[i].addRangeDevice(&sonarDev[i]);
            robot[i].enableSonar();
        }
        else
        {
            robot[i].disableSonar();
        }

        // For some reason, this needs to initialize AFTER connecting
        robotData[i] = CRobotInfo(myRuntimeSettings->name[i], i, myRuntimeSettings->velocity);

    //}

    // If the beacon exists, add it to the system
    if (strcmp(myRuntimeSettings->beacon_name.c_str(), "") != 0 )
    {
        beaconData = CRobotInfo(myRuntimeSettings->beacon_name, 0, 0);
    }

}

void CSystemData::startRobots()
{
    //start each robot, velocity is handled by each action
    for (int i = 0; i< numRobots; i++)
    {
        robot[i].runAsync(true);
        robot[i].lock();

        robot[i].comInt(ArCommands::ENABLE, 1);
        robot[i].comInt(ArCommands::SOUNDTOG, 0);

        robot[i].unlock();
    }
}

void CSystemData::updateSystemData(const vicon_reader::CoordData::ConstPtr &msg)
{
    // Update the time we're getting our information
    updateTime = ros::Time::now().toNSec();

    // Give the data to the appropriate robot structure
    for (int i = 0; i < numRobots; i++)
    {
        if(robotData[i].name.compare(msg->name) == 0)
        {
            robotData[i].update(msg->x, msg->y, msg->heading, msg->frame,
                                msg->time, msg->latency, ros::Time::now().toNSec());
        }
        else if (beaconData.name.compare(msg->name) == 0)
        {
            beaconData.update(msg->x, msg->y, msg->heading, msg->frame,
                              msg->time, msg->latency, ros::Time::now().toNSec());
        }
    }
}

    void CSystemData::updateMatlabData(const std_msgs::Float32::ConstPtr& msg){

// Beam angle range is -50 to 50 degrees
// Beam angle confidence range is 0 to 1

    //ROS_INFO("Beam angle: [%f], Confidence level: [%f]", msg->beamAngle*50, msg->beamAngleConfidence);
    u_control = msg->data;

}




std::string CSystemData::checkLogName(std::string fileName)
{
    std::time_t t = std::time(NULL); //for our timestamping
    std::string timestamp = std::asctime(std::localtime(&t));

    //create the new log file, check if the path in the settings file has a file extension txt, else rename it and use just the path
    if (fileName.substr((fileName.length()-3), 3) =="txt")
    {
        std::cout<<"proper text file, leaving name the same."<<std::endl;
        return fileName;
    }
    else if (fileName.substr(fileName.length()-1,1) =="/")
    {
        fileName = fileName.append(timestamp.substr(0,timestamp.length()-2))+"_log.txt"; //remove the newline from the string and add extension

        //replace spaces with underscores
        while(fileName.find(" ")!=std::string::npos)
        {
            fileName.replace(fileName.find(" "),1,"_");
        }

        //replace colons with underscores
        while(fileName.find(":")!=std::string::npos)
        {
            fileName.replace(fileName.find(":"),1,"_");
        }

        std::cout<<"Log file set to: "<<fileName<<std::endl;

        return fileName;
    }
    else
    {
        std::cout<<"Can't handle given file path, giving standard path: "<<std::endl;
        std::cout<<"~/log.txt"<<std::endl;
        return "~/log.txt";
    }
}


void CSystemData::openLog()
{
    logFile.open(logFileStr.c_str());
    if (logFile.is_open())
    {
        ROS_INFO("Log file open: %s", logFileStr.c_str());
        startTime = ros::Time::now().toNSec();
    }
    else
    {
        ROS_WARN("Could not open log file: %s", logFileStr.c_str());
    }
}


void CSystemData::closeLog()
{
    if (logFile.is_open())
    {
        logFile.close();
    }
    ROS_INFO("Closing log file: %s", logFileStr.c_str());

}


void CSystemData::writeSysLog()
{
    if (logFile.is_open())
    {
        logFile << updateTime << "," << beaconData.coord_x << "," << beaconData.coord_y << ",";
        for (int i = 0; i < numRobots; i++)
        {
            logFile <<  robotData[i].r.xComponent << "," << robotData[i].r.yComponent << ","
                    << robotData[i].heading << ",";
        }
        logFile << "\n";
    }
    else
        ROS_WARN("Could not write to log file: %s", logFileStr.c_str());
}


void CSystemData::systemExit()
{
    exitLoop = true;
    closeLog();
}


void CSystemData::increaseVelocity()
{
    int i = 0;

    for(i = 0; i<=numRobots; i++)
    {
        // Check for max velocity
        if(robotData[i].velocity <= 700)
        {
            robotData[i].velocity = robotData[i].velocity + 10.00;
        }
    }

    ROS_INFO("INCREASING VELOCITY");
}


void CSystemData::decreaseVelocity()
{
    int i = 0;

    for(i = 0; i<=numRobots; i++)
    {
        // Check for min velocity
        if(robotData[i].velocity >= 0)
        {
            robotData[i].velocity = robotData[i].velocity - 10.00;
        }
    }

    ROS_INFO("DECREASING VELOCITY");
}


void CSystemData::stopVelocity()
{
    int i = 0;

    for(i = 0; i<=numRobots; i++)
    {
        robotData[i].velocity = 0;
    }

}

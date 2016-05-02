#include "CRobotInfo.h"

CRobotInfo::CRobotInfo()
{
    name = "anonymous";
    coord_x = 0;
    coord_y = 0;
    heading = 0;
    lastFrame = 0;
    lastROSTime = 0;
    viconLatency = 0;
    lastUpdateTime = 0;
    velocity = 0;

    // Update Vectors
    r = CVector(0, 0);
    x = CVector(0, 0);
    y = CVector(0, 0);

    isUpdated = false;
}

CRobotInfo::CRobotInfo(std::string myName, int indx, double vel)
{
    //initialize all values to zero
    name = myName;
    robotIndex = indx;

    velocity = vel;
    coord_x = 0;
    coord_y = 0;
    heading = 0;
    lastFrame = 0;
    lastROSTime = 0;
    viconLatency = 0;
    lastUpdateTime = 0;

    // Update Vectors
    r = CVector(0, 0);
    x = CVector(0, 0);
    y = CVector(0, 0);

    isUpdated = false;


}

CRobotInfo::~CRobotInfo()
{
    //dtor
}

CRobotInfo& CRobotInfo::operator=(const CRobotInfo& other)
{
    coord_x = other.coord_x;
    coord_y = other.coord_y;
    coord_z = other.coord_z;
    r = other.r;
    x = other.x;
    y = other.y;

    heading = other.heading;
    lastFrame = other.lastFrame;
    lastROSTime = other.lastROSTime;
    viconLatency = other.viconLatency;
    lastUpdateTime = other.lastUpdateTime;
    velocity = other.velocity;
    robotIndex = other.robotIndex;
    name = other.name;
    isUpdated = other.isUpdated;

    return *this;
}

void CRobotInfo::update(double _x, double _y, double hdg, double lFrame,
double lastRTime, double viconLat, double lUpdateTime )
{
    // Update coordinate info
    coord_x = _x;
    coord_y = _y;
    heading = hdg;
    lastFrame = lFrame;
    lastROSTime = lastRTime;
    viconLatency = viconLat;
    lastUpdateTime = lUpdateTime;

    // Update Vectors
    r = CVector(coord_x, coord_y);
    x = CVector(std::cos(D2R*heading), std::sin(D2R*heading));
    y = CVector(-1*std::sin(D2R*heading), std::cos(D2R*heading));

    isUpdated = true;

}

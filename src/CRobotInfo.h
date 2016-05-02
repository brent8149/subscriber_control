#ifndef CROBOTINFO_H
#define CROBOTINFO_H

#include <string>
#include "CVector.h"

#define PI 3.14159265
#define D2R ((double)PI/180)
#define R2D ((double)180/PI)

class CRobotInfo
{
 public:
        double coord_x, coord_y, coord_z, heading, lastFrame, lastROSTime, viconLatency, lastUpdateTime;
        double velocity;
        CVector r, x, y;


        int robotIndex;
        std::string name;

        bool isUpdated;

        // Default Contructor, initialize all values to 0
        CRobotInfo();

        // Default Contructor, initialize all values to 0 but give a name
        CRobotInfo(std::string myName, int indx, double vel);

        // Destructor
        virtual ~CRobotInfo();

        // Operators
        CRobotInfo& operator= (const CRobotInfo& other);


        //=====================================================
        // update()
        //
        // A simple way to update data, where it uses the Vicon
        // data to make the x,y, and r vectors for you
        //=====================================================
        void update(double _x, double _y, double hdg, double lFrame, double lastRTime, double viconLat, double lUpdateTime );


    protected:
    private:
};

#endif // CROBOTINFO_H

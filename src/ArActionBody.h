#ifndef ArActionBody_H
#define ArActionBody_H

#include "Aria.h"
#include "ArAction.h"
#include "ariaTypedefs.h"
#include "CRobotInfo.h"
#include  <ros/ros.h>

// Globals for our control law
#define PI 3.14159265
#define D2R ((double)PI/180)
#define R2D ((double)180/PI)

//=====================================================
// General Description:
//
// Ok.  So the goal of this class is to encapsulate an "atom" control with reference
// to the MDLE.  It should contain all necessary pointers to data to compute the next
// control upon a fire(). It should be self contained, and all appopriate parameters should
// be passed upon initialization.
//=====================================================
class ArActionBody : public ArAction
{
public:

    //=====================================================
    // ArActionKinect()
    //
    // This is the constructor.  This is where the magic happens.  We'll instantiate
    // everything we need in order for the control to function properly.  Note that
    // the parameters are passed via pointer, so when they're updated outside, there
    // are fresh values inside here.
    //=====================================================
AREXPORT ArActionBody(const char * name, float *pos_x, float *pos_y, float *pos_z);

    AREXPORT virtual ~ArActionBody();

    //=====================================================
    // fire()
    //
    // This is our fire action, which is computed every control cycle
    // it returns an action of what we'd like the robot to do.  It is
    // desired because the resolver may choose to ignore this control law
    //=====================================================
    AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);

    //=====================================================
    // getDesired()
    //
    // Simply return the desired action, i.e. what we'd like the robot to do
    // after the last fire() was done.
    //=====================================================
    AREXPORT virtual ArActionDesired *getDesired(void)
    {
        return &myDesiredAction;
    }

protected:

    ArActionDesired myDesiredAction; //the thing we want the robot to do after fire()

    // Latency variables.  Will be updated every fire().
    double latency_vicon;
    long int lastROSTime, lastUpdateTime, lastFireTime;
    int latency_aria;

    //=====================================================
    // updateAriaLatency()
    //
    // update the aria latency.  Get the robot, figure out the latency, then update
    // the data.
    //=====================================================
    void updateAriaLatency(void);

    //=====================================================
    // updateState
    //
    // Copy the data from the pointer to the data array with state variables
    // to the class's copy
    //=====================================================
    void updateState(void);


private:
    float * pos_x;
    float * pos_y;
    float * pos_z;

    //=====================================================
    // computeControl()
    //
    // This is where we computer the control based on the local state data
    // We'll assume the following parameters have been updated and given to us,
    // then we'll crunch the rest:
    //
    // This computes the CB Beacon control law, by the way.
    //=====================================================
    void computeBodyControl(void);

};

#endif // ArActionBody_H

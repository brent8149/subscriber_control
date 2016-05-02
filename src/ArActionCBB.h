#ifndef ARACTIONCBB_H
#define ARACTIONCBB_H

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
class ArActionCBB : public ArAction
{
public:

    //=====================================================
    // ArActionCBB()
    //
    // This is the constructor.  This is where the magic happens.  We'll instantiate
    // everything we need in order for the control to function properly.  Note that
    // the parameters are passed via pointer, so when they're updated outside, there
    // are fresh values inside here.
    //=====================================================
    AREXPORT ArActionCBB(const char* name, CRobotInfo &prevAgent, CRobotInfo &curAgent, CRobotInfo &nextAgent,
                         CRobotInfo &beacon, float _alpha, float _alpha_0, float _lambda,
                         float _mu, float cycFreq);

    AREXPORT virtual ~ArActionCBB();

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

    bool isFirstFire;
    bool isUpdated;
    bool isFirstErrorCalc;

    //=====================================================
    // Control and State Variables
    //=====================================================
    // Parameters
    float alpha;
    float alpha_0;
    double tstep, updateTime;
    float cycleFreq;
    float lambdaWeight;

    // Relevant data for computing the control
    double rVel;
    float velocity;

    double heading_new, heading_prev, radii;

    CVector x_new, y_new, r_expected, r_error, r_prev, y_prev;
    CVector r, r_to_neighbor, r_bi, x, y, pa_r_to_neighbor;

    // Next agent position, beacon position, and previous agent position
    CVector na_r, r_beacon, pa_r;

    double na_theta; // next agent theta

    double rho_ij, curvature, kappa, theta, kappa_beacon, nu, mu, heading;


    //=====================================================
    // Relevant Agent Information
    //=====================================================

    // This is where ALL of the state data (and parameters) are going to live
    CRobotInfo beaconInfo;
    CRobotInfo paInfo; //previous agent
    CRobotInfo naInfo;
    CRobotInfo myInfo;

    // This is pointer to the outside (updating) robot data
    CRobotInfo* p_beaconInfo;
    CRobotInfo* p_paInfo;
    CRobotInfo* p_naInfo;
    CRobotInfo* p_myInfo;

    //=====================================================
    // computeControl()
    //
    // This is where we computer the control based on the local state data
    // We'll assume the following parameters have been updated and given to us,
    // then we'll crunch the rest:
    //
    // This computes the CB Beacon control law, by the way.
    //=====================================================
    void computeControl(void);

};

#endif // ARACTIONCBB_H

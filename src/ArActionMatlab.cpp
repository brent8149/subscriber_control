#include "ArActionMatlab.h"

//=====================================================
//
// Constructors and Destructors
//
//=====================================================
AREXPORT ArActionMatlab::ArActionMatlab(const char * name, CRobotInfo &curAgent, float *u_ctrl): ArAction(name, "Control")
{
    // store the pointers to the updating robots
       u_control = u_ctrl;
       p_myInfo  = &curAgent;
       myInfo      = *p_myInfo;

       //first_fire = true;
    //
    // Set updated for our first go around the fire()
    myDesiredAction.setVel(myInfo.velocity);

}

AREXPORT ArActionMatlab::~ArActionMatlab()
{
    //dtor
}


//=====================================================
//
// Member functions
//
//=====================================================
AREXPORT ArActionDesired *ArActionMatlab::fire(ArActionDesired currentDesired)
{
    myDesiredAction.reset();

    //
    // Update Aria latency
    //
   // updateAriaLatency();


    //
    // Update state
    //
   // updateState();

    //
    // Compute controls and set the desired action
    //
    computeControl();

    //
    // Output for debugging
    //

   // ROS_INFO("Speed-: %f", myInfo.velocity);

   /* ROS_INFO("Turning Rate-: %f", rVel);
    ROS_INFO("Curvature-: %f", curvature);
    ROS_INFO("kappa-: %f", kappa);
    ROS_INFO("kappa w.r.t beacon-: %f", kappa_beacon);
    ROS_INFO("Length of r: %f mm", r_prev.mod());
    ROS_INFO("r expected: %f ", r_expected.mod());
*/

    return &myDesiredAction;
}

void ArActionMatlab::computeControl()
{
   // if (first_fire) {
        myDesiredAction.setVel(myInfo.velocity);
     //   first_fire = false;
//}

    if ((double)* u_control == -1000)
        myDesiredAction.setVel(0.0);
    else {
        ROS_INFO("Rotation-: %f",(double)* u_control );

        myDesiredAction.setRotVel( ((double) *u_control )* myInfo.velocity/1000.00);
}


    //myDesiredAction.setRotVel(*beamAngle);
}

#include "ArActionCBB.h"

//=====================================================
//
// Constructors and Destructors
//
//=====================================================
AREXPORT ArActionCBB::ArActionCBB(const char* name, CRobotInfo &prevAgent, CRobotInfo &curAgent, CRobotInfo &nextAgent,
                                  CRobotInfo &beacon, float _alpha, float _alpha_0, float _lambda, float _mu, float cycFreq): ArAction(name, "CB-Beacon Pursuit law")
{
    // store the pointers to the updating robots
    p_paInfo        = &prevAgent;
    p_naInfo        = &nextAgent;
    p_myInfo        = &curAgent;
    p_beaconInfo    = &beacon;

    // copy the actual data to local arrays
    paInfo      = *p_paInfo;
    naInfo      = *p_naInfo;
    myInfo      = *p_myInfo;
    beaconInfo  = *p_beaconInfo;

    // add parameter values to the system
    alpha = _alpha;
    alpha_0 = _alpha_0;
    lambdaWeight = _lambda;
    cycleFreq = cycFreq;
    velocity = myInfo.velocity;
    mu = _mu;

    //
    // Set updated for our first go around the fire()
    isUpdated = false;
    isFirstFire = true;
    isFirstErrorCalc = true;


}

AREXPORT ArActionCBB::~ArActionCBB()
{
    //dtor
}


//=====================================================
//
// Member functions
//
//=====================================================
AREXPORT ArActionDesired *ArActionCBB::fire(ArActionDesired currentDesired)
{

    myDesiredAction.reset();

    //
    // Update Aria latency
    //
    updateAriaLatency();


    //
    // Update state
    //
    updateState();

    //
    // Compute controls and set the desired action
    //
    computeControl();


    //
    // Output for debugging
    //

    ROS_INFO("Speed-: %f", velocity);
    ROS_INFO("Turning Rate-: %f", rVel);
    ROS_INFO("Curvature-: %f", curvature);
    ROS_INFO("kappa-: %f", kappa);
    ROS_INFO("kappa w.r.t beacon-: %f", kappa_beacon);
    ROS_INFO("Length of r: %f mm", r_prev.mod());
    ROS_INFO("r expected: %f ", r_expected.mod());


    return &myDesiredAction;
}

void ArActionCBB::updateAriaLatency()
{
    // get the robot we're attached to, and update the local latency
    latency_aria = getRobot()->getLatency();
}

void ArActionCBB::updateState()
{
    // Dereference the pointer to the external data structure, then copy that object to
    // the local object
    paInfo      = *p_paInfo;
    naInfo      = *p_naInfo;
    myInfo      = *p_myInfo;
    beaconInfo  = *p_beaconInfo;

    x = myInfo.x;
    y = myInfo.y;
    r = myInfo.r;
    velocity = myInfo.velocity;
    heading = myInfo.heading;


    // Now translate the raw data to data we'll need for the controls, specifically...
    r_to_neighbor = (myInfo.r - naInfo.r) * (0.001);
    rho_ij        = r_to_neighbor.mod();
    r_bi          = (beaconInfo.r - myInfo.r) * (0.001);
    pa_r_to_neighbor = (paInfo.r - myInfo.r)* (0.001);

}

void ArActionCBB::computeControl()
{
    // if it's our first time firing...
    if( isFirstFire == true )
    {
        r_expected = r;
        r_prev = r;
        y_prev = y;
        heading_prev = heading;
    }

    // Computes the relevant angles: Kappa, Theta and Kappa_ib
    kappa = std::atan2(y * (r_to_neighbor*(-1)), x * (r_to_neighbor*(-1)));

    theta = std::atan2(y * (pa_r_to_neighbor), x * pa_r_to_neighbor);

    na_theta = std::atan2( naInfo.y * r_to_neighbor, naInfo.x * r_to_neighbor );

    kappa_beacon = std::atan2(y * r_bi, x*r_bi);

    // Computes our actual feedback control
    curvature = (1 - lambdaWeight) * (mu * std::sin(kappa - alpha) + (1/rho_ij)*
                (std::sin(kappa) + std::sin(na_theta))) + mu*lambdaWeight*
                std::sin(kappa_beacon - alpha_0);

    // the original writes to the sys log here

    // now set the desired action
    rVel = velocity*(0.001)*R2D*curvature;
    myDesiredAction.setRotVel(rVel);
    myDesiredAction.setVel(velocity);

    // calculate error
    r_error = r_expected - r;

    tstep = 1/cycleFreq;

    if(isFirstFire == false )
    {
        r_prev = r_expected;

        y_prev = y_new;

        heading_prev = heading_new;

        heading_new = heading_prev + rVel*tstep;

        radii = 1/curvature;

        x_new = CVector(std::cos(D2R*heading_new), std::sin(D2R*heading_new));

        y_new = CVector(-1*std::sin(D2R*heading_new), std::cos(D2R*heading_new));

        r_expected =  r_prev + (y_prev - y_new) * radii;

    }

    // we're done.
    isFirstFire = false;
    isUpdated = false;

}

#include "ArActionBody.h"

//=====================================================
//
// Constructors and Destructors
//
//=====================================================
AREXPORT ArActionBody::ArActionBody(const char * name, float *posx, float *posy, float *posz): ArAction(name, "Kinect Body Control")
{

       pos_x = posx;
       pos_y = posy;
       pos_z = posz;

}

AREXPORT ArActionBody::~ArActionBody()
{
    //dtor
}


//=====================================================
//
// Member functions
//
//=====================================================
AREXPORT ArActionDesired *ArActionBody::fire(ArActionDesired currentDesired)
{

    myDesiredAction.reset();
    computeBodyControl();
    return &myDesiredAction;
}

void ArActionBody::computeBodyControl()
{
    if (*pos_x != 0 || *pos_z != 0){
        myDesiredAction.setVel(100.0 * (*pos_z));
        myDesiredAction.setRotVel((*pos_x)*100);
    } else {
        myDesiredAction.setRotVel(0);
        myDesiredAction.setVel(100.0);
    }

}

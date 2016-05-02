#ifndef CRUNTIMESETTINGS_H
#define CRUNTIMESETTINGS_H

#include <string.h>
#include <sstream>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <fstream> //file io - of course

#define PI 3.14159265
#define D2R ((double)PI/180)
#define R2D ((double)180/PI)


//=====================================================
// General Description:
//
// This is a class (more of a structure) for importing
// our runtime settings.  Using the constructor will import
// the "settings.txt" file in the local directory
//=====================================================

class CRuntimeSettings
{
public:

// Constructor/Destructors
//=====================================================
// CRuntimeSettings(bool verbose):
    /*
    This function will serve to import our settings from a text file and spit back a structure of useful information
    if verbose activated it will print the inported settings.
    File Format:  Setting = value
    There MUST be a space between the setting, equals and the value
    The name of the setting is NOT case sensitive
    After NUMROBOTS = x, then IP1 = x1 \n IP2 = x2 ... then Port1 = y.... then Name1 = x1 \n Name2 = x2...
    Each setting needs to be its own line!

    For ease of parsing, alpha neighbor and alpha beacon are the denominator of the fraction
    (pi/alphaneighbor) and (pi/alphabeacon)
    */
//=====================================================
    CRuntimeSettings(bool verbose);
    virtual ~CRuntimeSettings();

    //
    // Fields
    //
    int numRobots;
    int port[40];
    std::string host[40]; //arbitrarily set for 40 robots
    std::string name[40]; //arbitrarily set for 40 robots
    bool logging;
    float cycleFreq;
    std::string logpath;
    std::string logname;
    std::string beacon_name;
    float velocity;
    float mu;
    float leaderLambda;
    float alphaBeacon;
    float alphaNeighbor;
    int withSonar;


protected:
private:
};

#endif // CRUNTIMESETTINGS_H

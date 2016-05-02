#include "CRuntimeSettings.h"

CRuntimeSettings::CRuntimeSettings(bool verbose)
{
    char settingType[40]; //lines from the imported settings will be stored here
    char settingValue[40]; //the value for the setting will be here
    char dummyVar[5]; // for the dummy equals
    std::ifstream settingsFile("settings.txt"); //this will open our settings file!
    char* ptr;

    std::istringstream mystream;
    std::string str_num; //these large string arrays were put here to deal with some interesting
    std::string str_cf; //cycle frequency container
    std::string str_port[40];
    std::string str_vel;
    std::string str_mu;
    std::string str_leaderLambda;
    std::string str_alphaBeacon;
    std::string str_alphaNeighbor;
    std::string str_withSonar;

     logging = false; //default no logging

    int i=0;//our counter

    //
    //initialize defaults
    //
     cycleFreq = 25.0;
     logpath = "/home/kate/Desktop/kp_log.txt";

    if(!settingsFile)
    {
        std::cout<<"Could not open settings.txt - continuing with default parameters!"<<std::endl;
        sleep(1);
    }
    else
    {
        std::cout<<"Successfully opened settings file, importing settings..."<<std::endl;

        while(!settingsFile.eof())
        {
            //read line by line
            settingsFile>> settingType >> dummyVar >> settingValue;

            //convert the "settingType" to upper case for usability
            ptr=settingType;
            while (*ptr)
            {
                if(*ptr!=' ')
                {
                    *ptr = toupper(*ptr);
                }
                ptr++;
            }

            //number of robots, default to 1 if invalid, must be followed by ip's and ports
            if(strcmp(settingType,"NUMROBOTS") == 0)
            {
                str_num = settingValue;
                mystream.str(str_num);
                if ( ! (mystream >>  numRobots) )  numRobots = 1;
                if(verbose)
                {
                    std::cout<<"NumRobots:"<< numRobots<<std::endl;
                }
                //now it will grab the IP's that MUST be listed, one for each robot
                i=0;
                while(i!= numRobots)
                {
                    settingsFile>> settingType >> dummyVar >> settingValue;
                     host[i]=settingValue;

                    if(verbose)
                    {
                        std::cout<<"Robot "<<i<<" IP is: "<< host[i]<<std::endl;
                    }
                    i++;
                }
                //now it will grab the ports in Order listed, corresponding to each robot
                i=0;
                while(i!= numRobots)
                {
                    mystream.clear();
                    settingsFile>> settingType >> dummyVar >> settingValue;
                    str_port[i] = settingValue;
                    mystream.str(str_port[i]);
                    if ( ! (mystream >>  port[i]) )  numRobots = 8101;
                    if(verbose)
                    {
                        std::cout<<"Robot "<<i<<" port is: "<< port[i]<<std::endl;
                    }
                    i++;
                }
                //now it will grab the names that MUST be listed, one for each robot
                i=0;
                while(i!= numRobots)
                {
                    settingsFile>> settingType >> dummyVar >> settingValue;
                     name[i]=settingValue;
                    if(verbose)
                    {
                        std::cout<<"Robot "<<i<<" name is: "<< name[i]<<std::endl;
                    }
                    i++;
                }
            }
            else if (strcmp(settingType,"LOGGING") == 0)
            {
                if(strcmp(settingValue,"yes") == 0)
                {
                     logging = true;
                    if(verbose)
                    {
                        std::cout<<"Logging enabled."<<std::endl;
                    }
                }
            }
            else if((strcmp(settingType,"CYCLEFREQ")==0) || (strcmp(settingType,"CYCLEFREQUENCY")==0))
            {
                mystream.clear();
                str_cf = settingValue;
                mystream.str(str_cf);
                if ( ! (mystream >>  cycleFreq) )  cycleFreq = 25;
                if(verbose)
                {
                    std::cout<<"Cycle Fequency is "<< cycleFreq<<std::endl;
                }
            }
            else if(strcmp(settingType,"LOGPATH")==0)
            {
                 logpath = settingValue;
                if(verbose)
                {
                    std::cout<<"Logging Path is: "<< logpath<<std::endl;
                }
            }
            else if(strcmp(settingType,"BEACONNAME")==0)
            {
                 beacon_name = settingValue;
                if(verbose)
                {
                    std::cout<<"Beacon name is: "<< beacon_name<<std::endl;
                }
            }
            else if((strcmp(settingType,"VEL")==0)|| (strcmp(settingType,"VELOCITY")==0))
            {
                mystream.clear();
                str_vel = settingValue;
                mystream.str(str_vel);
                if ( ! (mystream >>  velocity) )  velocity = 100;
                if(verbose)
                {
                    std::cout<<"Velocity is: "<< velocity<<std::endl;
                }
            }
            else if(strcmp(settingType,"MU")==0)
            {
                mystream.clear();
                str_mu = settingValue;
                mystream.str(str_mu);
                if ( ! (mystream >>  mu) )  mu = 1.00;
                if(verbose)
                {
                    std::cout<<"Mu is: "<< mu<<std::endl;
                }
            }
            else if(strcmp(settingType,"LOGNAME") == 0)
            {
                 logname = settingValue;
                if(verbose)
                {
                    std::cout<<"LOGFILE name is: "<< logname<<std::endl;
                }
            }
            else if(strcmp(settingType,"LEADERLAMBDA") == 0)
            {
                mystream.clear();
                str_leaderLambda = settingValue;
                mystream.str(str_leaderLambda);

                //configure default setting if it fails
                if ( ! (mystream >>  leaderLambda) )  leaderLambda = .5;

                if(verbose)
                {
                    std::cout<<"leaderLambda is: "<< leaderLambda<<std::endl;
                }
            }
            else if(strcmp(settingType,"ALPHANEIGHBOR") == 0)
            {
                mystream.clear();
                str_alphaNeighbor = settingValue;
                mystream.str(str_alphaNeighbor);

                //configure default setting if it fails
                if ( ! (mystream >>  alphaNeighbor) )  alphaNeighbor = PI/3;
                else
                {
                     alphaNeighbor = ((double) (1.00*PI)/ alphaNeighbor);
                }
                if(verbose)
                {
                    std::cout<<"AlphaNeighbor is: "<< alphaNeighbor<<std::endl;
                }
            }
            else if(strcmp(settingType,"ALPHABEACON") == 0)
            {
                mystream.clear();
                str_alphaBeacon = settingValue;
                mystream.str(str_alphaBeacon);

                //configure default setting if it fails
                if ( ! (mystream >>  alphaBeacon) )  alphaBeacon = .5;
                else
                {
                     alphaBeacon =((double) (1.00*PI)/ alphaBeacon);
                }

                if(verbose)
                {
                    std::cout<<"AlphaBeacon is: "<< alphaBeacon<<std::endl;
                }
            }
            else if(strcmp(settingType,"WITHSONAR") == 0)
            {
                mystream.clear();
                str_withSonar = settingValue;
                mystream.str(str_withSonar);

                //configure default setting if it fails
                if ( ! (mystream >>  withSonar) )  withSonar = 1;

                if(verbose)
                {
                    std::cout<<"WithSonar: "<< withSonar<<std::endl;
                }
            }
        }
    }
    settingsFile.close();
    if(verbose)
    {
        std::cout<<"Done importing setings!"<<std::endl<<"==============================================="<<std::endl;
    }

}

CRuntimeSettings::~CRuntimeSettings()
{
    //dtor
}

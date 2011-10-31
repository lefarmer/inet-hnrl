/* -*- mode:c++ -*- ********************************************************
 * file:        WifiRobotMobility.h
 *
 * authors:     Sion O'Boyle
 * 				Ramya Sasidharan
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *              (C) 2010 School of Engineering at Swansea University,
 *              Wales, UK.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 **************************************************************************/


#ifndef WIFI_ROBOT_MOBILITY_H
#define WIFI_ROBOT_MOBILITY_H

#include <omnetpp.h>

#include "BasicMobility.h"
#include "AbstractRadioExtended.h"
#include "Ieee80211Frame_m.h"
#include "Ieee80211MgmtFrames_m.h"
//#include "Radio80211aControlInfo_m.h"
#include "Coord.h"
#include "ChannelControl.h"

/**
 * @brief Controls all movement related things of a host
 *
 * Parameters to be specified in omnetpp.ini
 *  - vHost : Speed of a host [m/s]
 *  - updateInterval : Time interval to update the hosts position
 *  - x, y : Starting position of the host, -1 = random
 *
 * @ingroup mobility
 * @author Steffen Sroka, Marc Loebbers, Daniel Willkomm
 * @sa ChannelControl
 */
class INET_API WifiRobotMobility : public BasicMobility
{

  protected:
    /** @brief Velocity of the host*/
    double vHost;
//
//    /** @brief Pointer to ChannelControl -- these two must know each other */
//    ChannelControl *cc;
//
      /** @brief Time interval to update the hosts position*/
    double updateInterval;

    /** @brief If true, the host doesn't move*/
    bool stationary;

    /** @brief parameters to handle the movement of the host*/
    /*@{*/
    Coord targetPos;
    Coord newPos;
    Coord stepSize;
    int numSteps;
    int step;
    /*@}*/

    // TODO: Declare variables here
    int count[3];
    double PR[3];
    float xp, yp, dist, dist2, P1, P2, a, b, xj, yj;
    double distance;
    float distanceCompare;
    float distanceCompare2;
    int count1, count2;
    double minx, miny, RN, Sumtot, SumRD, randNoise;
     double PR1,PR2,PR3;
    float xtar, ytar, xs, ys;
    float xnew, ynew;
    int xtemp, ytemp;
    double pathLossAlpha;
    std::string APOne, APTwo;



    // For Kalman filter

     double temp;
    double x[];
     double pl[];
     //the noise in the system
     double PN ;//process noise covariance
   	 double MN ;//measurement noise covariance
     double Pred[];
     double xl[];
     double xt[];
     double pt[];
     double kg[];
   	 double x_est[];

   	//FOR TRUE DISTNCE
   	double td[];
   	double td1,td2,td3;

  protected:
    /** @brief Initializes mobility model parameters.*/
    virtual void initialize(int);

    /** @brief Handle messages from PHY layer as well as self-messages*/
    virtual void handleMessage(cMessage *msg);

    /** @brief Called upon arrival of a self messages*/
    virtual void handleSelfMsg(cMessage *msg);

    /** @brief Called upon arrival of an AirFrameExtended from PHY layer */
    virtual void handleLowerMsg(AirFrameExtended *airFrame);

    /** @brief Calculate the target position to move to*/
    virtual void setTargetPosition();

    /** @brief Move the host*/
    virtual void move();

    /** @brief Get target position for the host*/
    virtual Coord getTargetPosition();

    virtual Coord getnewTargetPosition();

    virtual Coord getNewPos();
    // ** @brief Get Random noise to simulate ideal world*/


};

#endif

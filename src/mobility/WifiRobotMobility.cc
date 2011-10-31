/***************************************************************************
 * file:        WifiRobotMobility.cc
 *
 * authors:     Sion O'Boyle
 *              Ramya Sasidharan
 *              Kyeong Soo (Joseph) Kim
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *              (C) 2010-2011 College of Engineering, Swansea University,
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

#include "WifiRobotMobility.h"
#include "FWMath.h"
//#include "string.h"
//#include <math.h>
#include "ChannelControl.h"

using namespace std;

Define_Module(WifiRobotMobility)
;

/**
 * Reads the updateInterval and the velocity
 *
 * If the host is not stationary it calculates a random position and
 * schedules a timer to trigger the first movement
 */

void WifiRobotMobility::initialize(int stage) {
	BasicMobility::initialize(stage);

	distanceCompare = 1e8;
	distanceCompare2 = 1e8;
	count1 = 0;
	count2 = 0;
	PR1 = 0;
	PR2 = 0;
	PR3 = 0;
	pl[0] = 0;
	xtar = 579;
	ytar = 380;
	Sumtot = 0;
	PN = 0.0234;
	MN = 0.667;
	xp = 0;
	yp = 0;
	xs = 50;
	ys = 130;
	td1 = td[0];
	td2 = td[1];
	td3 = td[2];
	temp = normal(0, 1);
	EV<< "initializing WifiRobotMobility stage " << stage << endl;

	//	double pathLossAlpha;
	if (stage == 0)
	{
		updateInterval = par("updateInterval");
		vHost = par("vHost");

		// if the initial speed is lower than 0, the node is stationary
		stationary = (vHost <= 0);

		//calculate the target position of the host if the host moves
		if (!stationary)
		{
			setTargetPosition();
			//host moves the first time after some random delay to avoid synchronized movements
			scheduleAt(simTime() + uniform(0, updateInterval), new cMessage("move"));
		}
	}
}

void WifiRobotMobility::handleMessage(cMessage * msg) {
	if (msg->isSelfMessage()) {
		handleSelfMsg(msg);
	} else {
		if (dynamic_cast<AirFrame *> (msg) != NULL) {
			handleLowerMsg(check_and_cast<AirFrame *> (msg));
		} else {
			error(
					"WifiRobotMobility module can receive only AirFrame from the PHY layer");
		}
	}
}

/**
 * The only self message possible is to indicate a new movement. If
 * host is stationary this function is never called.
 */
void WifiRobotMobility::handleSelfMsg(cMessage* msg) {
	//	move();
	updatePosition();
	scheduleAt(simTime() + updateInterval, msg);
}

/**
 * Handle an Ieee80211BeaconFrame from the PHY layer.
 */
void WifiRobotMobility::handleLowerMsg(AirFrame *airFrame) {

	cMessage *msg = airFrame->decapsulate();

	if (dynamic_cast<Ieee80211BeaconFrame *> (msg) != NULL) {
		// handles only beacon frames now
		Ieee80211BeaconFrame *beacon = check_and_cast<Ieee80211BeaconFrame *> (
				msg);
		std::string ssid = (beacon->getBody()).getSSID();

		double carrierFrequency = airFrame->getCarrierFrequency();
		double powerReceived = airFrame->getPowRec();
		double powerSent = airFrame->getPSend();
		double snr = airFrame->getSnr();
		EV<< "Frame received from " << ssid.c_str() << ":" << endl;
		// Ideal power received calculated from the robot
		EV << "- Power received = " << powerReceived << endl;
		EV << "- Power sent = " << powerSent << endl;
		EV << "- SNR = " << snr << endl;

		const double speedOfLight = 300000000.0;
		double waveLength = speedOfLight / carrierFrequency;

		// Calculated distance in the basis of gathering the power received and calculating
		// using the parameters of power sent and the wavelength.
		double distance =sqrt((powerSent * waveLength * waveLength / (16 * M_PI * M_PI * powerReceived)));
		EV << "distance = " << distance << endl;
		EV << " ssid is from " << ssid.c_str() << endl;
		// Storing power received with regards to the access point
		if (ssid.compare(0, 3, "AP1", 0, 3) == 0)
		{
			PR[0] = powerReceived;
			count[0]++;
		}
		else if (ssid.compare(0, 3, "AP2", 0, 3) == 0)
		{
			PR[1] = powerReceived;
			count[1]++;
		}
		else if (ssid.compare(0, 3, "AP3", 0, 3) == 0)
		{
			PR[2] = powerReceived;
			count[2]++;
		}
		else
		{
			EV << "Error in SSID comparison: no matching SSID found" << endl;
			opp_error("Error in SSID comparison: no matching SSID found");
		}

		// Outputting the power results
		EV << " PR1 is " << PR[0] << " PR2 is " << PR[1] << "PR3 is "<< PR[2] << endl;

		//adding noise
		for(int i=0;i<3;i++) {
			if ( count[i] >= 1 && count[i] <= 3) {
				PR[i]+= temp>=0 ? temp: 0;
				EV << " PR1 is " << PR[0] << " PR2 is " << PR[1] << "PR3 is "<< PR[2] << endl;
				//using kalman filter to filter the noise
				//initialize with a measurement
				x[i]=PR[i];
				//do a prediction
				xt[i] = x[i];
				pt[i] = pl[i] + PN;
				//calculate the Kalman gain
				kg[i] = pt[i] * (1.0/(pt[i] + MN));
				//correct
				x_est[i] = xt[i] + kg[i] * (PR[i]- xt[i]);
				Pred[i] = (1- kg[i]) * pt[i];
				//outputing true received power
				EV << "kalman filter " << x_est[i] << endl;
				//update the last
				pt[i] = Pred[i];
				xt[i] = x_est[i];
			}
			if (count[0]==3&& count[1]==3 && count[2]==3) {
				td[i] =sqrt((powerSent * waveLength * waveLength / (16 * M_PI * M_PI *x_est[i])));
				EV << "truedistance = " << td[i] << endl;
				td1=td[0];
				td2=td[1];
				td3=td[2];
				//using trilateration technique
				P1 = (pow(td1,2) - pow(td3,2)) - (pow(579,2) - pow(24,2)) - (pow(380,2) - pow(380,2));
				EV << "P1 is " << P1 << endl;

				b = P1 / (2 * -555);
				xj = b;
				EV << " xj is " << xj << endl;

				P2 = (pow(td2,2) - pow(td3,2)) - (pow(579,2) - pow(24,2)) - (pow(19,2) - pow(380,2));
				EV << "P2 is " << P2 << endl;

				a = 2* xj * -555;
				yj = P2 - a;
				yj = yj / (2 * 361);

				EV << " yj is " << yj << endl;
				// finding the best representation of coordinates
				dist = xs - xj;
				dist2 = ys - yj;
				if(dist < distanceCompare && dist2 < distanceCompare2) {
					minx = xj;
					miny = yj;
					distanceCompare = dist;
					distanceCompare2 = dist2;
				}
				// outputting the minimum error of estimated coordinates
				EV << " minx is"<< minx << endl;
				EV << " miny is " << miny << endl;
				EV << " dist is "<< dist << " and dist2 is " << dist2 << endl;

				move();
				//intialize count
				count[0]=0;
				count[1]=0;
				count[2]=0;
				distanceCompare = 1e8;
				distanceCompare2 = 1e8;
			}
		}
	}

	delete msg;
	delete airFrame;
}
		/**
		 * Calculate a new random position and the number of steps the host
		 * needs to reach this position
		 */
void WifiRobotMobility::setTargetPosition() {

	targetPos = getTargetPosition();
	double distance = pos.distance(targetPos);
	double totalTime = distance / vHost;
	numSteps = FWMath::round(totalTime / updateInterval);
	stepSize = (targetPos - pos) / numSteps;

	distance = vHost * updateInterval;
	EV<< " distance is equal to " << distance << endl;
	step = 0;

	EV<< "distance=" << distance << " xpos= " << targetPos.x << " ypos=" <<targetPos.y
        << "totalTime=" << totalTime << " numSteps=" << numSteps << " vHost=" << vHost << endl;
}

Coord WifiRobotMobility::getTargetPosition() {
	Coord targetPos;
	targetPos.x = 579;
	targetPos.y = 380;
	return targetPos;
}

Coord WifiRobotMobility::getnewTargetPosition() {
	Coord targetPos;
	targetPos.x = 140;
	targetPos.y = 200;
	return targetPos;
}

Coord WifiRobotMobility::getNewPos() {
	Coord newPos;
	newPos.x = 579;
	newPos.y = 380;
	return newPos;
}

/**
 * Move the host if the destination is not reached yet. Otherwise
 * calculate a new random position
 */
void WifiRobotMobility::move() {
	step++;

	if (step <= numSteps) {
		EV<< " before pos.x was " << pos.x << endl;
		EV << " before pos.y was " << pos.y << endl;

		xnew = pos.x + ((xtar - minx)/numSteps);
		ynew = pos.y + ((ytar - miny)/numSteps);

		newPos.x = xnew;
		newPos.y = ynew;

		pos.x = newPos.x;
		xs = pos.x;
		EV << "xs is =  " << xs << endl;
		EV << " pos.x is " << pos.x << endl;

		pos.y = newPos.y;
		ys = pos.y;
		EV << "pos.y is " << pos.y << endl;
		EV << "pos = " << pos << endl;
		// makes sure the robot stays within the playground region
		getTargetPosition();
		if (pos.x >= getPlaygroundSizeX()) {
			pos.x = 2*getPlaygroundSizeX() - pos.x;
			targetPos.x = 2*getPlaygroundSizeY() - targetPos.x;
		}
		if (pos.y >= getPlaygroundSizeY())
		{
			pos.y = 2*getPlaygroundSizeY() - pos.y;
			targetPos.y = 2*getPlaygroundSizeY() - targetPos.y;
		}
		getTargetPosition();
		if(pos.x >= targetPos.x && pos.y >= targetPos.y) {
			EV << endl << "HOST HAS REACHED TARGET POSITION" << endl << endl;
			EV << "Host is charging" << endl << endl << "Host has Recharged" << endl;
			endSimulation();
		}
		updatePosition();

		EV<< "stepping forward. step #=" << step << " xpos= " << pos.x << " ypos= "<< pos.y << endl;

	}
	else
	{
		EV << "destination reached.\n xpos= " << pos.x << " ypos= " << pos.y << endl;

		setTargetPosition();
	}
}

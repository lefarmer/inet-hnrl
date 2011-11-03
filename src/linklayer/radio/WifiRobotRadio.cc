//
// Copyright (C) 2011 Kyeong Soo (Joseph) Kim
// Copyright (C) 2006 Andras Varga, Levente Meszaros
// Based on the Mobility Framework's SnrEval by Marc Loebbers
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//


#include "WifiRobotRadio.h"
#include "FWMath.h"
#include "PhyControlInfo_m.h"
#include "Ieee80211Consts.h"  //XXX for the COLLISION and BITERROR msg kind constants
//#include "Radio80211aControlInfo_m.h"
//#include "BasicBattery.h"

Define_Module(WifiRobotRadio);

void WifiRobotRadio::initialize(int stage)
{
	Ieee80211Radio::initialize(stage);

	EV<< "Initializing WifiRadio, stage=" << stage << endl;

	if (stage == 0)
	{
		mobilityOut = findGate("mobilityOut");
	}
}

void WifiRobotRadio::sendUp(AirFrame *airframe)
{
	// send a copy of the air frame (including IEEE 802.11 frame)
	// to the mobility module of Wifi Robot for localization
	// (added by Kyeong Soo (Joseph) Kim)
	AirFrame *copy = airframe->dup();
	send(copy, mobilityOut);

    Ieee80211Radio::sendUp(airframe);
    // cPacket *frame = airframe->decapsulate();
    // Radio80211aControlInfo * cinfo = new Radio80211aControlInfo;
    // cinfo->setSnr(airframe->getSnr());
    // cinfo->setLossRate(airframe->getLossRate());
    // cinfo->setRecPow(airframe->getPowRec());
    // frame->setControlInfo(cinfo);

    // delete airframe;
    // EV << "sending up frame " << frame->getName() << endl;
    // send(frame, uppergateOut);
}


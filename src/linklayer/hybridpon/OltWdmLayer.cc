///
/// @file   OltWdmLayer.cc
/// @author Kyeong Soo (Joseph) Kim <kyeongsoo.kim@gmail.com>
/// @date   Jan/21/2010
///
/// @brief  Declares 'OltWdmLayer' class for a hybrid TDM/WDM-PON OLT.
///
/// @remarks Copyright (C) 2009-2010 Kyeong Soo (Joseph) Kim. All rights reserved.
///
/// @remarks This software is written and distributed under the GNU General
///          Public License Version 2 (http://www.gnu.org/licenses/gpl-2.0.html).
///          You must not remove this notice, or any other, from this software.
///


// #define DEBUG_WDM_LAYER


#include <stdlib.h>
#include "OltWdmLayer.h"

// Register modules.
Define_Module(OltWdmLayer)

void OltWdmLayer::initialize()
{
}

void OltWdmLayer::handleMessage(cMessage *msg)
{
#ifdef DEBUG_WDM_LAYER
	ev << getFullPath() << ": handleMessage called" << endl;
#endif

	// get the full name of arrival gate
	std::string inGate = msg->getArrivalGate()->getFullName();

	if (inGate.compare(0, 6, "muxg$i") == 0)
	{
		// optical frame from the MUX gate (i.e., the optical fiber).

		OpticalFrame *opticalFrame = check_and_cast<OpticalFrame *> (msg);
		int ch = opticalFrame->getLambda();

#ifdef DEBUG_WDM_LAYER
		ev << getFullPath() << ": optical frame with a WDM channel = " << ch << endl;
#endif

		// decapsulate a PON frame and send it to the upper layer
		HybridPonUsFrame *frame = check_and_cast<HybridPonUsFrame *> (
				opticalFrame->decapsulate());
		frame->setChannel(ch);
		send(frame, "demuxg$o", ch); //ownership problem here or up there?
		delete opticalFrame;
	}
	else
	{
		// PON frame from the DEMUX gate (i.e., the upper layer)

		HybridPonDsFrame *frame = check_and_cast<HybridPonDsFrame *> (msg);
		int ch = frame->getChannel();

		// encapsulate a PON frame into an optical frame and send it to the PON I/F
		OpticalFrame *opticalFrame = new OpticalFrame();
		opticalFrame->setLambda(ch);
		opticalFrame->encapsulate(frame);
		send(opticalFrame, "muxg$o", ch);
	}
}

void OltWdmLayer::finish()
{
}

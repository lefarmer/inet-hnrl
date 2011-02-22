//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "ONUMacCtl.h"

Define_Module(ONUMacCtl);

ONUMacCtl::ONUMacCtl(){
	startTxMsg=0;
	stopTxMsg=0;
	syncMsg=0;
}

ONUMacCtl::~ONUMacCtl(){

	cancelAndDelete(startTxMsg);
	cancelAndDelete(stopTxMsg);
	cancelAndDelete(syncMsg);

}

void ONUMacCtl::initialize()
{
	clock_reg=0;
	start_reg=0;
	len_reg=0;
	numGates = 0;



	startTxMsg = new cMessage("startTxMsg", STARTTXMSG);
	stopTxMsg = new cMessage("stopTxMsg", STOPTXMSG);
	transmitState = TX_OFF;
	txrate=GIGABIT_ETHERNET_TXRATE;


	// Link to MAC layer
	emac = dynamic_cast<EPON_mac *>(getNeighbourOnGate("lowerLayerOut"));
	if (!emac)
		error("No MAC layer found connected under ONU MAC Control Layer");


	// Init Stats
	numFramesFromHL=0;
	numFramesFromLL=0;
	numFramesFromHLDropped=0;
	fragmentedTime=0;

    WATCH(numFramesFromHL);
    WATCH(numFramesFromLL);
    WATCH(numFramesFromHLDropped);
    WATCH(fragmentedTime);

    WATCH(clock_reg);
    WATCH(start_reg);
    WATCH(len_reg);
    WATCH(transmitState);


	// MPCP
    transmitState = TX_INIT;

    // Initialize the Q mgmt module
    queue_mod = dynamic_cast<IEponQueueMgmt *>(getNeighbourOnGate("upperLayerOut"));
    if (!queue_mod)
    	error("ONUMacCtl: An IEponQueueMgmt is needed above mac control");


}

void ONUMacCtl::handleMessage(cMessage *msg)
{

	// Update clock from simTime
	// The clock MUST be in sync
	// when stop is scheduled. Stop
	// is called only when entering IDLE
	clockSync();

	// Self Message
	if (msg->isSelfMessage())
	{
		EV << "Self-message " << msg << " received\n";

		if (msg == startTxMsg){
			if (transmitState == TX_IDLE) {std::cout<<*msg<<endl; delete msg; error("?"); }
			startTxOnPON();
		}
		else if (msg == stopTxMsg)
			handleStopTxPeriod();
		else
			error("Unknown self message received!");

		return;
	}

	if (msg->getKind() == WAKEUPMSG){
		EV << "Wake UP message received..." << endl;
		// Re-schedule tx period
		// Check if we are idle, if yes start transmission
		if (transmitState == TX_IDLE){
			EV << "ONUMacCtl: CHANGING STATE FROM _IDLE_ TO _ON_\n";
			// Cancel Previous DeadLine
			cancelEvent(stopTxMsg);
			startTxOnPON();
		}
		// Check if we are asleep
		else if (transmitState == TX_SLEEP){
			EV << "ONUMacCtl: CHANGING STATE FROM _TX_SLEEP_ TO _??_\n";
			// IF start_reg is in the future just reschedule startTX
			// ELSE shift the clock the lost slots...
			if (clock_reg < start_reg){
				cancelEvent(startTxMsg);
				scheduleStartTxPeriod();
			}else if (clock_reg == start_reg){
				cancelEvent(stopTxMsg);
				startTxOnPON();
			}else {
				// Super Slot ...
				uint64_t slotTime16ns = (double)slotLength/16*slotNumber;
				uint32_t lostSlots = ceil((double)(clock_reg - start_reg)/slotTime16ns);
				shiftStartXSlots(lostSlots);
			}
		}
		delete msg;
		return;
	}


	// Network Message
	cGate *ingate = msg->getArrivalGate();
	EV << "Frame " << msg << " arrived on port " << ingate->getName() << "...\n";

	if (ingate->getId() ==  gate( "lowerLayerIn")->getId()){
		processFrameFromMAC(msg);
	}
	else if (ingate->getId() ==  gate( "upperLayerIn")->getId()){
		processFrameFromHigherLayer(msg);
	}else{
		EV << "ONUMacCtl: Message came FROM THE WRONG DIRRECTION???? Dropping\n";
		delete msg;
	}

}

void ONUMacCtl::processFrameFromHigherLayer(cMessage *msg){
	EV << "ONUMacCtl: Outgoing message, forwarding...\n";
	numFramesFromHL++;

	// Enqueue frame ...
	EV << "Packet " << msg << " arrived from higher layers, sending\n";
	if (start_reg == 0 ){
		EV<<"Frame arrived in pre-conf stage "<<endl;
		send(msg, "lowerLayerOut");
	}



}

void ONUMacCtl::processFrameFromMAC(cMessage *msg){


	EV << "ONUMacCtl: Incoming message from PON\n";
	EthernetIIFrame * frame = dynamic_cast<EthernetIIFrame *>(msg);


	if (frame && frame->getEtherType() == MPCP_TYPE){

		// Check that the frame is for us...
		if (frame->getDest() != emac->getMACAddress() && !frame->getDest().isBroadcast()){
			EV << "MPCP not for us... dropping\n";
			delete frame;
			return;
		}

		processMPCP(frame );
	}


	send(msg,"upperLayerOut");
	numFramesFromLL++;
}


void ONUMacCtl::processMPCP(EthernetIIFrame *frame ){
	EV << "ONUMacCtl: MPCP Frame processing\n";
	MPCP * mpcp = check_and_cast<MPCP *>(frame);
	//EPON_LLidCtrlInfo * nfo = dynamic_cast<EPON_LLidCtrlInfo *>(frame->getControlInfo());


	// DONT... DONT EVEN THINK OF IT
//	EV << "ONUMacCtl: Updating our clock from: "<<clock_reg<<" to ";
//	clock_reg=mpcp->getTs();
//	EV << clock_reg << endl;

	switch (mpcp->getOpcode())
	{

		case MPCP_GATE:
		{
			MPCPGate * gate = check_and_cast<MPCPGate *>(frame);
			EV << "ONUMacCtl: Type is MPCP_GATE\n";

			if (gate->getListLen() == 0) {
				EV << "ONUMacCtl: !!! NO ALLOCATION FOR US :-( (bitches...)";
				break;
			}


			// Update with 1st alloc
			start_reg = gate->getStartTime(0);
			len_reg = gate->getDuration(0);
			slotLength = gate->getSlotTime();
			slotNumber = gate->getSlotsNum();


			/**
			 *  NOTE: Announced time IS NOT ALWAYS ON THE FUTURE
			 *  IF we want the announced times to be on the future
			 *  then the DBA algorithms MUST CONSIDER RTT. For simplicity
			 *  we do accept to loose some slots...
			 */
			if ( numGates == 0){
				EV << "ONUMacCtl: MPCP_GATE arrived, IT IS INITIAL - MAC: "<<frame->getDest()<<endl;
				// Cancel ALL an scheduled TX
				cancelEvent(startTxMsg);
				cancelEvent(stopTxMsg);
				scheduleStartTxPeriod();
			}else{

				if (clock_reg>start_reg + len_reg){
					// Time assigned is in past
					EV << "ONUMacCtl: MPCP_GATE arrived, calculating lost slots"<<endl;

					// Super Slot ...
					uint64_t slotTime16ns = (double)slotLength/16*slotNumber;
					uint32_t lostSlots = ceil((double)(clock_reg - start_reg)/slotTime16ns);
					shiftStartXSlots(lostSlots);
				}else if (clock_reg>=start_reg && clock_reg < start_reg + len_reg){
					EV << "ONUMacCtl: MPCP_GATE arrived, it is our time"<<endl;
					// Now is our time
					scheduleStopTxPeriod();
					startTxOnPON();
				}else{
					// Time assigned is in future
					EV << "ONUMacCtl: MPCP_GATE arrived, no lost slots"<<endl;
					scheduleStartTxPeriod();
				}
			}


			numGates++;
			break;
		}
		default:
			break;
	};

}


void ONUMacCtl::scheduleStartTxPeriod(){

	EV << "\n==ONUMacCtl: CHANGING STATE FROM ?? TO _OFF_\n";
	transmitState = TX_OFF;


	/*
	 *  start_reg shifted clock_reg did not
	 *  HANDLE AND RETURN.
	 *
	 *  NOTE: 	This should occur only on shifted registers.
	 *  		MPCP could have the same result but we are fixing
	 *  		it by shifting the clock on receive.
	 */
	if (start_reg!=0 && (uint64_t)start_reg+len_reg<clock_reg) {
		EV<<"ONUMacCtl: Start_reg shifted clock_reg did not"<<endl;
		dumpReg();
		//error("start_reg+len_reg<clock_reg ");


		simtime_t nextTx;

		nextTx.setRaw(simTime().raw() + // NOW
				MPCPTools::ns16ToSimTime(MPCP_CLOCK_MAX - clock_reg + // Remaining (not shifted clock)
						start_reg
					));
		EV<<"...."<<(MPCP_CLOCK_MAX - clock_reg +start_reg)<<endl;
		EV<<"scheduleStartTxPeriod: "<<nextTx<<endl;

		cancelEvent(startTxMsg);
		scheduleAt(nextTx, startTxMsg);
		return;
	}

	simtime_t nextTx;
	// Start the next moment...
	nextTx.setRaw(simTime().raw());

	// If start time is ahead sim time Start now
	// clock_reg == MPCPTools::simTimeToNS16()....
	if (start_reg>=clock_reg){
		nextTx.setRaw(simTime().raw()+								//Now (NOTE: not the clock_reg)
					  MPCPTools::ns16ToSimTime(start_reg-clock_reg) //Remaining time to Start
					 );
		EV << "ONUMacCtl: Start scheduled. Clock: "<<clock_reg<<" Start: "<<start_reg<<endl;
	}else{
		EV << "ONUMacCtl: Starting NOW. Clock: "<<clock_reg<<" Start: "<<start_reg<<endl;
	}


	cancelEvent(startTxMsg);
	scheduleAt(nextTx, startTxMsg);
}


/*
 * 			     length
 *              ------------
 * time --------|----|-----|------
 *         start^    \     ^stop
 *         		 	  \now
 */
void ONUMacCtl::scheduleStopTxPeriod(){

	// You cannot calculate next time from registers...
	// Simulation time may be different (regs are shifted)
	// So calculate the remaining time...
	simtime_t stopTX;
	stopTX.setRaw( simTime().raw() + 						 // Now
			MPCPTools::ns16ToSimTime(start_reg +len_reg - clock_reg) // Remaining
	);
	dumpReg();
	EV << "Scheduled after (ns16): " << (start_reg +len_reg - clock_reg) <<endl;
	EV << "Scheduled after  (raw): " << MPCPTools::ns16ToSimTime(start_reg +len_reg - clock_reg) <<endl;

	// Just in case..
	cancelEvent(stopTxMsg);
	scheduleAt(stopTX, stopTxMsg);
}


void ONUMacCtl::handleStopTxPeriod(){

	// Try not to call it all the time
	if (queue_mod->isEmpty()) {

		transmitState = TX_SLEEP;
		cancelEvent(startTxMsg);
		EV << "\n==ONUMacCtl: CHANGING STATE FROM ?? TO _SLEEP_\n";

		return;
	}

	transmitState = TX_OFF;
	EV << "\n==ONUMacCtl: CHANGING STATE FROM ?? TO _OFF_\n";

	// Shift the start_reg
	shiftStart1Slot();
}



/**
 * Directly called for start Messages
 * and called if a frame arrives in IDLE.
 */
void ONUMacCtl::startTxOnPON(){
	/**
	 * Check to be sure...
	 *
	 * Sometimes because we add 0.001 to startTx schedule, results to
	 * clock_reg = start+len +1
	 *
	 * So, shift a slot.
	 */
	if (clock_reg>start_reg+len_reg && start_reg!=0) {
		shiftStart1Slot();
		std::cout<<"SHIT: "<<simTime()<<endl;
		return;
	}

	/**
	 * Check that the MAC layer is not in TX mode
	 *
	 * This happens on the case of rapid MPCP messages.
	 * What we do is to wait for the minimum frame length
	 * bit time which is : ((double)64*8)/txrate.
	 *
	 * NOTE:
	 * that +1 is needed to the nextTx because the 2 messages
	 * (EndTransmission and startTx) are scheduled at the same
	 * EXACT time. Thus simulator will parse first the latest
	 * (LIFO) which is the startTx and we will have to wait 1
	 * round more.
	 */
	if (emac->getQueueLength()>0){
		EV<<"DELAYing MAC busy: "<<simTime()<<endl;
		cancelEvent(startTxMsg);
		simtime_t delayTx = ((double)64*8)/txrate;
		simtime_t timerem;
		timerem.setRaw( MPCPTools::ns16ToSimTime( start_reg+len_reg-clock_reg ));
//		std::cout<<timerem<<" - "<<delayTx<<endl;
		if (timerem>delayTx){
//			std::cout<<"  reScheduling on : "<<delayTx+simTime()<<endl;
			scheduleAt(delayTx+simTime(), startTxMsg);
//			std::cout<<"Clock: "<<clock_reg<<endl;
//			std::cout<<"Start: "<<start_reg<<endl;
//			std::cout<<"Till: "<<start_reg+len_reg<<endl;
//			std::cout<<"Sched: "<<MPCPTools::simTimeToNS16(delayTx.raw()+simTime().raw())<<endl;

		}else{
//			std::cout<<"  shifting slot, no time this time "<<endl;
			shiftStart1Slot();
		}
		return;
	}


	EV << "\n==ONUMacCtl: CHANGING STATE FROM ?? TO _ON_\n";
	transmitState = TX_ON;

	uint32_t nextMsgSize =  queue_mod->getNextFrameSize();
	if (queue_mod->isEmpty() || nextMsgSize==0) {
		// MPCP period
		if (start_reg == 0){
			transmitState = TX_OFF;
			EV << "\n==ONUMacCtl: CHANGING STATE FROM _ON_ TO _OFF_\n";
			return;
		}
		transmitState = TX_IDLE;
		// Schedule stop
		// To get out of IDLE state if no packets are queued.
		EV << "\n==ONUMacCtl: CHANGING STATE FROM _ON_ TO _IDLE_ (next msg size: "<<queue_mod->getNextFrameSize()<<")\n";
		// cancel previous IDLE state
		cancelEvent(stopTxMsg);
		cancelEvent(startTxMsg);
		scheduleStopTxPeriod();
		return;
	}

	// Calculate TX and remaining time
	// NOTE: Change here for more bandwidth
	if (nextMsgSize<64) nextMsgSize=64;
	uint32_t bytes=nextMsgSize+PREAMBLE_BYTES+SFD_BYTES;
	bytes+=INTERFRAME_GAP_BITS/8;

	// TODO: Add laser on/off delay
	simtime_t timereq = ((double)bytes*8)/txrate;
	EV << "Total Bytes: "<<bytes<<" Total bits: "<<bytes*8<<" TX RATE: "<<txrate<<endl;

	simtime_t timerem;
	timerem.setRaw( MPCPTools::ns16ToSimTime( start_reg+len_reg-clock_reg ));
//std::cout<<start_reg+len_reg-clock_reg<<endl;
//std::cout<<"S+L>"<<(start_reg+len_reg)<<endl;
//std::cout<<"Clk>"<<clock_reg<<endl;
	EV << "*** It will take (sim): "<<timereq<<endl;
	EV << "*** We Still have (sim): "<<timerem<<endl;
	EV << "*** It will take (ns16): "<<MPCPTools::simTimeToNS16(timereq.raw())<<endl;
	EV << "*** We Still have (ns16): "<<MPCPTools::simTimeToNS16(timerem.raw())<<endl;
	dumpReg();

	// If we dont have time break
	if (timerem<timereq){
		// TODO: log fragmented time
		EV << "ONUMacCtl: Fragmented TimeSlot... Frame is too big\n";
		EV << "\n==ONUMacCtl: CHANGING STATE FROM _ON_ TO _OFF_\n";
		transmitState = TX_OFF;
		fragmentedTime += timerem;
		// Shift the start_reg
		shiftStart1Slot();
		// cancel the stop message
		// (else we lose a time slot)
		cancelEvent(stopTxMsg);

		return;
	}

	cPacket * msg = dynamic_cast<cPacket *>(queue_mod->getNextFrame());
	// Check the user implementation of the Queues
	if (!msg){
		error(  "Shit... we should never reach here: "
				"Your algorithm in Queue Module is faulty: "
				"Reported that the Q is not empty BUT "
				"getNextFrame() returned NULL frame?!?");
		return;
	}
	// Add CURRENT TIME STAMP
	MPCP * mpcp = dynamic_cast<MPCP *>(msg);
	if (mpcp){
		mpcp->setTs(clock_reg);
	}

	// We have enough time for the frame...
	//Send
	send(msg, "lowerLayerOut");
	numFramesFromHL++;


	EV << "*** Current simTime (RAW): \t"<<simTime().raw()<<endl;
	EV << "*** Next Message simTime(RAW): \t"<<simTime().raw() + timereq.raw()<<endl;
	EV << "*** Current simTime (ns16): \t"<<MPCPTools::simTimeToNS16(simTime().raw())<<endl;
	EV << "*** Next Message simTime(ns16): \t"<<MPCPTools::simTimeToNS16(simTime().raw() + timereq.raw())<<endl;
	simtime_t nextTx;
	// LOOK at the NOTE on the begging
	nextTx = simTime() + timereq;
//	std::cout<<nextTx<<endl;
//
	nextTx+= 0.001/(double)txrate;
//	std::cout<<nextTx<<endl<<"-----------------"<<endl;
	cancelEvent(startTxMsg);
	scheduleAt(nextTx, startTxMsg);

/*
	EV << "\n==ONUMacCtl: CHANGING STATE FROM ?? TO _ON_\n";
	transmitState = TX_ON;

	cancelEvent(startTxMsg);
	simtime_t timerem;
	timerem.setRaw( MPCPTools::ns16ToSimTime( start_reg+len_reg-clock_reg ));
	// Send All frame of time slot
	while(true){
		if (queue_mod->isEmpty()) {
			// MPCP period
			if (start_reg == 0){
				transmitState = TX_OFF;
				EV << "\n==ONUMacCtl: CHANGING STATE FROM _ON_ TO _OFF_\n";
				return;
			}
			transmitState = TX_IDLE;
			// Schedule stop
			// To get out of IDLE state if no packets are queued.
			EV << "\n==ONUMacCtl: CHANGING STATE FROM _ON_ TO _IDLE_\n";
			// cancel previous IDLE state
			cancelEvent(stopTxMsg);
			cancelEvent(startTxMsg);
			scheduleStopTxPeriod();
			return;
		}


		// Calculate TX and remaining time
		// NOTE: Change here for more bandwidth
		uint64_t txrate=GIGABIT_ETHERNET_TXRATE;
		uint32_t bytes=queue_mod->getNextFrameSize()+PREAMBLE_BYTES+SFD_BYTES;

		bytes+=INTERFRAME_GAP_BITS/8;

		// TODO: Add laser on/off delay
		simtime_t timereq = ((double)(bytes*8))/txrate;

		EV << "Bytes: "<<bytes<<" bits: "<<bytes*8<<" TX RATE: "<<txrate<<endl;
		EV << "*** It will take (sim): "<<timereq<<endl;
		EV << "*** We Still have (sim): "<<timerem<<endl;
		EV << "*** It will take (ns16): "<<MPCPTools::simTimeToNS16(timereq.raw())<<endl;
		EV << "*** We Still have (ns16): "<<MPCPTools::simTimeToNS16(timerem.raw())<<endl;
		//dumpReg();

		// If we dont have time break
		if (timerem<timereq){
			EV << "ONUMacCtl: Fragmented TimeSlot... Frame is too big\n";
			EV << "\n==ONUMacCtl: CHANGING STATE FROM _ON_ TO _OFF_\n";
			transmitState = TX_OFF;
			fragmentedTime += timerem;
			// Shift the start_reg
			shiftStart1Slot();
			// cancel the stop message
			// (else we lose a time slot)
			cancelEvent(stopTxMsg);

			return;
		}


		cPacket * msg = dynamic_cast<cPacket *>(queue_mod->getNextFrame());

		// Check the user implementation of the Queues
		if (!msg){
			error(  "Shit... we should never reach here: "
					"Your algorithm in Queue Module is faulty: "
					"Reported that the Q is not empty BUT "
					"getNextFrame() returned NULL frame?!?");
			return;
		}

		// Add CURRENT TIME STAMP
		MPCP * mpcp = dynamic_cast<MPCP *>(msg);
		if (mpcp){
			mpcp->setTs(clock_reg);
		}

		// We have enough time for the frame...
		//Send
		send(msg, "lowerLayerOut");
		timerem-=timereq;
		numFramesFromHL++;

	}*/

}


void ONUMacCtl::clockSync(){

	EV << "\n\n============= CLOCK: ";
	// NOTE: Clock is in sync with the simulation clock
	// a small skew can be added here (random)


	// Reset start/stop
	uint32_t simT = MPCPTools::simTimeToNS16();


	// No clock shift
	if (simT>=clock_reg){
		// Update the clock
		clock_reg = simT;
		EV << clock_reg << "=======================\n";
		return;
	}

	/// Multi shift
	if (start_reg<simT){
		EV << "MULTI SHIFT =====================\n" <<endl;
		clock_reg = simT;
		// Super Slot ...
		uint64_t slotTime16ns = (double)slotLength/16*slotNumber;
		uint32_t lostSlots = ceil((double)(clock_reg - start_reg)/slotTime16ns);
		shiftStartXSlots(lostSlots);
		return;
	}

	// Single shift (1 reg loop)

	/**
	 * Check Slot Shift...
	 * NOTE: skew introduced
	 */

	// Reschedule the clock
	// In ANY case start reg is wrong
	cancelEvent(startTxMsg);
	cancelEvent(stopTxMsg);

	EV << " SHIFT =======================\n";
	dumpReg();
	uint64_t superSlotTime16ns = ((double)slotLength/16)*slotNumber;
	uint32_t remclk = MPCP_CLOCK_MAX - clock_reg;
	EV << "Remaining Clock: "<< remclk<<endl;
	EV << "Time Lost: "<< simT+remclk <<endl;
	uint64_t spSlotsLost       = ceil((double)( simT+remclk )/superSlotTime16ns);
	EV << "Slots Lost: "<< spSlotsLost <<endl;

	// Update the clock
	clock_reg = simT;
	shiftStartXSlots(spSlotsLost);

	dumpReg();

	// We are IN the transmission area
	if (clock_reg>=start_reg && clock_reg < start_reg+len_reg){
		EV << ".... IN TX PERIOD "<<endl;
		startTxOnPON();
	// We are in the same slot AFTER the tx area
	}else if (clock_reg>start_reg+len_reg){
		EV << " .... SHIFT IS NEEDED "<<endl;
		shiftStart1Slot();
	// We are in the same slot BEFORE the tx area
	}else {
		EV << ".... SCHED. NEXT PERIOD "<<endl;
		scheduleStartTxPeriod();
	}


	EV << "===========================================\n\n\n";

}



void ONUMacCtl::shiftStart1Slot(){

	uint64_t slotTime16ns = ((double)slotLength/16)*slotNumber;

	EV << "Increasing Start Reg.: old=" << start_reg
		<<" + " << slotTime16ns;

	// NOTE: start_reg may OVERFLOW TOOooo...  tsouf
	start_reg = ((uint64_t)start_reg+slotTime16ns)%MPCP_CLOCK_MAX;


	EV<<" = "<<start_reg<<" Clock Now: "<<clock_reg<<endl;
	EV<<"SlotLength: "<<slotTime16ns<<endl<<endl;


	// Try not to call it all the time
	if (queue_mod->isEmpty()) {

		transmitState = TX_SLEEP;
		EV << "\n==ONUMacCtl: CHANGING STATE FROM ?? TO _SLEEP_\n";
		cancelEvent(startTxMsg);
		return;
	}

	// Reschedule next TX
	scheduleStartTxPeriod();
}

void ONUMacCtl::shiftStartXSlots(uint32_t X){
	uint64_t slotTime16ns = (double)slotLength/16*slotNumber;

	EV << "Increasing Clock: old=" << start_reg
		<<" + " << X*slotTime16ns;

	start_reg = ((uint64_t)start_reg+X*slotTime16ns)%MPCP_CLOCK_MAX;


	EV<<" = "<<start_reg<<endl;
	EV<<"Lost Slots: "<<X<<" SlotLength: "<<slotTime16ns<<endl<<endl;

	// Reschedule next TX
	scheduleStartTxPeriod();
}


void ONUMacCtl::dumpReg(){
	EV << "Sim(ns16): "<<MPCPTools::simTimeToNS16()<<endl;
	EV << "Clock: "<<clock_reg<<endl;
	EV << "Start: "<<start_reg<<endl;
	EV << "Length: "<<len_reg<<endl;
}



void ONUMacCtl::finish ()
{
    simtime_t t = simTime();
    recordScalar("simulated time", t);
    recordScalar("messages handled", numFramesFromHL+numFramesFromLL);
    recordScalar("Dropped Frames From HL", numFramesFromHLDropped);
    double fragBits = fragmentedTime.dbl()/(1/GIGABIT_ETHERNET_TXRATE);
    recordScalar("FragmentedBits", fragBits);
    if (t>0) {
        recordScalar("frames/sec", (numFramesFromHL+numFramesFromLL)/t);
        recordScalar("drops/sec", numFramesFromHLDropped/t);
        recordScalar("FragmentedBits/sec", fragBits/t);
    }
}

cModule * ONUMacCtl::getNeighbourOnGate(const char * g){
	return gate(g)->getNextGate()->getOwnerModule();
}


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

#include "OLTMacCtl.h"


Define_Module(OLTMacCtl);

OLTMacCtl::~OLTMacCtl(){
	cancelAndDelete(txEnd);
}

void OLTMacCtl::initialize()
{
	clock_reg=0;

	// Watch the clock!!!
	// (AND TRY NOT TO TOUCH IT)
    WATCH(clock_reg);


    txEnd = new cMessage("TxEnd",TXENDMSG);
    transmitState=TX_IDLE;
    WATCH(transmitState);


    // Initialize the Q mgmt module
    queue_mod = dynamic_cast<IEponQueueMgmt *>(getNeighbourOnGate("upperLayerOut"));
    if (!queue_mod)
        	error("ONUMacCtl: A IEponQueueMgmt is needed above mac control");

}

void OLTMacCtl::handleMessage(cMessage *msg)
{

	// Do clock_reg sync
	clockSync();

	// Self Message
	if (msg->isSelfMessage())
	{
		EV << "Self-message " << msg << " received\n";
		if (msg->getKind() == TXENDMSG)
			handleTxEnd();
		else
			EV << "UnKnown Self Message\n";

		return;
	}

	// Wake Up message
	if (msg->getKind() == WAKEUPMSG){
		EV << "OLTMacCtl::WAKE UP Received" <<endl;
		// handle tx end will react like
		// a transmission just finished ==
		// check the Q module and keep on sending.
		if (transmitState == TX_IDLE)
			handleTxEnd();
		// Be clean
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
		EV << "OLTMacCtl: Message came FROM THE WRONG DIRRECTION???? Dropping\n";
		delete msg;
	}
}

void OLTMacCtl::processFrameFromHigherLayer(cMessage *msg){
	EV << "OLTMacCtl: Incoming to PON area...\n";
	EthernetIIFrame * frame = dynamic_cast<EthernetIIFrame *>(msg);
	if (frame && frame->getEtherType() == MPCP_TYPE){
		MPCP * mpcp = check_and_cast<MPCP *>(msg);
		mpcp->setTs(MPCPTools::simTimeToNS16());
	}
	doTransmit(msg);
}

void OLTMacCtl::processFrameFromMAC(cMessage *msg){
	EV << "OLTMacCtl: Incoming message, forwarding to higher layer\n";

	// Not Control
	send(msg,"upperLayerOut");
	numFramesFromLL++;
}

/**
 * Handle transmissions and queue.
 */
void OLTMacCtl::doTransmit(cMessage * msg){
	cPacket * packet = check_and_cast<cPacket *>(msg);

	if (!packet)
			error("Queue module returned NULL frame");


	// Calculate TX and remaining time
	// NOTE: Change here for more bandwidth
	uint64_t txrate=GIGABIT_ETHERNET_TXRATE;

	uint32_t nextMsgSize =  packet->getByteLength();
	if (nextMsgSize == 0) {
		std::cout<<packet<<endl;
		error("Message size 0");
	}
	uint32_t bytes=nextMsgSize+PREAMBLE_BYTES+SFD_BYTES;
	bytes+=INTERFRAME_GAP_BITS/8;


	// TODO: Add laser on/off delay
	simtime_t timereq= ((double)(bytes*8)/txrate);
	EV << "Bytes: "<<bytes<<" bits: "<<bytes*8<<" TX RATE: "<<txrate<<endl;
	EV << "TX State: "<<transmitState<<endl;
	EV << "Scheduled after "<<(double)(bytes*8)/txrate<<"ns"<<endl;

	if (transmitState == TX_IDLE){
		transmitState=TX_SENDING;
		// Calculate the tx time
		EV << "EndTx Scheduled after "<< timereq.raw() << " time_now: "<<simTime().raw()<<endl;
		scheduleAt(simTime()+timereq, txEnd);
		EV << " Sending..."<<endl;
		send(msg, "lowerLayerOut");
	}
	else{
		error ("OLTMacCtl: Packet Arrived from higher layer"
				"while we where in transmission. This should never happen"
				"(normally).");
	}
}


void OLTMacCtl::handleTxEnd(){
	transmitState=TX_IDLE;
	if (queue_mod->isEmpty()) return;

	doTransmit(dynamic_cast<cMessage *>(queue_mod->getNextFrame()));
}

/*
 * Keep track of the clock register
 */
void OLTMacCtl::clockSync(){
	uint32_t simT = MPCPTools::simTimeToNS16();
	clock_reg=simT%MPCP_CLOCK_MAX;
}


// TOOLS
cModule * OLTMacCtl::getNeighbourOnGate(const char * g){
	return gate(g)->getNextGate()->getOwnerModule();
}

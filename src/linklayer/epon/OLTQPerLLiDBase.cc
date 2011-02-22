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

#include "OLTQPerLLiDBase.h"


OLTQPerLLiDBase::OLTQPerLLiDBase(){}

OLTQPerLLiDBase::~OLTQPerLLiDBase(){
	for (uint32_t i=0; i<pendingAck.size(); i++){
		cancelAndDelete((cMessage *)pendingAck[i]);
	}

	for (uint32_t i=0; i<pon_queues.size(); i++){
		pon_queues[i].clean();
	}
	pon_queues.clear();
}

void OLTQPerLLiDBase::initialize()
{
	// ONU table
	onutbl = NULL;
	onutbl = dynamic_cast<ONUTable *>( findModuleUp("onuTable"));
	if (!onutbl)
		error("Shit... no ONU table found ?!?!");


	// Service List
	serviceList=NULL;
	if (dynamic_cast<ServiceConfig *>( findModuleUp("serviceConfig")) != NULL){
		serviceList = &(dynamic_cast<ServiceConfig *>( findModuleUp("serviceConfig"))->srvs);
	}

	// Parameters
	regAckTimeOut = par("regAckTimeOut");
	regAckTimeOut/=1000;
	// in ns
	slotLength=par("slotLength");
	slotNumber=par("slotNumber");
	// Convert to ns16 time
	// (to be added in the MPCP frames)
	regTimeInt = par("regTimeInt");
	regTimeInt*=(double)1000 / 16;
	queueLimit = par("queueLimit");

	// Create Q for BroadCasts
	QueuePerMacLLid tmp_qpml;
	tmp_qpml.setServiceName("BroadCast");
	tmp_qpml.prior = 0.0;
	tmp_qpml.ml.llid = LLID_EPON_BC;
	addSortedMacLLID(tmp_qpml);

	allQsEmpty=true;
	nextQIndex = 0;


	// WATCH
	WATCH_VECTOR(pon_queues);
	sendMPCPGateReg();
}

void OLTQPerLLiDBase::handleMessage(cMessage *msg)
{

	// Self Message
	if (msg->isSelfMessage())
	{
		EV << "Self-message " << msg << " received\n";
		if (msg->getKind() == ONUPENACK)
			handleRegTimeOut(msg);
		else
			EV << "UnKnown Self Message\n";

		return;
	}

	// Network Message
	cGate *ingate = msg->getArrivalGate();
	EV << "Frame " << msg << " arrived on port " << ingate->getName() << "...\n";



	if (ingate->getId() ==  gate( "lowerLayerIn")->getId()){
		processFrameFromLowerLayer(msg);
	}
	else if (ingate->getId() ==  gate( "upperLayerIn")->getId()){
		processFrameFromHigherLayer(msg);
	}else{
		EV << "OLTMacCtl: Message came FROM THE WRONG DIRRECTION???? Dropping\n";
		delete msg;
	}
}

void OLTQPerLLiDBase::processFrameFromHigherLayer(cMessage *msg){
	EV << "OLT_Q_mgmt_PerLLiD: Incoming from higher layer...\n";

	/*
	 * Notify the lower layer.
	 */
	if (allQsEmpty){
		EV << "Direct tx: "<<msg<<endl;
		send(new cMessage("WakeUpAssHole",WAKEUPMSG), "lowerLayerOut");
	}

	EPON_LLidCtrlInfo *nfo = dynamic_cast<EPON_LLidCtrlInfo *>(msg->getControlInfo());

	// If frame has no info or is BC add to the BroadCast Q
	int Q_index=-1;
	if (!nfo || nfo->llid == LLID_EPON_BC){
		Q_index = getIndexForLLID(LLID_EPON_BC);
	}else{
		Q_index = getIndexForLLID(nfo->llid);
	}

	// Check if we can forward frame
	if (Q_index == -1){
		EV << "*** WRONG LLID : DROPPING" <<endl;
		delete msg;
		return;
	}

	// Update the incoming rate BEFORE discarding message
	// in case the Q is full.
	cPacket *pkt = dynamic_cast<cPacket *>(msg);
	pon_queues[Q_index].vec->numIncomingBits+=pkt->getBitLength();

	// Check that is not full
	if (pon_queues[Q_index].length() >= pon_queues[Q_index].queueLimit){
		pon_queues[Q_index].vec->numBytesDropped+=pkt->getByteLength();
		pon_queues[Q_index].vec->recordVectors();
		delete msg;
		return;
	}else{
		// Record the incoming bytes
		pon_queues[Q_index].vec->recordVectors();
	}
	// Add it
	pon_queues[Q_index].insert(msg);



}

void OLTQPerLLiDBase::processFrameFromLowerLayer(cMessage *msg){
	EV << "OLT_Q_mgmt_PerLLiD: Incoming from lower layer\n";

	EthernetIIFrame * frame = dynamic_cast<EthernetIIFrame *>(msg);
	if (frame && frame->getEtherType() == MPCP_TYPE){
		processMPCP(frame );
		return;
	}

	// Not Control
	send(msg,"upperLayerOut");
}



/**
 * Do MPCP stuff
 */
void OLTQPerLLiDBase::processMPCP(EthernetIIFrame *frame ){
	EV << "OLTMacCtl: MPCP Frame processing\n";
	MPCP * mpcp = check_and_cast<MPCP *>(frame);
	switch (mpcp->getOpcode())
	{
		case MPCP_REGREQ:
		{
			MPCPRegReq * req = check_and_cast<MPCPRegReq *>(frame);
			MPCPRegister *rspframe = new MPCPRegister();
			rspframe->setOpcode(MPCP_REGISTER);
			rspframe->setName("MPCPRegister");
			rspframe->setEtherType(MPCP_TYPE);
			rspframe->setDest(req->getSrc());
			rspframe->setPtpNumReg(req->getPtpNumReq());

			EV << "Log the LLIDs temporarly and add a TO timer\n";

			ONUTableEntry te;
			te.setId(req->getSrc());
			// Parse and set LLIDS
			rspframe->setLLIDsArraySize(req->getPtpNumReq());

			for (int i=0; i<req->getPtpNumReq(); i++){
				// 0xFFF = 4095 intrand -> [0-4095)
				uint32_t llid_tmp=(uint32_t)intrand(4095);
				while (te.addLLID(llid_tmp) <0 ){
					EV << "*** RANDOM GEN FAILED!!!!???\n";
					llid_tmp=(uint32_t)intrand(4095);
				}

				rspframe->setLLIDs(i,llid_tmp);
			}

			rspframe->setByteLength(MPCP_HEADER_LEN+MPCP_LIST_LEN+req->getPtpNumReq()*MPCP_LLID_LEN);

			//temptbl.addONU(te);
			temptbl.push_back(te);


			// NOTE: do NOT send... enqueue
			//send(rspframe,"lowerLayerOut");
			processFrameFromHigherLayer(rspframe);

			// Create and add TO self message
			cMessage *tmpmsg=new cMessage("OnuTOMsg", ONUPENACK);
			pendingAck.push_back(tmpmsg);
			scheduleAt(simTime()+regAckTimeOut, tmpmsg);

			break;
		}
		case MPCP_REGACK:
		{
			doOnuRegistration(mpcp->getSrc());
			break;
		}
		default:
			EV << "Unrecognized MPCP OpCode";
	};

	delete frame;
}


void OLTQPerLLiDBase::doOnuRegistration(MACAddress mac){
	// Find Entry Index....
	for (uint32_t i=0; i<temptbl.size(); i++){
		if (temptbl.at(i).getId() == mac){
			EV << "OLTMacCtl: ONU (MAC: "<<mac<<") request "<< i << " Registered"<<endl;
			// Clear message and move tb entry to the global
			cancelAndDelete(pendingAck[i]);
			pendingAck.erase(pendingAck.begin()+i);

			// Copy
			ONUTableEntry en=temptbl.at(i);
			onutbl->addONU(en);

			// Remove from tmp
			//temptbl.removeONU(i);
			temptbl.erase(temptbl.begin()+i);
			createONU_Q(en);

			break;
		}
	}


}

void OLTQPerLLiDBase::handleRegTimeOut(cMessage *msg){
	// Find Message Index....
	for (uint32_t i=0; i<pendingAck.size(); i++){
		if (pendingAck[i] == msg){
			EV << "OLTMacCtl: ONU request "<< i << " TimeOut"<<endl;
			// Clear message and tb entry
			cancelAndDelete((cMessage *)pendingAck[i]);
			pendingAck.erase(pendingAck.begin()+i);
			//temptbl.removeONU(i);
			temptbl.erase(temptbl.begin()+i);
		}
	}
}


void OLTQPerLLiDBase::createONU_Q(ONUTableEntry &en){

	mac_llid tmp_ml;
	tmp_ml.mac = en.getId();
	// Scan llids
	for (int j=0; j<en.getLLIDsNum(); j++){

		tmp_ml.llid = en.getLLID(j);

		QueuePerMacLLid tmp_q;
		tmp_q.ml = tmp_ml;
		char tmp_name[20];
		sprintf(tmp_name,"Default-%d",tmp_ml.llid);

		tmp_q.prior = 0.0;

		if (serviceList) {
			sprintf(tmp_name,"%s-%d",serviceList->at(j).name.c_str(),tmp_ml.llid);
			tmp_q.prior = serviceList->at(j).priority;
		}

		tmp_q.setServiceName(tmp_name);
		addSortedMacLLID(tmp_q);


	}

	// Do Upstream DBA and calculate tx timers
	DoUpstreamDBA();
	// Announce these timers to the ONUs
	SendGateUpdates();
}

void OLTQPerLLiDBase::addSortedMacLLID(QueuePerMacLLid tmp_qpml){
	// Just add it if empty...
	if (pon_queues.empty()) {
		pon_queues.push_back(tmp_qpml);
		return;
	}

	// Add in the proper possition
	for (PonQueues::iterator it = pon_queues.begin(); it != pon_queues.end(); it++){
		if ((*it).prior < tmp_qpml.prior){
			pon_queues.insert(it, tmp_qpml);
			return;
		}
	}

	// If not added till here... add to the end
	pon_queues.push_back(tmp_qpml);
}

int OLTQPerLLiDBase::getIndexForLLID(uint16_t llid){
	for (uint32_t i=0; i<pon_queues.size(); i++)
		if (pon_queues[i].ml.llid == llid) return i;

	return -1;
}

bool OLTQPerLLiDBase::existsInPONQueues(mac_llid ml){
	// MY FIND...
	bool found=false;
	for(PonQueues::const_iterator it = pon_queues.begin(); it != pon_queues.end(); ++it)
	{
		if ((mac_llid)it->ml == ml) {
			found=true;
			break;
		}
	}

	return found;
}

/**
 * Fair Allocation per MAC-LLID.
 */
void OLTQPerLLiDBase::DoUpstreamDBA(){
	/*
	 * We do not need to allocate for the broadcast
	 * Q on the upstream... so -1. Downstream is
	 * different...
	 */
	int numOfMACsLLIDs = pon_queues.size()-1;

	// Check IDLE
	if (numOfMACsLLIDs == 0) return;


	/**
	 * NOTE:
	 * regTime/16 + slotNumber*slotLength/16 < 65536
	 */
	double slots_per_ml = (double) slotNumber/numOfMACsLLIDs;
	uint32_t curpointer=regTimeInt;
	double each_llid = slots_per_ml*slotLength/16;

	// For each ONU generate start and length values
	for (int i=0; i<onutbl->getTableSize(); i++){
		CommitedTime ct;
		ct.length=each_llid*onutbl->getEntry(i)->getLLIDsNum();
		ct.start=curpointer;
		onutbl->getEntry(i)->setComTime(ct);
		curpointer+=each_llid*onutbl->getEntry(i)->getLLIDsNum();
	}

}

void OLTQPerLLiDBase::sendMPCPGateReg(){
	MPCPGate *gt = new MPCPGate();
	gt->setEtherType(MPCP_TYPE);
	gt->setOpcode(MPCP_GATE);
	gt->setName("MPCPGate(Reg)");
	MPCPTools::setGateLen(*gt, 1);

	gt->setDuration(0, regTimeInt);
	gt->setDest(MACAddress::BROADCAST_ADDRESS);
	gt->setSlotTime(slotLength);
	gt->setSlotsNum(slotNumber);

	// Header + List + (start + Len) + slotNum + slotLen
	gt->setByteLength(MPCP_HEADER_LEN+MPCP_LIST_LEN+MPCP_TIMERS_LEN+MPCP_SLOTINFO_LEN);

	send(gt, "lowerLayerOut");

}

void OLTQPerLLiDBase::SendGateUpdates(){

	for (int i=onutbl->getTableSize()-1; i>=0; i--){
		MPCPGate *gt = new MPCPGate();
		gt->setName("MPCPGate");
		gt->setEtherType(MPCP_TYPE);
		gt->setOpcode(MPCP_GATE);
		MPCPTools::setGateLen(*gt, 1);

		gt->setSlotTime(slotLength);
		gt->setSlotsNum(slotNumber);

		gt->setDest(onutbl->getEntry(i)->getId());
		gt->setStartTime(0, onutbl->getEntry(i)->getComTime().start);
		gt->setDuration(0, onutbl->getEntry(i)->getComTime().length);

		// Header + List + (start + Len) + slotNum + slotLen
		gt->setByteLength(MPCP_HEADER_LEN+MPCP_LIST_LEN+MPCP_TIMERS_LEN+MPCP_SLOTINFO_LEN);

		// Instead of sending the frames,
		// Q them up in the BC queue
		//processFrameFromHigherLayer(gt);
		QForContainer * q = &pon_queues[pon_queues.size()-1];
		if (q->isEmpty())
			q->insert(gt);
		else
			q->insertAfter(q->front(), gt);
		// Notify MAC
		send(new cMessage("WakeUpAssHole",WAKEUPMSG), "lowerLayerOut");
	}
}

/**
 * Check is all the Qs are empty. IF yes set
 * the all empty variable. This must be done after
 * the frame request from the lower layer.
 */
void OLTQPerLLiDBase::checkIfAllEmpty(){
	if (pon_queues.size() == 0) allQsEmpty = true;
	if (pon_queues[0].isEmpty()
			&& pon_queues.size() == 1) allQsEmpty = true;


	for (uint32_t i=0; i< pon_queues.size(); i++){
		if (!pon_queues[i].isEmpty()){
			allQsEmpty = false;
			return;
		}
	}

	allQsEmpty = true;
}


cModule * OLTQPerLLiDBase::findModuleUp(const char * name){
	cModule *mod = NULL;
	for (cModule *curmod=this; !mod && curmod; curmod=curmod->getParentModule())
	     mod = curmod->getSubmodule(name);
	return mod;
}

void OLTQPerLLiDBase::finish(){
	simtime_t t = simTime();

	for (uint32_t i=0; i<pon_queues.size(); i++){
		std::string srvName =pon_queues[i].getServiceName();
		std::string name=srvName +" BytesDropped";
		recordScalar(name.c_str(), pon_queues[i].vec->numBytesDropped  );
		name=srvName +" BytesSent";
		recordScalar(name.c_str(), pon_queues[i].vec->numBytesSent  );

		if (t>0)
		{
			name=srvName +" bytes dropped/sec";
			recordScalar(name.c_str(), pon_queues[i].vec->numBytesDropped/t);
			name=srvName +" bytes sent/sec";
			recordScalar(name.c_str(), pon_queues[i].vec->numBytesSent/t);
		}
	}
}

QueuePerMacLLid *
OLTQPerLLiDBase::getFastestQForMac(const MACAddress & mac){
	for (uint32_t i=0; i<pon_queues.size(); i++){
		if (pon_queues[i].ml.mac.compareTo(mac) == 0)
			return &pon_queues[i];
	}
	return NULL;
}

QueuePerMacLLid *
OLTQPerLLiDBase::getFastestQForMac(const std::string & mac){

	for (uint32_t i=0; i<pon_queues.size(); i++){
		if (pon_queues[i].ml.mac.str() == mac) return &pon_queues[i];
	}
	return NULL;
}

uint16_t
OLTQPerLLiDBase::getFastestLLiDForMac(const std::string & mac){

	for (uint32_t i=0; i<pon_queues.size(); i++){
		if (pon_queues[i].ml.mac.str() == mac) return pon_queues[i].ml.llid;
	}
	return NULL;
}


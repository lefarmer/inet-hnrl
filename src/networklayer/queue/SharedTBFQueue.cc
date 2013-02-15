//
// Copyright (C) 2012 Kyeong Soo (Joseph) Kim
// Copyright (C) 2005 Andras Varga
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


#include "SharedTBFQueue.h"

Define_Module(SharedTBFQueue);

SharedTBFQueue::SharedTBFQueue()
{
//    queues = NULL;
//    numQueues = NULL;
}

SharedTBFQueue::~SharedTBFQueue()
{
    for (int i=0; i<numQueues; i++)
    {
        delete queues[i];
        cancelAndDelete(conformityTimer[i]);
    }
}

void SharedTBFQueue::initialize()
{	
    PassiveQueueBase::initialize();

	// configuration
	frameCapacity = par("frameCapacity");
	numQueues = par("numQueues");
	mtu = par("mtu").longValue()*8; // in bit
	peakRate = par("peakRate"); // in bps
	threshValue = 0.95;
	earliestThreshTime = getMaxTime();

    // state
    const char *classifierClass = par("classifierClass");
    classifier = check_and_cast<IQoSClassifier *>(createOne(classifierClass));
    classifier->setMaxNumQueues(numQueues);

    outGate = gate("out");

    // set subqueues
    queues.assign(numQueues, (cQueue *)NULL);
    meanBucketLength.assign(numQueues, 0); // was bucketSize
    peakBucketLength.assign(numQueues, mtu);
    lastTime.assign(numQueues, simTime());
    conformityFlag.assign(numQueues, false);
    conformityTimer.assign(numQueues, (cMessage *)NULL);
	isActive.assign(numQueues, true);
	meanRate.assign(numQueues, 0.0);
	modRate.assign(numQueues, 0.0);
	
    for (int i=0; i<numQueues; i++)
    {
        char buf[32];
        sprintf(buf, "queue-%d", i);
        queues[i] = new cQueue(buf);
        conformityTimer[i] = new cMessage("Conformity Timer", i);   // message kind carries a queue index
    }

    // state: RR scheduler
    currentQueueIndex = 0;

    // statistic
    warmupFinished = false;
    numQueueReceived.assign(numQueues, 0);
    numQueueDropped.assign(numQueues, 0);
//	numQueueShaped.assign(numQueues, 0);
    numQueueUnshaped.assign(numQueues, 0);
    numQueueSent.assign(numQueues, 0);
	
	meanRateTotal = 0.0;
	
	// TODO: This par() usage might not work at all, test it ASAP
	for (int i=0; i<numQueues; i++)
	{
		meanRate[i] = par("meanRate" + i); // in bps
	//	bucketSize = par("bucketSize").longValue()*8LL; // TODO: Add an algorithm to determine bucketSize from meanRate
		bucketSize[i] = par("bucketSize").longValue()*8LL; // in bit
		modRate[i] = 0.0;
		meanRateTotal += meanRate[i];
	}
	
	// get contribution values (between 0 and 1)
	for (int i=0; i<numQueues; i++)
	{
		contribution[i] = meanRate[i] / meanRateTotal;
	}
}



// =============
//  Q U E U E S
// =============

void SharedTBFQueue::handleMessage(cMessage *msg)
{
    if (warmupFinished == false)
    {   // start statistics gathering once the warm-up period has passed.
        if (simTime() >= simulation.getWarmupPeriod()) {
            warmupFinished = true;
            for (int i = 0; i < numQueues; i++)
            {
                numQueueReceived[i] = queues[i]->getLength();   // take into account the frames/packets already in queues
            }
        }
    }

    if (msg->isSelfMessage())
    {   // Conformity Timer expires
        int queueIndex = msg->getKind();    // message kind carries a queue index
		
		if (queueIndex >= 0) // conformity message
		{
			conformityFlag[queueIndex] = true;
			if (packetRequested > 0)
			{
				cMessage *msg = dequeue();
				if (msg != NULL)
				{
					packetRequested--;
					if (warmupFinished == true)
					{
						numQueueSent[currentQueueIndex]++;
					}
					sendOut(msg);
				}
				else
				{
					error("%s::handleMessage: Error in round-robin scheduling", getFullPath().c_str());
				}
			}
		}
		else // queue threshold message
		{
			for (int i=0; i<numQueues; i++)
			{
				if (meanBucketLength[i] > bucketSize[i] * threshValue)
				{
					isActive[i] = false;
				}
				else
				{
					isActive[i] = true;
				}
			}
			update();
		}
    }
    else
    {   // a frame arrives
        int queueIndex = classifier->classifyPacket(msg);
        cQueue *queue = queues[queueIndex];
        if (warmupFinished == true)
        {
            numQueueReceived[queueIndex]++;
        }
        int pktLength = (check_and_cast<cPacket *>(msg))->getBitLength();
// DEBUG
        ASSERT(pktLength > 0);
// DEBUG
        if (queue->isEmpty())
        {
            if (isConformed(queueIndex, pktLength))
            {
                if (warmupFinished == true)
                {
                    numQueueUnshaped[queueIndex]++;
                }
                if (packetRequested > 0)
                {
                    packetRequested--;
                    if (warmupFinished == true)
                    {
                        numQueueSent[queueIndex]++;
                    }
                    currentQueueIndex = queueIndex;
                    sendOut(msg);
                }
                else
                {
                    bool dropped = enqueue(msg);
                    if (dropped)
                    {
                        if (warmupFinished == true)
                        {
                            numQueueDropped[queueIndex]++;
                        }
                    }
                    else
                    {
                        conformityFlag[queueIndex] = true;
                    }
                }
            }
            else
            {
//                numQueueShaped[queueIndex]++;
                bool dropped = enqueue(msg);
                if (dropped)
                {
                    if (warmupFinished == true)
                    {
                        numQueueDropped[queueIndex]++;
                    }
                }
                else
                {
                    triggerConformityTimer(queueIndex, pktLength);
                    conformityFlag[queueIndex] = false;
                }
            }
        }
        else
        {
            bool dropped = enqueue(msg);
            if (dropped)
            {
                if (warmupFinished == true)
                {
                    numQueueDropped[queueIndex]++;
                }
            }
        }

        if (ev.isGUI())
        {
            char buf[40];
            sprintf(buf, "q rcvd: %d\nq dropped: %d", numQueueReceived[queueIndex], numQueueDropped[queueIndex]);
            getDisplayString().setTagArg("t", 0, buf);
        }
    }
}

bool SharedTBFQueue::enqueue(cMessage *msg)
{
    int queueIndex = classifier->classifyPacket(msg);
    cQueue *queue = queues[queueIndex];

    if (frameCapacity && queue->length() >= frameCapacity)
    {
        EV << "Queue " << queueIndex << " full, dropping packet.\n";
        delete msg;
        return true;
    }
    else
    {
        queue->insert(msg);
        return false;
    }
}

cMessage *SharedTBFQueue::dequeue()
{
    bool found = false;
    int startQueueIndex = (currentQueueIndex + 1) % numQueues;  // search from the next queue for a frame to transmit
    for (int i = 0; i < numQueues; i++)
    {
       if (conformityFlag[(i+startQueueIndex)%numQueues])
       {
           currentQueueIndex = (i+startQueueIndex)%numQueues;
           found = true;
           break;
       }
    }

    if (!found)
    {
        // TO DO: further processing?
        return NULL;
    }

    cMessage *msg = (cMessage *)queues[currentQueueIndex]->pop();

    // TO DO: update statistics

    // conformity processing for the HOL frame
    if (!queues[currentQueueIndex]->isEmpty())
    {
        int pktLength = (check_and_cast<cPacket *>(queues[currentQueueIndex]->front()))->getBitLength();
        if (isConformed(currentQueueIndex, pktLength))
        {
            conformityFlag[currentQueueIndex] = true;
        }
        else
        {
            conformityFlag[currentQueueIndex] = false;
            triggerConformityTimer(currentQueueIndex, pktLength);
        }
    }
    else
    {
        conformityFlag[currentQueueIndex] = false;
    }

    // return a packet from the scheduled queue for transmission
    return (msg);
}

void SharedTBFQueue::requestPacket()
{
    Enter_Method("requestPacket()");

    cMessage *msg = dequeue();
    if (msg==NULL)
    {
        packetRequested++;
    }
    else
    {
        if (warmupFinished == true)
        {
            numQueueSent[currentQueueIndex]++;
        }
        sendOut(msg);
    }
}

bool SharedTBFQueue::isConformed(int queueIndex, int pktLength)
{
    Enter_Method("isConformed()");

// DEBUG
    EV << "Last Time = " << lastTime[queueIndex] << endl;
    EV << "Current Time = " << simTime() << endl;
    EV << "Packet Length = " << pktLength << endl;
// DEBUG

    // update states
    simtime_t now = simTime();
    unsigned long long meanTemp = meanBucketLength[queueIndex] + (unsigned long long)(meanRate[queueIndex]*(now - lastTime[queueIndex]).dbl() + 0.5);
    meanBucketLength[queueIndex] = (long long)((meanTemp > bucketSize[queueIndex]) ? bucketSize[queueIndex] : meanTemp);
    unsigned long long peakTemp = peakBucketLength[queueIndex] + (unsigned long long)(peakRate*(now - lastTime[queueIndex]).dbl() + 0.5);
    peakBucketLength[queueIndex] = int((peakTemp > mtu) ? mtu : peakTemp);
    lastTime[queueIndex] = now;

    if (pktLength <= meanBucketLength[queueIndex])
    {
        if  (pktLength <= peakBucketLength[queueIndex])
        {
            meanBucketLength[queueIndex] -= pktLength;
            peakBucketLength[queueIndex] -= pktLength;
            return true;
        }
    }
    return false;
}

// trigger TBF conformity timer for the HOL frame in the queue,
// indicating that enough tokens will be available for its transmission
void SharedTBFQueue::triggerConformityTimer(int queueIndex, int pktLength)
{
    Enter_Method("triggerConformityCounter()");

    double meanDelay = (pktLength - meanBucketLength[queueIndex]) / meanRate[queueIndex] + modRate[queueIndex];
    double peakDelay = (pktLength - peakBucketLength[queueIndex]) / peakRate;

// DEBUG
    EV << "Packet Length = " << pktLength << endl;
    dumpTbfStatus(queueIndex);
    EV << "Delay for Mean TBF = " << meanDelay << endl;
    EV << "Delay for Peak TBF = " << peakDelay << endl;
    EV << "Current Time = " << simTime() << endl;
    EV << "Counter Expiration Time = " << simTime() + max(meanDelay, peakDelay) << endl;
// DEBUG

    scheduleAt(simTime() + max(meanDelay, peakDelay), conformityTimer[queueIndex]);
}

void SharedTBFQueue::dumpTbfStatus(int queueIndex)
{
    EV << "Last Time = " << lastTime[queueIndex] << endl;
    EV << "Current Time = " << simTime() << endl;
    EV << "Token bucket for mean rate/burst control " << endl;
    EV << "- Bucket size [bit]: " << bucketSize[queueIndex] << endl;
	EV << "- Mean rate [bps]: " << meanRate[queueIndex] << endl;
    EV << "- Bucket length [bit]: " << meanBucketLength[queueIndex] << endl;
    EV << "Token bucket for peak rate/MTU control " << endl;
    EV << "- MTU [bit]: " << mtu << endl;
    EV << "- Peak rate [bps]: " << peakRate << endl;
    EV << "- Bucket length [bit]: " << peakBucketLength[queueIndex] << endl;
}

void SharedTBFQueue::finish()
{
    unsigned long sumQueueReceived = 0;
    unsigned long sumQueueDropped = 0;
    unsigned long sumQueueShaped = 0;
    unsigned long sumQueueUnshaped = 0;

    for (int i=0; i < numQueues; i++)
    {
        std::stringstream ss_received, ss_dropped, ss_shaped, ss_sent;
        ss_received << "packets received by per-VLAN queue[" << i << "]";
        ss_dropped << "packets dropped by per-VLAN queue[" << i << "]";
        ss_shaped << "packets shaped by per-VLAN queue[" << i << "]";
        ss_sent << "packets sent by per-VLAN queue[" << i << "]";
        recordScalar((ss_received.str()).c_str(), numQueueReceived[i]);
        recordScalar((ss_dropped.str()).c_str(), numQueueDropped[i]);
        recordScalar((ss_shaped.str()).c_str(), numQueueReceived[i]-numQueueUnshaped[i]);
        recordScalar((ss_sent.str()).c_str(), numQueueSent[i]);
        sumQueueReceived += numQueueReceived[i];
        sumQueueDropped += numQueueDropped[i];
        sumQueueShaped += numQueueReceived[i] - numQueueUnshaped[i];
    }
    recordScalar("overall packet loss rate of per-VLAN queues", sumQueueDropped/double(sumQueueReceived));
    recordScalar("overall packet shaped rate of per-VLAN queues", sumQueueShaped/double(sumQueueReceived));
}



// =====================
//  C O N T R O L L E R
// =====================

void SharedTBFQueue::update()
{
	double tempSharedRateUsage = 0.0;
	unsigned long long meanTemp;
	unsigned long long peakTemp;
	
	// update own shared bucket first
	
	// TODO: Complete this, similar to isConformed()
	
//	simtime_t now = simTime();
	simtime_t tempThreshTime = simTime();
	
	// TODO: Cancel ALL current timers
	
	sharedRate = 0.0;
	
	// gather rates from donating subscribers
	for (int i=0; i<numQueues; i++)
	{
		if (!isActive[i])
		{
			modRate[i] = 0.0;
			sharedRate += meanRate[i];
		}
	}
	
	// then distribute rates among active subscribers
	for (int i=0; i<numQueues; i++)
	{
		if (isActive[i])
		{	
			modRate[i] = sharedRate * contribution[i];
			tempSharedRateUsage += modRate[i];
		}
	}
	
	sharedRate -= tempSharedRateUsage;
	
	// disribute the remaining sharedRate to active subscribers based on the sharedRate algorithm
	//---
	
	// if the algorithm cannot apply right now, leave sharedRate to be given to sharedBucket at next update()
	//---
	
	earliestThreshTime = getMaxTime();
	
	// find new timers for all queues
	for (int i=0; i<numQueues; i++)
	{
		if (isActive[i])
		{
			// FIXME this is wrong
			tempThreshTime = ((bucketSize[i] * threshValue) - meanBucketLength[i]) / (meanRate[i] + modRate[i])
			if (tempThreshTime < earliestThreshTime)
				earliestThreshTime = tempThreshTime;
		}
		triggerConformityTimer(i, (check_and_cast<cPacket *>(queues[i]->front()))->getBitLength());
	}
	sheduleAt(earliestThreshTime+100, conformityTimer[-1]);
}



// ===================
//  S C H E D U L E R
// ===================

// TODO: schedule()

void SharedTBFQueue::prioritySchedule(cMessage *msg, int priority) // msg or *msg?
{
	//---
}

void SharedTBFQueue::sendOut(cMessage *msg)
{
    send(msg, outGate);
}
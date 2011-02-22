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

#ifndef __ONUMACCTL_H__
#define __ONUMACCTL_H__

#include <omnetpp.h>
#include <string.h>
#include "EtherFrame_m.h"
#include "MPCP_codes.h"
#include "EPON_messages_m.h"
#include "MACAddress.h"
#include "MPCPTools.h"
#include "EPON_mac.h"
#include "IEponQueueMgmt.h"

// Self-message kind values
#define STARTTXMSG          100
#define STOPTXMSG			101
#define SFTSTRMSG			103

// Signal Q module to get new frame
#define GETFRAMEMSG			200
#define WAKEUPMSG			201


// States
/// Laser is ON and transmitting
#define TX_ON				1
/// Laser is OFF - Not our timeslot
#define TX_OFF				2
/// Pre-MPCP state - Only during initialization
#define TX_INIT				3
/// It IS our timeslot but there are not frames to Tx (or there are fragmented)
#define TX_IDLE				4
/// It may or may not be our turn BUT ALL Qs ARE EMPTY (no need to schedule startTx)
#define TX_SLEEP			5



/**
 * ONUMacCtl class controls the below MAC layer transmission times. This
 * version is fairly more complicated from the OLT one because we have
 * transmit in specific times. The module can come to IDLE or SLEEP states
 * which means that the clock and the start register are left back. (The
 * clock is updated on each message reception).
 *
 * Also this class has a pointer to the EPON_Q_mgmt module and it uses it
 * as a simple queue in order to get frames. Finally note that missing mac
 * addresses and LLIDs (from frames) are filled in here, cause the lower layer
 * expects them.
 */
class ONUMacCtl : public cSimpleModule
{
  protected:
	/// Tx Rate TODO: make it a parameter
	uint64_t txrate;
	/// Pointer to the mac layer for some control and use of MAC address
	EPON_mac * emac;
	// Registers are 32bit, 16ns granularity
	uint32_t clock_reg;
	uint32_t start_reg;
	uint32_t len_reg;
	int transmitState;
	int RTT;

	// Dynamic Parameters from OLT
	uint16_t slotLength;
	uint16_t slotNumber;

	// Hold the number of MPCP GATEs
	int numGates;

	// self messages
	cMessage *startTxMsg, *stopTxMsg, *syncMsg;

	// Queue
	IEponQueueMgmt * queue_mod;		// Upper Layer Q

	// Statistics
	int numFramesFromHL;
	int numFramesFromLL;
	int numFramesFromHLDropped;
	simtime_t fragmentedTime;

  public:
	ONUMacCtl();
	virtual ~ONUMacCtl();

  protected:
    virtual void initialize();
    virtual void finish();
    virtual void handleMessage(cMessage *msg);

    virtual void processFrameFromHigherLayer(cMessage *msg);
    virtual void processFrameFromMAC(cMessage *msg);
    virtual void processMPCP(EthernetIIFrame *frame );

    // Scheduling
    virtual void scheduleStartTxPeriod();
    virtual void scheduleStopTxPeriod();

    // Tx handling
    virtual void startTxOnPON();
    virtual void handleStopTxPeriod();


    // Clock Functions
    /**
     * Update the clock. Note that the clock can be
     * reseted back to 0 (zero) cause it is define
     * by MPCP as 32bit register (of 16 nanoseconds
     * granularity). This is also handled here.
     */
    virtual void clockSync();
    /**
     * This method shifts the start register for one
     * SuperSlot. SuperSlot is defined as the summation
     * of all the slot lengths.
     */
    virtual void shiftStart1Slot();
    /**
     * This method handles the start register when the
     * module wakes up from SLEEP state and it has
     * lost many SuperSlots.
     */
    virtual void shiftStartXSlots(uint32_t X);


    // DEBUG
    virtual void dumpReg();

    // TOOLS TODO: add to a different class
    virtual cModule * getNeighbourOnGate(const char * gate);
};

#endif

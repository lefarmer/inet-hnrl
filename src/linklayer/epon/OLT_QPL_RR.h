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

#ifndef OLT_QPL_RR_H_
#define OLT_QPL_RR_H_

#include <omnetpp.h>
#include "OLTQPerLLiDBase.h"


class OLT_QPL_RR : public OLTQPerLLiDBase{
public:
	virtual ~OLT_QPL_RR();

	virtual void initialize();

	// Inherited
		/**
		 * This method returns the next frame's size without removing it
		 * from the queue. Thus this method (and the getNextFrame()) control
		 * the queue's priorities and QoS. The default behavior is to do
		 * Round Robin on the queues. NOTE: RR is per frame...
		 *
		 * This could be overridden in a subclass and change the default
		 * behavior.
		 */
		virtual uint64_t getNextFrameSize();
		/**
		 * This method returns the next frame and removes it
		 * from the queue. Thus this method (and the getNextFrameSize()) control
		 * the queue's priorities and QoS. The default behavior is to do
		 * Round Robin on the queues. NOTE: RR is per frame...
		 *
		 * This could be overridden in a subclass and change the default
		 * behavior.
		 */
		virtual cPacket * getNextFrame();
		/**
		 * Returns if there is another frame for transmission. NOTE:
		 * In the current implementation (Round Robin) this is true only if
		 * all queues are empty. This is not though true when we apply _strict_
		 * QoS with standard committed data rate. The module may return true
		 * when no one has the right to tx (even if there are frames in the queues).
		 */
		virtual bool isEmpty();
};

#endif /* OLT_QPL_RR_H_ */
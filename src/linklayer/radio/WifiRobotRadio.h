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

#ifndef WIFIROBOTRADIO_H
#define WIFIROBOTRADIO_H

#include "Ieee80211Radio.h"

/**
 * Abstract base class for radio modules. Radio modules deal with the
 * transmission of frames over a wireless medium (the radio channel).
 * See the Radio module's NED documentation for an overview of radio modules.
 *
 * This class implements common functionality of the radio, and abstracts
 * out specific parts into two interfaces, IReceptionModel and IRadioModel.
 * The reception model is responsible for modelling path loss, interference
 * and antenna gain. The radio model is responsible for calculating frame
 * duration, and modelling modulation scheme and possible forward error
 * correction. Subclasses have to redefine the <tt>createReceptionModel()</tt>
 * and <tt>createRadioModel()</tt> methods to create and return appropriate
 * reception model and radio model objects.
 *
 * <b>History</b>
 *
 * The implementation is largely based on the Mobility Framework's
 * SnrEval and Decider modules. They have been merged into a single
 * module, multi-channel support, runtime channel and bitrate switching
 * capability added, and all code specific to the physical channel
 * and radio characteristics have been factored out into the IReceptionModel
 * and IRadioModel classes.
 *
 * @author Andras Varga, Levente Meszaros
 */
class INET_API WifiRobotRadio: public Ieee80211Radio
{
public:

protected:
	virtual void initialize(int stage);

	/** @brief Sends a message to the upper layer and mobility module */
	virtual void sendUp(AirFrame *airframe);

protected:
    /** @name Gate Ids */
	//@{
	int mobilityOut;
    //@}
};

#endif

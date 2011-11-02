//
// Copyright (C) 2011 Kyeong Soo (Joseph) Kim
// Copyright (C) 2005 Andras Varga
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

package inet.examples.wireless.WifiRobot;

import inet.networklayer.autorouting.FlatNetworkConfigurator;
import inet.nodes.wireless.WirelessAP;
import inet.nodes.wireless.WifiRobot;
import inet.world.ChannelControl;


network WifiRobotTest
{
    parameters:
        double playgroundSizeX;
        double playgroundSizeY;
        @display("bgb=600,400");
    submodules:
        robot: WifiRobot {
            @display("p=50,130;r=,,#707070");
        }
        ap1: WirelessAP {
            @display("p=24,380;r=,,#707070");
        }
        ap2: WirelessAP {
            @display("p=579,380;r=,,#707070");
        }
        ap3: WirelessAP {
            @display("p=579,19;r=,,#707070");
        }
        channelcontrol: ChannelControl {
            playgroundSizeX = playgroundSizeX;
            playgroundSizeY = playgroundSizeY;
            @display("p=60,50");
        }
        configurator: FlatNetworkConfigurator {
            @display("p=163,51");
        }
}

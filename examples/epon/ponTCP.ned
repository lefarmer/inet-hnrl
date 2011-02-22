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

import inet.linklayer.ethernet.EtherHub;
import inet.nodes.ethernet.EtherHost2;
import ned.DatarateChannel;
import inet.nodes.ethernet.EtherHost;
import inet.applications.dhcp.DHCPServer;
import inet.linklayer.ethernet.EthernetInterface;
import inet.nodes.ethernet.EtherSwitch;
import inet.nodes.inet.StandardHost;

import PON.ONU.*;
import PON.OLT.*;
import PON.common.*;
import PON.*;




import inet.networklayer.autorouting.FlatNetworkConfigurator;
network ponTCP
{
    types:
        channel GFX extends DatarateChannel
        {
            datarate = 1Gbps;
        }
    submodules:
        pon_Splitter: PON_Splitter {
            @display("p=230,142");
        }
        epon_onu: EPON_ONU {
            @display("p=432,66");
        }
        epon_onu3: EPON_ONU {
            @display("p=432,185");
        }
        epon_onu2: EPON_ONU {
            @display("p=432,123");
        }
        epon_olt: EPON_OLT {
            @display("p=141,142");
        }

        TCPHost1: StandardHost {
            @display("p=531,61");
        }
        TCPHost2: StandardHost {
            @display("p=526,127");
        }
        TCPHost3: StandardHost {
            @display("p=527,211");
        }
        Server: StandardHost {
            @display("p=69,89");
        }
        flatNetworkConfigurator: FlatNetworkConfigurator {
            @display("p=268,54");
        }
    connections:

        // to splitter
        epon_olt.epon <--> GFX <--> pon_Splitter.portu;
        pon_Splitter.portg++ <--> GFX <--> epon_onu.epon;
        pon_Splitter.portg++ <--> GFX <--> epon_onu2.epon;
        pon_Splitter.portg++ <--> GFX <--> epon_onu3.epon;


        TCPHost1.ethg++ <--> DatarateChannel <--> epon_onu.ethg;
        TCPHost2.ethg++ <--> DatarateChannel <--> epon_onu2.ethg;
        TCPHost3.ethg++ <--> DatarateChannel <--> epon_onu3.ethg;
        Server.ethg++ <--> DatarateChannel <--> epon_olt.ethg;
}

//
// Copyright (C) 2006 Andras Varga
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




package inet.nodes.wireless;

import inet.applications.pingapp.PingApp;
import inet.applications.tcpapp.TCPApp;
import inet.applications.udpapp.UDPApp;
import inet.base.NotificationBoard;
//import inet.linklayer.ethernet.EthernetInterface;
//import inet.linklayer.ieee80211.Ieee80211NicSTA;
import inet.linklayer.ieee80211.WifiRobotNic;
//import inet.linklayer.ppp.PPPInterface;
import inet.mobility.WifiRobotMobility;
import inet.networklayer.common.InterfaceTable;
import inet.networklayer.ipv4.RoutingTable;
import inet.nodes.inet.NetworkLayer;
import inet.transport.tcp.TCP;
import inet.transport.udp.UDP;
//import inet.linklayer.radio.WifiRobotRadio;


import inet.underTest.nodes.wireless.WirelessRouterExtended;
//
// Models a host with one wireless (802.11b) card in infrastructure mode.
// This module is basically a StandardHost with an Ieee80211NicSTA added.
// It should be used in conjunction with WirelessAP, or any other AP model
// which contains Ieee80211NicAP.
//
// @see WirelessAP, WirelessAPSimplified, WirelessAPWithEth, WirelessAPWithEthSimplified
// @see WirelessHost, WirelessHostSimplified
// @see MobileHost, MFMobileHost
//
module WifiRobot
{
    parameters:
        @node();
        @labels(node,ethernet-node,wifi-robot);
        @display("i=device/wifi_robot_s;bgb=486,432");
        int numTcpApps = default(0);
        int numUdpApps = default(0);
        string tcpAppType = default("");
        string udpAppType = default("");
        bool IPForward = default(false);
        string routingFile = default("");
    //        string mobilityType = default("WifiRobotMobility");
    gates:
//        inout pppg[] @labels(PPPFrame-conn);
//        inout ethg[] @labels(EtherFrame-conn);
        input radioIn @directIn;
    submodules:
        notificationBoard: NotificationBoard {
            parameters:
                @display("p=60,143");
        }
                interfaceTable: InterfaceTable {
                    parameters:
                        @display("p=60,223");
                }
                routingTable: RoutingTable {
                    parameters:
                        IPForward = IPForward;
                        routerId = "";
                        routingFile = routingFile;
                        @display("p=60,303");
                }
                tcpApp[numTcpApps]: <tcpAppType> like TCPApp {
                    parameters:
                        @display("p=180,63");
                }
                tcp: TCP {
                    parameters:
                        @display("p=180,150");
                }
                udpApp[numUdpApps]: <udpAppType> like UDPApp {
                    parameters:
                        @display("p=313,37");
                }
                udp: UDP {
                    parameters:
                        @display("p=313,150");
                }
                pingApp: PingApp {
                    parameters:
                        @display("p=440,150");
                }
                networkLayer: NetworkLayer {
                    parameters:
                        proxyARP = false;
                        @display("p=313,262;q=queue");
                    gates:
//                        ifIn[(sizeof(pppg)+sizeof(ethg))+1];
//                        ifOut[(sizeof(pppg)+sizeof(ethg))+1];
                        ifIn[1];
                        ifOut[1];
                }
        wlan: WifiRobotNic {
            parameters:
                @display("p=313,374;q=queue");
        }
        mobility: WifiRobotMobility {
            parameters:
                @display("p=58,374");
        }
//        radio: WifiRobotRadio {
//            parameters:
//                @display("p=96,240");
//        }
    connections allowunconnected:
        for i=0..numTcpApps-1 {
            tcpApp[i].tcpOut --> tcp.appIn++;
            tcpApp[i].tcpIn <-- tcp.appOut++;
        }

        tcp.ipOut --> networkLayer.tcpIn;
        tcp.ipIn <-- networkLayer.tcpOut;

        for i=0..numUdpApps-1 {
            udpApp[i].udpOut --> udp.appIn++;
            udpApp[i].udpIn <-- udp.appOut++;
        }

        udp.ipOut --> networkLayer.udpIn;
        udp.ipIn <-- networkLayer.udpOut;

        networkLayer.pingOut --> pingApp.pingIn;
        networkLayer.pingIn <-- pingApp.pingOut;

        radioIn --> wlan.radioIn;
        wlan.mobilityOut --> mobility.lowergateIn;
        
        wlan.uppergateOut --> networkLayer.ifIn[0];
        wlan.uppergateIn <-- networkLayer.ifOut[0];
}


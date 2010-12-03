/***************************************************************************
 *   Copyright (C) 2008 by Alfonso Ariza                                   *
 *   aarizaq@uma.es                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

///

#include "ManetRoutingBase.h"
#include "UDPPacket.h"
#include "IPControlInfo.h"
#include "IPv4InterfaceData.h"
#include "IPv6ControlInfo.h"
#include "Ieee802Ctrl_m.h"
#include "RoutingTableAccess.h"
#include "InterfaceTableAccess.h"

#define IP_DEF_TTL 32
#define UDP_HDR_LEN	8


void ManetRoutingBase::registerRoutingModule()
{
	InterfaceEntry *   ie;
	InterfaceEntry *   i_face;
	const char *name;

 	/* Set host parameters */
	int  num_80211=0;
	inet_rt = RoutingTableAccess ().get();
	inet_ift = InterfaceTableAccess ().get();
	nb = NotificationBoardAccess().get();

	for (int i = 0; i < inet_ift->getNumInterfaces(); i++)
	{
		ie = inet_ift->getInterface(i);
		name = ie->getName();
		if (strstr (name,"wlan")!=NULL)
		{
			i_face = ie;
			InterfaceIdentification interface;
			interface.interfacePtr = ie;
			interface.index = i;
			num_80211++;
			interfaceVector.push_back(interface);
		}
	}

	routerId = inet_rt->getRouterId();

	if (interfaceVector.size()==0 || interfaceVector.size() > (unsigned int)maxInterfaces)
		opp_error ("Manet routing protocol has found %i wlan interfaces",num_80211);

	const char *classname = getParentModule()->getClassName();
	mac_layer_=false;
	if (strcmp(classname,"Ieee80211Mesh")==0)
	{	
		hostAddress = interfaceVector[0].interfacePtr->getMacAddress();
		mac_layer_=true;
	}
	else
		hostAddress = interfaceVector[0].interfacePtr->ipv4Data()->getIPAddress();

	// One enabled network interface (in total)
}


bool ManetRoutingBase::isIpLocalAddress (const IPAddress& dest) const 
{
	return inet_rt->isLocalAddress(dest);
}



bool ManetRoutingBase::isLocalAddress (const Uint128& dest) const 
{
	if (!mac_layer_)
		return inet_rt->isLocalAddress(dest.getIPAddress());
	InterfaceEntry *   ie;
	for (int i = 0; i < inet_ift->getNumInterfaces(); i++)
	{
		ie = inet_ift->getInterface(i);
		Uint128 add = ie->getMacAddress();
		if (add==dest) return true;
	}
	return false;
}

void ManetRoutingBase::linkLayerFeeback()
{
	nb->subscribe(this, NF_LINK_BREAK);
}

void ManetRoutingBase::linkPromiscuous()
{
	nb->subscribe(this, NF_LINK_PROMISCUOUS);
}

void ManetRoutingBase::linkFullPromiscuous()
{
	nb->subscribe(this, NF_LINK_FULL_PROMISCUOUS);
}

void ManetRoutingBase::sendToIp (cPacket *msg, int srcPort, const Uint128& destAddr, int destPort,int ttl,const Uint128 &interface)
{
     sendToIp (msg,srcPort,destAddr,destPort,ttl,0,interface);
}

void ManetRoutingBase::processLinkBreak(const cPolymorphic *details){return;}
void ManetRoutingBase::processPromiscuous (const cPolymorphic *details){return;}
void ManetRoutingBase::processFullPromiscuous (const cPolymorphic *details){return;}

void ManetRoutingBase::sendToIp (cPacket *msg, int srcPort, const Uint128& destAddr, int destPort,int ttl,double delay,const Uint128 &iface)
{
	InterfaceEntry  *ie = NULL;
	if (mac_layer_)
	{
		Ieee802Ctrl *ctrl = new Ieee802Ctrl;
		MACAddress macadd = (MACAddress) destAddr;
		IPAddress add = destAddr.getIPAddress();
		if (iface!=0)
		{
			ie = getInterfaceWlanByAddress (iface); // The user want to use a pre-defined interface
		}
		else
			ie = interfaceVector[interfaceVector.size()-1].interfacePtr;

		if (IPAddress::ALLONES_ADDRESS==add)
			ctrl->setDest(MACAddress::BROADCAST_ADDRESS);
		else
			ctrl->setDest(macadd);

		if (ctrl->getDest()==MACAddress::BROADCAST_ADDRESS)
		{
			for (unsigned int i = 0;i<interfaceVector.size()-1;i++)
			{
// It's necessary to duplicate the the control info message and include the information relative to the interface
				Ieee802Ctrl *ctrlAux = ctrl->dup();
				ie = interfaceVector[i].interfacePtr;
				cPacket *msgAux = msg->dup();
// Set the control info to the duplicate packet
				if (ie)
					ctrlAux->setInputPort(ie->getInterfaceId());
				msgAux->setControlInfo(ctrlAux);
				sendDelayed(msgAux,delay,"to_ip");
				
			}
			ie = interfaceVector[interfaceVector.size()-1].interfacePtr;
		}

		if (ie)
			ctrl->setInputPort(ie->getInterfaceId());
		msg->setControlInfo(ctrl);
		sendDelayed(msg,delay,"to_ip");
		return;
	}

	UDPPacket *udpPacket = new UDPPacket(msg->getName());
	udpPacket->setByteLength(UDP_HDR_LEN);
	udpPacket->encapsulate(msg);
	//IPvXAddress srcAddr = interfaceWlanptr->ipv4Data()->getIPAddress();

	if (ttl==0)
	{
        // delete and return 
		delete msg;
		return;
	}
	// set source and destination port
	udpPacket->setSourcePort(srcPort);
	udpPacket->setDestinationPort(destPort);
	

	if (iface!=0)
	{
		ie = getInterfaceWlanByAddress (iface); // The user want to use a pre-defined interface
	}

	//if (!destAddr.isIPv6())
	if (true)
	{
		// send to IPv4
		IPAddress add = destAddr.getIPAddress();
		IPAddress  srcadd;


// If found interface We use the address of interface
		if (ie)              
			srcadd = ie->ipv4Data()->getIPAddress();
		else
			srcadd = hostAddress.getIPAddress();

		EV << "Sending app packet " << msg->getName() << " over IPv4." << " from " << 
			add.str() << " to " << add.str() << "\n";
		IPControlInfo *ipControlInfo = new IPControlInfo();
		ipControlInfo->setDestAddr(add);
		//ipControlInfo->setProtocol(IP_PROT_UDP);
		ipControlInfo->setProtocol(IP_PROT_MANET);

		ipControlInfo->setTimeToLive(ttl);
		udpPacket->setControlInfo(ipControlInfo);

		if (ie!=NULL)
			ipControlInfo->setInterfaceId(ie->getInterfaceId());

		if (add == IPAddress::ALLONES_ADDRESS && ie == NULL)
		{
// In this case we send a broadcast packet per interface
			for (unsigned int i = 0;i<interfaceVector.size()-1;i++)
			{
				ie = interfaceVector[i].interfacePtr;
				srcadd = ie->ipv4Data()->getIPAddress();
// It's necessary to duplicate the the control info message and include the information relative to the interface
				IPControlInfo *ipControlInfoAux = new IPControlInfo(*ipControlInfo);
				ipControlInfoAux->setInterfaceId(ie->getInterfaceId());
				ipControlInfoAux->setSrcAddr(srcadd);
				UDPPacket *udpPacketAux = udpPacket->dup();
// Set the control info to the duplicate udp packet
				udpPacketAux->setControlInfo(ipControlInfo);
				sendDelayed(udpPacketAux,delay,"to_ip");
			}
			ie = interfaceVector[interfaceVector.size()-1].interfacePtr;
			srcadd = ie->ipv4Data()->getIPAddress();
			ipControlInfo->setInterfaceId(ie->getInterfaceId());
		}
		ipControlInfo->setSrcAddr(srcadd);
		sendDelayed(udpPacket,delay,"to_ip");
	}
	else
	{
		// send to IPv6
		EV << "Sending app packet " << msg->getName() << " over IPv6.\n";
		IPv6ControlInfo *ipControlInfo = new IPv6ControlInfo();
		// ipControlInfo->setProtocol(IP_PROT_UDP);
		ipControlInfo->setProtocol(IP_PROT_MANET);
		ipControlInfo->setSrcAddr((IPv6Address) hostAddress);
		ipControlInfo->setDestAddr((IPv6Address)destAddr);
		ipControlInfo->setHopLimit(ttl);
		// ipControlInfo->setInterfaceId(udpCtrl->InterfaceId()); FIXME extend IPv6 with this!!!
		udpPacket->setControlInfo(ipControlInfo);
		sendDelayed(udpPacket,delay,"to_ip");
	}
	// totalSend++;
}


void ManetRoutingBase::omnet_chg_rte (const struct in_addr &dst, const struct in_addr &gtwy, const struct in_addr &netm,
		  short int hops,bool del_entry)
{
	omnet_chg_rte (dst.s_addr,gtwy.s_addr,netm.s_addr,hops,del_entry);
}

void ManetRoutingBase::omnet_chg_rte (const struct in_addr &dst, const struct in_addr &gtwy, const struct in_addr &netm,
		  short int hops,bool del_entry,const struct in_addr &iface)
{
	omnet_chg_rte (dst.s_addr,gtwy.s_addr,netm.s_addr,hops,del_entry,iface.s_addr);
}

bool ManetRoutingBase::omnet_exist_rte (struct in_addr dst)
{
 Uint128 add = omnet_exist_rte (dst.s_addr);
 if (add==0) return false;
 else if (add==(Uint128)IPAddress::ALLONES_ADDRESS) return false;
 else return true;
}

void ManetRoutingBase::omnet_chg_rte (const Uint128 &dst, const Uint128 &gtwy, const Uint128 &netm,short int hops,bool del_entry,const Uint128 &iface)
{
	/* Add route to kernel routing table ... */
	IPAddress desAddress((uint32_t)dst);
	IPRoute *entry=NULL;

	if (mac_layer_)
		return;

	bool found = false;
	for (int i=inet_rt->getNumRoutes(); i>0 ; --i)
	{
		const IPRoute *e = inet_rt->getRoute(i-1);
		if (desAddress == e->getHost())
		{
			if (del_entry && !found)
			{
				if (!inet_rt->deleteRoute(e))
					opp_error ("Aodv omnet_chg_rte can't delete route entry");  
			}
			else
			{
				found = true;
				entry = const_cast<IPRoute*>(e);
			}
		}
	}


	if (del_entry)
	   return;

	if (!found)
		entry = new   IPRoute();

	IPAddress netmask((uint32_t)netm);
	IPAddress gateway((uint32_t)gtwy);
	if (netm==0)
		netmask = IPAddress((uint32_t)dst).getNetworkMask().getInt();

	/// Destination
	entry->setHost(desAddress);
	/// Route mask
	entry->setNetmask (netmask);
	/// Next hop
	entry->setGateway (gateway);
	/// Metric ("cost" to reach the destination)
	entry->setMetric(hops);
	/// Interface name and pointer

	entry->setInterface(getInterfaceWlanByAddress(iface));

	/// Route type: Direct or Remote
	if(entry->getGateway().isUnspecified())
		entry->setType (IPRoute::DIRECT);
	else
		entry->setType (IPRoute::REMOTE);
	/// Source of route, MANUAL by reading a file,
	/// routing protocol name otherwise
	entry->setSource(IPRoute::MANET);

	if (!found)
		inet_rt->addRoute(entry);
}

//
// Check if it exists in the ip4 routing table the address dst
// if it doesn't exist return ALLONES_ADDRESS
// 
Uint128 ManetRoutingBase::omnet_exist_rte (Uint128 dst)
{
	/* Add route to kernel routing table ... */
	IPAddress desAddress((uint32_t)dst);
	const IPRoute *e=NULL;
	if (mac_layer_)
		return (Uint128) 0;
   	for (int i=inet_rt->getNumRoutes(); i>0 ; --i)
   	{
		e = inet_rt->getRoute(i-1);
		if (desAddress == e->getHost())
			return e->getGateway().getInt();
	}
	return (Uint128)IPAddress::ALLONES_ADDRESS;
}

//
// Erase all the enties int Ip4 the routing table
//
void ManetRoutingBase::omnet_clean_rte ()
{
	const IPRoute *entry;
	if (mac_layer_)
		return;
	// clean the route table wlan interface entry
	for (int i=inet_rt->getNumRoutes()-1;i>=0;i--)
	{
		entry= inet_rt->getRoute(i);	
		if (strstr (entry->getInterface()->getName(),"wlan")!=NULL)
		{
			inet_rt->deleteRoute(entry);
		}
	}
}

//
// generic receiveChangeNotification, the protocols must implemet processLinkBreak and processPromiscuous only
//
void ManetRoutingBase::receiveChangeNotification(int category, const cPolymorphic *details)
{
	Enter_Method("Manet llf");
	if (category == NF_LINK_BREAK)
	{
		if (details==NULL)
			return;
		processLinkBreak(details);
	}
	else if (category == NF_LINK_PROMISCUOUS)
	{
		processPromiscuous(details);
	}
	else if (category == NF_LINK_FULL_PROMISCUOUS)
	{
		processFullPromiscuous(details);
	}
}

/*
  Replacement for gettimeofday(), used for timers.
  The timeval should only be interpreted as number of seconds and
  fractions of seconds since the start of the simulation.
*/
int ManetRoutingBase::gettimeofday(struct timeval *tv, struct timezone *tz)
{
	double current_time;
	double tmp;
	/* Timeval is required, timezone is ignored */
	if (!tv)
	return -1;
	current_time =SIMTIME_DBL(simTime());
	tv->tv_sec = (long)current_time; /* Remove decimal part */
	tmp = (current_time - tv->tv_sec) * 1000000;
	tv->tv_usec = (long)(tmp+0.5);
	if (tv->tv_usec>1000000)
	{
		tv->tv_sec++;
		tv->tv_usec -=1000000;
	}
	return 0;
}

//
// Get the index of interface with the same address that add
//
int ManetRoutingBase::getWlanInterfaceIndexByAddress (Uint128 add)  
{
	if (add==0) 
		return interfaceVector[0].index;

	for (unsigned int i=0;i<interfaceVector.size();i++)
	{
		if (mac_layer_)
		{
			if (interfaceVector[i].interfacePtr->getMacAddress() ==  add.getMACAddress())
				return interfaceVector[i].index;
		}
		else
		{
			if (interfaceVector[i].interfacePtr->ipv4Data()->getIPAddress() ==  add.getIPAddress())
				return interfaceVector[i].index;
		}
	}
	return -1;
}

//
// Get the interface with the same address that add
//
InterfaceEntry * ManetRoutingBase::getInterfaceWlanByAddress(Uint128 add) const
{
	if (add==0) 
		return interfaceVector[0].interfacePtr;
	
	for (unsigned int i=0;i<interfaceVector.size();i++)
	{
		if (mac_layer_)
		{
			if (interfaceVector[i].interfacePtr->getMacAddress() ==  add.getMACAddress())
				return interfaceVector[i].interfacePtr;
		}
		else
		{
			if (interfaceVector[i].interfacePtr->ipv4Data()->getIPAddress()==add.getIPAddress ())
				return interfaceVector[i].interfacePtr;
		}
	}
	return NULL;
}

//
// Get the index used in the general interface table
//
int ManetRoutingBase::getWlanInterfaceIndex (int i) const {
	if (i >= 0 && i <  (int) interfaceVector.size()) 
		return interfaceVector[i].index;
	else
		return -1;
}

//
// Get the i-esime wlan interface 
//
InterfaceEntry * ManetRoutingBase::getWlanInterfaceEntry (int i) const 
{
	if (i >= 0 && i < (int)interfaceVector.size()) 
		return interfaceVector[i].interfacePtr;
	else
		return NULL;
}


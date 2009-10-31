/*
 * Copyright (c)  2008,2009, Cairo University, Egypt.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgment:
 *	This product includes software developed by the Faculty of computers and information at Cairo University.
 * 4. Neither the name of the University nor of the faculty may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#ifndef __ns_amp_PAL802_11_h__
#define __ns_amp_PAL802_11_h__

#include "queue.h"
#include "mac-802_11.h"
//#include "lmp-link.h"
//#include "rendpoint.h"
//#include "bt-channel.h"



class L2CAPChannel;
class ConnectionHandle;
class PAL802_11;//class LMP;
//class LMPLink;

class PAL802_11:public BiConnector {
    friend class BTNode;

public:
    uchar PAL_Version_=0x01;//TODO:get from Bluetooth assigned numbers
	u_int16_t PAL_Sub_version_=0x0001;//vendor specified
	u_int32_t Total_Bandwidth_=30000;//for now assign 30Mbps //Bandwidth unit is Kbps (4octets)
	u_int32_t Max_Guaranteed_Bandwidth_; //= Total_Bandwidth	- sum of all active connections
	u_int32_t Min_Latencay_=50;//TODO: get value from vol5 part A p17 //for now assign 50microsecond //latency unit is microseconds(4octets)
	uchar Max_PDU_Size_=2312;// bytes
	uchar Controller_Type_=0x01;//802.11 AMP
	u_int16_t palCapabilities_=0x0000;//Guaranteed service type is not supported by this PAL(2octets)
	u_int16_t AMP_ASSOC_Length_=0x0000;//for now set to 0 //max size in octets of the requested AMP Assoc Structure(2octets)
	u_int32_t Max_Flush_Timeout_=0xFFFFFFFF;//Max time period in microseconds AMP device attempt to transmit a frame on a guaranteed link (currently set to infinity)
	u_int32_t Best_Effort_Flush_Timeout_=0xFFFFFFFF;//Max time period in microseconds AMP device attempt to transmit a frame on a best effort link (currently set to infinity)
	uchar Link_Quality_=0xFF;// Range 0x00<N<0xFF where 0x00 is link not available
	int8_t RSSI_=0xAA;//arriving signal strength in dBm (0x81 or -127dBm is not available) best case -42dBm or 0xAA
	uchar Short_Range_Mode_=0x00;//disabled change to 0x01 to enable

	////////////////////////////////////
    //          Constants	          //
    ////////////////////////////////////
	int ConnectionAcceptTimeOut=5;//in seconds
	int Max80211PALPDUSize=1492;//in octets
	int Max80211AMPASSOCLen=672;//in Octets
	int MinGUserPrio=4;//min priority in the guaranteed link
	int MaxGUserPrio=7;//max priority in the guaranteed link
	int BEUserPrio0=0;//priority in the best effort link
	int BEUserPrio1=3;//priority in the best effort link
	int Max80211BeaconPeriod=2000;//in millisecond
	int Max80211AMPASSOCLen=672;//in Octets
	int ShortRangeModePowerMax_=4;//in dBm

	////////////////////////////////////
    //          Data Structures       //
    ////////////////////////////////////
	//AMP_ASSOC
	///////////////////////////////////

	//+-------------------------------------------------+
	//	TypeID	|		Length		|		Value		|
	//+-------------------------------------------------+
	//	1 Octet	|		2 Octets	|		Variable	|
	//+-------------------------------------------------+
	struct AMP_ASSOC{
		enum TypeID {
			MAC_Address=0x01,//length = 0x0006
			Prefered_Channel_List=0x02,//length = N
			Connected_Channel=0x03,//length = N
			PAL_Capabilities_list=0x04,//length = 0x0004
			PAL_Version=0x05//length = 0x0005
		};
		TypeID typeID_;
		u_int16_t length_;
		uchar* value_;

		AMP_ASSOC(TypeID typeID,u_int16_t length,uchar* value):typeID_(typeID),length_(length),value_(value) {}

	};



	////////////////////////////////////
	//          HCI Interface         //
	////////////////////////////////////
	//HCI Events
	///////////////////////////////////
	struct PALEvent:public Event {
	    enum EventType {
	    	Channel_Selected,
	    	Short_Range_Mode_Change_Completed,
	    	Create_Physical_Link,
	    	Accept_Physical_Link,
	    	Connection_Accept_Timeout,
	    	MAC_Start_Completed,
	    	MAC_Start_Failed,
	    	MAC_Connect_Completed,
	    	MAC_Connect_Failed,
	    	MAC_Media_Disconnection_Indeication,
	    	HCI_Disconnect_Physical_Link_Request,
	    	Handshake_Fails,
	    	Handshake_Succeeds,
	    	HCI_Command_Status,
	    	HCI_Command_Complete,
	    	HCI_Physical_Link_Complete,
	    	HCI_Physical_Link_Disconnect_Complete,
	    	HCI_Physical_link_Loss_Early_Warning,
	    	HCI_Physical_Link_Recovery,
	    	HCI_Logical_Link_Complete,
	    	HCI_Disconnection_Logical_Link_Complete,
	    	HCI_Flow_Spec_Modify_complete
	    };
	    EventType eventType;
	    AMP_ASSOC amp_assoc_;
	    uchar Short_Range_Mode_;

	    PALEvent(AMP_ASSOC amp_assoc):eventType(Channel_Selected),amp_assoc_(amp_assoc) {}
	    PALEvent(uchar Short_Range_Mode):eventType(Short_Range_Mode_Change_Completed),Short_Range_Mode_(Short_Range_Mode) {}

	};

	enum PhysicalLinkStates{
		Disconnected,
		Starting,
		Connecting,
		Disconnecting,
		Authenticating,
		Connected
	};

	enum LogicalLinkStatus{
		Command_Disallowed=0x0C,
		Success=0x00
	};

  protected:
	//send packet to the L2CAP
    void sendUp(Packet *, Handler *);

  public:

    Mac802_11 * mac_;
    L2CAP *l2cap_;
    BTNode *node_;
    Bd_info *_my_info;
    Bd_info *_bd;		// bt device database

    void checkLink();
    PAL802_11();
    void on();
    void reset();
    void _init();
    ////////////////////////////////////
    //          HCI Interface         //
    ////////////////////////////////////
	//HCI Commands
	///////////////////////////////////
	void HCI_Reset();
	uchar* HCI_Read_Local_Version_Info();
	uchar* HCI_Read_Local_AMP_Info();
	uchar* HCI_Read_Local_AMP_Assoc();
	int HCI_Read_Failed_Contact_Counter(/*logical link*/);
	uchar HCI_Read_Link_Quality();
	int HCI_Read_RSSI();
	uchar HCI_Read_Best_Effort_Flush_Timeout();
	uchar* HCI_Write_Remote_AMP_Assoc();
	void HCI_Write_Best_Effort_Flush_Timeout(uchar);
	void HCI_Flow_Spec_modify();
	void HCI_Physical_link_Loss_Early_Warning();
	void HCI_Physical_Link_Recovery();
	void HCI_Short_Range_mode();
	void HCI_Create_Physical_Link();
	void HCI_Accept_Physical_Link();
	void HCI_Channel_Select();
	void HCI_Disconnect_Physical_Link();
	void HCI_Create_Logical_link();
	void HCI_Accept_Logical_link();
};


#endif				// __ns_amp_wifi-palp_h__

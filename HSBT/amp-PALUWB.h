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

#ifndef __ns_amp_PALUWB_h__
#define __ns_amp_PALUWB_h__


#include "amp-PAL.h"
#include "packet.h"
#include "mac-ifcontrol.h"


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
	struct ASSOCUWB{
		enum TypeID {
			MAC_Address=0x01,//length = 0x0006
			Prefered_Channel_List=0x02,//length = N
			Connected_Channel=0x03,//length = N
			PAL_Capabilities_list=0x04,//length = 0x0004
			PAL_Version=0x05//length = 0x0005
		};
		TypeID typeID_;
		u_int16_t length_;
		u_int8_t* value_;

		ASSOCUWB(TypeID typeID,u_int16_t length,u_int8_t* value):typeID_(typeID),length_(length),value_(value) {}
		ASSOCUWB(){}
	};

	////////////////////////////////////
	//          HCI Interface         //
	////////////////////////////////////
	//HCI Events
	///////////////////////////////////
	struct PALUWBEvent:public Event {
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
	    	MAC_Connection_Cancel_indecation,
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
	    ASSOCUWB amp_assoc_;
	    uint8_t short_Range_Mode_;

	    PALUWBEvent(ASSOCUWB amp_assoc):eventType(Channel_Selected),amp_assoc_(amp_assoc) {}
	    PALUWBEvent(uint8_t Short_Range_Mode):eventType(Short_Range_Mode_Change_Completed),short_Range_Mode_(Short_Range_Mode) {}

	};





class L2CAPChannel;
class ConnectionHandle;
class AMPConnection;

class PALUWB:public PAL {
    friend class BTNode;
    friend class  A2MP;
public:
  u_int32_t Max_Guaranteed_Bandwidth_; //= Total_Bandwidth	- sum of all active connections
  u_int32_t Min_Latencay_;//value = mac DIFS + CWmin //for now assign 50microsecond //latency unit is microseconds(4octets)
  u_int32_t Max_PDU_Size_;// in octets largest allowed L2CAP PDU size//2312//
  u_int16_t AMP_ASSOC_Length_;//max size in octets of the requested AMP Assoc Structure(2octets)
  NsObject* ifq_;

	enum LogicalLinkStatus{
		Command_Disallowed=0x0C,
		Success=0x00
	};

	static const u_int8_t PAL_Version_=0x01;//TODO:get from Bluetooth assigned numbers
	static const u_int16_t PAL_Company_Identifier_ = 0x0001;
	static const u_int16_t PAL_Sub_version_ =0x0001;//vendor specified
	static const u_int32_t Total_Bandwidth_ =1320000000;//for now assign 1320Mbps //Bandwidth unit is Kbps (4octets)
	static const u_int8_t Controller_ID_ =0x02;//UWB AMP
	static const u_int8_t Controller_Type_ =0x02;//UWB AMP
	static const u_int16_t palCapabilities_ =0x0000;//Guaranteed service type is not supported by this PAL(2octets)
	static const u_int32_t Max_Flush_Timeout_ =0xFFFFFFFF;//Max time period in microseconds AMP device attempt to transmit a frame on a guaranteed link (currently set to infinity)
	static const u_int32_t Best_Effort_Flush_Timeout_ =0xFFFFFFFF;//Max time period in microseconds AMP device attempt to transmit a frame on a best effort link (currently set to infinity)
	static const u_int8_t Link_Quality_ =0x00;// Range 0x00<N<0xFF where 0x00 is link quality indicator is  not available
	static const u_int8_t RSSI_ =0x81;//arriving signal strength in dBm (0x81 or -127dBm is indicator not available) best case -42dBm or 0xAA
	static const u_int8_t Short_Range_Mode_ =0x00;//disabled change to 0x01 to enable

	////////////////////////////////////
	//          Constants	          //
	////////////////////////////////////
	static const u_int8_t ConnectionAcceptTimeOut =5;//in seconds
	static const u_int32_t Max80211PALPDUSize =1492;//in octets
	static const u_int8_t MinGUserPrio =4;//min priority in the guaranteed link
	static const u_int8_t MaxGUserPrio =7;//max priority in the guaranteed link
	static const u_int8_t BEUserPrio0 =0;//priority in the best effort link
	static const u_int8_t BEUserPrio1 =3;//priority in the best effort link
	static const u_int32_t Max80211BeaconPeriod =2000;//in millisecond
	static const u_int32_t Max80211AMPASSOCLen =672;//in Octets
	static const u_int8_t ShortRangeModePowerMax_ =4;//in dBm

public:
    PALUWB();

	//send packet to the L2CAP
    void sendUp(Packet *, Handler *);
    void sendDown(AMPConnection*,Packet *);
    int command(int argc, const char*const* argv);

    void on();//fixme : see if they can be written once for all PALs
    void _init();
    Version_Info* HCI_Read_Local_Version_Info();
    AMP_Info* HCI_Read_Local_AMP_Info();
    u_int8_t* HCI_Read_Local_AMP_Assoc();
    void HCI_Write_Remote_AMP_Assoc(AMPConnection*,u_int8_t*);
    void HCI_Reset();
    int HCI_Read_Failed_Contact_Counter();
    u_int8_t HCI_Read_Link_Quality();

    u_int8_t HCI_Read_RSSI();

     void HCI_Short_Range_mode();
     void HCI_Write_Best_Effort_Flush_Timeout(u_int8_t);
     u_int8_t HCI_Read_Best_Effort_Flush_Timeout();

    //Events
     void Physical_link_Loss_Early_Warning();
     void Physical_Link_Recovery();
     void Channel_Selected();
     void Short_Range_Mode_Change_Completed() ;

    //Physical Link Manager functions
    //Implements operations on physical link includes physical link creation/acceptance/deletion plus channel selection
    //, security establishment and maintenance
     PhysLinkCompleteStatus HCI_Create_Physical_Link(AMPConnection*);
     PhysLinkCompleteStatus HCI_Accept_Physical_Link(AMPConnection*);
     void HCI_Disconnect_Physical_Link();

    //Actions
     void Determine_Selected_Channel() ;
     void Signal_MAC_Start_On_Channel(/*physical channel*/) ;
     //void MAC_Connect(AMPConnection*) ;
     void MAC_Initiate_Handshake(AMPConnection*) ;
     void Cancel_MAC_Connect_Operation(/*physical channel*/) ;
     void Signal_MAC_Start_To_Disconnect(/*physical channel*/) ;
    //Logical Link Manager functions
    //Implements operations on logical link includes logical link creation/deletion and applying QoS
     void HCI_Flow_Spec_modify();
     void HCI_Create_Logical_link(AMPConnection* c);
     void HCI_Accept_Logical_link();
     void HCI_Disconnect_Logical_link();
    //Data Manager functions
    //Perform operations on data packets includes : transmit/receive/buffer management
     void Encapsulate_Packet() ;
     void recv(Packet*, Handler*);
};


#endif				// __ns_amp_PALUWB_h__

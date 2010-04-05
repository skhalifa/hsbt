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

#include <string.h>

#include "amp-PAL802_11.h"

static class PAL802_11class:public TclClass {
  public:
	PAL802_11class():TclClass("PAL/802_11") {}
    TclObject *create(int, const char *const *) {
	return (new PAL802_11());
    }
} class_PAL802_11;

PAL802_11::PAL802_11(){
	printf("*** PAL 802.11 constructor.\n");
}
int PAL802_11::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
	if(argc == 2)
	{
		if(strcmp(argv[1],"_init")==0)
		{
			this->_init();
			return (TCL_OK);
		}
	}
	if (argc == 3) {
		if (strcmp(argv[1], "l2cap") == 0) {
			 l2cap_= (L2CAP*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "mac") == 0) {
			mac_ = (Mac*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "btnode") == 0) {
			btnode_ = (BTNode*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "a2mp") == 0) {
			a2mp_ = (A2MP*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "netif") == 0) {
			netif_ = (WirelessPhy*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
	}
}
void PAL802_11::on(){

}
void PAL802_11::_init(){


	Max_Guaranteed_Bandwidth_ = ((Mac802_11*)mac_)->dataRate_/1000;//Fixme: the guaranteed bandwidth should equals total bandwidth - used bandwidth
	Min_Latencay_ = ((Mac802_11*)mac_)->phymib_.getDIFS()+((Mac802_11*)mac_)->phymib_.getCWMin();
	Max_PDU_Size_ = Max80211PALPDUSize;
	AMP_ASSOC_Length_ = Max80211AMPASSOCLen;
	controllerID_ = Controller_ID_;
	controllerType_ = Controller_Type_;
	controllerStatus_ = RadioHasHighCapacityLeft;
	physicalLinkState_ = Disconnected;
	printf("PAL INIT BW = %i\n",Max_Guaranteed_Bandwidth_);
}

void PAL802_11::sendUp(Packet *p, Handler *h){

}

Version_Info* PAL802_11::HCI_Read_Local_Version_Info(){
	return new Version_Info(PAL_Version_,PAL_Company_Identifier_,PAL_Sub_version_);
}

AMP_Info* PAL802_11::HCI_Read_Local_AMP_Info()
{
	printf("Later BW = %d\n",Max_Guaranteed_Bandwidth_);
	return new AMP_Info(Total_Bandwidth_,Max_Guaranteed_Bandwidth_,Min_Latencay_,Max_PDU_Size_,Controller_Type_,palCapabilities_,AMP_ASSOC_Length_,Max_Flush_Timeout_,Best_Effort_Flush_Timeout_);
}
u_int8_t* PAL802_11::HCI_Read_Local_AMP_Assoc()
{
	ASSOC802_11** assoc = new ASSOC802_11*[5];
	printf("My MAC address is %i\n",(u_int8_t*)mac_->addr());
	assoc[0] = new ASSOC802_11(assoc[0]->MAC_Address,0x0006,(u_int8_t*)mac_->addr());
	assoc[1] = new ASSOC802_11(assoc[1]->PAL_Capabilities_list,0x0004,(u_int8_t*)0x00000000);
	//FixME : read the freq from the netif
	assoc[2] = new ASSOC802_11(assoc[2]->Prefered_Channel_List,0x0001,(u_int8_t*)0x0B);//always connect to channel 11 2.462GHZ
	//FixME : check for other connections
	assoc[3] = new ASSOC802_11(assoc[3]->Connected_Channel,0x0001,(u_int8_t*)0x00);//always send no connections
	assoc[4] = new ASSOC802_11(assoc[4]->PAL_Version,0x0005,(u_int8_t*)HCI_Read_Local_Version_Info());

	return (u_int8_t*)assoc;
}

void PAL802_11::HCI_Write_Remote_AMP_Assoc(u_int8_t* ampAssoc){
	this->remoteAMPAssoc_ = ampAssoc;
}
void PAL802_11::HCI_Reset(){
	//TODO: destroy all existing AMP Physical links
}
int PAL802_11::HCI_Read_Failed_Contact_Counter(){
	//fixme: should be per logical link
	return 0;//as the flush counter will never expire
}
u_int8_t PAL802_11::HCI_Read_Link_Quality(){
	//fixme:get link quality from MAC
	return Link_Quality_;//it return 0x00 ie link quality indicator is not available
}

u_int8_t PAL802_11::HCI_Read_RSSI(){
	//fixme:get link RSSI from MAC
	return RSSI_;//it return 0x81 ie  indicator is not available
}

 void PAL802_11::HCI_Short_Range_mode(){}
 void PAL802_11::HCI_Write_Best_Effort_Flush_Timeout(u_int8_t){}
 u_int8_t PAL802_11::HCI_Read_Best_Effort_Flush_Timeout(){ return NULL;}

//Events
 void PAL802_11::Physical_link_Loss_Early_Warning(){}
 void PAL802_11::Physical_Link_Recovery(){}
 void PAL802_11::Channel_Selected(){}
 void PAL802_11::Short_Range_Mode_Change_Completed() {}

//Physical Link Manager functions
//Implements operations on physical link includes physical link creation/acceptance/deletion plus channel selection
//, security establishment and maintenance
 PhysLinkCompleteStatus PAL802_11::HCI_Create_Physical_Link(u_int8_t* remote_amp_assoc){
	 /*
	  * 1) Determine the selected channel
	  * (if MAC not in selected channel)
		  * 2a) Request MAC to start on channel
	  * (if MAC is in selected channel)
		  * 2b) Send HCI Channel Select Event
	  * 3) set Connection accept timeout
	  * 4) set NeedPhysLinkCompleteEvent
	  * 5) set PhysLinkCompleteStatus=0x00 (no error)
	  */

	 /*
	  * If no suitable channel
	  * 1) Determine the selected channel
	  * 2) send HCI Physical link complete event with status set to "connection rejected due to limited resources 0x0D"
	  */

	 //ToDo : change the netif freq to the freq in the assoc resp
		/*
		 * ASSOC802_11** assoc = (ASSOC802_11**) remote_amp_assoc;
		printf("Dest Mac address : %i\n",assoc[0]->value_);
		printf("Dest PAL Cap : %i\n",assoc[1]->value_);
		printf("Dest pref channel : %i\n",assoc[2]->value_);
		printf("Dest connected channel : %i\n",assoc[3]->value_);
		printf("Dest PAL version : %i\n",((Version_Info*)assoc[4]->value_)->PAL_Version);
		*/
	 physicalLinkState_ = Starting;
	 if(!netif_->Is_node_on())
	 {
		 netif_->node_on();
	 }
	 if(netif_->Is_node_on())
	 {
		 physicalLinkState_ = Connecting;
		 return NoError;
	 }
	 else
	 {
		 physicalLinkState_ = Disconnected;
		 return LimitedResources;
	 }

 }
 PhysLinkCompleteStatus PAL802_11::HCI_Accept_Physical_Link(u_int8_t* remote_amp_assoc){
	 physicalLinkState_ = Starting;
	 if(!netif_->Is_node_on())
	 {
		 netif_->node_on();
	 }
	 if(netif_->Is_node_on())
	 {
		 physicalLinkState_ = Connecting;
		 return NoError;
	 }
	 else
	 {
		 physicalLinkState_ = Disconnected;
		 return LimitedResources;
	 }


 }
 void PAL802_11::HCI_Disconnect_Physical_Link(){}

//Actions
 void PAL802_11::Determine_Selected_Channel() {}
 void PAL802_11::Signal_MAC_Start_On_Channel(/*physical channel*/) {}
 void PAL802_11::MAC_Connect() {
	 //This function will make the MAC send a RTS message to the peering MAC
	 printf("Sending MAC Packet to %i\n",((ASSOC802_11**)this->remoteAMPAssoc_)[0]->value_);
	 u_int8_t* dap = ((ASSOC802_11**)this->remoteAMPAssoc_)[0]->value_;
		Packet *p = Packet::alloc(10);
		char *mh = (char*)p->access(hdr_mac::offset_);
		mac_->hdr_src(mh, mac_->addr());
		mac_->hdr_type(mh, ETHERTYPE_IP);
		mac_->hdr_dst((char*) HDR_MAC(p), (int)dap);

		hdr_pal *sh = HDR_PAL(p);
		sh->protocol_ = Link_Supervision_Request;
		sh->identifier_ = 1;
		sh->Length_ = 1;
		//send auth packet
		Scheduler& s = Scheduler::instance();
		// let mac decide when to take a new packet from the queue.
		s.schedule(((Mac802_11*)mac_), p, 0);


 }

 void PAL802_11::MAC_Initiate_Handshake(/*physical channel*/) {}
 void PAL802_11::Cancel_MAC_Connect_Operation(/*physical channel*/) {}
 void PAL802_11::Signal_MAC_Start_To_Disconnect(/*physical channel*/) {}
//Logical Link Manager functions
//Implements operations on logical link includes logical link creation/deletion and applying QoS
 void PAL802_11::HCI_Flow_Spec_modify(){}
 void PAL802_11::HCI_Create_Logical_link(){}
 void PAL802_11::HCI_Accept_Logical_link(){}
 void PAL802_11::HCI_Disconnect_Logical_link(){}
//Data Manager functions
//Perform operations on data packets includes : transmit/receive/buffer management
 void PAL802_11::Encapsulate_Packet() {
 }
//
//	 void PAL802_11::sendDown(Packet* p)
//	 {
//	 	hdr_cmn *ch = HDR_CMN(p);
//	 	hdr_ip *ih = HDR_IP(p);
//
//	 	nsaddr_t dst = (nsaddr_t)Address::instance().get_nodeaddr(ih->daddr());
//
//	 	hdr_ll *llh = HDR_LL(p);
//	 	char *mh = (char*)p->access(hdr_mac::offset_);
//
//	 	llh->seqno_ = ++seqno_;
//	 	llh->lltype() = LL_DATA;
//
//	 	mac_->hdr_src(mh, mac_->addr());
//	 	mac_->hdr_type(mh, ETHERTYPE_IP);
//	 	int tx = 0;
//
//	 	switch(ch->addr_type()) {
//
//	 	case NS_AF_ILINK:
//	 		mac_->hdr_dst((char*) HDR_MAC(p), ch->next_hop());
//	 		break;
//
//	 	case NS_AF_INET:
//	 		dst = ch->next_hop();
//	 		/* FALL THROUGH */
//
//	 	case NS_AF_NONE:
//
//	 		if (IP_BROADCAST == (u_int32_t) dst)
//	 		{
//	 			mac_->hdr_dst((char*) HDR_MAC(p), MAC_BROADCAST);
//	 			break;
//	 		}
//	 		/* Assuming arptable is present, send query */
//	 		if (arptable_) {
//	 			tx = arptable_->arpresolve(dst, p, this);
//	 			break;
//	 		}
//
//
//	 	default:
//
//	 		int IPnh = (lanrouter_) ? lanrouter_->next_hop(p) : -1;
//	 		if (IPnh < 0)
//	 			mac_->hdr_dst((char*) HDR_MAC(p),macDA_);
//	 		else if (varp_)
//	 			tx = varp_->arpresolve(IPnh, p);
//	 		else
//	 			mac_->hdr_dst((char*) HDR_MAC(p), IPnh);
//	 		break;
//	 	}
//
//	 	if (tx == 0) {
//	 		Scheduler& s = Scheduler::instance();
//	 	// let mac decide when to take a new packet from the queue.
//	 		s.schedule(downtarget_, p, delay_);
//	 	}
//	 }

 void PAL802_11::recv(Packet* p, Handler* callback = 0){
	 printf("\n\n\nPAL802.11 has just received a packet from MAC 802.11\n\n\n");
		hdr_pal *sh = HDR_PAL(p);

		char buf[1024];
		int len = 1024;
		uchar* req;

		char *mh = (char*)p->access(hdr_mac::offset_);
		printf("%d PAL::recv() from %d - %s packet with code %x\n", mac_->hdr_dst(mh,-2),mac_->hdr_src(mh,-2), sh->dump(buf, len), sh->protocol_);

		switch (sh->protocol_) {
		case L2CAP_DATA:

			break;
		case Link_Supervision_Request:{
			physicalLinkState_=Connected;
			//Send Link supervision reply
			 printf("Sending MAC Packet to %i\n",mac_->hdr_src(mh,-2));
				Packet *rp = Packet::alloc(10);
				char *rmh = (char*)rp->access(hdr_mac::offset_);
				mac_->hdr_src(rmh, mac_->hdr_dst(mh,-2));
				mac_->hdr_type(rmh, ETHERTYPE_IP);
				mac_->hdr_dst((char*) HDR_MAC(rp),mac_->hdr_src(mh,-2));

				hdr_pal *rsh = HDR_PAL(rp);
				rsh->protocol_ = Link_Supervision_Reply;
				rsh->identifier_ = 1;
				rsh->Length_ = 1;
				//send auth packet
				Scheduler& s = Scheduler::instance();
				// let mac decide when to take a new packet from the queue.
				s.schedule(((Mac802_11*)mac_), rp, 0);
		}
		break;
		case Link_Supervision_Reply:
			physicalLinkState_=Connected;
			//notify A2MP to create the logical link

			break;
		default:
				fprintf(stderr, "%d PAL received packed with invalid protocol %d\n",
						mac_->hdr_dst(mh,-2), sh->protocol_);
			}

			Packet::free(p);

	 // on recieving auth req packet send it up to the A2MP and send auth resp to the source
	 // on recieving auth resp packet send it up to the A2MP
	 //if()
	/* if(physicalLinkState_ == Connected)
	 		 l2cap_->recv(p,callback);
	 else if(physicalLinkState_ == Connecting)
		 physicalLinkState_=Connected;*/


 }

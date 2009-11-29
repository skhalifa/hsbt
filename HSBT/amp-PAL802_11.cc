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
	}
}
void PAL802_11::on(){

}
void PAL802_11::_init(){
	a2mp_->pal_[a2mp_->ampNumber_-1] = this;
	Max_Guaranteed_Bandwidth_ = ((Mac802_11*)mac_)->bandwidth();//Fixme: the guaranteed bandwidth should equals total bandwidth - used bandwidth
	Min_Latencay_ = ((Mac802_11*)mac_)->phymib_.getDIFS()+((Mac802_11*)mac_)->phymib_.getCWMin();
	Max_PDU_Size_ = Max80211PALPDUSize;
	AMP_ASSOC_Length_ = Max80211AMPASSOCLen;
	printf("*** PAL 802.11 _init.\n");
}

void PAL802_11::sendUp(Packet *p, Handler *h){

}

Version_Info* PAL802_11::HCI_Read_Local_Version_Info(){
	return new Version_Info(PAL_Version_,PAL_Sub_version_);
}

AMP_Info* PAL802_11::HCI_Read_Local_AMP_Info()
{
	return new AMP_Info(Total_Bandwidth_,Max_Guaranteed_Bandwidth_,Min_Latencay_,Max_PDU_Size_,Controller_Type_,palCapabilities_,AMP_ASSOC_Length_,Max_Flush_Timeout_,Best_Effort_Flush_Timeout_);
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
 void PAL802_11::HCI_Create_Physical_Link(){}
 void PAL802_11::HCI_Accept_Physical_Link(){}
 void PAL802_11::HCI_Disconnect_Physical_Link(){}

//Actions
 void PAL802_11::Determine_Selected_Channel() {}
 void PAL802_11::Signal_MAC_Start_On_Channel(/*physical channel*/) {}
 void PAL802_11::MAC_Connect(/*physical channel*/) {}
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
 void PAL802_11::Encapsulate_Packet() {}

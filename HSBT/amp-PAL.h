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

#ifndef __ns_amp_PAL_h__
#define __ns_amp_PAL_h__

//#include "queue.h"
//#include "mac-802_11.h"
//#include "lmp-link.h"
//#include "rendpoint.h"
//#include "bt-channel.h"
//#include "mac.h"


class PAL:public BiConnector {
    friend class BTNode;

  protected:
	//send packet to the L2CAP
    void sendUp(Packet *, Handler *);

  public:

    A2MP *a2mp_;
    Bd_info *_my_info;	//fixme : what to do with it??
    Bd_info *_bd;		// bt device database //fixme : what to do with it??
    bd_addr_t ad;

    virtual void setup(bd_addr_t ad,A2MP * a2mp) = 0 ;//fixme : see if they can be written once for all PALs
    virtual void on()  = 0;//fixme : see if they can be written once for all PALs
    virtual void _init() = 0;//fixme : see if they can be written once for all PALs
    ////////////////////////////////////
    //          HCI Interface         //
    ////////////////////////////////////
	//HCI Commands
	///////////////////////////////////

    //PAL Manager functions
    //implements global operations includes responding to host requests for AMP info and performing PAL reset
    virtual uchar* HCI_Read_Local_Version_Info()= 0;
    virtual uchar* HCI_Read_Local_AMP_Info()= 0;
    virtual void HCI_Reset()= 0;
    virtual int HCI_Read_Failed_Contact_Counter(/*logical link*/)= 0;
    virtual uchar HCI_Read_Link_Quality()= 0;
    virtual int HCI_Read_RSSI()= 0;
    virtual void HCI_Short_Range_mode()= 0;
    virtual void HCI_Write_Best_Effort_Flush_Timeout(uchar)= 0;
    virtual uchar HCI_Read_Best_Effort_Flush_Timeout()= 0;

    //Events
    virtual void Physical_link_Loss_Early_Warning()= 0;
    virtual void Physical_Link_Recovery()= 0;
    virtual void Channel_Selected()= 0;
    virtual void Short_Range_Mode_Change_Completed() =0;

    //Physical Link Manager functions
    //Implements operations on physical link includes physical link creation/acceptance/deletion plus channel selection
    //, security establishment and maintenance
    virtual void HCI_Create_Physical_Link()= 0;
    virtual void HCI_Accept_Physical_Link()= 0;
    virtual void HCI_Disconnect_Physical_Link()= 0;

    //Actions
    virtual void Determine_Selected_Channel() =0;
    virtual void Signal_MAC_Start_On_Channel(/*physical channel*/) =0;
    virtual void MAC_Connect(/*physical channel*/) =0;
    virtual void MAC_Initiate_Handshake(/*physical channel*/) =0;
    virtual void Cancel_MAC_Connect_Operation(/*physical channel*/) =0;
    virtual void Signal_MAC_Start_To_Disconnect(/*physical channel*/) =0;
    //Logical Link Manager functions
    //Implements operations on logical link includes logical link creation/deletion and applying QoS
    virtual void HCI_Flow_Spec_modify()= 0;
    virtual void HCI_Create_Logical_link()= 0;
    virtual void HCI_Accept_Logical_link()= 0;
    virtual void HCI_Disconnect_Logical_link()= 0;
    //Data Manager functions
    //Perform operations on data packets includes : transmit/receive/buffer management
    virtual void Encapsulate_Packet() =0;


    //virtual uchar* HCI_Read_Local_AMP_Assoc()= 0;
    //virtual uchar* HCI_Write_Remote_AMP_Assoc()= 0;

};


#endif				// __ns_amp_PAL_h__

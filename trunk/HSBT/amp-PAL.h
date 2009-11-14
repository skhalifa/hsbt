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
#include "mac.h"



class L2CAPChannel;
class ConnectionHandle;
class PAL;
//class LMP;
//class LMPLink;

class PAL:public BiConnector {
    friend class BTNode;

  protected:
	//send packet to the L2CAP
    void sendUp(Packet *, Handler *);

  public:


    Mac *mac_;
    L2CAP *l2cap_;
    A2MP *a2mp_;
    BTNode *btnode_;
    Bd_info *_my_info;
    Bd_info *_bd;		// bt device database
    bd_addr_t ad;

    PAL();
    virtual void setup(bd_addr_t ad, Mac * mac, L2CAP * l2cap,A2MP * a2mp, BTNode * btnode) = 0 ;
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


#endif				// __ns_amp_PAL_h__

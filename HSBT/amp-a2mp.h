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

#ifndef __ns_amp_a2mp_h__
#define __ns_amp_a2mp_h__

// Alternative Mac/Phy (AMP) Manager Protocol (A2MP)
// Reference : Bluetooth Specification V3.0+HS volume 3 part E
/*******************************************************************************
GENERAL DESCRIPTION
====================
The AMP Manager Protocol (A2MP) provides a means for one device to solicit
information regarding AMP capabilities from another device. Each device contains
an abstract entity called an AMP Manager which uses the AMP Manager
Protocol to communicate with a peer AMP Manager on another device using a fixed
L2CAP channel called A2MP channel.
*********************************************************************************/
/********************************************************************************
Packet Format
=============
	Octet 0		Octet 1			Octet 2	Octet 3
 -----------------------------------------------
|	Code	|	Identifier	|		Length		|
 -----------------------------------------------
|					Data			 			|
 -----------------------------------------------

**********************************************************************************/
#include "bi-connector.h"
#include "l2cap.h"
#include "agent.h"
#include "baseband.h"
#include "lmp.h"


//================
//  Codes
//================
#define A2MP_CommandReject					0x01
#define A2MP_DiscoverRequest				0x02
#define A2MP_DiscoverResponse				0x03
#define A2MP_ChangeNotify					0x04
#define A2MP_ChangeResponse					0x05
#define A2MP_GetInfoRequest					0x06
#define A2MP_GetInfoResponse				0x07
#define A2MP_Get_AMP_AssocRequest			0x08
#define A2MP_Get_AMP_AssocResponse			0x09
#define A2MP_CreatePhysicalLinkRequest		0x0A
#define A2MP_CreatePhysicalLinkResponse		0x0B
#define A2MP_DisconnectPhysicalLinkRequest	0x0C
#define A2MP_DisconnectPhysicalLinkResponse	0x0D


//==================
//Controller Types
//==================
#define BR_EDR	0x00
#define WIFI	0x01
#define UWB		0x02

//==================
//Controller Status
//==================
#define RadioAvailableButPoweredDown	0x00 // radio need to be powered up to use
#define RadioNotShared					0x01 //used when radio is powered up to indicate that the radio is only used by bluetooth
#define RadioHasNoCapacityLeft			0x02 //used when radio is powered up to indicate that the no bandwidth available to be used by bluetooth
#define RadioHasLowCapacityLeft			0x03 //used when radio is powered up to indicate that the from 0% to 30% of bandwidth is available to be used by bluetooth
#define RadioHasMediumCapacityLeft		0x04 //used when radio is powered up to indicate that the from 30% to 70% of bandwidth is available to be used by bluetooth
#define RadioHasHighCapacityLeft		0x05 //used when radio is powered up to indicate that the from 70% to 100% of bandwidth is available to be used by bluetooth
#define RadioHasFullCapacityLeft		0x06 //used when radio is powered up to indicate that the full bandwidth is available to be used by bluetooth

struct hdr_a2mp {
    uchar code_;		// defined above
    uint16_t identifier_;// valid values 0x01 - 0xFF
    uint16_t Length_;
    // length field indescates the size in octets of the data field of the packet only
    //	Total packet size  == Length + 1 + 1 + 2;
    double timeout_;

    static int offset_;
    inline static int &offset() { return offset_; }
    inline static hdr_a2mp *access(Packet * p) {
	return (hdr_a2mp *) p->access(offset_);
    }

    char *dump(char *buf, int len) {
	snprintf(buf, len, "code:%d identifier:%d length%d",
		 code_, identifier_, Length_);
	buf[len - 1] = '\0';
	return buf;
    }
};


class A2MP;

class A2MPTimer:public Handler {
  public:
	A2MPTimer(A2MP * a2mp):a2mp_(a2mp) {}
    void handle(Event *);

  private:
    A2MP * a2mp_;
};

class A2MPInqCallback:public Handler {
  public:
    A2MPInqCallback(A2MP * a2mp):a2mp_(a2mp) {}
    void handle(Event *);

  private:
    A2MP * a2mp_;
};

class A2MP:public Connector {
  public:
    class Connection {
      public:
	class A2MP * a2mp_;
	Connection *next_;
	L2CAPChannel *cid_;
	int daddr_;
	int ready_;
	PacketQueue q_;

	Connection(A2MP * a2mp, L2CAPChannel * c = 0)
	 : a2mp_(a2mp), next_(0), cid_(c), daddr_(-1), ready_(0), q_() {}

	void send() {
	    Packet *p;
	    // FIXME: check timeout of pkt first
	    while ((p = q_.deque())) {
		cid_->enque(p);
	    }
	}
    };

    A2MP();
    void setup(bd_addr_t ad, LMP * l, L2CAP * l2, BTNode * node);
    void sendToAll();
    void recv(Packet *, L2CAPChannel * ch);
    int command(int argc, const char *const *argv);

    Connection *connect(bd_addr_t addr);
    void channel_setup_complete(L2CAPChannel * ch);
    Connection *addConnection(L2CAPChannel * ch);
    void removeConnection(A2MP::Connection * c);
    Connection *lookupConnection(bd_addr_t addr);
    Connection *lookupConnection(L2CAPChannel * ch);

    void inq_complete();
    Packet *gen_a2mp_pkt(uchar *, int);
    void inquiry();
    void inqAndSend(Packet *);


    void A2MP_CommandRej(Packet *, L2CAPChannel *);
    void A2MP_DiscoverReq();
    void A2MP_DiscoverRsp(Packet *, L2CAPChannel *);
    void A2MP_ChangeNtfy(uchar *, int);
    void A2MP_ChangeRsp(Packet *, L2CAPChannel *);
    void A2MP_GetInfoReq(L2CAPChannel *,uchar *, int);
    void A2MP_GetInfoRsp(Packet *, L2CAPChannel *);
    void A2MP_Get_AMP_AssocReq(L2CAPChannel *,uchar *, int);
    void A2MP_Get_AMP_AssocRsp(Packet *, L2CAPChannel *);
    void A2MP_CreatePhysicalLinkReq(L2CAPChannel *,uchar *, int);
    void A2MP_CreatePhysicalLinkRsp(Packet *, L2CAPChannel *);
    void A2MP_DisconnectPhysicalLinkReq(uchar *, int);
    void A2MP_DisconnectPhysicalLinkRsp(Packet *, L2CAPChannel *);


    bd_addr_t bd_addr_;
    L2CAP *l2cap_;
    LMP *lmp_;
    BTNode *btnode_;

    int packetType_;
    A2MPTimer timer_;
    A2MPInqCallback inqCallback_;
    Event ev_;
    double lastInqT_;
    double inqTShred_;
    int inInquiry_;

    Connection *conn_;
    int numConn_;
    int connNumShred_;
    Bd_info *nbr_;
    int numNbr_;

    int identifier_;
    uchar controllerId_;
    PacketQueue q_;
};

#endif				// __ns_amp_a2mp_h__

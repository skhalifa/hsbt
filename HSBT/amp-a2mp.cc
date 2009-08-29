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

#include "amp-a2mp.h"
#include "baseband.h"


int hdr_a2mp::offset_;

static class A2MPHeaderClass:public PacketHeaderClass {
  public:
    A2MPHeaderClass():PacketHeaderClass("PacketHeader/A2MP", sizeof(hdr_a2mp)) {
	bind_offset(&hdr_a2mp::offset_);
    }
} class_a2mphdr;

static class A2MPclass:public TclClass {
  public:
    A2MPclass():TclClass("A2MP") {}
    TclObject *create(int, const char *const *) {
	return (new A2MP());
    }
} class_a2mp_agent;

void A2MPTimer::handle(Event * e)
{
}

void A2MPInqCallback::handle(Event *)
{
    a2mp_->inq_complete();
}

A2MP::A2MP()
: timer_(this), inqCallback_(this), lastInqT_(-9999),
inqTShred_(60), inInquiry_(0), conn_(0), numConn_(0), connNumShred_(1),
nbr_(0), numNbr_(0), identifier_(0), controllerId_(0x01), q_()
{
    //bind("randomizeSlotOffset_", &randomizeSlotOffset_);
    //bind("enable_clkdrfit_in_rp_", &enable_clkdrfit_in_rp_);
	}

void A2MP::inq_complete()
{
    if (nbr_) {
	lmp_->destroyNeighborList(nbr_);
    }
    nbr_ = lmp_->getNeighborList(&numNbr_);
    inInquiry_ = 0;
    // waitForInq_ = 0;
    // make_connections();

    if (numNbr_ <= 0) {
	printf("*** Ooops, no neighbors are found. Inquiry failed.\n");
	return;
    }
    Bd_info *wk = nbr_;
    for (int i = 0; i < numNbr_; i++) {
	connect(wk->bd_addr_);
	wk = wk->next_;
    }
    sendToAll();
}

A2MP::Connection * A2MP::lookupConnection(bd_addr_t addr)
{
    Connection *wk = conn_;
    while (wk) {
	if (wk->daddr_ == addr) {
	    return wk;
	}
	wk = wk->next_;
    }
    return NULL;
}

A2MP::Connection * A2MP::addConnection(L2CAPChannel * ch)
{
    Connection *c = new Connection(this, ch);
    c->next_ = conn_;
    conn_ = c;
    numConn_++;
    return c;
}

void A2MP::removeConnection(A2MP::Connection * c)
{
    if (c == conn_) {
	conn_ = conn_->next_;
	delete c;
	return;
    }

    Connection *par = conn_;
    Connection *wk = conn_->next_;
    while (wk) {
	if (wk == c) {
	    par->next_ = wk->next_;
	    delete c;
	    return;
	}
	par = wk;
	wk = wk->next_;
    }
}

void A2MP::channel_setup_complete(L2CAPChannel * ch)
{
    Connection *c = lookupConnection(ch->_bd_addr);
    if (!c) {
	c = addConnection(ch);
    }
    c->ready_ = 1;
    c->send();
}

A2MP::Connection * A2MP::connect(bd_addr_t addr)
{
	//todo: Still under testing
    Connection *c;
    if ((c = lookupConnection(addr))) {
	return c;
    }

    L2CAPChannel *ch = l2cap_->L2CA_ConnectReq(addr, PSM_A2MP);

    c = addConnection(ch);
    if (ch->ready_) {
	c->ready_ = 1;
    }

    return c;
}

void A2MP::setup(bd_addr_t ad, LMP * l, L2CAP * l2, BTNode * node)
{
    bd_addr_ = ad;
    l2cap_ = l2;
    lmp_ = l;
    btnode_ = node;
}

int A2MP::command(int argc, const char *const *argv)
{
    if (argc == 2) {
	if (!strcmp(argv[1], "change")) {
	    uchar req[128];
	    int len = 1;
	    A2MP_ChangeNtfy(req, len);
	    return (TCL_OK);
	}
	if (!strcmp(argv[1], "discover")) {
		    A2MP_DiscoverReq();
		    return (TCL_OK);
	}
	if (!strcmp(argv[1], "disconnect")) {
			assert(conn_);
			uchar localControllerId=0x01;
			uchar remoteControllerId=0x01;
		    uchar req[2]={localControllerId,remoteControllerId};
		    int len = 2;
		    A2MP_DisconnectPhysicalLinkReq(req,len);
		    return (TCL_OK);
		}

    }
    return Connector::command(argc, argv);
}

void A2MP::inquiry()
{
    printf("%d A2MP start inquiry().\n", bd_addr_);
    lmp_->HCI_Inquiry(lmp_->giac_, 4, 7);	// 4x1.28s
    inInquiry_ = 1;
    lmp_->addInqCallback(&inqCallback_);
}

void A2MP::inqAndSend(Packet * p)
{
    Scheduler & s = Scheduler::instance();
    double now = s.clock();

    q_.enque(p);
    if (lastInqT_ + inqTShred_ < now && !inInquiry_) {
	inquiry();		// start inquiry process
	return;
    }

    int i;
    if (numConn_ < connNumShred_) {
	Bd_info *wk = nbr_;
	for (i = 0; i < numNbr_ - 1; i++) {
	    connect(wk->bd_addr_);
	}
    }
    sendToAll();
}

void A2MP::sendToAll()
{
    assert(conn_);

    Connection *c;
    Packet *p;

    while ((p = q_.deque())) {
	c = conn_;
	for (int i = 0; i < numConn_ - 1; i++) {	// bcast
	    c->cid_->enque(p->copy());
	    c = c->next_;
	}
	c->cid_->enque(p);
    }
}

Packet *A2MP::gen_a2mp_pkt(uchar * req, int len)
{
    Packet *p = Packet::alloc(len);
    hdr_a2mp *sh = HDR_A2MP(p);
    hdr_cmn *ch = HDR_CMN(p);
    ch->ptype() = PT_A2MP;
    ch->size() = len + 4;//4 octets header
    sh->Length_= len;
    memcpy(p->accessdata(), req, len);

    return p;
}

void A2MP::recv(Packet * p, L2CAPChannel * ch)
{
    hdr_a2mp *sh = HDR_A2MP(p);

    char buf[1024];
    int len = 1024;
    uchar* req;

    printf("%d A2MP::recv() from %d - %s packet with code %x\n", bd_addr_, ch->remote(),
	   sh->dump(buf, len),sh->code_);

    switch (sh->code_) {

    case A2MP_CommandReject:
		printf("Got A2MP_CommandReject.\n");
		//TODO:Handle command reject
    	abort();//temp
		break;

    case A2MP_DiscoverRequest:
		printf("Got A2MP_DiscoverRequest.\n");
		A2MP_DiscoverRsp(p, ch);
		break;

    case A2MP_DiscoverResponse:
		printf("Got A2MP_DiscoverResponse.\n");
		//TODO: loop on the controller list retrieved and issue get info request for each but the first one (BR/EDR)
		//A2MP_GetInfoReq(p,ch,req,len);
		break;

    case A2MP_ChangeNotify:
		printf("Got A2MP_ChangeNotify.\n");
		//TODO:Handle Change Notify
		A2MP_ChangeRsp(p,ch);
		break;

    case A2MP_ChangeResponse:
		printf("Got A2MP_ChangeResponse.\n");
		break;

    case A2MP_GetInfoRequest:
	printf("Got A2MP_GetInfoRequest.\n");
		A2MP_GetInfoRsp(p, ch);
		break;

    case A2MP_GetInfoResponse:
		printf("Got A2MP_GetInfoResponse.\n");
		//TODO:choose the AMP/controllerID other than BR/EDR controllerID=0x00 and send assoc request
		//uchar controllerID[1]={0x01};
		req=new uchar[1];
		req[0]=0x01;
		A2MP_Get_AMP_AssocReq(ch,req,1);
		break;

    case A2MP_Get_AMP_AssocRequest:
		printf("Got A2MP_Get_AMP_AssocRequest.\n");
		A2MP_Get_AMP_AssocRsp(p, ch);
		break;

    case A2MP_Get_AMP_AssocResponse:
		printf("Got A2MP_Get_AMP_AssocResponse.\n");
		//TODO: get local controller ID
//		uchar localControllerId=0x01;
//		uchar remoteControllerId=p->accessdata()[0];
//		uchar* assocStructure = new uchar[p->datalen()-2];
//		if(p->datalen()-2>0)
//		{
//			assocStructure = p->accessdata()[2];
//		}
//		uchar req={localControllerId,remoteControllerId,assocStructure};
		req=new uchar[p->datalen()];
		req[0]=0x01;
		req[1]=p->accessdata()[0];
		for(int i=2;i<p->datalen();i++)
		{
			req[i]=p->accessdata()[i];
		}
		A2MP_CreatePhysicalLinkReq(ch,req,p->datalen());
		break;

    case A2MP_CreatePhysicalLinkRequest:
		printf("Got A2MP_CreatePhysicalLinkRequest.\n");
		//TODO: PAL open the specified AMP radio
		A2MP_CreatePhysicalLinkRsp(p, ch);
		break;

    case A2MP_CreatePhysicalLinkResponse:
		printf("Got A2MP_CreatePhysicalLinkResponse.\n");
		//TODO: PAL create AMP physical link
		break;

    case A2MP_DisconnectPhysicalLinkRequest:
		printf("Got A2MP_DisconnectPhysicalLinkRequest.\n");
		//TODO: PAL shutdown the specified AMP radio
		A2MP_DisconnectPhysicalLinkRsp(p, ch);
		break;

    case A2MP_DisconnectPhysicalLinkResponse:
		printf("Got A2MP_DisconnectPhysicalLinkResponse.\n");
		//TODO: PAL disconnect AMP physical link
		break;

    default:
		fprintf(stderr, "%d A2MP received packed with invalid Code %d\n",
			lmp_->bb_->bd_addr_, sh->code_);
		A2MP_CommandRej(p,ch);
    }

    Packet::free(p);
}


void A2MP::A2MP_CommandRej(Packet * p, L2CAPChannel * ch){
	//Sent as a response to receiving a packet with invalid code
    hdr_a2mp *sh = HDR_A2MP(p);

    int len = 2;//data field is divided to Reason (2 octets) and data (0 octets)
    uchar rep[] = {0x0000};//Reason="0x0000" as  data field value (Command not recognized)

    Packet *resp = gen_a2mp_pkt(rep, len);
    hdr_a2mp *sh_resp = HDR_A2MP(resp);
    sh_resp->code_ = A2MP_CommandReject;
    sh_resp->identifier_ = sh->identifier_;
    sh_resp->Length_=len;


    ch->enque(resp);
}
void A2MP::A2MP_DiscoverReq(){
	//Sent to peer AMP Manger to obtain the list os the available Controllers supported by that peer
	int len = 4;//data field is divided to MTU/MPS size (2 octets) and External Feature Mask (2 octets)
	uchar req[] = {0x029E,0x0000};//MTU_MPS >= 670 and EFM =0x0000
	Packet *p = gen_a2mp_pkt(req, len);
    hdr_a2mp *sh = HDR_A2MP(p);
    sh->code_ = A2MP_DiscoverRequest;
    sh->identifier_ = identifier_++;
    sh->Length_=len;
    printf("Attempt to establish A2MP channel over L2CAP.\n");
    inqAndSend(p);
}
void A2MP::A2MP_DiscoverRsp(Packet * p, L2CAPChannel * ch){
	//Shall indicate the IDs, types and Status of the controllers that are available on the local device
    hdr_a2mp *sh = HDR_A2MP(p);
    int len = 7;//data field is divided to MTU/MPS size (2 octets) , External Feature Mask (2 octets) and controller list (3 octets per control list entry)
    //Control list 0 (BR/EDR) controller ID = 0x00 , controller type = 0x00 and controller status = full capacity 0x06
    uchar controlList1[]={0x00,BR_EDR,RadioHasFullCapacityLeft};
    //TODO: request from the PAL to send the controller lists
    uchar rep[]={p->accessdata()[0],p->accessdata()[1],0x0000,controlList1[0],controlList1[1],controlList1[2]};

    Packet *resp = gen_a2mp_pkt(rep, len);
    hdr_a2mp *sh_resp = HDR_A2MP(resp);
    sh_resp->code_ = A2MP_DiscoverResponse;
    sh_resp->identifier_ = sh->identifier_;
    sh_resp->Length_=len;

    ch->enque(resp);
}
void A2MP::A2MP_ChangeNtfy(uchar * req, int len){
	//Sent if there is a change in the number or status of supported controllers
	//after the device has sent a Discovery Response and before the A2MP channel is disconnected
	Packet *p = gen_a2mp_pkt(req, len);
    hdr_a2mp *sh = HDR_A2MP(p);
    sh->code_ = A2MP_ChangeNotify;
    sh->identifier_ = identifier_++;
    sh->Length_=len;
    assert(conn_);
    conn_->cid_->enque(p);
    //inqAndSend(p);
}
void A2MP::A2MP_ChangeRsp(Packet * p, L2CAPChannel * ch){
	//Sent as an ACK to the change Notification
    hdr_a2mp *sh = HDR_A2MP(p);

    uchar rep[128];
    int len = 0;

    Packet *resp = gen_a2mp_pkt(rep, len);
    hdr_a2mp *sh_resp = HDR_A2MP(resp);
    sh_resp->code_ = A2MP_ChangeResponse;
    sh_resp->identifier_ = sh->identifier_;
    sh_resp->Length_=len;

    ch->enque(resp);
}
void A2MP::A2MP_GetInfoReq(L2CAPChannel * ch,uchar * req, int len){
	//Sent to request more info about a certain AMP controller for every controller in the discovery response or in the change notification
    Packet *p = gen_a2mp_pkt(req, len);
    hdr_a2mp *sh = HDR_A2MP(p);
    sh->code_ = A2MP_GetInfoRequest;
    sh->identifier_ = identifier_++;
    sh->Length_=len;
    ch->enque(p);
    //inqAndSend(p);
}
void A2MP::A2MP_GetInfoRsp(Packet * p, L2CAPChannel * ch){
	//Sent in response to the GET INFO REQUEST it holds controller info
    hdr_a2mp *sh = HDR_A2MP(p);
    int len = 18;
    //data field is divided to Controller ID (1 octet) ,Status (1 octet) , Total Bandwidth (4 octets) , Max Guaranteed Bandwidth (4 octets) , Min Latency (4 octets) ,  PAL Capabilities (2 octets) and AMP_Assoc Structure Size (2 octets)
    uchar controllerId = p->accessdata()[0];
    //TODO: request from the HCI to send the controller lists
    int controllerIdValid=1;
    uchar status;
    uchar* rep;
    if(controllerIdValid == 0 || controllerId == 0x00){
    	status=0x01; // controller ID is invalid or equals 0
    	len=2;
    	rep=new uchar[2];
    	rep[0]=controllerId;
    	rep[1]=status;
    }else{
    	status=0x00;
    	//TODO: query HCI for the controller info
    	uchar totalBandwidth[] = {0x000186A0};//for now assign 100Mbps //Bandwidth unit is Kbps (4octets)
    	uchar guaranteedBandwidth[] = {0x0000C350};//for now assign 50Mbps //Bandwidth unit is Kbps(4octets)
    	uchar minLatencay[] = {0x00000032};//for now assign 50microsecond //latency unit is microseconds(4octets)
    	//TODO: get PAL capabilities by issuing HCI Read Local AMP Info Vol2 part E section 7.5.8
    	uchar palCapabilities[]={0x0000};//for now set to 0(2octets)
    	uchar assocStructureSize[]={0x0000};//for now set to 0 //max size in octets of the requested AMP Assoc Structure(2octets)
    	//uchar temp[]={controllerId,status,totalBandwidth,guaranteedBandwidth,minLatencay,palCapabilities,assocStructureSize};
    	rep=new uchar[18];
    	rep[0]=controllerId;
    	rep[1]=status;
    	rep[2]=totalBandwidth[0];
    	rep[3]=totalBandwidth[1];
    	rep[4]=totalBandwidth[2];
    	rep[5]=totalBandwidth[3];
    	rep[6]=guaranteedBandwidth[0];
    	rep[7]=guaranteedBandwidth[1];
    	rep[8]=guaranteedBandwidth[2];
    	rep[9]=guaranteedBandwidth[3];
    	rep[10]=minLatencay[0];
    	rep[11]=minLatencay[1];
    	rep[12]=minLatencay[2];
    	rep[13]=minLatencay[3];
    	rep[14]=palCapabilities[0];
    	rep[15]=palCapabilities[1];
    	rep[16]=assocStructureSize[0];
    	rep[17]=assocStructureSize[1];
    }

    Packet *resp = gen_a2mp_pkt(rep, len);
    hdr_a2mp *sh_resp = HDR_A2MP(resp);
    sh_resp->code_ = A2MP_GetInfoResponse;
    sh_resp->identifier_ = sh->identifier_;
    sh_resp->Length_=len;

    ch->enque(resp);
}
void A2MP::A2MP_Get_AMP_AssocReq(L2CAPChannel * ch,uchar * req, int len){
    Packet *p = gen_a2mp_pkt(req, len);
    hdr_a2mp *sh = HDR_A2MP(p);
    sh->code_ = A2MP_Get_AMP_AssocRequest;
    sh->identifier_ = identifier_++;
    sh->Length_=len;

    ch->enque(p);
    //inqAndSend(p);
}
void A2MP::A2MP_Get_AMP_AssocRsp(Packet * p, L2CAPChannel * ch){
    //get AMP Assoc structure from the PAL and send it as a reply to the request
	hdr_a2mp *sh = HDR_A2MP(p);
	int len = 4;
    //data field is divided to Controller ID (1 octet) ,Status (1 octet) and AMP_Assoc Structure  (2 or more octets)
    uchar controllerId = p->accessdata()[0];
    //TODO: request from the HCI to send the controller lists
    int controllerIdValid=1;
    uchar status;
    uchar* rep;
    if(controllerIdValid == 0 || controllerId == 0x00){
    	status=0x01; // controller ID is invalid or equals 0
    	len=2;
    	rep=new uchar[2];
    	rep[0]=controllerId;
    	rep[1]=status;
    	//uchar temp[]={controllerId,status};
    	//rep=tmp;
    }else{
    	status=0x00;
    	//TODO: get PAL capabilities by issuing HCI Read Local AMP Info Vol2 part E section 7.5.8
    	uchar assocStructure[]={0x0000};//for now set to 0 // the requested AMP Assoc Structure(2 octets)
    	//uchar temp[]={controllerId,status,assocStructure};
    	//rep=tmp;
    	rep=new uchar[4];
    	rep[0]=controllerId;
    	rep[1]=status;
    	rep[2]=assocStructure[0];
    	rep[3]=assocStructure[1];
    }


    Packet *resp = gen_a2mp_pkt(rep, len);
    hdr_a2mp *sh_resp = HDR_A2MP(resp);
    sh_resp->code_ = A2MP_Get_AMP_AssocResponse;
    sh_resp->identifier_ = sh->identifier_;
    sh_resp->Length_=len;

    ch->enque(resp);
}
void A2MP::A2MP_CreatePhysicalLinkReq( L2CAPChannel * ch,uchar * req, int len){
	//TODO: communicate with the PAL to create the actual AMP physical link
    Packet *p = gen_a2mp_pkt(req, len);
    hdr_a2mp *sh = HDR_A2MP(p);
    sh->code_ = A2MP_CreatePhysicalLinkRequest;
    sh->identifier_ = identifier_++;
    sh->Length_=len;

    ch->enque(p);
    //inqAndSend(p);
}
void A2MP::A2MP_CreatePhysicalLinkRsp(Packet * p, L2CAPChannel * ch){
	hdr_a2mp *sh = HDR_A2MP(p);
	int len = 3;
    //data field is divided to local Controller ID (1 octet),remote Controller ID (1 octet) and Status (1 octet)
	uchar localcontrollerId = p->accessdata()[1];
	uchar remotecontrollerId = p->accessdata()[0];
    //TODO: request from the HCI to send the controller lists
	int controllerIdValid=1;
    uchar status;
    if(controllerIdValid == 0 || localcontrollerId == 0x00){
    	status=0x01; // controller ID is invalid or equals 0
    }
    else if(sh->code_==A2MP_DisconnectPhysicalLinkRequest)
    {
    	status=0x04;//Failed - AMP Disconnect Physical link request packet received
    }
    else{
    	//TODO:Add other failure codes
    	status=0x00;//IF code is valid the link is established so other status are ignored for now
    }
    uchar rep[]={localcontrollerId,remotecontrollerId,status};
    Packet *resp = gen_a2mp_pkt(rep, len);
    hdr_a2mp *sh_resp = HDR_A2MP(resp);
    sh_resp->code_ = A2MP_CreatePhysicalLinkResponse;
    sh_resp->identifier_ = sh->identifier_;
    sh_resp->Length_=len;

    ch->enque(resp);
}
void A2MP::A2MP_DisconnectPhysicalLinkReq(uchar * req, int len){
	//used to abort or cancel AMP link
    Packet *p = gen_a2mp_pkt(req, len);
    hdr_a2mp *sh = HDR_A2MP(p);
    sh->code_ = A2MP_DisconnectPhysicalLinkRequest;
    sh->identifier_ = identifier_++;
    assert(conn_);
    conn_->cid_->enque(p);
    //inqAndSend(p);
}
void A2MP::A2MP_DisconnectPhysicalLinkRsp(Packet * p, L2CAPChannel * ch){
	hdr_a2mp *sh = HDR_A2MP(p);
	int len = 3;
    //data field is divided to local Controller ID (1 octet),remote Controller ID (1 octet) and Status (1 octet)
	uchar localcontrollerId = p->accessdata()[1];
	uchar remotecontrollerId = p->accessdata()[0];
    //TODO: request from the HCI to send the controller lists
	int controllerIdValid=1;
    uchar status;
    if(controllerIdValid == 0 || localcontrollerId == 0x00){
    	status=0x01; // controller ID is invalid or equals 0
    }
    /*else if(physical link does not exist)
    {
    	status=0x02;//TODO: Failed - NO physical exists and no physical link creation is in progress
    }*/
    else{
    	status=0x00;//IF code is valid the link is disconnected so other status are ignored for now
    }
    uchar rep[]={localcontrollerId,remotecontrollerId,status};
    Packet *resp = gen_a2mp_pkt(rep, len);
    hdr_a2mp *sh_resp = HDR_A2MP(resp);
    sh_resp->code_ = A2MP_DisconnectPhysicalLinkResponse;
    sh_resp->identifier_ = sh->identifier_;
    sh_resp->Length_=len;

    ch->enque(resp);
}


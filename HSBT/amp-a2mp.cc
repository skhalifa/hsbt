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
/* Class Usage
 *
 * set a2mp [$node0 set a2mp_]
 * $a2mp discover $node1
 */

#include <string.h>

#include "amp-a2mp.h"
#include "baseband.h"

int hdr_a2mp::offset_;

static class A2MPHeaderClass: public PacketHeaderClass {
public:
	A2MPHeaderClass() :
		PacketHeaderClass("PacketHeader/A2MP", sizeof(hdr_a2mp)) {
		bind_offset(&hdr_a2mp::offset_);
	}
} class_a2mphdr;

static class A2MPclass: public TclClass {
public:
	A2MPclass() :
		TclClass("A2MP") {
	}
	TclObject *create(int, const char * const *) {
		return (new A2MP());
	}
} class_a2mp_agent;

void A2MPTimer::handle(Event * e) {
}

void A2MPInqCallback::handle(Event *) {
	a2mp_->inq_complete();
}

A2MP::A2MP() :
	timer_(this), inqCallback_(this), lastInqT_(-9999), inqTShred_(60),
			inInquiry_(0), conn_(0), numConn_(0), connNumShred_(1), nbr_(0),
			numNbr_(0), identifier_(0), controllerId_(0x01), q_(),
			ampNumber_(0){
	bind("ampNumber_", &ampNumber_);

}

void A2MP::inq_complete() {
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

A2MP::Connection * A2MP::lookupConnection(bd_addr_t addr) {
	printf("lookup connection\n");
	Connection *wk = conn_;
	while (wk) {
		printf("check with %i \n",wk->daddr_);
		if (wk->daddr_ == addr) {
			printf("connection found to address %i\n",addr);
			return wk;
		}
		wk = wk->next_;
	}
	printf("NO connection found\n");
	return NULL;
}

A2MP::Connection * A2MP::addConnection(L2CAPChannel * ch) {
	printf("A2MP::addConnection\n");
	Connection *c = new Connection(this, ch);
	c->next_ = conn_;
	conn_ = c;
	numConn_++;
	return c;
}

void A2MP::removeConnection(A2MP::Connection * c) {
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

void A2MP::channel_setup_complete(L2CAPChannel * ch) {
	Connection *c = lookupConnection(ch->_bd_addr);
	if (!c) {
		c = addConnection(ch);
	}
	c->ready_ = 1;
	c->send();
}

A2MP::Connection * A2MP::connect(bd_addr_t addr) {
	printf("A2MP::Connect\n");
	Connection *c;
	if ((c = lookupConnection(addr))) {
		printf("Connection found\n");
		return c;
	}

	L2CAPChannel *ch = l2cap_->L2CA_ConnectReq(addr, PSM_A2MP);
	printf("No Connection found add connection\n");
	c = addConnection(ch);
	printf("Connection added and ready = %i\n",ch->ready_);
	if (ch->ready_) {
		c->ready_ = 1;
	}

	return c;
}

void A2MP::setup(bd_addr_t ad, LMP * l, L2CAP * l2, BTNode * node) {
	bd_addr_ = ad;
	l2cap_ = l2;
	lmp_ = l;
	btnode_ = node;
	printf("*** A2MP setup with address %i.\n", bd_addr_);
}

int A2MP::command(int argc, const char * const *argv) {
	if (argc == 2) {
		if (!strcmp(argv[1], "change")) {
			uchar req[128];
			int len = 1;
			A2MP_ChangeNtfy(req, len);
			return (TCL_OK);
		}
		if (!strcmp(argv[1], "disconnect")) {
			assert(conn_);
			uchar localControllerId = 0x01;
			uchar remoteControllerId = 0x01;
			uchar req[2] = { localControllerId, remoteControllerId };
			int len = 2;
			A2MP_DisconnectPhysicalLinkReq(req, len);
			return (TCL_OK);
		}

	}
	if (argc == 3) {
		if (strcmp(argv[1], "add-pal") == 0) {
			pal_[ampNumber_] = (PAL*) TclObject::lookup(argv[2]);
			ampNumber_++;
			return (TCL_OK);
		}
		if (!strcmp(argv[1], "discover")) {
			if (ampNumber_ == 0)
				return (TCL_ERROR);
			BTNode *dest = (BTNode *) TclObject::lookup(argv[2]);
			A2MP_DiscoverReq(dest->bb_->bd_addr_);
			A2MP::Connection* c = lookupConnection(dest->bb_->bd_addr_);
			c->discoveryOnly_ = true;
			return (TCL_OK);
		}
	}
	return Connector::command(argc, argv);
}

void A2MP::findMatchingControllers(A2MP::Connection * c){
	printf("findMatchingControllers\n");
	u_int8_t *candidateAMP = new u_int8_t[MAX_AMP_NUMBER];
		int k = 0;
		printf("ampNumber_ %i\n",ampNumber_);
		for (int i = 0; i < ampNumber_; i++) {
			printf("c->dAMP_Count_ %i\n",c->dAMP_Count_);
			//The first AMP is skipped because it is the BR_EDR controller
			for (int j = 1; j < c->dAMP_Count_+1; j++) {
				printf("pal_[i]->controllerType_ = %i && c->dci_[j].controller_Type__= %i\n",pal_[i]->controllerType_,c->dci_[j].controller_Type__);
				if (pal_[i]->controllerType_ == c->dci_[j].controller_Type__) {
					printf("match found of type %i\n", c->dci_[j].controller_ID_);
					candidateAMP[k] = c->dci_[j].controller_ID_;
					k++;
				}
			}
		}
		if (k == 0){
			printf("NO match found\n");
		}
		else{
			c->infoReqSent_=k;
			for(int l=0;l<k;l++){
				printf("send request number %i to controller %i\n",l,candidateAMP[l]);
				A2MP_GetInfoReq(c->cid_,candidateAMP[l]);//Send info request to all matching PALs/Controllers
			}
		}

}
//FixMe : remove this function no longer used
void A2MP::inquiry() {
	printf("%d A2MP start inquiry().\n", bd_addr_);
	lmp_->HCI_Inquiry(lmp_->giac_, 4, 7); // 4x1.28s
	inInquiry_ = 1;
	lmp_->addInqCallback(&inqCallback_);
}
//FixMe : remove this function no longer used
void A2MP::inqAndSend(Packet * p) {
	Scheduler & s = Scheduler::instance();
	double now = s.clock();

	q_.enque(p);
	if (lastInqT_ + inqTShred_ < now && !inInquiry_) {
		inquiry(); // start inquiry process
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
//FixMe : remove this function no longer used
void A2MP::sendToAll() {
	assert(conn_);

	Connection *c;
	Packet *p;

	while ((p = q_.deque())) {
		c = conn_;
		for (int i = 0; i < numConn_ - 1; i++) { // bcast
			c->cid_->enque(p->copy());
			c = c->next_;
		}
		c->cid_->enque(p);
	}
}

Packet *A2MP::gen_a2mp_pkt(uchar * req, int len) {
	Packet *p = Packet::alloc(len);
	hdr_a2mp *sh = HDR_A2MP(p);
	hdr_cmn *ch = HDR_CMN(p);
	ch->ptype() = PT_A2MP;
	ch->size() = len + 4;//4 octets header
	sh->Length_ = len;
	printf("A2MP packet length is %i", len);
	memcpy(p->accessdata(), req, len);
	return p;
}

void A2MP::recv(Packet * p, L2CAPChannel * ch) {
	hdr_a2mp *sh = HDR_A2MP(p);

	char buf[1024];
	int len = 1024;
	uchar* req;

	printf("%d A2MP::recv() from %d - %s packet with code %x\n", bd_addr_,
			ch->remote(), sh->dump(buf, len), sh->code_);

	switch (sh->code_) {

	case A2MP_CommandReject:
		printf("Got A2MP_CommandReject.\n");
		break;

	case A2MP_DiscoverRequest:
		printf("Got A2MP_DiscoverRequest.\n");
		A2MP_DiscoverRsp(p, ch);
		break;

	case A2MP_DiscoverResponse: {
		printf("Got A2MP_DiscoverResponse.\n");
		//loop on the controller list retrieved and issue get info request for each but the first one (BR/EDR)

		Discovry_Rsp* rep = (Discovry_Rsp*) p->accessdata();
		Controller_Info *testcl = rep->controller_Info_;
		for (int i = 0; i < (int) rep->AMP_Count_ + 1; i++) {
			printf("%i : controller ID = %i , Controller Type = %i , Status = %i \n",
					i, testcl[i].controller_ID_, testcl[i].controller_Type__,
					testcl[i].controller_Status_);
		}
		printf("connection to address ch->_bd_addr %i",ch->_bd_addr);
		Connection *c = lookupConnection(ch->_bd_addr);
		printf("connection to address c->daddr_ %i",c->daddr_);
		c->dAMP_Count_ = rep->AMP_Count_;
		c->dci_ = rep->controller_Info_;
		if (!c->discoveryOnly_) {
			findMatchingControllers(c);
		}
	}
		break;

	case A2MP_ChangeNotify:
		printf("Got A2MP_ChangeNotify.\n");
		//TODO:Handle Change Notify
		A2MP_ChangeRsp(p, ch);
		break;

	case A2MP_ChangeResponse:
		printf("Got A2MP_ChangeResponse.\n");
		break;

	case A2MP_GetInfoRequest:
		printf("Got A2MP_GetInfoRequest.\n");
		A2MP_GetInfoRsp(p, ch);
		break;

	case A2MP_GetInfoResponse:{

		//Choose the AMP/controllerID based on the maxGaranteedBandwidth other than BR/EDR controllerID=0x00 and send assoc request
		Connection* c = lookupConnection(ch->_bd_addr);
		c->infoRspRecv_++;
		printf("Got A2MP_GetInfoResponse %i/%i. controller (%i) with maxBW %i\n",c->infoRspRecv_,c->infoReqSent_,((Info_Rsp* )p->accessdata())->controllerID_,((Info_Rsp* )p->accessdata())->maxGaranteedBandwidth_);
		if(c->dPAL_Info_== NULL)
			c->dPAL_Info_ = new Info_Rsp((Info_Rsp* )p->accessdata());
		else if(c->dPAL_Info_->maxGaranteedBandwidth_< ((Info_Rsp* )p->accessdata())->maxGaranteedBandwidth_)
			c->dPAL_Info_ = new Info_Rsp((Info_Rsp* )p->accessdata());
		printf("Candidate controller (%i) of bandwidth %i\n",c->dPAL_Info_->controllerID_,c->dPAL_Info_->maxGaranteedBandwidth_);
		if(c->infoReqSent_ == c->infoRspRecv_)
		{
			c->infoReqSent_=0;
			c->infoRspRecv_=0;

			for(int i=1;i<c->dAMP_Count_+1;i++)
			{

				if(c->dci_[i].controller_ID_ == c->dPAL_Info_->controllerID_)
				{
					for(int j=0;j<ampNumber_;j++)
					{
						if(pal_[j]->controllerType_ == c->dci_[i].controller_Type__){
							c->localPalID_=j;
							c->remotePalID_=c->dPAL_Info_->controllerID_;
						}
					}
					break;
				}
			}
			printf("Local pal number is %i\n",c->localPalID_);
			printf("Remote pal number is %i\n",c->remotePalID_);
			printf("c->dPAL_Info_->controllerID_ %i\n",c->dPAL_Info_->controllerID_);
			A2MP_Get_AMP_AssocReq(ch, c->dPAL_Info_->controllerID_);
		}

	}
		break;

	case A2MP_Get_AMP_AssocRequest:
		printf("Got A2MP_Get_AMP_AssocRequest.\n");
		A2MP_Get_AMP_AssocRsp(p, ch);
		break;

	case A2MP_Get_AMP_AssocResponse:{
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
		Connection* c = lookupConnection(ch->_bd_addr);
		AssocRsp* rsp = (AssocRsp* )p->accessdata();
		printf("local pal ID %i\n",c->localPalID_);
		printf("remote pal ID %i\n",c->remotePalID_);
		printf("remote pal from pal info ID %i\n",c->dPAL_Info_->controllerID_);
		printf("pal_[c->localPalID_]->controllerStatus_ %i\n",pal_[c->localPalID_]->controllerStatus_);
		PhysLinkCompleteStatus physLinkCompleteStatus = pal_[c->localPalID_]->HCI_Create_Physical_Link(rsp->amp_assoc_);
		if(physLinkCompleteStatus == NoError)
		{
			pal_[c->localPalID_]->HCI_Write_Remote_AMP_Assoc(rsp->amp_assoc_);
			printf("remote mac address is %i\n",((ASSOC802_11**)rsp->amp_assoc_)[0]->value_);
			A2MP_CreatePhysicalLinkReq(ch,c);
		}
		else
			printf("Unable to start MAC\n");
	}
		break;

	case A2MP_CreatePhysicalLinkRequest:
	{
		printf("Got A2MP_CreatePhysicalLinkRequest.\n");
		//PAL open the specified AMP radio
		Connection* c = connect(ch->_bd_addr);
		CreatePhyLinkReq* req = (CreatePhyLinkReq*) p->accessdata();
		c->localPalID_=req->remoteControllerID_;
		c->remotePalID_ = req->localControllerID_;
		printf("receiver : c->localPalID_= %i , c->remotePalID_ = %i \n",c->localPalID_,c->remotePalID_);

		CreatePhyLinkRspStatus status = InvalidControllerId;
		for (int i = 0; i < ampNumber_; i++) {
			printf("pal_[i]->controllerID_ %i and  c->localPalID_ %i\n",pal_[i]->controllerID_,c->localPalID_);
			if (pal_[i]->controllerID_ == c->localPalID_) {
				status = PhyCreateSuccess;//if a matching controller is found change status to success
				c->localPalID_ = i;
				break;
			}
		}

		if(status == PhyCreateSuccess){
			PhysLinkCompleteStatus physLinkCompleteStatus = pal_[c->localPalID_]->HCI_Accept_Physical_Link(req->amp_assoc_);
			if(physLinkCompleteStatus == NoError)
			{
				printf("success\n");
				pal_[c->localPalID_]->HCI_Write_Remote_AMP_Assoc(req->amp_assoc_);
			}
			else{
				printf("Unable to start MAC\n");
				status = FailedUnableToStartLinkCreation;
			}
		}

		A2MP_CreatePhysicalLinkRsp(p,ch,c,status);
	}
		break;

	case A2MP_CreatePhysicalLinkResponse:
	{
		printf("Got A2MP_CreatePhysicalLinkResponse.\n");
		//TODO: PAL create AMP physical link
		Connection* c = lookupConnection(ch->_bd_addr);
		pal_[c->localPalID_]->MAC_Connect();
	}
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
		A2MP_CommandRej(p, ch);
	}

	Packet::free(p);
}

void A2MP::A2MP_CommandRej(Packet * p, L2CAPChannel * ch) {
	//Sent as a response to receiving a packet with invalid code
	hdr_a2mp *sh = HDR_A2MP(p);

	int len = 2;//data field is divided to Reason (2 octets) and data (0 octets)
	uchar rep[] = { 0x0000 };//Reason="0x0000" as  data field value (Command not recognized)

	Packet *resp = gen_a2mp_pkt(rep, len);
	hdr_a2mp *sh_resp = HDR_A2MP(resp);
	sh_resp->code_ = A2MP_CommandReject;
	sh_resp->identifier_ = sh->identifier_;
	sh_resp->Length_ = len;

	ch->enque(resp);
}
void A2MP::A2MP_DiscoverReq(bd_addr_t dest_add) {
	//Sent to peer AMP Manger to obtain the list os the available Controllers supported by that peer
	//int len = 4;//data field is divided to MTU/MPS size (2 octets) and External Feature Mask (2 octets)
	//uchar req[] = {0x029E,0x0000};//MTU_MPS >= 670 and EFM =0x0000
	Discovry_Req* req = new Discovry_Req(0x029E, 0x0000);
	Packet *p = gen_a2mp_pkt((uchar*) req, sizeof(Discovry_Req));
	hdr_a2mp *sh = HDR_A2MP(p);
	sh->code_ = A2MP_DiscoverRequest;
	sh->identifier_ = identifier_++;
	sh->Length_ = sizeof(Discovry_Req);
	connect(dest_add);
	assert(conn_);
	conn_->cid_->enque(p);

	//    inqAndSend(p);

}

void A2MP::A2MP_DiscoverRsp(Packet * p, L2CAPChannel * ch) {
	//Shall indicate the IDs, types and Status of the controllers that are available on the local device
	hdr_a2mp *sh = HDR_A2MP(p);
	//int len = 7;//data field is divided to MTU/MPS size (2 octets) , External Feature Mask (2 octets) and controller list (3 octets per control list entry)
	//Control list 0 (BR/EDR) controller ID = 0x00 , controller type = 0x00 and controller status = full capacity 0x06

	Discovry_Req* req = (Discovry_Req*) p->accessdata();
	Controller_Info *ci = new Controller_Info[ampNumber_ + 1];
	ci[0].controller_ID_ = BR_EDR_ID;
	ci[0].controller_Type__ = BR_EDR;
	ci[0].controller_Status_ = RadioHasFullCapacityLeft;//Fixme : the controller status info
	for (int i = 0; i < ampNumber_; i++) {
		ci[i + 1].controller_ID_ = pal_[i]->controllerID_;
		ci[i + 1].controller_Type__ = pal_[i]->controllerType_;
		ci[i + 1].controller_Status_ = pal_[i]->controllerStatus_;//Fixme : the controller status info
	}
	//A hack is used here where the number or PALs is transmitted in the discovery response
	Discovry_Rsp* rep = new Discovry_Rsp(req->MTU_MPS_, req->Ext_Feature_Mask_,
			ci, ampNumber_);

	Packet *resp = gen_a2mp_pkt((uchar*) rep, sizeof(Discovry_Rsp));
	hdr_a2mp *sh_resp = HDR_A2MP(resp);
	sh_resp->code_ = A2MP_DiscoverResponse;
	sh_resp->identifier_ = sh->identifier_;
	sh_resp->Length_ = sizeof(Discovry_Rsp);

	ch->enque(resp);
}
void A2MP::A2MP_ChangeNtfy(uchar * req, int len) {
	//Sent if there is a change in the number or status of supported controllers
	//after the device has sent a Discovery Response and before the A2MP channel is disconnected
	Packet *p = gen_a2mp_pkt(req, len);
	hdr_a2mp *sh = HDR_A2MP(p);
	sh->code_ = A2MP_ChangeNotify;
	sh->identifier_ = identifier_++;
	sh->Length_ = len;
	assert(conn_);
	conn_->cid_->enque(p);
	//inqAndSend(p);
}
void A2MP::A2MP_ChangeRsp(Packet * p, L2CAPChannel * ch) {
	//Sent as an ACK to the change Notification
	hdr_a2mp *sh = HDR_A2MP(p);

	uchar rep[128];
	int len = 0;

	Packet *resp = gen_a2mp_pkt(rep, len);
	hdr_a2mp *sh_resp = HDR_A2MP(resp);
	sh_resp->code_ = A2MP_ChangeResponse;
	sh_resp->identifier_ = sh->identifier_;
	sh_resp->Length_ = len;

	ch->enque(resp);
}
void A2MP::A2MP_GetInfoReq(L2CAPChannel * ch, u_int8_t controllerID) {
	//Sent to request more info about a certain AMP controller for every controller in the discovery response or in the change notification
	printf("get info of controller %i\n",controllerID);
	uchar* req = &controllerID;
	Packet *p = gen_a2mp_pkt(req, 1);
	printf("packet formed");
	hdr_a2mp *sh = HDR_A2MP(p);
	sh->code_ = A2MP_GetInfoRequest;
	sh->identifier_ = identifier_++;
	sh->Length_ = 1;
	ch->enque(p);
	printf("A2MP_GetInfoReq sent on l2cap channel ");
	//inqAndSend(p);
}
void A2MP::A2MP_GetInfoRsp(Packet * p, L2CAPChannel * ch) {
	//Sent in response to the GET INFO REQUEST it holds controller info
	hdr_a2mp *sh = HDR_A2MP(p);
	//int len = 18;
	//data field is divided to Controller ID (1 octet) ,Status (1 octet) , Total Bandwidth (4 octets) , Max Guaranteed Bandwidth (4 octets) , Min Latency (4 octets) ,  PAL Capabilities (2 octets) and AMP_Assoc Structure Size (2 octets)
	u_int8_t controllerId = (u_int8_t) p->accessdata()[0];
printf("response controller ID is %i\n",controllerId);
	Info_Rsp* rsp = new Info_Rsp();
	rsp->controllerID_ = controllerId;
	rsp->status_ = 0x01;//by default the status is set to invalid controller ID

	for (int i = 0; i < ampNumber_; i++) {
		if (pal_[i]->controllerID_ == controllerId) {
			rsp->status_ = 0x00;//if a matching controller is found change status to success
			AMP_Info* ampInfo = pal_[i]->HCI_Read_Local_AMP_Info();
			rsp->totalBandwidth_ = ampInfo->Total_Bandwidth;
			rsp->maxGaranteedBandwidth_ = ampInfo->Max_Guaranteed_Bandwidth;
			rsp->minLatency_ = ampInfo->Min_Latency;
			rsp->PAL_Capabilities_ = ampInfo->PAL_Capabilities;
			rsp->AMP_AssocStructureSize_ = ampInfo->AMP_Assoc_Length;
			break;
		}

	}
	Packet *resp = gen_a2mp_pkt((uchar *) rsp, sizeof(Info_Rsp));
	hdr_a2mp *sh_resp = HDR_A2MP(resp);
	sh_resp->code_ = A2MP_GetInfoResponse;
	sh_resp->identifier_ = sh->identifier_;
	sh_resp->Length_ = sizeof(Info_Rsp);

	ch->enque(resp);
}
void A2MP::A2MP_Get_AMP_AssocReq(L2CAPChannel * ch, u_int8_t controllerID) {
	printf("A2MP::A2MP_Get_AMP_AssocReq to controller %i",controllerID);
	uchar* req = &controllerID;
	Packet *p = gen_a2mp_pkt(req, 1);
	hdr_a2mp *sh = HDR_A2MP(p);
	sh->code_ = A2MP_Get_AMP_AssocRequest;
	sh->identifier_ = identifier_++;
	sh->Length_ = 1;

	ch->enque(p);
	//inqAndSend(p);
}
void A2MP::A2MP_Get_AMP_AssocRsp(Packet * p, L2CAPChannel * ch) {
	//get AMP Assoc structure from the PAL and send it as a reply to the request
	hdr_a2mp *sh = HDR_A2MP(p);
	//int len = 4;
	//data field is divided to Controller ID (1 octet) ,Status (1 octet) and AMP_Assoc Structure  (2 or more octets)
	u_int8_t controllerId = (u_int8_t)p->accessdata()[0];
	printf("Asso rsp controller Id %i\n",controllerId);
	AssocRsp* rsp = new AssocRsp();
	rsp->controllerID_ = controllerId;
	rsp->status_ = 0x01;//by default the status is set to invalid controller ID
	for (int i = 0; i < ampNumber_; i++) {
		if (pal_[i]->controllerID_ == controllerId) {
			rsp->status_ = 0x00;//if a matching controller is found change status to success
			//Request from the Controller to send the Assoc struct
			rsp->amp_assoc_  = pal_[i]->HCI_Read_Local_AMP_Assoc();
			break;
		}
	}

	Packet *resp = gen_a2mp_pkt((uchar*)rsp, sizeof(AssocRsp));
	hdr_a2mp *sh_resp = HDR_A2MP(resp);
	sh_resp->code_ = A2MP_Get_AMP_AssocResponse;
	sh_resp->identifier_ = sh->identifier_;
	sh_resp->Length_ =  sizeof(AssocRsp);
	ch->enque(resp);
}
void A2MP::A2MP_CreatePhysicalLinkReq(L2CAPChannel * ch,Connection* c) {

	printf("A2MP::A2MP_CreatePhysicalLinkReq \n");
	CreatePhyLinkReq* req = new CreatePhyLinkReq(c->localPalID_,c->dPAL_Info_->controllerID_,pal_[c->localPalID_]->HCI_Read_Local_AMP_Assoc());
	printf("In createphyreq c->localPalID_ %i and remote c->dPAL_Info_->controllerID_ %i\n",c->localPalID_,c->dPAL_Info_->controllerID_);
	Packet *p = gen_a2mp_pkt((u_int8_t*)req, sizeof(CreatePhyLinkReq));
	hdr_a2mp *sh = HDR_A2MP(p);
	sh->code_ = A2MP_CreatePhysicalLinkRequest;
	sh->identifier_ = identifier_++;
	sh->Length_ = sizeof(CreatePhyLinkReq);

	ch->enque(p);
	//inqAndSend(p);
}
void A2MP::A2MP_CreatePhysicalLinkRsp(Packet * p,L2CAPChannel * ch ,Connection* c,CreatePhyLinkRspStatus status) {

	//data field is divided to local Controller ID (1 octet),remote Controller ID (1 octet) and Status (1 octet)
	hdr_a2mp *sh = HDR_A2MP(p);
	CreatePhyLinkRsp* rep = new CreatePhyLinkRsp(c->localPalID_,c->remotePalID_,status);
	Packet *resp = gen_a2mp_pkt((u_int8_t*)rep, sizeof(CreatePhyLinkRsp));
	hdr_a2mp *sh_resp = HDR_A2MP(resp);
	sh_resp->code_ = A2MP_CreatePhysicalLinkResponse;
	sh_resp->identifier_ = sh->identifier_;
	sh_resp->Length_ = sizeof(CreatePhyLinkRsp);

	ch->enque(resp);
}
void A2MP::A2MP_DisconnectPhysicalLinkReq(uchar * req, int len) {
	//used to abort or cancel AMP link
	Packet *p = gen_a2mp_pkt(req, len);
	hdr_a2mp *sh = HDR_A2MP(p);
	sh->code_ = A2MP_DisconnectPhysicalLinkRequest;
	sh->identifier_ = identifier_++;
	assert(conn_);
	conn_->cid_->enque(p);
	//inqAndSend(p);
}
void A2MP::A2MP_DisconnectPhysicalLinkRsp(Packet * p, L2CAPChannel * ch) {
	hdr_a2mp *sh = HDR_A2MP(p);
	int len = 3;
	//data field is divided to local Controller ID (1 octet),remote Controller ID (1 octet) and Status (1 octet)
	uchar localcontrollerId = p->accessdata()[1];
	uchar remotecontrollerId = p->accessdata()[0];
	//TODO: request from the HCI to send the controller lists
	int controllerIdValid = 1;
	uchar status;
	if (controllerIdValid == 0 || localcontrollerId == 0x00) {
		status = 0x01; // controller ID is invalid or equals 0
	}
	/*else if(physical link does not exist)
	 {
	 status=0x02;//TODO: Failed - NO physical exists and no physical link creation is in progress
	 }*/
	else {
		status = 0x00;//IF code is valid the link is disconnected so other status are ignored for now
	}
	uchar rep[] = { localcontrollerId, remotecontrollerId, status };
	Packet *resp = gen_a2mp_pkt(rep, len);
	hdr_a2mp *sh_resp = HDR_A2MP(resp);
	sh_resp->code_ = A2MP_DisconnectPhysicalLinkResponse;
	sh_resp->identifier_ = sh->identifier_;
	sh_resp->Length_ = len;

	ch->enque(resp);
}


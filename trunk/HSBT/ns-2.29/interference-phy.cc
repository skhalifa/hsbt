/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- 
 *
 * Copyright (c) 2004, Joerg Widmer, EPFL
 * Copyright (c) 2005, 2006 Ruben Merz, EPFL
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
 *    must display the following acknowledgement:
 *	This product includes software developed by the Laboratory for
 *      Computer Communications and Applications (LCA), EPFL, Switzerland
 * 4. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Header: /cvs/ns-2_26-uwb/mac/interference-phy.cc,v 1.27 2006/03/22 13:01:21 uid15352 Exp $
 *
 * partly based on wireless-phy
 */

#include <math.h>
#include <phy.h>
#include <propagation.h>
#include <modulation.h>
#include <wireless-phy.h>
#include <interference-phy.h>
#include <packet.h>
#include <assert.h>
#include <random.h>
#include "mac-ifcontrol.h"
#include "mac.h" // for MAC_BROADCAST (code)

#define INTERFERENCEPHY_DEBUG
#define PREAMBLE_SYNC_DEBUG

/* ======================================================================
   InterferencePhy TCL
   ====================================================================== */
static class InterferencePhyClass: public TclClass {
public:
        InterferencePhyClass() : TclClass("Phy/WirelessPhy/InterferencePhy") {}
        TclObject* create(int, const char*const*) {
                return (new InterferencePhy);
        }
} class_InterferencePhy;

/* ======================================================================
   Timers
   ====================================================================== */
void
PhyIdleTimer::expire(Event *e)
{
	phy_->idle_handler();
}

void
PhyRecvTimer::expire(Event *e)
{
	phy_->recv_handler();
}

void
PhySyncTimer::expire(Event *e)
{
	phy_->sync_handler();
}

/* ======================================================================
   InterferencePhy Interface
   ====================================================================== */
InterferencePhy::InterferencePhy() : WirelessPhy(), first_ifpkt(0), first_syncpkt(0), sync_length_(0), pktRx_(0),
				     sync_thresh_(-84),
				     tx_ths_(-1), rx_ths_(-1),
				     code_id_(MODULATION_NUM_CODES - 1), if_state(PHY_IDLE),
				     idle_timer(this), recv_timer(this), sync_timer(this)
{
	bind("frequency_range_", &frequency_range_);
	bind("noise_", &noise_);
	bind("use_timehopping_", &use_timehopping_);
	bind("preamble_time_", &preamble_time_);
	bind_bw("bitrate_", &bitrate_);


	sync_thresh_ = pow(10, sync_thresh_/10)/1000;

#ifdef PREAMBLE_SYNC_DEBUG
	printf(" (PHY) preamble_time_ = %.9f\n", preamble_time_);
	printf(" (PHY) sync_thresh_   = %.16f (%2.2f dBm)\n", sync_thresh_, 10*log10(sync_thresh_*1000));
#endif

}
 
int
InterferencePhy::command(int argc, const char*const* argv)
{
	TclObject *obj;

	if(argc == 3) {
		if (strcmp(argv[1], "modulation") == 0) {
			assert(modulation_ == 0);
			obj = TclObject::lookup(argv[2]);
			modulation_ = (Modulation*) obj;
			return TCL_OK;
		}
	}
	return WirelessPhy::command(argc, argv);
}


int InterferencePhy::is_idle() {
	int res = (if_state == PHY_IDLE);
	return res;
}

int InterferencePhy::is_recv() {
	int res = (if_state == PHY_RECV);
	return res;
}

int InterferencePhy::is_send() {
	int res = (if_state == PHY_SEND);
	return res;
}

int InterferencePhy::is_sync() {
	int res = (if_state == PHY_SYNC);
	return res;
}

int InterferencePhy::is_right_ths(Packet *p) {
	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);

	int res = !use_timehopping_ || (hdr_ifc->tx_ths_ == rx_ths_) || (hdr_ifc->tx_ths_ == own_ths_) || ((u_int32_t)hdr_ifc->tx_ths_ == MAC_BROADCAST);
	return res;
}

void
InterferencePhy::idle_handler()
{
	if_state = PHY_IDLE;
}

void
InterferencePhy::recv_handler()
{
	assert(pktRx_);
	//#ifdef INTERFERENCEPHY_DEBUG
	//printf("%0.9f %i (PHY) end of reception\n", Scheduler::instance().clock(), index_);
	//#endif
	if_state = PHY_IDLE;
	// hand over packets with errors -> should be dropped by MAC layer
	sendUp(pktRx_);
	uptarget_->recv(pktRx_, (Handler*) 0); // Send to the MAC

	/*
	if (sendUp(pktRx_) == 0) {
		Packet::free(pktRx_);
	} else {
		uptarget_->recv(pktRx_, (Handler*) 0);
	}
	*/
}

// rmz
void
InterferencePhy::sync_handler()
{

	if_state = PHY_RECV;

	/*
	 * 1. Get all possible packets from the sync list
	 *
	 * 2. (a) collision case: we drop all of the packets, but still
	 * add them to the interference list 3.
	 *
	 * 2. (b) select the one with the maximum power and send it to
	 * the MAC, the others are added to the interference list
	 *
	 */

	//int select = (int)round(0.5 + Random::uniform(sync_length_));
	int select = 1+ (Random::random() % sync_length_);

#ifdef PREAMBLE_SYNC_DEBUG
	double now = Scheduler::instance().clock();
	printf("%0.9f %i (PHY) Sync timer expired (packet selected = %d / %d)\n",
	       now, index_, select, sync_length_);
#endif

	startRecvPkt(select);
	sync_length_ = 0;
}

void
InterferencePhy::recv(Packet* p, Handler* h)
{
	struct hdr_cmn *hdr = HDR_CMN(p);
	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);
	double now = Scheduler::instance().clock();

	// For debugging purpose
	//struct hdr_mac *mh = HDR_MAC(p);
	//fprintf(stderr,"phy::recv: DA = %d, SA = %d, type = %x, direction = %d\n",mh->macDA_,mh->macSA_,mh->ftype_, hdr->direction());
	/*
	 * Handle outgoing packets
	 */
	switch (hdr->direction()) {
	case hdr_cmn::DOWN :
		/*
		 * The receiving PHY will handle EOT and take care of
		 * interference. if_state is set to PHY_SEND for the
		 * duration of the packet. The MAC should check before
		 * handing over packets. When busy, they'll simply be
		 * dropped!
		 */
		if (!is_idle()) {
			Packet::free(p);
#ifdef INTERFERENCEPHY_DEBUG
			printf("%0.9f %i (PHY) THS rx%i tx%i own%i pkt%i id %i / DROP BUSY (send) sync_length = %d\n",
			       now, index_, rx_ths_, tx_ths_, own_ths_, hdr_ifc->tx_ths_, hdr->uid(), sync_length_);
#endif
		} else {
			/* assume uncoded tx (no channel code) if txtime not set by upper layer */
			if (hdr->txtime() <= 0)
				hdr->txtime() = hdr->size() * 8.0 / bitrate_;
			// TH sequence
			if (use_timehopping_)
				hdr_ifc->tx_ths_ = tx_ths_;
			
			// channel code
			hdr_ifc->code_id_ = code_id_;

			// Add the preamble
			addPreamble(p);

#ifdef INTERFERENCEPHY_DEBUG
			printf("%0.9f %i (PHY) THS rx%i tx%i own%i pkt%i id %i / SEND (code %i, psize %i, txtime %0.9f)\n",
			       now, index_, rx_ths_, tx_ths_, own_ths_, hdr_ifc->tx_ths_, hdr->uid(),
			       hdr_ifc->code_id_, hdr->size(), hdr->txtime());
#endif
			if_state = PHY_SEND;
			idle_timer.resched(txtime(p));
			trans_end_ = now + txtime(p);

			sendDown(p);
		}
		return;
	case hdr_cmn::UP :
		// fall through, default direction is up
	default:
		// if the node is in sleeping mode or the energy goes to ZERO, drop the packet
		int em_fail_ = em() && (em()->sleep() || (em()->node_on() != true) || (em()->energy() <= 0));

		// calculate receive power
		assert(propagation_);
		PacketStamp s;
		s.stamp((MobileNode*)node(), ant_, 0, lambda_);
		double Pr = propagation_->Pr(&p->txinfo_, &s, this);

		// JCW
		//printf("%0.9f Pr %e Thresh %e %e\n", Scheduler::instance().clock(), Pr, CSThresh_, RXThresh_);

		assert(preamble_time_ < txtime(p));
		
		if (!is_send() && !is_recv() &&  !em_fail_ && (Pr > sync_thresh_)) {

			int right_ths = is_right_ths(p);

			if (right_ths) {
				// attempt sync with local packets only
				if_state = PHY_SYNC;
				/*
				 * We can receive the packet if we're idle, it's on
				 * the THS we're listening to (if THSs are used), and
				 * we have enough power (power above detect threshold
				 * currently disabled)
				 */
				// trans_end_ is set temporary until the end of the first receiving packet
				// syncronization could fail
				rx_start_ = now;
				//trans_end_ = rx_end_ = now + txtime(p);
				trans_end_ = rx_end_ = now + preamble_time_;

				sync_timer.resched(preamble_time_);
				/*
				if (sync_length_ == 0) {
					rx_start_ = now;
					trans_end_ = rx_end_ = now + txtime(p);
					//trans_end_ = rx_end_ = now + preamble_time_;

					sync_timer.resched(preamble_time_);
				}
				if (now + preamble_time_ > trans_end_) {
					trans_end_ = rx_end_ = now + preamble_time_;
				}
				*/
				insertSyncListEntry(p, now, now + txtime(p), Pr, right_ths);
				sync_length_++;

#ifdef PREAMBLE_SYNC_DEBUG
				printf("%0.9f %i (PHY) Sync timer scheduled at %0.9f (sync_length = %d / recv at %.9f)\n",
				       now, index_,now + preamble_time_, sync_length_, now+txtime(p));
				if (sync_length_ > 1) {
					printf("%0.9f %i (PHY) Preamble collision (sync_length = %d)\n",
					       now, index_, sync_length_);
#endif
				}
			} else {
				// Wrong THS drop and add as interference
#ifdef INTERFERENCEPHY_DEBUG
				printf("%0.9f %i (PHY) THS rx%i tx%i own%i pkt%i id %i",
				       now, index_, rx_ths_, tx_ths_, own_ths_, hdr_ifc->tx_ths_, hdr->uid());
				printf(" drop wrong THS\n");
#endif

				/* while receiving/transmitting/sleeping we just
				 * update the interference list */
				dropPkt(p, now, now+txtime(p), Pr);
			}

		} else {

#ifdef INTERFERENCEPHY_DEBUG
			printf("%0.9f %i (PHY) THS rx%i tx%i own%i pkt%i id %i ",
			       now, index_, rx_ths_, tx_ths_, own_ths_, hdr_ifc->tx_ths_, hdr->uid());
			if (Pr < sync_thresh_) {
				printf(" / DROP Pr < sync_thresh_ (%.16f / %.16f)\n", Pr, sync_thresh_);
			} else {
				if (!is_idle()) {
					printf(" / DROP busy (%s until %.9f)\n",
					       (if_state == PHY_SEND) ? "send" : "recv", trans_end_);
				} else {
					printf(" / DROP ENERGY\n");
				}
			}
#endif
			/* while receiving/transmitting/sleeping we just
			 * update the interference list */
			dropPkt(p, now, now+txtime(p), Pr);

		}
		break;
	}

}

int
InterferencePhy::sendUp(Packet *p)
{
	hdr_cmn *hdr = HDR_CMN(p);
	int pkt_recvd = 1;

	/*
	 * Sanity Check
	 */
	assert(initialized());

	if(modulation_) {
		// adjust interference by processing gain
		rx_interference_ = avgInterferencePower() / (frequency_range_ / bitrate_);

		hdr->errbitcnt() = modulation_->BitError(rx_power_, rx_interference_, noise_, p);
		hdr->error() = (hdr->errbitcnt() ? 1 : 0);
		if (hdr->error())
			pkt_recvd = 0; // Random experiments failed

	} else if (rx_power_ < RXThresh_ || rx_power_ / maxInterferencePower() < pow(10, CPThresh_ / 10.0)) {
		// This is the old 802.11 like code which has not been tested! Use at own risk.

		/*
		 * We can detect, but not successfully receive
		 * this packet.
		 */
		hdr->error() = 1;
		pkt_recvd = 0;
	}
	
	/*
	 * Decrease energy if packet successfully received
	 */
	if(pkt_recvd && em()) {
		double rcvtime = hdr_cmn::access(p)->txtime();

		double start_time = MAX(channel_idle_time_, NOW);
		double end_time = MAX(channel_idle_time_, NOW+rcvtime);
		double actual_rcvtime = end_time-start_time;

		if (start_time > update_energy_time_) {
			em()->DecrIdleEnergy(start_time-update_energy_time_,
					     P_idle_);
			update_energy_time_ = start_time;
		}
		
		em()->DecrRcvEnergy(actual_rcvtime,Pr_consume_);
		if (end_time > channel_idle_time_) {
			status_ = CHAN_RECV;
		}

		channel_idle_time_ = end_time;
		update_energy_time_ = end_time;

		/*
		  hdr_diff *dfh = HDR_DIFF(p);
		  printf("Node %d receives (%d, %d, %d) energy %lf.\n",
		  node()->address(), dfh->sender_id.addr_, 
		  dfh->sender_id.port_, dfh->pk_num, node()->energy());
		*/
		
		if (em()->energy() <= 0) {  
			// saying node died
			em()->setenergy(0);
			((MobileNode*)node())->log_energy(0);
		}
	}
	
	return pkt_recvd;
}


void
InterferencePhy::dropPkt(Packet *p, double start, double end, double Pr) {

	insertInterferenceListEntry(start, end, Pr);
	Packet::free(p);
}



double
InterferencePhy::avgInterferencePower()
{
	double avgPower = 0;
	InterferenceListEntry *i = first_ifpkt;

// #ifdef INTERFERENCEPHY_DEBUG
// 	double now = Scheduler::instance().clock();
// 	printf("%0.9f %i (PHY) avgInterferencePower:\n",
// 	       now, index_);
// #endif


	while(i) {
		if (i->used) {
			if(i->rxEndTime > rx_start_) {
				//The packet i  collides with the useful packet:
				//we evaluate the duration of the collision

				double interferingInterval = MIN(rx_end_, i->rxEndTime) - MAX(rx_start_, i->rxStartTime);
// #ifdef INTERFERENCEPHY_DEBUG
// 				printf("         %0.9f / MIN(%0.9f, %0.9f) - MAX(%0.9f, %0.9f)\n",
// 				       interferingInterval, rx_end_, i->rxEndTime, rx_start_, i->rxStartTime);
// #endif
				assert(!(interferingInterval < 0));
				//assert(interferingInterval > 0);
				
				//The two packets collide: we evaluate the contribution to the interference power
				double collisionTimeCoefficient = interferingInterval / (rx_end_ - rx_start_);
				avgPower += i->receivedPower * collisionTimeCoefficient;
			} else {
				//The packet i can not cause collisions anymore: we free the slot
				i->used = 0;

				/*
				  removeInterferenceListEntry(prev, i);
				  // i is removed and the pointer updated
				  if (prev)
				  i = prev->next;
				  else
				  i = first_ifpkt;
				*/
			}
		}
		i = i->next;
	}
	
	return avgPower;
}

double
InterferencePhy::maxInterferencePower()
{
	double totalInterferencePower = 0;
	InterferenceListEntry *i = first_ifpkt;

	while(i) {
		if (i->used) {
			if(i->rxEndTime > rx_start_) {
				totalInterferencePower = MAX(totalInterferencePower, i->receivedPower);
			} else {
				i->used = 0;
			}
		}
		i = i->next;
	} 

	return totalInterferencePower;
}

void
InterferencePhy::startRecvPkt(int select)
{
	SyncListEntry *i = first_syncpkt;
	double now = Scheduler::instance().clock();
	int k = 0;

	while(i) {
		if (i->used) {
			k++;
			if (k == select) {

				assert(i->right_ths);
				// Schedule packet for real reception
				// XXX check
				pktRx_ = i->p;
				rx_start_ = i->rxStartTime;
				trans_end_ = rx_end_ = i->rxEndTime;
				rx_power_ = i->Pr;
				/*
				 * schedule reception of full packet
				 */
				double delay = i->rxEndTime-now;
				if(delay < 0){
					delay =0.00000001;
				}
				recv_timer.resched(delay);

#ifdef INTERFERENCEPHY_DEBUG
				struct hdr_cmn *hdr = HDR_CMN(i->p);
				struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(i->p);
				printf("%0.9f %i (PHY) THS rx%i tx%i own%i pkt%i id %i / recv at %.9f, power RX %f dBm TX %f dBm\n",
				       now, index_, rx_ths_, tx_ths_, own_ths_, hdr_ifc->tx_ths_, hdr->uid(), i->rxEndTime,
				       10.0*log10(i->Pr*1000), 10.0*log10(i->p->txinfo_.getTxPr()*1000));
#endif

			} else {

#ifdef PREAMBLE_SYNC_DEBUG
				struct hdr_cmn *hdr = HDR_CMN(i->p);
				struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(i->p);
				printf("%0.9f %i (PHY) THS rx%i tx%i own%i pkt%i id %i",
				       now, index_, rx_ths_, tx_ths_, own_ths_, hdr_ifc->tx_ths_, hdr->uid());
				printf(" DROP end of sync\n");
#endif

				dropPkt(i->p, i->rxStartTime, i->rxEndTime, i->Pr);

			}
			i->used = 0;
		}
		i = i->next;
	} 

}


void
InterferencePhy::insertInterferenceListEntry(double start, double end, double power)
{
	InterferenceListEntry *prev = 0;
	InterferenceListEntry *i = first_ifpkt;

	// look for free slot
	while (i) {
		if (i->used) {
			prev = i;
			i = i->next;
		} else {
			break;
		}
	}
	if (i == 0) {
		i = new InterferenceListEntry;
		if (prev)
			prev->next = i;
		else
			first_ifpkt = i;
		i->next = 0;
	}
	i->receivedPower = power;
	i->rxStartTime = start;
	i->rxEndTime = end;
	i->used = 1;
}

void
InterferencePhy::insertSyncListEntry(Packet *p, double start, double end, double Pr, int right_ths)
{
	SyncListEntry *prev = 0;
	SyncListEntry *i = first_syncpkt;


	//printf("%12.11f insert\n", Scheduler::instance().clock());
	// look for free slot
	//#ifdef PREAMBLE_SYNC_DEBUG
	//double now = Scheduler::instance().clock();
	//printf("%0.9f %i (PHY) SyncList add\n",
	//       now, index_);
	//#endif

	while (i) {
		//printf("SyncList i = %d\n",i);
		if (i->used) {
			prev = i;
			i = i->next;
		} else {
			break;
		}
	}
	if (i == 0) {
		i = new SyncListEntry;
		//printf("New SyncList i = %d\n",i);
		if (prev)
			prev->next = i;
		else
			first_syncpkt = i;
		i->next = 0;
	}
	i->p = p;
	i->rxStartTime = start;
	i->rxEndTime = end;
	i->Pr = Pr;
	i->right_ths = right_ths;
	i->used = 1;


}



// void
// InterferencePhy::removeInterferenceListEntry(InterferenceListEntry *prev, InterferenceListEntry *current)
// {
// 	assert(current);
// 	// set last to previous entry (or to 0 if it was the only entry)
// 	if (current == last_ifpkt)
// 		last_ifpkt = prev;
// 	if (current == first_ifpkt) {
// 		first_ifpkt = current->next;
// 	} else {
// 		assert(prev);
// 		prev->next = current->next;	
// 	}
// 	delete current;		
// }

void
InterferencePhy::addPreamble(Packet *p)
{
	struct hdr_cmn *ch = HDR_CMN(p);

// #ifdef PREAMBLE_SYNC_DEBUG
// 	printf("%0.9f %i (PHY) Adding preamble (%0.9f -> ",
// 	       Scheduler::instance().clock(), index_, ch->txtime());
// #endif

	ch->txtime() = ch->txtime() + preamble_time_;

// #ifdef PREAMBLE_SYNC_DEBUG
// 	printf("%0.9f)\n", ch->txtime());
// #endif

}

double
InterferencePhy::txtime(Packet *p)
{
	struct hdr_cmn *ch = HDR_CMN(p);
	double t = ch->txtime();
	assert(t > 0.0);
	return t;
}

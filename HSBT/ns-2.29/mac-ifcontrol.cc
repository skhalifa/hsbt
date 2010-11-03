/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
 * Copyright (c) 2004, Joerg Widmer, EPFL
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
 * $Header: /cvs/ns-2_26-uwb/mac/mac-ifcontrol.cc,v 1.35 2006/07/03 08:38:34 uid15352 Exp $
 *
 * partly based on mac-802.11
 */

#include <assert.h>

#include "delay.h"
#include "connector.h"
#include "packet.h"
#include "random.h"
#include "mobilenode.h"

#include "arp.h"
#include "ll.h"
#include "mac.h"
#include "mac-ifcontrol.h"
#include "cmu-trace.h"
#include "modulation.h"

// HACK: just add one (since 1st index_ == 0) and multiply dst ID by 10000 to create private code.
#define PRIVATE_CODE(s, d) ((s+1) + (d+1) * 10000)

#define THS_DISABLED -3
#define NO_MAC_ADDR -4

#define PIGGYBACK_DATA

#define MACIF_DEBUG

// doesn't include RTS (since the node must have already received this)
#define MAX_TXTIME (txtime(phymib_.getCTSlen()) + txtime(phymib_.getACKlen()) \
		    + txtime(MAC_IF_MAX_PACKETSIZE) + 4 * DSSS_MaxPropagationDelay)



int hdr_mac_ifcontrol::offset_;

static class MACIFControlHeaderClass : public PacketHeaderClass {
public:
        MACIFControlHeaderClass() : PacketHeaderClass("PacketHeader/MacIFControl",
					     sizeof(hdr_mac_ifcontrol)) {
		bind_offset(&hdr_mac_ifcontrol::offset_);
	}
} class_hdr_mac_ifcontrol;


/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class Mac_IFControlClass : public TclClass {
public:
	Mac_IFControlClass() : TclClass("Mac/IFControl") {}
	TclObject* create(int, const char*const*) {
		return (new Mac_IFControl());
	}
} class_mac_IFControl;

/* ======================================================================
   Mac  and Phy MIB Class Functions
   ====================================================================== */

PHY_IF_MIB::PHY_IF_MIB(Mac_IFControl *parent)
{
	/*
	 * Bind the phy mib objects.  Note that these will be bound
	 * to Mac/802_11 variables
	 */

	parent->bind("CWMin_", &CWMin);
	parent->bind("CWMax_", &CWMax);
	parent->bind("SlotTime_", &SlotTime);
	parent->bind("SIFS_", &SIFSTime);
	parent->bind("PreambleLength_", &PreambleLength);
	parent->bind("PLCPHeaderLength_", &PLCPHeaderLength);
	parent->bind_bw("PLCPDataRate_", &PLCPDataRate);

printf("\n\n\n Slot time = %f\n\n\n",SlotTime);

}

MAC_IF_MIB::MAC_IF_MIB(Mac_IFControl *parent)
{
	/*
	 * Bind the phy mib objects.  Note that these will be bound
	 * to Mac/802_11 variables
	 */
	
	parent->bind("RTSThreshold_", &RTSThreshold);
	parent->bind("ShortRetryLimit_", &ShortRetryLimit);
	parent->bind("LongRetryLimit_", &LongRetryLimit);
}

/* ======================================================================
   Mac Class Functions
   ====================================================================== */
Mac_IFControl::Mac_IFControl() :
	Mac(), phymib_(this), macmib_(this),
	mhSend_(this), mhBackoff_(this), mhIdle_(this)
{

	tx_state_ = MAC_IDLE;
	last_action = MAC_IDLE;
	last_mac = NO_MAC_ADDR; // no previous communication partner
	last_time = 0;
	wait_for_idle = 0;
	forward = 0;
	pktRTS_ = 0;
	pktCTRL_ = 0;

	cw_ = phymib_.getCWMin();
	ssrc_ = slrc_ = 0;
	
	sta_seqno_ = 1;
	cache_ = 0;
	cache_node_count_ = 0;

	// reset rate cache
	for (int i = 0; i < MAC_IF_RATE_CACHE; ++i) {
		rate_cache[i].time = 0;
		rate_cache[i].address = (int)MAC_BROADCAST;
	}

	bind("fixed_code", &fixed_code);


	// Use PHY bitrate_ instead!

	//Tcl& tcl = Tcl::instance();
	//tcl.evalf("Mac/IFControl set bitrate_");
	//if (strcmp(tcl.result(), "0") != 0) 
	//	bind_bw("bitrate_", &bitrate_);
	//else
	//	bitrate_ = bandwidth_;
}


int
Mac_IFControl::command(int argc, const char*const* argv)
{
	if (argc == 3) {
		if (strcmp(argv[1], "log-target") == 0) {
			logtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			if(logtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		} else if(strcmp(argv[1], "nodes") == 0) {
			if(cache_) return TCL_ERROR;
			cache_node_count_ = atoi(argv[2]);
			cache_ = new Host[cache_node_count_ + 1];
			assert(cache_);
			bzero(cache_, sizeof(Host) * (cache_node_count_+1 ));
			return TCL_OK;
		} else if (strcmp(argv[1], "netif") == 0) {
			// Initialize the InterferencePhy
			// JCW
			netif_ = (InterferencePhy*) TclObject::lookup(argv[2]);
			netif_->tx_ths() = 0;
			netif_->rx_ths() = index_;
			netif_->own_ths() = THS_DISABLED; // disabled (only set during RTS/DATA transmission)
			netif_->code_id() = MODULATION_NUM_CODES - 1; // start with most powerful code

			// HACK for CACDMA
			//data_power_ = netif_->getPt();
			default_power_ = netif_->getPt();
			return TCL_OK;
		}
	}
	return Mac::command(argc, argv);
}

/* ======================================================================
   Debugging Routines
   ====================================================================== */
void
Mac_IFControl::trace_pkt(Packet *p) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_mac *dh = HDR_MAC(p);

	fprintf(stderr, "\t[ %2x %2x %2x ] %x %s %d\n",
		dh->ftype_, dh->macDA_, dh->macSA_,
		index_, packet_info.name(ch->ptype()), ch->size());
}

void
Mac_IFControl::dump(char *fname)
{
	fprintf(stderr,
		"\n%s --- (INDEX: %d, time: %2.9f)\n",
		fname, index_, Scheduler::instance().clock());

	fprintf(stderr,
		"\ttx_state_: %x, idle: %d\n",
		tx_state_, is_idle());

	fprintf(stderr,
		"\tpktTx_: %x, pktRTS_: %x, pktCTRL_: %x, callback: %x\n",
		(int) pktTx_, (int) pktRTS_,
		(int) pktCTRL_, (int) callback_);

	fprintf(stderr,
		"\tBackoff: %d (%d), Timer: %d\n",
		mhBackoff_.busy(), mhBackoff_.paused(),
		mhSend_.busy());
	fprintf(stderr,
		"\tBackoff Expire: %f\n",
		mhBackoff_.expire());
}


/* ======================================================================
   Misc Routines
   ====================================================================== */

inline int
Mac_IFControl::is_idle()
{
	//printf("%.9f %i is idle (tx %x, PHY %i)\n", Scheduler::instance().clock(),
	//       index_, tx_state_, netif_->is_idle());

	if(tx_state_ != MAC_IDLE)
		return 0;
	// JCW
	if(netif_->is_idle() == 0)
		return 0;

	return 1;
}

inline void
Mac_IFControl::set_tx_state(MacState x)
{
	/*
#ifdef MACIF_DEBUG
	printf("%.9f %i set tx state (idle %i, busy %i, paused %i) state (%x -> %x)\n", Scheduler::instance().clock(),
	       index_, is_idle(), mhBackoff_.busy(), mhBackoff_.paused(),
	       tx_state_, x);
#endif
	*/
	tx_state_ = (x);
	//if(is_idle() && mhBackoff_.paused())
	//	mhBackoff_.resume();
	//if(!is_idle() && mhBackoff_.busy() && !mhBackoff_.paused())
	//	mhBackoff_.pause();
}

inline void
Mac_IFControl::set_ths(Packet *p)
{
	// TIME HOPPING CODE
	// for now, data and RTS use the receivers code while CTS and
	// ACK are sent with the senders code (i.e. all is encoded
	// using the DATA receiver); concurrent tx with the same code
	// don't create a collisions for now (random TH and non-synced
	// senders)

	struct hdr_mac *mh = HDR_MAC(p);

	switch(mh->ftype_) {
	case MF_RTS:
		assert(mh->macDA_ != (int)MAC_BROADCAST);
		// receive CTS on private code
		netif_->rx_ths() = PRIVATE_CODE(index_, mh->macDA_);
		// send on receiver's code
		netif_->tx_ths() = mh->macDA_;
		break;
	case MF_CTS:
		assert(mh->macDA_ != (int)MAC_BROADCAST);
		netif_->rx_ths() = PRIVATE_CODE(mh->macDA_, index_);
		netif_->tx_ths() = PRIVATE_CODE(mh->macDA_, index_);
		break;
	case MF_DATA:
		// listen for ACK on receiver's code unless it was a broadcast (-> reset THS)
		if (mh->macDA_ != (int)MAC_BROADCAST)
			netif_->rx_ths() = mh->macDA_;
		else
			netif_->rx_ths() = index_;

		// send on private code (only when RTS was sent)
		if((u_int32_t) HDR_CMN(p)->size() >= macmib_.getRTSThreshold() &&
		   mh->macDA_ != (int)MAC_BROADCAST) {
			netif_->tx_ths() = PRIVATE_CODE(index_, mh->macDA_);
		} else {
			netif_->tx_ths() = mh->macDA_;
		}
		break;
	case MF_ACK:
		assert(mh->macDA_ != (int)MAC_BROADCAST);
		// fall through
	case MF_SIGIDLE:
		// use own code for tx, reset rx THS to own code as well
		netif_->rx_ths() = index_;
		netif_->tx_ths() = index_;
		break;
	default:
		fprintf(stderr, "ERROR in packet type: %f %i rx-code %i tx-code %i set code (%i)\n",
			Scheduler::instance().clock(),
			index_, netif_->rx_ths(), netif_->tx_ths(), mh->ftype_);
		exit(-1);
	}
	//printf("%.9f %i rx-code %i tx-code %i set code (%i)\n", Scheduler::instance().clock(),
	//       index_, netif_->rx_ths(), netif_->tx_ths(), mh->ftype_);
}

inline int
Mac_IFControl::calc_rate(Packet *p)
{
	// PHY LAYER CODING (INCREMENTAL REDUNDANCY)
	// use most powerful code for control packets and broadcasts,
	// adjust code for data packets.

	struct hdr_mac *mh = HDR_MAC(p);
	int dst = mh->macDA_;
	double now = Scheduler::instance().clock();
	int code = -1;

	switch(mh->ftype_) {
	case MF_CTS:
	case MF_SIGIDLE:
	case MF_ACK:
	case MF_RTS:
		code = MODULATION_NUM_CODES - 1; // use most powerful code
		break;
	case MF_DATA:
		if (dst == (int)MAC_BROADCAST) {
			code = MODULATION_NUM_CODES - 1; // use most powerful code
		} if (fixed_code >= 0) {
			code = fixed_code;
		} else {
			// search for dst in rate cache
			int oldest = 0, found = -1;
			for (int i = 0; i < MAC_IF_RATE_CACHE; ++i) {
				if (rate_cache[i].time < rate_cache[oldest].time)
					oldest = i;
				if (rate_cache[i].address == dst)
					found = i;
			}

			if (found == -1) {
				// not found --> insert (overwrite oldest entry)
				rate_cache[oldest].address = dst;
				rate_cache[oldest].rate = MODULATION_NUM_CODES - 1; // use most powerful code
				found = oldest;
			} else if (now - rate_cache[found].time > MAC_IF_RATE_CACHE_TIMEOUT) {
				// entry too old
				rate_cache[found].rate = MODULATION_NUM_CODES - 1;
			}
			code = rate_cache[found].rate;
			rate_cache[found].time = now;
		}
		break;
	default:
		fprintf(stderr,"ERROR in packet type: %f %i rx-code %i tx-code %i set code (%i)\n",
			Scheduler::instance().clock(),
			index_, netif_->rx_ths(), netif_->tx_ths(), mh->ftype_);
		exit(-1);
	}
	//printf("%.9f %i rx-code %i tx-code %i set code (%i)\n", Scheduler::instance().clock(),
	//       index_, netif_->rx_ths(), netif_->tx_ths(), mh->ftype_);

	return code;
}

inline void
Mac_IFControl::updateRateCache(int dst, int rate)
{
	double now = Scheduler::instance().clock();

	// search for dst in rate cache
	int oldest = 0, found = -1;
	for (int i = 0; i < MAC_IF_RATE_CACHE; ++i) {
		if (rate_cache[i].time < rate_cache[oldest].time)
			oldest = i;
		if (rate_cache[i].address == dst)
			found = i;
	}

	if (found == -1) {
		// not found --> insert (overwrite oldest entry)
		found = oldest;
		rate_cache[found].address = dst;
	}
	rate_cache[found].time = now;

	//rate_cache[found].rate = rate;
	if (rate < rate_cache[found].rate) {
		if (rate_cache[found].rate > 0)
			--rate_cache[found].rate;
	} else if (rate > rate_cache[found].rate) {
		rate_cache[found].rate = MIN(rate_cache[found].rate * 2, rate);
		if (rate_cache[found].rate > MODULATION_NUM_CODES - 1)
			rate_cache[found].rate = MODULATION_NUM_CODES - 1;
	}
}

void
Mac_IFControl::tx_resume()
{
	assert(mhSend_.busy() == 0);

	// idle could be running if we sent data, got data instead of
	// ACK, then send ACK but are still waiting for the IDLE
	// assert(mhIdle_.busy() == 0);

	set_tx_state(MAC_IDLE);

	if (pktCTRL_) {
		// disable listening on own THS (only during RTS and DATA)
		netif_->own_ths() = THS_DISABLED;

		/*
		 *  Need to send a CTS, ACK, or SIGIDLE.
		 */
		if (mhBackoff_.busy()) {
			// usually backoff after SIGIDLE or
			// the PHY was busy when we tried to send an RTS
			// -> cancel backoff, RTS will be sent at next tx_resume()
#ifdef MACIF_DEBUG
			printf("%.9f %i PHY busy, sending RTS later\n", Scheduler::instance().clock(),
			       index_);
#endif
			mhBackoff_.stop();
		}
		
		if (check_pktCTRL() == -1) {
			fprintf(stderr, "check_pktCTRL failed\n");
			exit(-1);
		}
		return;
	}
	// set own THS to also listen for incoming RTS (and DATA)
	// all RTS except for one coming from the node we sent our RTS to will be discarded
	// (listening necessary to avoid temporary deadlocks)
	netif_->own_ths() = index_;

        if (pktRTS_) {
		if (mhBackoff_.busy() == 0) {
			// wait for IDLE if we want to send to our last communication partner
			struct hdr_mac *mh = HDR_MAC(pktRTS_);
			if (last_mac == mh->macDA_ && last_action != MAC_IDLE
			    && Scheduler::instance().clock() < last_time + MAX_TXTIME) {
				// we need to explicitly set the state since no RTS packet is
				// sent yet, but we're also no longer idle
				set_tx_state(MAC_RTS);

				// listen for IDLE
				netif_->rx_ths() = mh->macDA_;

				// timer will be canceled when IDLE is received and RetransmitRTS() is called
				if (mhIdle_.busy() == 0) {
					// max transaction time + random offset
					double t = MAX_TXTIME + (Random::random() % cw_) * phymib_.getSlotTime();
					printf("%.9f %i MAX_TXTIME cw = %d\n", Scheduler::instance().clock(),index_,cw_);
#ifdef MACIF_DEBUG
					printf("%.9f %i RTS to prev. comm. partner (%i); waiting for IDLE until %.9f (%f)\n",
					       Scheduler::instance().clock(), index_, mh->macDA_,
					       Scheduler::instance().clock() + t, t);
#endif
					mhIdle_.start(t);
				} else if (mhIdle_.paused()) {
					mhIdle_.resume();
				}
				// only wait for IDLE once
				last_action = MAC_IDLE;
				last_mac = NO_MAC_ADDR; // no previous communication partner
			} else {
				if (check_pktRTS() == -1) {
					fprintf(stderr, "check_pktRTS failed\n");
					exit(-1);
				}
			}
		}
		return;
	}
	if(pktTx_) {
		// backoff could be running after sending idle
		if (mhBackoff_.busy() == 0) {
			// wait for IDLE if we want to send to our last communication partner
			// AND we didn't send an RTS
			struct hdr_mac *mh = HDR_MAC(pktTx_);
			if (mh->macDA_ != (int)MAC_BROADCAST
			    && (u_int32_t) HDR_CMN(pktTx_)->size() < macmib_.getRTSThreshold()
			    && last_mac == mh->macDA_ && last_action != MAC_IDLE
			    && Scheduler::instance().clock() < last_time + MAX_TXTIME) {
				// we need to explicitly set the state since no data packet is
				// sent yet, but we're also no longer idle
				set_tx_state(MAC_SEND);

				// listen for IDLE
				netif_->rx_ths() = mh->macDA_;

				// timer will be canceled when IDLE is received and RetransmitRTS() is called
				if (mhIdle_.busy() == 0) {
					// max transaction time + random offset
					double t = MAX_TXTIME + (Random::random() % cw_) * phymib_.getSlotTime();
#ifdef MACIF_DEBUG
					printf("%.9f %i MAX_TXTIME cw = %d\n", Scheduler::instance().clock(),index_,cw_);
					printf("%.9f %i DATA to prev. comm. partner (%i); waiting for IDLE until %.9f (%f)\n",
					       Scheduler::instance().clock(), index_, mh->macDA_,
					       Scheduler::instance().clock() + t, t);
#endif
					mhIdle_.start(t);
				} else if (mhIdle_.paused()) {
					mhIdle_.resume();
				}
				// only wait for IDLE once
				last_action = MAC_IDLE;
				last_mac = NO_MAC_ADDR; // no previous communication partner
			} else {
				if (check_pktTx() == -1) {
					fprintf(stderr, "check_pktTx failed\n");
					exit(-1);
				}
			}
		}
		return;
	} 
	if (callback_) {
#ifdef MACIF_DEBUG
		printf("%.9f %i callback (%i)\n", Scheduler::instance().clock(),
		       index_, (int) callback_);
#endif
		Handler *h = callback_;
		callback_ = 0;
		h->handle((Event*) 0);
		return;
	}

	/*
	  idle implicit in ACK
	
	// nothing to ...
	if (last_action == MAC_RECV) {
		// last action was recv and node has nothing to send
		// --> send IDLE and wait for max. backoff time 
		sendSIGIDLE(index_);
		assert(!mhBackoff_.busy());
		if (check_pktCTRL() == -1) {
			fprintf(stderr, "check_pktCTRL failed\n");
			exit(-1);
		}
	}
	*/
	last_action = MAC_IDLE;
	last_mac = NO_MAC_ADDR; // no previous communication partner
}


/*
 * txtime()	- pluck the precomputed tx time from the packet header
 */
double
Mac_IFControl::txtime(Packet *p)
{
	 struct hdr_cmn *ch = HDR_CMN(p);
	 double t = ch->txtime();
	 assert (t > 0.0);
	 return t;
}

 
/*
 * txtime()	- calculate tx time for packet of size "psz" bytes 
 *		  with channel code channel_code"
 */
double
Mac_IFControl::txtime(int psz, int channel_code)
{
	double dsz = psz - phymib_.getPLCPhdrLen();
	int plcp_hdr = phymib_.getPLCPhdrLen() << 3;
	int datalen = (int)dsz << 3;

	assert (dsz > 0);

	double basicrate = netif_->ppm_rate(MODULATION_NUM_CODES - 1);
//printf("\n\n\n\n\n MODULATION_NUM_CODES = %i\n\n\n\n\n",MODULATION_NUM_CODES);
//printf("\n\n\n\n\n basicrate = %f\n\n\n\n\n",basicrate);
	double datarate = netif_->ppm_rate(channel_code);
//printf("\n\n\n\n\n channel_code = %i\n\n\n\n\n",channel_code);
//printf("\n\n\n\n\n datarate = %f\n\n\n\n\n",datarate);

//printf("\n\n\n\n\n bit rate = %f\n\n\n\n\n",netif_->bitrate());
	assert(basicrate > 0 && datarate > 0);
	double t = (((double)plcp_hdr)/(netif_->bitrate() * basicrate)) +
		(((double)datalen)/(netif_->bitrate() * datarate));
	return(t);
}


/* ======================================================================
   Timer Handler Routines
   ====================================================================== */
void
Mac_IFControl::backoffHandler()
{
	//printf("%.9f %i backoff handler, %x\n", Scheduler::instance().clock(),
	//       index_, tx_state_);

	//assert(pktCTRL_ == 0);

	//if(pktCTRL_) {
	//	assert(mhSend_.busy());
	//	return;
	//}

	//if(check_pktRTS() == 0)
	//	return;

	//if(check_pktTx() == 0)
	//	return;

	// we shouldn't accept an RTS while we'ew waiting to send our own RTS or data apcket
	assert(pktCTRL_ == 0);
	// call tx_resume to ensure that last_mac is checked accordingly
	tx_resume();
}

void
Mac_IFControl::sendHandler()
{
	// timeout of the send timer
	switch(tx_state_) {
	/*
	 * Sent a RTS, but did not receive a CTS.
	 */
	case MAC_RTS:
		assert(pktRTS_);
		// make sure we wait for an idle instead of sending directly
		assert(last_mac == HDR_MAC(pktRTS_)->macDA_ && last_action != MAC_IDLE
		       && Scheduler::instance().clock() < last_time + MAX_TXTIME);
#ifdef MACIF_DEBUG
		printf("%.9f %i send timeout -> retransmit RTS\n",
		       Scheduler::instance().clock(), index_);
#endif
		RetransmitRTS();
		break;
	/*
	 * Sent a CTS, but did not receive a DATA packet.
	 */
	case MAC_CTS:
		assert(pktCTRL_);
		Packet::free(pktCTRL_); pktCTRL_ = 0;
		// reset recv THS
		netif_->rx_ths() = index_;
		break;
	/*
	 * Sent DATA, but did not receive an ACK packet.
	 */
	case MAC_SEND:
		assert(pktTx_);
		// make sure we wait for an idle instead of sending directly
		// unless it's a broadcast
		assert((HDR_MAC(pktTx_)->macDA() == (int)MAC_BROADCAST) ||
			(last_mac == HDR_MAC(pktTx_)->macDA_ && last_action != MAC_IDLE
			 && Scheduler::instance().clock() <= last_time + MAX_TXTIME));
#ifdef MACIF_DEBUG
	printf("%.9f %i send timeout -> retransmit DATA\n",
		       Scheduler::instance().clock(), index_);
#endif
		RetransmitDATA();
		break;
	/*
	 * Sent private ACK, now send the public IDLE
	 */
	case MAC_ACK:
		// send IDLE signal
		assert(pktCTRL_);
		Packet::free(pktCTRL_); pktCTRL_ = 0;

		// to optimize forwarding, try to send after receive -> don't send IDLE yet
		//sendSIGIDLE(index_);

		break;
	/*
	 * Sent IDLE SIGNAL, and now ready to resume transmission.
	 */
	case MAC_SIGIDLE:
		/*
		 * Backoff before sending again.
		 */
		// reset recv THS
		netif_->rx_ths() = index_;

		rst_cw();
		assert(mhBackoff_.busy() == 0);

		// max possible timeout to allow others to contact sender

		// RMZ: randomize
		/*
		mhBackoff_.start((Random::random() % phymib_.getCWMax()) * phymib_.getSlotTime()
				 + (Random::uniform() * 2 * DSSS_MaxPropagationDelay));
		*/

		mhBackoff_.start(phymib_.getCWMax() * phymib_.getSlotTime()
				 + 2 * DSSS_MaxPropagationDelay);

		assert(pktCTRL_);
		Packet::free(pktCTRL_); pktCTRL_ = 0;
		break;
	case MAC_IDLE:
		break;
	default:
		assert(0);
	}
	tx_resume();
}

void
Mac_IFControl::idleHandler()
{
	// waited for IDLE that didn't come -> send RTS or DATA

	// backoff timer can't be running but
	// it might still be paused if we didn't hear an IDLE signal
	assert(mhBackoff_.busy() == 0 || mhBackoff_.paused());
	
	if (mhBackoff_.paused())
		mhBackoff_.stop();
		
	if (pktRTS_) {
#ifdef MACIF_DEBUG
		printf("%.9f %i idle timeout -> retransmit RTS\n",
		       Scheduler::instance().clock(), index_);
#endif
	} else if (pktTx_) {
#ifdef MACIF_DEBUG
		printf("%.9f %i idle timeout -> retransmit DATA\n",
		       Scheduler::instance().clock(), index_);
#endif
	} else {
		assert(0);
	}
	tx_resume();
}


/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */

inline void
Mac_IFControl::transmit(Packet* p, double t)
{
        set_ths(p);       /* set TH code on interface */

	/* 
	 * set channel code (FEC)
	 * packet rate set in sendData() only for data packets (unicast and broadcast)!
	 *
	 */
	struct hdr_mac *mh = HDR_MAC(p);
	if (mh->ftype_ == MF_DATA) {
		netif_->code_id() = packet_rate;
	} else {
		netif_->code_id() = MODULATION_NUM_CODES - 1;
	}

#ifdef MACIF_DEBUG
	printf("%.9f %i transmit ", Scheduler::instance().clock(),
	       index_);
	switch (mh->ftype_) {
	case MF_RTS: printf("RTS");break;
	case MF_CTS: printf("CTS");break;
	case MF_DATA: printf("DATA");break;
	case MF_ACK: printf("ACK");break;
	case MF_SIGIDLE: printf("SIGIDLE");break;
	default: printf("UNKN");
	}
	printf(" to %i (timeout %.9f) cw = %d\n", mh->macDA(), t,cw_);
#endif

        /*
         * pass the packet on the "interface" which will in turn
         * place the packet on the channel.
         */

	// HACK for CACDMA
	/*
	if (fixed_code >= 0) {
		if (mh->ftype_ == MF_DATA) {
			if (data_power_ < default_power_)
				netif_->setPt(data_power_);
			else
				netif_->setPt(default_power_);
		} else {
			netif_->setPt(default_power_);
		}
	}
	*/
        downtarget_->recv(p->copy(), this);
        mhSend_.start(t);
}

int
Mac_IFControl::check_pktCTRL()
{
	if(pktCTRL_ == 0)
		return -1;
	if(tx_state_ == MAC_CTS || tx_state_ == MAC_ACK)
		return -1;
					  
	struct hdr_mac *mh = HDR_MAC(pktCTRL_);
	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(pktCTRL_);
	double timeout;

	switch(mh->ftype_) {
	/*
	 *  If the medium is not IDLE, don't send the CTS.
	 */
	case MF_CTS:
		if(!is_idle()) {
			drop(pktCTRL_, DROP_MAC_BUSY);
			pktCTRL_ = 0;
			return 0;
		}
		set_tx_state(MAC_CTS);
		
		/*
		 * timeout:  cts + data tx time calculated by
		 *           adding cts tx time to the cts duration
		 *           minus ack tx time -- this timeout is
		 *           a guess since it is unspecified
		 *           (note: mh->dh_duration == cf->cf_duration)
		 */
		timeout = txtime(phymib_.getCTSlen())
			+ DSSS_MaxPropagationDelay			// XXX
			// sender could include code ID (and psize) in
			// RTS -> use proper pkt txtime instead of upper bound
			+ txtime(MAC_IF_MAX_PACKETSIZE)
			+ DSSS_MaxPropagationDelay;			// XXX
		break;
	case MF_ACK:
		// set IDLE flag (has to be done here instead of sendACK(),
		// since the RTR needs to have processed the received packet 
		// and maybe has scheduled it for forwarding by now)
		if (pktRTS_ || pktTx_ || (forward != 0)) {
			hdr_ifc->idle_ = 0;
#ifdef MACIF_DEBUG
			if (pktRTS_ || pktTx_) {
				struct hdr_cmn *hdr = HDR_CMN(pktTx_);
				struct hdr_mac *mh_pktTx = HDR_MAC(pktTx_);
				printf("%.9f %i BUSY ACK (packet %i to dst %i)\n",
				       Scheduler::instance().clock(), index_, hdr->uid(), mh_pktTx->macDA_);
			} else {
				printf("%.9f %i BUSY ACK (forward = %i)\n",
				       Scheduler::instance().clock(), index_, forward);
			}
#endif
		}
		else {
			hdr_ifc->idle_ = -1;
		}
		forward = 0;

		set_tx_state(MAC_ACK);
		timeout = txtime(phymib_.getACKlen()) + netif_->getPreambleTime(); //rmz

		// set last action when sending the ACK
		// --> no right to prefer sending packet after receiveing a broadcast
		last_action = MAC_RECV;
		last_mac = mh->macDA_;
		last_time = Scheduler::instance().clock() + timeout;
		break;
	case MF_SIGIDLE:
		set_tx_state((MacState)MAC_SIGIDLE);
		timeout = txtime(phymib_.getACKlen()) + netif_->getPreambleTime(); //rmz
		break;
	default:
		fprintf(stderr, "check_pktCTRL:Invalid MAC Control type %x\n", mh->ftype_);
		exit(1);
	}
        transmit(pktCTRL_, timeout);
	return 0;
}

int
Mac_IFControl::check_pktRTS()
{
	if(pktRTS_ == 0)
 		return -1;

	struct hdr_mac *mh = HDR_MAC(pktRTS_);
	double timeout;

	assert(mhBackoff_.busy() == 0);

 	switch(mh->ftype_) {
	case MF_RTS:
		// called only when idle or to retransmit RTS
		assert(tx_state_ == MAC_IDLE || tx_state_ == MAC_RTS);
		if(!netif_->is_idle()) {
			//inc_cw();
			// XXX this should be solved with a callback from the PHY!
			double now = Scheduler::instance().clock();
			assert(netif_->getTransEnd() > now);
			mhBackoff_.start(netif_->getTransEnd() - now + DSSS_MaxPropagationDelay);
			return 0;
		}
		set_tx_state(MAC_RTS);
		timeout = txtime(phymib_.getRTSlen())
			+ DSSS_MaxPropagationDelay
			+ txtime(phymib_.getCTSlen())
			+ DSSS_MaxPropagationDelay;
		break;
	default:
		fprintf(stderr, "check_pktRTS:Invalid MAC Control type %x\n", mh->ftype_);
		exit(1);
	}
	last_action = MAC_SEND;
	last_mac = mh->macDA_;
	last_time = Scheduler::instance().clock() + timeout;
        transmit(pktRTS_, timeout);
	return 0;
}

int
Mac_IFControl::check_pktTx()
{
	if(pktTx_ == 0)
		return -1;

	struct hdr_mac *mh = HDR_MAC(pktTx_);
	double timeout;
	
	assert(mhBackoff_.busy() == 0);

	switch(mh->ftype_) {
	case MF_DATA:
		assert(tx_state_ == MAC_IDLE);
		if(!netif_->is_idle()) {
			// the PHY can be busy when no RTS/CTS (->private THS) is used or if all use the common THS

			//inc_cw();
			// XXX this should be solved with a callback from the PHY!
			double now = Scheduler::instance().clock();
			//assert(netif_->getTransEnd() > now);
			assert(!(netif_->getTransEnd() - now < 0));
			mhBackoff_.start(netif_->getTransEnd() - now + DSSS_MaxPropagationDelay);
			return 0;
		}
		set_tx_state(MAC_SEND);
		if(mh->macDA_ != (int)MAC_BROADCAST)
			//timeout = txtime(pktTx_)
			timeout = txtime(pktTx_) + netif_->getPreambleTime()
				+ DSSS_MaxPropagationDelay		// XXX
				+ txtime(phymib_.getACKlen()) + netif_->getPreambleTime()
				+ DSSS_MaxPropagationDelay;		// XXX
		else
			//timeout = txtime(pktTx_);
			timeout = txtime(pktTx_) + netif_->getPreambleTime();
		break;
	default:
		fprintf(stderr, "check_pktTx:Invalid MAC Control type\n");
		//printf("pktRTS:%x, pktCTS/ACK:%x, pktTx:%x\n",pktRTS_, pktCTRL_,pktTx_);
		exit(1);
	}
	if(mh->macDA_ != (int)MAC_BROADCAST) {
		last_action = MAC_SEND;
		last_mac = mh->macDA_;
		last_time = Scheduler::instance().clock() + timeout;
	}
        transmit(pktTx_, timeout);
	return 0;
}

/*
 * Low-level transmit functions that actually place the packet onto
 * the channel.
 */
void
Mac_IFControl::sendRTS(int dst)
{
	Packet *p = Packet::alloc();
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac *mh = HDR_MAC(p);
	
	assert(pktTx_);
	assert(pktRTS_ == 0);

	/*
	 *  If the size of the packet is larger than the
	 *  RTSThreshold, then perform the RTS/CTS exchange.
	 *
	 *  XXX: also skip if destination is a broadcast
	 */
	if( (u_int32_t) HDR_CMN(pktTx_)->size() < macmib_.getRTSThreshold() ||
	    dst == (int)MAC_BROADCAST) {
		Packet::free(p);
		return;
	}

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getRTSlen();
	ch->iface() = -2;
	ch->error() = 0;

 	mh->ftype_ = MF_RTS;
	mh->macDA_ = dst;
	mh->macSA_ = index_;

	/* store rts tx time */
 	ch->txtime() = txtime(ch->size());

	pktRTS_ = p;
}

void
Mac_IFControl::sendCTS(int dst)
{
	Packet *p = Packet::alloc();
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac *mh = HDR_MAC(p);
	//	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);

	assert(pktCTRL_ == 0);

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getCTSlen();
	ch->iface() = -2;
	ch->error() = 0;

	mh->ftype_ = MF_CTS;
	mh->macDA_ = dst;
	mh->macSA_ = index_;
	
	/* store cts tx time */
	ch->txtime() = txtime(ch->size());
	
	// HACK for CACDMA
// 	if (fixed_code >= 0) {
// 		hdr_ifc->opt_power = netif_->CACDMApower();
// 	}
	
	pktCTRL_ = p;
}

void
Mac_IFControl::sendACK(int dst)
{
	Packet *p = Packet::alloc();
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac *mh = HDR_MAC(p);
	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);

	assert(pktCTRL_ == 0);

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getACKlen();
	ch->iface() = -2;
	ch->error() = 0;
	
	mh->ftype_ = MF_ACK;
	mh->macDA_ = dst;
	mh->macSA_ = index_;

	/* store ack tx time */
 	ch->txtime() = txtime(ch->size());
	
	// this assumes Modulation::best_code doesn't change between
	// data reception and creation of the ACK packet
	if (fixed_code < 0) {
		hdr_ifc->best_code = netif_->best_code();
	} else {
		hdr_ifc->best_code = fixed_code;
	}

	pktCTRL_ = p;
}

void
Mac_IFControl::sendSIGIDLE(int dst)
{
	/* similar to ACK creation */

	Packet *p = Packet::alloc();
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac *mh = HDR_MAC(p);

	assert(pktCTRL_ == 0);

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getACKlen();
	ch->iface() = -2;
	ch->error() = 0;
	
	mh->ftype_ = (MacFrameType)MF_SIGIDLE;
	mh->macDA_ = dst;
	mh->macSA_ = index_;

	/* store ack tx time */
 	ch->txtime() = txtime(ch->size());
	
	pktCTRL_ = p;
}

void
Mac_IFControl::sendDATA(Packet *p)
{
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac *mh = HDR_MAC(p);

	assert(pktTx_ == 0);

	/*
	 * Update the MAC header
	 */
	ch->size() += phymib_.getHdrLen11();

	mh->ftype_ = MF_DATA;

	// This is the only place where we set the data packet TX time
	// calc packet_rate which is used in transmit()

	if(mh->macDA_ != (int)MAC_BROADCAST) {
		/* store data tx time for unicast packets */
		packet_rate = calc_rate(p);
		ch->txtime() = txtime(ch->size(), packet_rate);
	} else {
		/* store data tx time for broadcast packets (see 9.6) */
		packet_rate = MODULATION_NUM_CODES - 1;
		ch->txtime() = txtime(ch->size(), packet_rate);
	}
	pktTx_ = p;
#ifdef MACIF_DEBUG
	printf("%.9f %i sendDATA to %i\n", Scheduler::instance().clock(), index_, mh->macDA_);
#endif
}

/* ======================================================================
   Retransmission Routines
   ====================================================================== */
void
Mac_IFControl::RetransmitRTS()
{
	assert(pktTx_);
	assert(pktRTS_);

	macmib_.RTSFailureCount++;

	ssrc_ += 1;			// STA Short Retry Count

	if(ssrc_ >= macmib_.getShortRetryLimit()) {
		// cancel timers and reset tx_state

		drop(pktRTS_, DROP_MAC_RETRY_COUNT_EXCEEDED); pktRTS_ = 0;
		/* tell the callback the send operation failed 
		   before discarding the packet */
		hdr_cmn *ch = HDR_CMN(pktTx_);
		if (ch->xmit_failure_) {
                        /*
                         *  Need to remove the MAC header so that 
                         *  re-cycled packets don't keep getting
                         *  bigger.
                         */
                        ch->size() -= phymib_.getHdrLen11();
                        ch->xmit_reason_ = XMIT_REASON_RTS;
                        ch->xmit_failure_(pktTx_->copy(),
                                          ch->xmit_failure_data_);
                }
		//printf("(%d)....discarding RTS:%x\n",index_,pktRTS_);
		drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED); pktTx_ = 0;
		ssrc_ = 0;
		rst_cw();

		// reset rx THS
		netif_->rx_ths() = index_;
#ifdef MACIF_DEBUG
		printf("%.9f %i rx-code %i tx-code %i (RTS fail)\n", Scheduler::instance().clock(), index_, netif_->rx_ths(), netif_->tx_ths());
#endif
	} else {
		inc_cw();
		//mhBackoff_.start(cw_, is_idle());
		//backoffHandler();
	}
}

void
Mac_IFControl::RetransmitDATA()
{
	struct hdr_cmn* ch = HDR_CMN(pktTx_);
	struct hdr_mac *mh = HDR_MAC(pktTx_);
	u_int32_t *rcount, thresh;

	assert(mhBackoff_.busy() == 0);
	assert(pktTx_);
	assert(pktRTS_ == 0);

	/*
	 *  Broadcast packets don't get ACKed and therefore
	 *  are never retransmitted.
	 */
	if(mh->macDA_ == (int)MAC_BROADCAST) {
		Packet::free(pktTx_); pktTx_ = 0;

		// we get here when the broadcast was transmitted
		// --> time to send an IDLE signal

		// for now disable IDLE signal after broadcast, it's
		// used for ARP and the ARP reply RTS will collide
		// with the idle signal (for unicast, the recv has to
		// wait for CONTENTION_PERIOD after the ACK)

		//sendSIGIDLE(index_);

		/*
		 * Backoff at end of TX.
		 */
		rst_cw();
		//backoffHandler();
		//mhBackoff_.start(cw_, is_idle());

		return;
	}

	macmib_.ACKFailureCount++;

	if((u_int32_t) ch->size() <= macmib_.getRTSThreshold()) {
		rcount = &ssrc_;
		thresh = macmib_.getShortRetryLimit();
	} else {
		rcount = &slrc_;
		thresh = macmib_.getLongRetryLimit();
	}

	(*rcount)++;

	// update channel code
	if((u_int32_t) ch->size() > macmib_.getRTSThreshold()) {
		// reset channel code upon each tx failure when RTS/CTS is used
		updateRateCache(mh->macDA_, MODULATION_NUM_CODES - 1);
	} else {
		// ... otherwise, TX failure is usually because recv node is busy
		if(*rcount > thresh) {
			// reset channel code only when giving up
			updateRateCache(mh->macDA_, MODULATION_NUM_CODES - 1);
		}
	}

	if(*rcount > thresh) {
		macmib_.FailedCount++;
		/* tell the callback the send operation failed 
		   before discarding the packet */
		hdr_cmn *ch = HDR_CMN(pktTx_);
		if (ch->xmit_failure_) {
                        ch->size() -= phymib_.getHdrLen11();
                        ch->xmit_reason_ = XMIT_REASON_ACK;
                        ch->xmit_failure_(pktTx_->copy(),
                                          ch->xmit_failure_data_);
                }

		drop(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED); pktTx_ = 0;
		//printf("(%d)DATA discarded: count exceeded\n",index_);
		*rcount = 0;
		rst_cw();

		// reset rx THS
		netif_->rx_ths() = index_;
#ifdef MACIF_DEBUG
		printf("%.9f %i rx-code %i tx-code %i (Data fail)\n", Scheduler::instance().clock(), index_, netif_->rx_ths(), netif_->tx_ths());
#endif
	}
	else {
		sendRTS(mh->macDA_);
		//printf("(%d)retxing data:%x..sendRTS..\n",index_,pktTx_);
		inc_cw();
		//backoffHandler();
		//mhBackoff_.start(cw_, is_idle());
	}
}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
Mac_IFControl::send(Packet *p, Handler *h)
{
	struct hdr_mac* mh = HDR_MAC(p);
	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);

	/* 
	 * drop the packet if the node is in sleep mode
	 XXX sleep mode can't stop node from sending packets
	 */
	EnergyModel *em = netif_->node()->energy_model();
	if (em && em->sleep()) {
		em->set_node_sleep(0);
		em->set_node_state(EnergyModel::INROUTE);
	}
	
	callback_ = h;
	sendDATA(p);
	sendRTS(mh->macDA_);

	/*
	 * Assign the data packet a sequence number.
	 */
	hdr_ifc->scontrol_ = sta_seqno_++;

	/*
	 *  If the medium is IDLE, we transmit.
	 */
	if (mhBackoff_.busy() == 0) {
		if (tx_state_ == MAC_IDLE) {
			if (netif_->is_idle()) {
				//backoffHandler();
				tx_resume();
			} else {
				// we are receiving or sending a packet -> backoff
				// XXX again, ideally this should be done with a callback from the PHY!
				assert(netif_->getTransEnd() > Scheduler::instance().clock());
				mhBackoff_.start(netif_->getTransEnd() + (Random::random() % cw_) * phymib_.getSlotTime());
			}
		}
	}
}

void
Mac_IFControl::recv(Packet *p, Handler *h)
{
	struct hdr_cmn *ch = HDR_CMN(p);
	/*
	 * Sanity Check
	 */
	assert(initialized());
	assert(p);

	//printf("%.9f %i recv\n", t, index_);

	/*
	 *  Handle outgoing packets.
	 */
	if(ch->direction() == hdr_cmn::DOWN) {
                send(p, h);
                return;
        }
	/*
	 *  Handle incoming packets.
	 */

	/*
	 * Check to see if this packet was received with enough
	 * bit errors that the current level of FEC still could not
	 * fix all of the problems - ie; after FEC, the checksum still
	 * failed.
	 */
	if( ch->error() ) {
		// HACK: don't drop ARP
		if (ch->ptype_ != PT_ARP) {
			drop(p, "ERR");
			Packet::free(p);
			return;
		}
	}

	struct hdr_mac *mh = HDR_MAC(p);
	int dst = mh->macDA_;
	int src = mh->macSA_;
	int type = mh->ftype_;

#ifdef MACIF_DEBUG
	if (type == MF_ACK) {
		struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);
		printf("%.9f %i This is ACK (src = %i, dst = %i, ", Scheduler::instance().clock(),index_,src,dst);
		if (hdr_ifc->idle_ == 0) {
			printf("BUSY)\n");
		} else {
			printf("IDLE)\n");
		}
	}
#endif



	if (dst != index_) {
		// The packet is not for us!!!
		// Make sense since ACK and IDLE are sent with the THS of the destination
		if (tx_state_ == MAC_RTS || tx_state_ == MAC_SEND) {
			struct hdr_mac *rf;
			if (tx_state_ == MAC_RTS) {
				// We sent an RTS
				assert(pktRTS_);
				rf = HDR_MAC(pktRTS_);
			} else {
				assert(pktTx_);
				rf = HDR_MAC(pktTx_);
			}

			if ((src == rf->macDA_) && ((type == MF_SIGIDLE) || (type == MF_ACK))) {
				/*
				 * The packets comes from our intended
				 * RTS destination and is an IDLE or
				 * ACK packet
				 * 
				 * check if it's an IDLE signal (through an idle flag in the ACK)
				 */
				struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);
				if ((type == MF_ACK) && (hdr_ifc->idle_ == 0)) {
					// Node is not idle
					// node will be busy transmitting it's own packet: reset IDLE timer,
					// cancel send timer, and pause backoff timer
#ifdef MACIF_DEBUG
					printf("%.9f %i Overheard BUSY ACK received, MAX_TXTIME cw = %d\n", Scheduler::instance().clock(),index_,cw_);
#endif
					if (mhBackoff_.busy() && !mhBackoff_.paused())
						mhBackoff_.pause();
					if (mhSend_.busy())
						mhSend_.stop();
					if (mhIdle_.busy())
						mhIdle_.stop();

					mhIdle_.start(MAX_TXTIME + (Random::random() % cw_) * phymib_.getSlotTime());

				} else {
					/*
					 * Not a ACK => IDLE PACKET
					*/
					// if we see a node's IDLE/ACK, this means its tx is
					// over and we can now contend for access to it
#ifdef MACIF_DEBUG
					printf("%.9f %i received IDLE, retrying RTS/DATA (dst %i) cw = %d\n",
					       Scheduler::instance().clock(), index_, rf->macDA_,cw_);
#endif
					// send or idle timer could be running -> stop timers
					if (mhSend_.busy())
						mhSend_.stop();
					if (mhIdle_.busy())
						mhIdle_.stop();
					
					// start / resume backoff
					if (mhBackoff_.busy()) {
						if (mhBackoff_.paused())
							mhBackoff_.resume();
					} else {
						mhBackoff_.start((Random::random() % cw_) * phymib_.getSlotTime());
					}
				}
			} else if ((dst == rf->macDA_) && (type == MF_RTS || type == MF_DATA)) {
				/*
				 * We overhear an RTS or DATA packet
				 * from another src but for our
				 * intended destination
				 */
				// wait until the tx from another nodes is over
#ifdef MACIF_DEBUG
				printf("%.9f %i destination busy, delaying RTS/DATA (dst %i)\n",
				       Scheduler::instance().clock(), index_, rf->macDA_);
#endif
				// check that the node didn't alreay pause the timer 
				// or the send timer is running instead of the backoff timer
				if (mhBackoff_.busy() && !mhBackoff_.paused()) {
					mhBackoff_.pause();
					if (mhIdle_.busy())
						mhIdle_.stop();
					// can be optimized
					// max transaction time + random offset
					mhIdle_.start(MAX_TXTIME + (Random::random() % cw_) * phymib_.getSlotTime());
					printf("%.9f %i MAX_TXTIME cw = %d\n", Scheduler::instance().clock(),index_,cw_);
				}
			}
		}
	}


	/* rmz: this is stuff from the regular 802.11 MAC */
        /* tap out - */
        if (tap_ && type == MF_DATA) 
		tap_->tap(p);
	/*
	 * Adaptive Fidelity Algorithm Support - neighborhood infomation 
	 * collection
	 *
	 * Hacking: Before filter the packet, log the neighbor node
	 * I can hear the packet, the src is my neighbor
	 */
	if (netif_->node()->energy_model() && 
	    netif_->node()->energy_model()->adaptivefidelity()) {
		netif_->node()->energy_model()->add_neighbor(src);
	}
	/*
	 * Address Filtering
	 */
	if(dst != index_ && dst != (int)MAC_BROADCAST) {
		/*
		 *  We don't want to log this event, so we just free
		 *  the packet
		 */
		// #ifdef MACIF_DEBUG
		// 		printf("%.9f %i Dropping packet (src %i - dst %i)\n",
		// 		       Scheduler::instance().clock(), index_, src, dst);
		// #endif
		Packet::free(p);
		return;
	}

#ifdef MACIF_DEBUG
	printf("%.9f %i recv ", Scheduler::instance().clock(),
	       index_);
	switch (type) {
	case MF_RTS: printf("RTS");break;
	case MF_CTS: printf("CTS");break;
	case MF_DATA: printf("DATA");break;
	case MF_ACK: printf("ACK");break;
	case MF_SIGIDLE: printf("SIGIDLE");break;
	default: printf("UNKN");
	}
	printf(" from %i\n", src);
#endif

	switch(type) {
	case MF_RTS:
		recvRTS(p);
		break;
	case MF_CTS:
		recvCTS(p);
		break;
	case MF_ACK:
		recvACK(p);
		break;
	case MF_DATA:
		recvDATA(p);
		break;
	default:
		fprintf(stderr, "recv: Invalid MAC Type %x\n", type);
		exit(1);
	}
}


void
Mac_IFControl::recvRTS(Packet *p)
{
	struct hdr_mac *mh = HDR_MAC(p);

	/*
	 * check if we get an RTS from the node we should receive data from
	 * in this case the data packet must have been lost, don't wait
	 */
 	if (tx_state_ == MAC_CTS) {
		assert(pktCTRL_);
		assert(mhSend_.busy());
		struct hdr_mac *mhctrl = HDR_MAC(pktCTRL_);
		if (mh->macSA() == mhctrl->macDA()) {
			// don't wait for data packet
			tx_state_ = MAC_IDLE;
			Packet::free(pktCTRL_); pktCTRL_ = 0;
			mhSend_.stop();
		}
	}

	/*
	 *  If the node is talking or responding to someone else, discard the RTS.
	 */
	int reply = (tx_state_ == MAC_IDLE && pktCTRL_ == 0);

	/*
	 * ... unless it gets an RTS from the node it sent an RTS to! (to avoid deadlock)
	 */
	if (tx_state_ == MAC_RTS) {
		assert(pktRTS_);
		if (HDR_MAC(pktRTS_)->macDA() == mh->macSA()) {
			reply = -1;
			
			// cancel timer for RTS
			assert(mhSend_.busy() || mhBackoff_.busy() || mhIdle_.busy());
			if (mhSend_.busy())
				mhSend_.stop();
			if (mhIdle_.busy())
				mhIdle_.stop();
			if (mhBackoff_.busy())
				mhBackoff_.stop();
		}
	}

	if (!reply) {
#ifdef MACIF_DEBUG
		printf("%.9f %i drop RTS (busy), state %x, pkt %i\n",
		       Scheduler::instance().clock(), index_, tx_state_, (int)pktCTRL_);
#endif
		drop(p, DROP_MAC_BUSY);
		return;
	}

	sendCTS(mh->macSA_);
	mac_log(p);

	tx_resume();
}


void
Mac_IFControl::recvCTS(Packet *p)
{
	if(tx_state_ != MAC_RTS) {
		drop(p, DROP_MAC_INVALID_STATE);
		return;
	}

	// HACK for CACDMA
	//if (fixed_code >= 0) {
	//	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);
	//	data_power_ = hdr_ifc->opt_power;
	//}

	assert(pktRTS_);
	Packet::free(pktRTS_); pktRTS_ = 0;

	assert(pktTx_);
	
	mhSend_.stop();

	/*
	 * The successful reception of this CTS packet implies
	 * that our RTS was successful.  Hence, we can reset
	 * the Short Retry Count and the CW.
	 */
	// XXX why was this commented out???
	ssrc_ = 0;
	rst_cw();

	mac_log(p);
	tx_resume();
}

void
Mac_IFControl::recvDATA(Packet *p)
{
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_mac *mh = HDR_MAC(p);
	struct hdr_ip *iph = HDR_IP(p);
	int dst, src, size;

	dst = mh->macDA_;
	src = mh->macSA_;
	size = ch->size();

	/*
	 * Adjust the MAC packet size - ie; strip
	 * off the mac header
	 */
	ch->size() -= phymib_.getHdrLen11();
	ch->num_forwards() += 1;


	/*
	 *  If we sent a CTS, clean up...
	 */
	if(dst != (int)MAC_BROADCAST) {
		if((u_int32_t)size >= macmib_.getRTSThreshold()) {
			if (tx_state_ == MAC_CTS) {
				assert(pktCTRL_);
				Packet::free(pktCTRL_); pktCTRL_ = 0;
				mhSend_.stop();
				/*
				 * Our CTS got through.
				 */
				//printf("(%d): RECVING DATA!\n",index_);

				// XXX why was this commented out???
				ssrc_ = 0;
				rst_cw();
			}
			else {
				drop(p, DROP_MAC_BUSY);
				//printf("(%d)..discard DATA\n",index_);
				return;
			}
			/*
			 * Looking at the IP address of the packet (rmz)
			 */
			if((dst == index_) && (iph->daddr() != index_) && (iph->daddr() != (int)MAC_BROADCAST)) {
				forward = 1;
#ifdef MACIF_DEBUG
				// Set the forward flag
				printf("%.9f %i Packet to forward to IP %i \n",
				       Scheduler::instance().clock(), index_, iph->daddr());
#endif
			}
			sendACK(src);
		}
		/*
		 *  We did not send a CTS and there's no
		 *  room to buffer an ACK.
		 */
		else {
			if(pktCTRL_) {
				drop(p, DROP_MAC_BUSY);
				return;
			}
			/*
			 * Looking at the IP address of the packet (rmz)
			 */
			if((dst == index_) && (iph->daddr() != index_) && (iph->daddr() != (int)MAC_BROADCAST)) {
				assert(forward == 0);
				forward = 1;
#ifdef MACIF_DEBUG
				// Set the forward flag
				printf("%.9f %i Packet to forward to IP %i \n",
				       Scheduler::instance().clock(), index_, iph->daddr());
#endif
			}
			sendACK(src);
		}
	}

	/* ============================================================
	    Make/update an entry in our sequence number cache.
	   ============================================================ */

	/* Changed by Debojyoti Dutta. This upper loop of if{}else was 
	   suggested by Joerg Diederich <dieder@ibr.cs.tu-bs.de>. 
	   Changed on 19th Oct'2000 */

        if(dst != (int)MAC_BROADCAST) {
                if (src < cache_node_count_) {
                        Host *h = &cache_[src];
			struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);

                        if(h->seqno && h->seqno == hdr_ifc->scontrol_) {
                                drop(p, DROP_MAC_DUPLICATE);
				// need to send the ACK again
				goto data_resume;
                        }
                        h->seqno = hdr_ifc->scontrol_;
                } else {
			static int count = 0;
			if (++count <= 10) {
				printf ("MAC_IFControl: accessing MAC cache_ array out of range (src %u, dst %u, size %d)!\n", src, dst, cache_node_count_);
				if (count == 10)
					printf ("[suppressing additional MAC cache_ warnings]\n");
			}
		}
	}

	/*

	 *  Pass the packet up to the link-layer.
	 *  XXX - we could schedule an event to account
	 *  for this processing delay.
	 */
	
	uptarget_->recv(p, (Handler*) 0);

 data_resume:
	//if (mhSend_.busy() == 0 && mhIdle_.busy() == 0)

	// cancel send timer and pause idle timer to be able to send the ACK immediately
	if (mhSend_.busy())
		mhSend_.stop();
	if (mhIdle_.busy() && !mhIdle_.paused())
		mhIdle_.pause();
	tx_resume();
}


void
Mac_IFControl::recvACK(Packet *p)
{	
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_mac *mh = HDR_MAC(p);
	struct hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);

	struct hdr_mac *mh_data = HDR_MAC(pktTx_);

	if(tx_state_ != MAC_SEND) {
		drop(p, DROP_MAC_INVALID_STATE);
		return;
	}

	// update channel code of the last data packet's dst to the
	// one indicated in the ACK
	// use code specified in the ACK and add safety margin to
	// improve probability of decoding if conditions worsen
	updateRateCache(mh_data->macDA_, hdr_ifc->best_code + MAC_IF_RATE_MARGIN);

	assert(pktTx_);
	Packet::free(pktTx_); pktTx_ = 0;

	mhSend_.stop();

	/*
	 * The successful reception of this ACK packet implies
	 * that our DATA transmission was successful.  Hence,
	 * we can reset the Short/Long Retry Count and the CW.
	 */
	if((u_int32_t) ch->size() <= macmib_.getRTSThreshold())
		ssrc_ = 0;
	else
		slrc_ = 0;

	// if receiver sends IDLE, reset last_mac; after sending the own IDLE, if the sender
	// wants to contact the same receiver it can do so immediately (other request will be earlier)
	if (hdr_ifc->idle_) {
		last_action = MAC_IDLE;
		last_mac = NO_MAC_ADDR;
#ifdef MACIF_DEBUG
		printf("%.9f %i received IDLE/ACK from %i\n",
		       Scheduler::instance().clock(), index_, mh->macSA());
#endif
	}

	mac_log(p);
	// send idle signal (dst oneself)
	// RMZ: removed SIGIDLE for tests!!!!
	sendSIGIDLE(index_);
	tx_resume();
}

/* ======================================================================
   Timers
   ====================================================================== */

void
MacTimerIFControl::start(double timeout)
{
	Scheduler &s = Scheduler::instance();
	assert(busy_ == 0);

	busy_ = 1;
	paused_ = 0;
	stime = s.clock();
	rtime = timeout;
	assert(rtime >= 0.0);

	s.schedule(this, &intr, rtime);
}

void
MacTimerIFControl::stop(void)
{
	Scheduler &s = Scheduler::instance();

	assert(busy_);

	if(paused_ == 0)
		s.cancel(&intr);

	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;
}

/* ======================================================================
   Send Timer
   ====================================================================== */
void    
TxTimerIFControl::handle(Event *)
{       
	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->sendHandler();
}

/* ======================================================================
   Idle Timer
   ====================================================================== */
void    
IdleTimerIFControl::handle(Event *)
{       
	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->idleHandler();
}

void
IdleTimerIFControl::pause()
{
	Scheduler &s = Scheduler::instance();

	double done = (s.clock() - stime);
	if (done < 0)
		done = 0;
	assert(busy_ && !paused_);

	paused_ = 1;
	rtime -= done;
	assert(rtime >= 0.0);

	s.cancel(&intr);
}


void
IdleTimerIFControl::resume()
{
	Scheduler &s = Scheduler::instance();

	assert(busy_ && paused_);
	paused_ = 0;
	stime = s.clock();
	assert(rtime >= 0.0);

       	s.schedule(this, &intr, rtime);
}

/* ======================================================================
   Backoff Timer
   ====================================================================== */
void
BackoffTimerIFControl::handle(Event *)
{
#ifdef MACIF_DEBUG
	printf("%.9f %i backoff handle\n", Scheduler::instance().clock(), mac->index_);
#endif

	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->backoffHandler();
}

void
BackoffTimerIFControl::start(double timeout)
{
#ifdef MACIF_DEBUG
	printf("%.9f %i backoff start, timeout %.9f (%.9f - %.9f)\n",
	       Scheduler::instance().clock(), mac->index_, timeout, rtime, rtime+stime);
#endif
	MacTimerIFControl::start(timeout);
}


void
BackoffTimerIFControl::pause()
{
#ifdef MACIF_DEBUG
	printf("%.9f %i backoff paused\n", Scheduler::instance().clock(), mac->index_);
#endif
	Scheduler &s = Scheduler::instance();

	int slots = (int) ((s.clock() - stime) / mac->phymib_.getSlotTime());
	if(slots < 0)
		slots = 0;
	assert(busy_ && !paused_);

	paused_ = 1;
	rtime -= (slots * mac->phymib_.getSlotTime());
	assert(rtime >= 0.0);

	s.cancel(&intr);
}


void
BackoffTimerIFControl::resume()
{
#ifdef MACIF_DEBUG
	printf("%.9f %i backoff resume\n", Scheduler::instance().clock(), mac->index_);
#endif
	Scheduler &s = Scheduler::instance();

	assert(busy_ && paused_);
	paused_ = 0;
	stime = s.clock();
	assert(rtime >= 0.0);

       	s.schedule(this, &intr, rtime);
}

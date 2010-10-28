/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-  *
 *
 * Copyright (c) 2004, Joerg Widmer, EPFL
 * Copyright (c) 2005, 2006, Ruben Merz, EPFL
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
 * $Header: /cvs/ns-2_26-uwb/mac/interference-phy.h,v 1.18 2006/02/09 16:24:19 uid15352 Exp $
 *
 * partly based on wireless-phy
 */

#ifndef ns_InterferencePhy_h
#define ns_InterferencePhy_h

#include "packet.h"
#include "timer-handler.h"
#include "wireless-phy.h"
#include "modulation.h"
#include "modulation-codedppm.h"

#define POWER_SPECTRAL_DENSITY 4e-21

enum PhyState {
	PHY_IDLE	= 0x00000,
	PHY_RECV 	= 0x00010,
	PHY_SEND 	= 0x00100,
	PHY_COLL	= 0x01000,
	PHY_SYNC	= 0x10000
};

/* 
 * to speed up simulations we never free entries but reuse unused
 * ones (-> have list of max. # of concurrent packets)
 */
struct InterferenceListEntry {
	InterferenceListEntry *next;
	double receivedPower;
	double rxEndTime;
	double rxStartTime;
	int used;
};

// rmz
struct SyncListEntry {
	SyncListEntry *next;
	Packet *p;
	double rxStartTime;
	double rxEndTime;
	double Pr;
	int right_ths;
	int used;
};


class InterferencePhy;

class PhyIdleTimer : public TimerHandler {
public:
	PhyIdleTimer(InterferencePhy *p) : TimerHandler() { phy_ = p; }
	virtual void expire(Event *e);
protected:
	InterferencePhy *phy_;
};

class PhyRecvTimer : public TimerHandler {
public:
	PhyRecvTimer(InterferencePhy *p) : TimerHandler() { phy_ = p; }
	virtual void expire(Event *e);
protected:
	InterferencePhy *phy_;
};

// rmz
class PhySyncTimer : public TimerHandler {
public:
	PhySyncTimer(InterferencePhy *p) : TimerHandler() { phy_ = p; }
	virtual void expire(Event *e);
protected:
	InterferencePhy *phy_;
};

class InterferencePhy : public WirelessPhy {
	friend class PhyIdleTimer;
	friend class PhyRecvTimer;
	friend class PhySyncTimer; // rmz
public:
	InterferencePhy();
	virtual int command(int argc, const char*const* argv);
	
	virtual void recv(Packet* p, Handler* h);
	virtual int sendUp(Packet *p);
	int is_idle();
	int is_recv();
	int is_send();
	int is_sync();
	inline int best_code() {
		CodedPPM* cppm = dynamic_cast<CodedPPM*>(modulation_);
		return (cppm ? (cppm->bestCode()) : -1);
	}

	inline int& tx_ths() { return (tx_ths_); }
	inline int& rx_ths() { return (rx_ths_); }
	inline int& own_ths() { return (own_ths_); }
	inline int& code_id() { return (code_id_); }
	inline double bitrate() { return bitrate_; }
	inline double ppm_rate(int code) {
		CodedPPM* cppm = dynamic_cast<CodedPPM*>(modulation_);
		return (cppm ? (cppm->ppm_rate(code)) : -1);
	}
	inline void setPt(double p) { Pt_ = p; }
	inline double getPt() { return Pt_; }
	inline double getTransEnd() { return trans_end_; }
	inline double getPreambleTime() { return preamble_time_; }

protected:
	double maxInterferencePower();
	double avgInterferencePower();

	void startRecvPkt(int select); // rmz
	void dropPkt(Packet *p, double start, double end, double Pr);

	int is_right_ths(Packet *p);

	void insertInterferenceListEntry(double start, double end, double power);
 	void insertSyncListEntry(Packet *p, double start, double end, double Pr, int right_ths);
	//void removeInterferenceListEntry(InterferenceListEntry *prev, InterferenceListEntry *current);
	void addPreamble(Packet *p); // rmz
	double txtime(Packet *p);

	void idle_handler();
	void recv_handler();
	void sync_handler(); // rmz

protected:
	//InterferenceListEntry *last_ifpkt;
	InterferenceListEntry *first_ifpkt;      /* head of interference list */
	SyncListEntry *first_syncpkt;            /* rmz: head of sync list */
	int sync_length_;                        /* rmz: length of the sync list */
	Packet *pktRx_;                          /* packet currently being received */
	double rx_start_;                        /* start time of packet reception */
	double rx_end_;                          /* end time of packet reception */
	double trans_end_;                       /* end time of packet reception or transmission */
	double rx_power_;                        /* receive power in W */
	double rx_interference_;                 /* received interference in W */
	double noise_;
	double frequency_range_;                 /* bandwidth in Hz */
	double bitrate_;                         /* coded bitrate on channel */
	int use_timehopping_;                    /* node uses quasi-orthogonal THS */

	double preamble_time_;                   /* rmz */
	double sync_thresh_;                     /* rmz */

	// the THS is set by the MAC layer
	int tx_ths_;                             /* TH sequence used for transmissions */
	int rx_ths_;                             /* TH sequence node is listening on */
	int own_ths_;                            /* node may listen on own TH sequence as well */
	int code_id_;                            /* index of the code (for error correction) */

	PhyState if_state;
	PhyIdleTimer idle_timer;
	PhyRecvTimer recv_timer;
	PhySyncTimer sync_timer;                 /* rmz */
};

#endif /* ns_InterferencePhy_h */

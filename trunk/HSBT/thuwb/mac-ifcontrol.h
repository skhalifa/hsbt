/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
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
 *
 * partly based on mac-802.11
 */

#ifndef ns_mac_ifcontrol_h
#define ns_mac_ifcontrol_h

#include "marshall.h"
#include "mac-802_11.h"
//#include "mac.h"

// JCW
#include "interference-phy.h"
#include "modulation.h"

#define MAC_IF_RTSThreshold		0		// bytes
// reduce CW min/max since they are now per receiver
#define MAC_IF_CWMin			4
#define MAC_IF_CWMax			31
#define MAC_IF_SlotTime			0.000020	// 20us
#define MAC_IF_RATE_CACHE		20		// hosts
#define MAC_IF_RATE_CACHE_TIMEOUT	1		// seconds
#define MAC_IF_RATE_MARGIN              2               // use more powerful code
#define MAC_IF_ShortRetryLimit		7		// retransmittions
#define MAC_IF_LongRetryLimit		7		// retransmissions

// after RTS, wait for txtime of this packet size at lowest rate code
#define MAC_IF_MAX_PACKETSIZE           1500            

// new packet frame type for sender's IDLE signal (-> log as UKWN)
#define MF_SIGIDLE		        0x0022
// and new MAC state
#define MAC_SIGIDLE                     0x2000

// INIT tx_code_(-2), code_id_(-2) ?

/* ======================================================================
   Frame Formats
   ====================================================================== */




struct hdr_mac_ifcontrol {
	int tx_ths_;  // different TH sequences for concurrent transmissions
	int code_id_; // id of the error correcting code

	int best_code;

	u_int32_t scontrol_; // taken from 802_11 header

	int idle_; // currently only used for ACK (to serve as implicit IDLE signal)

	// Header access methods
	static int offset_;
	inline static int& offset() { return offset_; }
	inline static hdr_mac_ifcontrol* access(const Packet* p) {
		return (hdr_mac_ifcontrol*) p->access(offset_);
	}
};


/* ======================================================================
   Timers
   ====================================================================== */
class Mac_IFControl;

class MacTimerIFControl : public Handler {
public:
	MacTimerIFControl(Mac_IFControl* m) : mac(m) {
		busy_ = paused_ = 0; stime = rtime = 0.0;
	}

	virtual void handle(Event *e) = 0;

	virtual void start(double timeout);
	virtual void stop(void);
	virtual void pause(void) { assert(0); }
	virtual void resume(void) { assert(0); }

	inline int busy(void) { return busy_; }
	inline int paused(void) { return paused_; }
	inline double expire(void) {
		return ((stime + rtime) - Scheduler::instance().clock());
	}

protected:
	Mac_IFControl	*mac;
	int		busy_;
	int		paused_;
	Event		intr;
	double		stime;	// start time
	double		rtime;	// remaining time
};


class BackoffTimerIFControl : public MacTimerIFControl {
public:
	BackoffTimerIFControl(Mac_IFControl *m) : MacTimerIFControl(m) {}

	void	start(double timeout);
	void	handle(Event *e);
	void	pause(void);
	void	resume();
};

class TxTimerIFControl : public MacTimerIFControl {
public:
	TxTimerIFControl(Mac_IFControl *m) : MacTimerIFControl(m) {}

	void	handle(Event *e);
};

class IdleTimerIFControl : public MacTimerIFControl {
public:
	IdleTimerIFControl(Mac_IFControl *m) : MacTimerIFControl(m) {}

	void	handle(Event *e);
	void	pause(void);
	void	resume();
};

struct RateCacheEntry {
	int address;
	int rate;
	double time;
};


/* ======================================================================
   Definitions
   ====================================================================== */

class PHY_IF_MIB {
public:
	PHY_IF_MIB(Mac_IFControl *parent);

	inline u_int32_t getCWMin() { return(CWMin); }
        inline u_int32_t getCWMax() { return(CWMax); }
	inline double getSlotTime() { return(SlotTime); }
	inline double getSIFS() { return(SIFSTime); }
	inline double getPIFS() { return(SIFSTime + SlotTime); }
	inline double getDIFS() { return(SIFSTime + 2 * SlotTime); }
	inline double getEIFS() {
		// see (802.11-1999, 9.2.10)
		return(SIFSTime + getDIFS()
                       + (8 *  getACKlen())/PLCPDataRate);
	}
	inline u_int32_t getPreambleLength() { return(PreambleLength); }
	inline double getPLCPDataRate() { return(PLCPDataRate); }
	
	inline u_int32_t getPLCPhdrLen() {
		return((PreambleLength + PLCPHeaderLength) >> 3);
	}

	inline u_int32_t getHdrLen11() {
		return(getPLCPhdrLen() + sizeof(struct hdr_mac802_11)
                       + ETHER_FCS_LEN);
	}
	
	inline u_int32_t getRTSlen() {
		return(getPLCPhdrLen() + sizeof(struct rts_frame));
	}
	
	inline u_int32_t getCTSlen() {
		return(getPLCPhdrLen() + sizeof(struct cts_frame));
	}
	
	inline u_int32_t getACKlen() {
		return(getPLCPhdrLen() + sizeof(struct ack_frame));
	}

 private:
	u_int32_t	CWMin;
	u_int32_t	CWMax;
	double		SlotTime;
	double		SIFSTime;
	u_int32_t	PreambleLength;
	u_int32_t	PLCPHeaderLength;
	double		PLCPDataRate;
};

class MAC_IF_MIB {
public:

	MAC_IF_MIB(Mac_IFControl *parent);

private:
	u_int32_t	RTSThreshold;
	u_int32_t	ShortRetryLimit;
	u_int32_t	LongRetryLimit;
public:
	u_int32_t	FailedCount;	
	u_int32_t	RTSFailureCount;
	u_int32_t	ACKFailureCount;
public:
       inline u_int32_t getRTSThreshold() { return(RTSThreshold);}
       inline u_int32_t getShortRetryLimit() { return(ShortRetryLimit);}
       inline u_int32_t getLongRetryLimit() { return(LongRetryLimit);}
};


/* ======================================================================
   The actual MAC_IFControl class.
   ====================================================================== */
class Mac_IFControl : public Mac {
	friend class BackoffTimerIFControl;
	friend class TxTimerIFControl;
	friend class IdleTimerIFControl;

public:
	Mac_IFControl();
	void		recv(Packet *p, Handler *h);
	virtual int		command(int argc, const char*const* argv);

protected:
	void	backoffHandler(void);
	void	sendHandler(void);
	void	idleHandler(void);
	void    set_ths(Packet *p);
	int     calc_rate(Packet *p);
	void    updateRateCache(int dst, int rate);

	inline virtual void     transmit(Packet* p, double t);

	void	sendCTS(int dst);

	virtual int		check_pktCTRL();
	virtual int		check_pktRTS();

	virtual void	recvRTS(Packet *p);
	virtual void	recvCTS(Packet *p);

	void	tx_resume(void);

	void mac_log(Packet *p) {
		logtarget_->recv(p, (Handler*) 0);
	}

private:
	/*
	 * Called by the timers.
	 */
	//int		check_pktCTRL();
	//int		check_pktRTS();
	int		check_pktTx();

	inline void set_tx_state(MacState x);

	/*
	 * Packet Transmission Functions.
	 */
	void	send(Packet *p, Handler *h);
	void 	sendRTS(int dst);
	//void	sendCTS(int dst);
	void	sendACK(int dst);
	void    sendSIGIDLE(int dst);
	void	sendDATA(Packet *p);
	void	RetransmitRTS();
	void	RetransmitDATA();

	/*
	 * Packet Reception Functions.
	 */
	//void	recvRTS(Packet *p);
	//void	recvCTS(Packet *p);
	void	recvACK(Packet *p);
	void	recvDATA(Packet *p);

	//void	tx_resume(void);

	inline int	is_idle(void);

	/*
	 * Debugging Functions.
	 */
	void		trace_pkt(Packet *p);
	void		dump(char* fname);

	/* JCW
	inline int initialized() {
		return (phymib_ && macmib_ && cache_ && logtarget_ && 
			Mac::initialized());
	}
	*/
	inline int initialized() {
		return (cache_ && logtarget_ && 
			netif_ && uptarget_ && downtarget_);
	}

// 	void mac_log(Packet *p) {
// 		logtarget_->recv(p, (Handler*) 0);
// 	}
	double txtime(Packet *p);
	double txtime(int psz, int channel_code);
	double txtime(int bytes) { return txtime(bytes, MODULATION_NUM_CODES - 1); }

	inline void inc_cw() {
		cw_ = (cw_ << 1) + 1;
		if(cw_ > phymib_.getCWMax())
			cw_ = phymib_.getCWMax();
	}
	inline void rst_cw() { cw_ = phymib_.getCWMin(); }
	inline double sec(double t) { return(t *= 1.0e-6); }
	inline u_int16_t usec(double t) {
		u_int16_t us = (u_int16_t)floor((t *= 1e6) + 0.5);
		return us;
	}

protected:
	PHY_IF_MIB		phymib_;
	MAC_IF_MIB		macmib_;

	// pointer to PHY layer to access is_idle(), best_code(), and bitrate_()
	InterferencePhy *netif_;            // network interface

	/*
	 * Mac Timers
	 */
	TxTimerIFControl	 mhSend_;	// outgoing packets
	BackoffTimerIFControl	 mhBackoff_;	// backoff timer
	IdleTimerIFControl	 mhIdle_;	// wait for IDLE signal


	/* ============================================================
	   Internal MAC State (accessible for the sub-class)
	   ============================================================ */
	MacState	tx_state_;	// outgoint state
	Packet		*pktRTS_;	// outgoing RTS packet
	Packet		*pktCTRL_;	// outgoing non-RTS packet

	int             fixed_code;     // no code adaptation

	float		default_power_;

private:

	/* ============================================================
	   Internal MAC State
	   ============================================================ */

	//MacState	tx_state_;	// outgoint state

	//Packet		*pktRTS_;	// outgoing RTS packet
	//Packet		*pktCTRL_;	// outgoing non-RTS packet

	int packet_rate;                // channel code for next data packet

	u_int32_t	cw_;		// Contention Window
	u_int32_t	ssrc_;		// STA Short Retry Count
	u_int32_t	slrc_;		// STA Long Retry Count

	//int             fixed_code;     // no code adaptation

	int		min_frame_len_;

	MacState        last_action;    // send/receive/idle (only for OPTIMIZE_FORWARDING)
	int             last_mac;       // MAC address with which the last rx/tx was (only for OPTIMIZE_FORWARDING)
	double          last_time;      // time when last rx/tx happened
	int             wait_for_idle;  // wait for IDLE instead of directly sending packet
	int             forward;        // rmz
	
	// HACK for CACDMA
	//float		data_power_;
	//float		default_power_;

	NsObject*	logtarget_;

	/* ============================================================
	   Duplicate Detection state
	   ============================================================ */
	u_int16_t	sta_seqno_;	// next seqno that I'll use
	int		cache_node_count_;
	Host		*cache_;

	/* Code rate cache */
	struct RateCacheEntry rate_cache[MAC_IF_RATE_CACHE];
};

#endif /* __mac_ifcontrol_h__ */

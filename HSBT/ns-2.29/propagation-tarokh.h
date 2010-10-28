/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
 * Copyright (c) 2004, Joerg Widmer, EPFL
 * Copyright (c) 2005,2006, Ruben Merz, EPFL
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
 */


#ifndef proptarokh_h
#define proptarokh_h

#include <packet-stamp.h>
#include <wireless-phy.h>
#include <propagation.h>
#include <rng.h>


class PropTarokh : public Propagation {
public:
	PropTarokh();
	~PropTarokh();
	virtual double Pr(PacketStamp *tx, PacketStamp *rx, WirelessPhy *ifp);
	virtual double getDist(double Pr, double Pt, double Gt, double Gr,
			       double hr, double ht, double L, double lambda);
	virtual int command(int argc, const char*const* argv);

protected:

	RNG *ranVar;	// random number generator for normal distribution
	double average_pathlossExp_;	// path-loss exponent
	double std_db_;		// shadowing deviation (dB),
	double dist0_;	// close-in reference distance
	int seed_;	// seed for random number generator
	double sigma_g_; //model parameters
	double mi_s_;
	double sigma_s_;

	double n1_low_; //range limits coefficients for random pathloss components
	double n1_high_;
	double n2_low_;
	double n2_high_;
	double n3_low_;
	double n3_high_;

};

#endif


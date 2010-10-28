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

#ifndef __modulation_codedppm_h__
#define __modulation_codedppm_h__

#include "packet.h"
#include "modulation.h"
#include <assert.h>

#define MODULATION_NUM_CODES 30
#define PPM_CONVERSION_COEFFICIENT (1.0/2.8)

// P_DELTA is delta * frequency range from Durisi
#define P_DELTA 0.1984
#define GAMMA -0.6183

class CodedPPM : public Modulation {

public:
	virtual int BitError(double Pr);
	virtual int BitError(double receivedPower, double interferencePower, double noisePower, Packet* p);
	inline int bestCode() { return best_code_; }

	inline double ppm_rate(int code) { 
		assert((code >= 0) && (code < MODULATION_NUM_CODES));
		return PPM_rate_table[code];
	}

protected:
	int calcBestCode(int code, double SNR, double x, int pktsize);
	double SNRtoBER(double SNR, int code);

	int best_code_;
	double SNR_;

	static double PPM_rate_table[];

private:
	// Probability of 1 bit error
	virtual double ProbBitError(double Pr);

	// Probability of n bit errors
	virtual double ProbBitError(double Pr, int n);

};

#endif /* __modulation_codedppm_h__ */


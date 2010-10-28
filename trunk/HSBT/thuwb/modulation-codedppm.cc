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

/* ======================================================================
   Pulse Position Modulation with Punctured Codes
   ====================================================================== */

#define PPM_TABLE_ROWS 41

//#define CODEDPPM_DEBUG

#include <math.h>
#include <stdlib.h>
#include <random.h>
#include "modulation.h"
#include "modulation-codedppm.h"
#include "packet.h"
#include "mac.h"
#include "mac-ifcontrol.h"

#include "ppm.cc"

double CodedPPM::PPM_rate_table[MODULATION_NUM_CODES] = {
 8.0/9.0,
 8.0/10.0,
 8.0/11.0,
 8.0/12.0,
 8.0/13.0,
 8.0/14.0,
 8.0/15.0,
 8.0/16.0,
 8.0/17.0,
 8.0/18.0,
 8.0/19.0,
 8.0/20.0,
 8.0/21.0,
 8.0/22.0,
 8.0/23.0,
 8.0/24.0,
 8.0/25.0,
 8.0/26.0,
 8.0/27.0,
 8.0/28.0,
 8.0/29.0,
 8.0/30.0,
 8.0/31.0,
 8.0/32.0,
 1.0/5.0,
 1.0/6.0,
 1.0/7.0,
 1.0/8.0,
 1.0/9.0,
 1.0/10.0
};

/*
static double PPM_rate_table[MODULATION_NUM_CODES] = {
	8.0/9.0, 4.0/5.0,  2.0/3.0,  4.0/7.0,
	1.0/2.0, 4.0/9.0,  4.0/10.0, 4.0/11.0,
	1.0/3.0, 4.0/13.0, 2.0/7.0,  4.0/15.0,
	1.0/4.0
};
*/

static class ModulationCodedPPMClass: public TclClass {
public:
        ModulationCodedPPMClass() : TclClass("Modulation/CodedPPM") {}
        TclObject* create(int, const char*const*) {
		return (new CodedPPM);
	}
} class_ModulationCodedPPM;

double
CodedPPM::SNRtoBER(double SNR, int code)
{
	double ber = 1.0;
	int row = 0;

	// outside table
	if (SNR < PPM_ber_table[code][0][0])
		return 1.0;

	// determine table row (SNR)
	while ((PPM_ber_table[code][row][0] < SNR) && (row < PPM_TABLE_ROWS))
		++row;

	// determine success for each of the codes with the given SNR
	if (row == 0)
		ber = PPM_ber_table[code][0][1];
	else if (row == PPM_TABLE_ROWS)
		ber = PPM_ber_table[code][PPM_TABLE_ROWS-1][1];
	else // linear interpolation
		ber = PPM_ber_table[code][row-1][1] + 
			(PPM_ber_table[code][row-1][1] - PPM_ber_table[code][row][1]) *
			(SNR - PPM_ber_table[code][row-1][0]) / (PPM_ber_table[code][row-1][0] - PPM_ber_table[code][row][0]);
	return ber;
}

int
CodedPPM::calcBestCode(int code, double SNR, double x, int pktsize) {
	double per, ber;
	int best = code;

	do {
		--best;
		if (best < 0)
			break;
		ber = SNRtoBER(SNR, best);
		per = 1.0 - pow(1 - ber, pktsize * 8.0);
	} while (x >= per);
	// the last one didn't work so the best code is the next more powerful one
	best += 1;
	if (best > MODULATION_NUM_CODES - 1)
		best = MODULATION_NUM_CODES - 1;
	return best;
}

int
CodedPPM::BitError(double receivedPower, double interferencePower, double noisePower, Packet* p)
{
	hdr_cmn *hdr = HDR_CMN(p);
	hdr_mac_ifcontrol *hdr_ifc = HDR_MAC_IFControl(p);

	double bitErrorProbability, packetErrorProbability;
	int numErrors, code;

	code = hdr_ifc->code_id_;

	//XXX: interferencePower multiplied by 1/Tf (== bitrate_) at PHY (only for CodedPPM)
	SNR_ = receivedPower / (P_DELTA * (1.0 - GAMMA)*(1.0 - GAMMA) * interferencePower + 2 * noisePower * (1.0 - GAMMA));

	// convert to dB
	if (SNR_ > 0) {
		SNR_ = 10 * log10(SNR_);
	} else {
		// make sure table lookup fails
		SNR_ = -10000000;
	}


	//double y = sqrt(SNR/2);
	//double bitErrorProbability = 0.5*erfc(y);

	bitErrorProbability = SNRtoBER(SNR_, code);
	packetErrorProbability = 1.0 - pow(1 - bitErrorProbability, hdr->size() * 8.0);

	double x = Random::uniform();
	if (x < packetErrorProbability) {
		numErrors = 1;
		best_code_ = -1; // "best code" invalid since packet had errors
	} else {
		numErrors = 0;
		best_code_ = calcBestCode(code, SNR_, x, hdr->size());
	}

#ifdef CODEDPPM_DEBUG
	struct hdr_mac *mh = HDR_MAC(p);
	printf("%.9f %i - %i (no. %i) SNR %.3f [Sig %.2f, Intf %.2f dBm], BER %.3e PER %.4f (%i bytes) %s code (%i/%i)\n",
	       Scheduler::instance().clock(), mh->macSA_, mh->macDA_, hdr->uid(), SNR_, 
	       10.0 * log10(receivedPower) + 30, 10.0 * log10(interferencePower) + 30,
	       bitErrorProbability, packetErrorProbability, hdr->size(),
	       numErrors == 0 ? "ok" : "err", code, best_code_);
#endif

	return numErrors;
}

/* not used! */
int
CodedPPM::BitError(double Pr)
{
	return 0;		// no bit errors
}

double
CodedPPM::ProbBitError(double)
{
	double Pe = 0.0;

	return Pe;
}

double
CodedPPM::ProbBitError(double, int)
{
	double Pe = 0.0;

	return Pe;
}

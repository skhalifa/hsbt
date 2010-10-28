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
 */


#include <math.h>

#include <delay.h>
#include <packet.h>

#include <packet-stamp.h>
#include <antenna.h>
#include <mobilenode.h>
#include <propagation.h>
#include <wireless-phy.h>
#include <propagation-tarokh.h>


static class PropTarokhClass: public TclClass {
public:
	PropTarokhClass() : TclClass("Propagation/Tarokh") {}
	TclObject* create(int, const char*const*) {
		return (new PropTarokh);
	}
} class_proptarokh;


PropTarokh::PropTarokh()
{
	bind("average_pathlossExp_", &average_pathlossExp_);
	bind("std_db_", &std_db_);
	bind("dist0_", &dist0_);
	bind("seed_", &seed_);

	bind("sigma_g_", &sigma_g_);
	bind("mi_s_", &mi_s_);
	bind("sigma_s_", &sigma_s_);

	bind("n1_low_", &n1_low_);
	bind("n1_high_", &n1_high_);
	bind("n2_low_", &n2_low_);
	bind("n2_high_", &n2_high_);
	bind("n3_low_", &n3_low_);
	bind("n3_high_", &n3_high_);


	ranVar = new RNG;
	ranVar->set_seed(RNG::PREDEF_SEED_SOURCE, seed_);
}


PropTarokh::~PropTarokh()
{
	delete ranVar;
}


double
PropTarokh::Pr(PacketStamp *t, PacketStamp *r, WirelessPhy *ifp)
{
	double L = ifp->getL();		// system loss
	double lambda = ifp->getLambda();   // wavelength

	double Xt, Yt, Zt;		// loc of transmitter
	double Xr, Yr, Zr;		// loc of receiver

	t->getNode()->getLoc(&Xt, &Yt, &Zt);
	r->getNode()->getLoc(&Xr, &Yr, &Zr);

	// Is antenna position relative to node position?
	Xr += r->getAntenna()->getX();
	Yr += r->getAntenna()->getY();
	Zr += r->getAntenna()->getZ();
	Xt += t->getAntenna()->getX();
	Yt += t->getAntenna()->getY();
	Zt += t->getAntenna()->getZ();

	double dX = Xr - Xt;
	double dY = Yr - Yt;
	double dZ = Zr - Zt;

	double dist = sqrt(dX * dX + dY * dY + dZ * dZ);

	// get antenna gain
	double Gt = t->getAntenna()->getTxGain(dX, dY, dZ, lambda);
	double Gr = r->getAntenna()->getRxGain(dX, dY, dZ, lambda);
	double Pr = 0;
	
	// reference distance has ~ 47db
	double Pr0 = Friis(t->getTxPr(), Gt, Gr, lambda, L, dist0_);
	
	if (dist < dist0_) {
		// Free Space below reference distance
		Pr = Friis(t->getTxPr(), Gt, Gr, lambda, L, dist);
		if (Pr > t->getTxPr())
			Pr = t->getTxPr();
	} else {
		//double powerLoss_db = avg_db + ranVar->normal(0.0, std_db_);
       
		double average_powerLoss_db = 10.0 * average_pathlossExp_ * log10(dist/dist0_); //10 * mi_gamma * log_10 (d) in ref.

		double n1 = ranVar->normal(0,1);
		while ( n1 < n1_low_ || n1 > n1_high_)
			n1 = ranVar->normal(0,1);
		double n2 = ranVar->normal(0,1);
		while ( n2 < n2_low_ || n2 > n2_high_)
			n2 = ranVar->normal(0,1);
		double n3 = ranVar->normal(0,1);
		while ( n3 < n3_low_ || n3 > n3_high_)
			n3 = ranVar->normal(0,1);
		
	
 		double random_powerLoss_db = 10 * n1 * sigma_g_ * log10(dist/dist0_) + n2 * mi_s_ + n2 * n3 * sigma_s_;
	
		// calculate the receiving power at dist
	
		Pr = Pr0 / pow(10.0, (average_powerLoss_db + random_powerLoss_db)/10.0);
	}	
	return Pr;
}

double
PropTarokh::getDist(double Pr, double Pt, double Gt, double Gr,
		    double hr, double ht, double L, double lambda)
{
	double loss_db, dist;
	//double loss_db = 10*log10(Pt * Gt * Gr * (hr * hr * ht * ht)/Pr);
	loss_db = 10*log10(Pt/(L*Pr));
	dist = pow(10.0, loss_db/(10.0*average_pathlossExp_));
	//fprintf(stdout, "propagation-tarokh.cc: loss %0.1f, dist %0.1f (%0.1f)\n", loss_db, dist, average_pathlossExp_);
	return dist;
}

int
PropTarokh::command(int argc, const char* const* argv)
{
	if (argc == 4) {
		if (strcmp(argv[1], "seed") == 0) {
			int s = atoi(argv[3]);
			if (strcmp(argv[2], "raw") == 0) {
				ranVar->set_seed(RNG::RAW_SEED_SOURCE, s);
			} else if (strcmp(argv[2], "predef") == 0) {
				ranVar->set_seed(RNG::PREDEF_SEED_SOURCE, s);
				// s is the index in predefined seed array
				// 0 <= s < 64
			} else if (strcmp(argv[2], "heuristic") == 0) {
				ranVar->set_seed(RNG::HEURISTIC_SEED_SOURCE, 0);
			}
			return(TCL_OK);
		}
	}
	
	return Propagation::command(argc, argv);
}


Baseband set debug_ 0
L2CAP set debug_ 0
LMP set debug_ 0
Mac/BNEP set debug_ 0
SDP set debug_ 0
A2MP set debug_ 0
PAL/802_11 set debug_ 0 
BTChannel set debug_ 0

Baseband set T_w_inquiry_scan_ 0
Baseband set T_inquiry_scan_ 0
Baseband set T_w_page_scan_ 0
Baseband set T_page_scan_ 0
Baseband set inquiryTO_ 0
Baseband set inq_max_num_responses_ 0
Baseband set pageTO_ 0
Baseband set SR_mode_ 0
Baseband set N_page_ 0
Baseband set N_inquiry_ 0
Baseband set backoffParam_ 0
Baseband set backoffParam_s_ 0
Baseband set Page_Scan_Type_ 0
Baseband set Inquiry_Scan_Type_ 0
Baseband set ver_ 0
Baseband set driftType_ 0
Baseband set clkdrift_ 0
Baseband set collisionDist_ 0

Baseband set useDynamicTpoll_ 0
Baseband set pollReserveClass_ 0

#Baseband set pmEnabled_ 0
#Baseband set energy_ 0
#Baseband set activeEnrgConRate_ 0
#Baseband set energyMin_ 0
#Baseband set activeTime_ 0
#Baseband set startTime_ 0
#Baseband set numTurnOn_ 0
#Baseband set warmUpTime_ 0

Baseband set energyMin_ 0.1
Baseband set energy_ 1
Baseband set activeEnrgConRate_ 1E-4
Baseband set activeTime_ 0
Baseband set startTime_ 0
Baseband set warmUpTime_ 0.0002
Baseband set numTurnOn_ 0
Baseband set trxTurnOnTime_ 0

#Baseband set ext_inqconn_ 0

LMP set T_w_inquiry_scan_ 0
LMP set T_inquiry_scan_ 0
LMP set T_w_page_scan_ 0
LMP set T_page_scan_ 0
LMP set giac_ 0
LMP set inq_max_period_length_ 0
LMP set inq_min_period_length_ 0
LMP set inquiry_length_ 0
LMP set inq_num_responses_ 0
LMP set nb_timeout_ 0
LMP set nb_dist_ 0
LMP set takeover_ 0
LMP set scan_after_inq_ 0
LMP set NPage_manual_ 0
LMP set NInqury_manual_ 0

LMP set defaultTSniff_ 0
LMP set defaultSniffAttempt_ 0
LMP set defaultSniffTimeout_ 0

LMP set supervisionTO_ 0
LMP set supervisionEnabled_ 0
LMP set idleSchred_ 0
LMP set defaultHoldTime_ 0
LMP set minHoldTime_ 0
LMP set maxHoldTime_ 0
LMP set autoOnHold_ 0
LMP set idleCheckEnabled_ 0
LMP set idleCheckIntv_ 0
LMP set nullTriggerSchred_ 0
LMP set failTriggerSchred_ 0

LMP set defaultPktType_ 0
LMP set defaultRecvPktType_ 0
LMP set allowRS_ 0

LMP set pageStartTO_ 0
LMP set inqStartTO_ 0
LMP set scanStartTO_ 0

LMP set scanWhenOn_ 0
LMP set lowDutyCycle_ 0

L2CAP set ifq_limit_ 0

Agent/SCO set packetType_ 0
Agent/SCO set initDelay_ 0

Mac/BNEP set onDemand_ 0
Node/BTNode set enable_clkdrfit_in_rp_ 0
Node/BTNode set randomizeSlotOffset_ 0
Node/BTNode set initDelay_ 0
Node/BTNode set X_ 0
Node/BTNode set Y_ 0
Node/BTNode set Z_ 0

A2MP set ampNumber_ 0

Node/BTNode instproc init args {
        $self instvar mac_ bnep_ sdp_ a2mp_ l2cap_ lmp_ bb_ phy_ ll_ \
		arptable_ classifier_ dmux_ entry_ ragent_ \
		energyModel_ initialEnergy_ txPower_ rxPower_ \
	    	idlePower_ sleepPower_ transitionPower_ transitionTime_ \
	    	level1_ level2_
	
        eval $self next $args

	set bnep_ [new Mac/BNEP]
	set mac_ $bnep_
	set sdp_ [new SDP]
	set a2mp_ [new A2MP]
	set l2cap_ [new L2CAP]
	set lmp_ [new LMP]
	set bb_ [new Baseband]
	set phy_ [new BTChannel]
	set ll_ [new LL]

	set arptable_ [new ARPTable $self $bnep_]	;# not used

	$ll_ mac $bnep_
	# $ll_ arptable $arptable_	
	$ll_ up-target [$self entry]

	$ll_ down-target $bnep_
	$bnep_ up-target $ll_
	$bnep_ down-target $l2cap_
	$l2cap_ up-target $bnep_
	$l2cap_ down-target $lmp_
	$lmp_ up-target $l2cap_
#	$lmp_ down-target $bb_
	$bb_ up-target $lmp_
	$bb_ down-target $phy_
	$phy_ up-target $bb_

	set ns [Simulator instance]

	set namtracefd [$ns get-nam-traceall]
	if {$namtracefd != "" } {
	    $self namattach $namtracefd
	    puts $namtracefd "n -t * -s [AddrParams addr2id $args] -x [$self set X_] -y [$self set Y_] -Z 0 -z 0.6  -v circle -c black"
	}

	set tracefd [$ns get-ns-traceall]
	if {[Simulator set MacTrace_] == "ON" && $tracefd != "" } {
		# puts "tracefile : $tracefd"
		set sndT [bt-trace Send MAC $self]
		$bb_ down-target $sndT
		$sndT target $phy_
		$mac_ drop-target [bt-trace Drop MAC $self]
		set rcvT [bt-trace Recv MAC $self]
		$phy_ up-target $rcvT
		$rcvT target $bb_

		if {$namtracefd != "" } {
		    $sndT namattach $namtracefd
		    $rcvT namattach $namtracefd
		}
	}

	$classifier_ defaulttarget $ll_
	
	########################## Setup up the Node ############################
	$self setup [AddrParams addr2id $args] $phy_ $bb_ $lmp_ $l2cap_ $bnep_ $sdp_ $a2mp_
}


##############Start Add 802.11 interface#################
# How to add PAL to the bluetooth node
#
#set val(chan)   Channel/WirelessChannel    ;# channel type
#set val(prop)   Propagation/TwoRayGround   ;# radio-propagation model
#set val(palType) PAL/802_11
#set topo       [new Topography]
#set chan [new $val(chan)];#Create wireless channel
#$node add-PAL $val(palType) $topo $chan $val(prop)
		
Node/BTNode instproc add-PAL {palType version topo channel pmodel \
				txPower_ rxPower_ idlePower_ sleepPower_ \
				transitionPower_ transitionTime_ } {
	if {$palType == "PAL/802_11"} {
		if { $version != "802.11b" && $version != "802.11g"} {
			error "Currently only these 802.11 versions are supported : 802.11b and 802.11g"
		}
		
		set a2mp_ [$self set a2mp_]
		set l2cap_ [$self set l2cap_]
		set bnep_ [$self set bnep_]
		
		set ns [Simulator instance]
		set imepflag [$ns imep-support]
		
		set ifq		[new Queue/DropTail/PriQueue]		;# interface queue
		set pal_	[new $palType]
		set prop	[new $pmodel]
		set netif	[new Phy/WirelessPhy]		;# interface
		set mac		[new Mac/802_11]		;# mac layer
        	set ant		[new Antenna/OmniAntenna]
		
		
			#
	# Interface Queue
	#
	$ifq target $mac
	$ifq set limit_ 99999999
	#$pal_ down-target $ifq
	#$ifq drop-target $drpT
		if { $version == "802.11b" } {
########################################################## 802.11b #######################################
#		The following parameters are set to simulate 802.11b as specified in http://www.joshuarobinson.net/docs/ns-802_11b.html
#		and Simulate 802.11b Channel within NS2  by Wu Xiuchao, SOC, NUS wuxiucha@comp.nus.edu.sg (http://www.comp.nus.edu.sg/~wuxiucha/research/reactive/publication/Simulate80211ChannelWithNS2.pdf)
		$ant set Gt_ 1 ;#//Transmit antenna gain
		$ant set Gr_ 1 ;#//Receive antenna gain
		$netif set L_ 1.0 ;#//System Loss Factor
		$netif set freq_ 2.462e9 ;#//channel-11. 2.463GHz
		$netif set bandwidth_ 11Mb ;#//Data Rate
		$netif set Pt_ 0.031622777 ;#//Transmit Power
		$netif set CPThresh_ 10.0 ;#//Collision Threshold
		$netif set CSThresh_ 5.011872e-12 ;#//Carrier Sense Power
		$netif set RXThresh_ 5.82587e-09 ;#//Receive Power Threshold; calculated under TwoRayGround model by tools from NS2.
		$mac set ShortRetryLimit_       7               ;# retransmittions
		$mac set LongRetryLimit_        4               ;# retransmissions
#		Almost all commercial 802.11b cards have the RTS/CTS exchange turned off by default. 
#		This is not a bad decision since I think most people's home wlan networks are simple enough 
#		so that the RTS/CTS really is just unnecessary overhead. 
#		NS by default has this feature turned on, so we probably want to tell NS not to use this feature. 
#		This means that an RTS will only be sent for packets that are bigger than 3000 bytes, which should be never. 
#		Note: if you want RTS/CTS on, then set this value to zero.
		$mac set RTSThreshold_ 3000 ;#send RTS for packets larger than 3000 byte (disable RTS)
#		Every packet is sent with a preamble, which is just a known pattern of bits at the beginning of the packet 
#		so that the receiver can sync up and be ready for the real data. 
#		This preamble must be sent at the basic rate (1 Mbps), according to the official standard. 
#		But there are two different kinds of preambles, short and long - referring to the length of the sync field. 
#		The long preamble has a field size of 128 bits, while the short preamble is only 56 bits. 
#		I would guess this short preamble option came about as hardware progressed and transceivers got better at locking on to a signal. 
#		NS is set by default to use the long preamble. 
#		To support short preambles in NS.
#		Note: there are 16 other bits in the preamble that aren't affected by the short/long distinction. 
#		To go back to long, change this value to 144.
		$mac set PreambleLength_        72             ;# 72 bit
#		NS, by default, has the data rate for the MAC set at 2 Mbps. 
#		But cards are faster now. My cards are 802.11b, which means they're 11 Mbps, 
#		The card can send at 1, 2, 5.5, or 11 Mbps.  
#		Mobile nodes will always send their packets at dataRate_. 
		$mac set dataRate_ 11Mb
		$mac set basicRate_ 1Mb ;#Rate for Control Frames
########################################################## End 802.11b #######################################
		} elseif { $version == "802.11g"} {
########################################################## 802.11g #######################################		
		$mac set SlotTime_              0.000020  ;# 20us
		$mac set SIFS_                  0.000009  ;# 9us
		$mac set RTSThreshold_          3000      ;# bytes
		$mac set ShortRetryLimit_      7        ;# retransmissions
		$mac set LongRetryLimit_        4        ;# retransmissions                                               
		$mac set dataRate_ 54Mb                  ;# 802.11 data transmission rate
		$mac set basicRate_ 6Mb                  ;# 802.11 basic transmission rate
		$mac set amc_ 1                          ;# 802.11 Multirate_speed
		
		# --- Configuration PHY 802.11g ---
		$ant set Gt_    1              ;# transmitter antenna gain
		$ant set Gr_    1              ;# receiver antenna gain
		$netif set freq_      2.472e9        ;# channel frequency (Hz)
		$netif set L_          1.0
		$mac set CWMin_                15
		$mac set CWMax_                1023
		$mac set PreambleLength_        96        ;# 96 bit
		$mac set PLCPHeaderLength_      24        ;# 24 bits
		$mac set PLCPDataRate_          6.0e6    ;# 1Mbps
########################################################## End 802.11g #######################################	
		}
		set namfp [$ns get-nam-traceall]
		
		######### I have no idea what are the following lines needed for ##########
		# errProc_ and FECProc_ are an option unlike other 
	        # parameters for node interface
        	$ns instvar inerrProc_ outerrProc_ FECProc_
		if ![info exist inerrProc_] {
			set inerrProc_ ""
		}
		if ![info exist outerrProc_] {
			set outerrProc_ ""
		}
		if ![info exist FECProc_] {
			set FECProc_ ""
		}
		set inerr ""
		if {$inerrProc_ != ""} {
			set inerr [$inerrProc_]
		}
		set outerr ""
		if {$outerrProc_ != ""} {
			set outerr [$outerrProc_]
		}
		set fec ""
		if {$FECProc_ != ""} {
			set fec [$FECProc_]
		}
		###########################################################################
		
		###############################IMEP Beaconing #############################
#	        if {$imepflag == "ON" } {              
#			# IMEP layer
#			set imep [new Agent/IMEP [$btnode_ id]]
#			set drpT [$self mobility-trace Drop "RTR"]
#			if { $namfp != "" } {
#				$drpT namattach $namfp
#			}
#			$imep drop-target $drpT
#			$ns at 0.[$btnode_ id] "$imep start"   ;# start beacon timer
#	        }
#		if {$imepflag == "ON" } {
#			$imep recvtarget [$self entry]
#			$imep sendtarget $ll
#			$ll up-target $imep
#	        } else {
#			$ll up-target [$self entry]
#		}
		#############################################################################
		#
		# Local Variables
		#
		set nullAgent_ [$ns set nullAgent_]
		
		
		#
		# A2MP Layer
		#
		$a2mp_ add-pal $pal_
		
		#
		# PAL Layer
		#
		#$pal_($t) up-target $l2cap_
		#$pal_($t) down-target $mac
		$pal_ l2cap $l2cap_
		$pal_ mac $mac
		$pal_ btnode $self
		$pal_ a2mp $a2mp_
		$pal_ netif $netif
		$pal_ ifq $ifq
	
		#
		# Interface Queue
		# TODO : do those changes by adding another interface queue to the L2CAP
	#	$ifq target $mac
	#	$ifq set limit_ $qlen
	#	if {$imepflag != ""} {
	#		set drpT [$self mobility-trace Drop "IFQ"]
	#	} else {
	#		set drpT [cmu-trace Drop "IFQ" $self]
	#        }
	#	$ifq drop-target $drpT
	#	if { $namfp != "" } {
	#		$drpT namattach $namfp
	#	}
	#	if {[$ifq info class] == "Queue/XCP"} {		
	#		$mac set bandwidth_ [$ll set bandwidth_]
	#		$mac set delay_ [$ll set delay_]
	#		$ifq set-link-capacity [$mac set bandwidth_]
	#		$ifq queue-limit $qlen
	#		$ifq link $ll
	#		$ifq reset
	#		
	#	}
	
		#
		# Mac Layer
		#

		
		$mac up-target $pal_
		$mac netif $netif
		
	
		if {$outerr == "" && $fec == ""} {
			$mac down-target $netif
		} elseif {$outerr != "" && $fec == ""} {
			$mac down-target $outerr
			$outerr target $netif
		} elseif {$outerr == "" && $fec != ""} {
			$mac down-target $fec
			$fec down-target $netif
		} else {
			$mac down-target $fec
			$fec down-target $outerr
			$err target $netif
		}
		#Implement the god for bluetooth nodes
#		Quoted from CMU document on god, "God (General Operations Director) is the object 
#		that is used to store global information about the state of the environment, 
#		network or nodes that an omniscent observer would have, but that should not be made known 
#		to any participant in the simulation." Currently, God object stores the total number of mobilenodes 
#		and a table of shortest number of hops required to reach from one node to another. 
#		The next hop information is normally loaded into god object from movement pattern files,
#		before simulation begins, since calculating this on the fly during simulation runs can be quite time consuming.
#		However, in order to keep this example simple we avoid using movement pattern files and 
#		thus do not provide God with next hop information. The usage of movement pattern files and 
#		feeding of next hop info to God shall be shown in the example in the next sub-section.
#
#		The procedure create-god is defined in ~ns/tcl/mobility/com.tcl, 
#		which allows only a single global instance of the God object to be created during a simulation. 
#		In addition to the evaluation functionalities, the God object is called internally by MAC objects in mobilenodes.
#		So even though we may not utilise God for evaluation purposes,(as in this example) we still need to create God. 

			set god_ [God instance]
		       # if {$mactype == "Mac/802_11"} {
				$mac nodes [$god_ num_nodes]
			#}
		
		
		#
		# Network Interface
		#
		#if {$fec == ""} {
	        #		$netif up-target $mac
		#} else {
	        #		$netif up-target $fec
		#	$fec up-target $mac
		#}
		

		$netif channel $channel
		if {$inerr == "" && $fec == ""} {
			$netif up-target $mac
		} elseif {$inerr != "" && $fec == ""} {
			$netif up-target $inerr
			$inerr target $mac
		} elseif {$err == "" && $fec != ""} {
			$netif up-target $fec
			$fec up-target $mac
		} else {
			$netif up-target $inerr
			$inerr target $fec
			$fec up-target $mac
		}
	
		$netif propagation $prop	;# Propagation Model
		#$netif node $pal_		;#Test: add the interface to the 802_11PAL which extends mobilenode Bind PAL <---> interface
		$netif node $self		;# Bind node <---> interface (checked (mac/wirelessphy.cc): OK)
		$netif antenna $ant
		$netif NodeOff
		
		
		#
		# Network interface energy model
		#
		# set transmission power
	        if [info exists txPower_] {
			$netif setTxPower $txPower_
	        }
		# set receiving power
	        if [info exists rxPower_] {
			$netif setRxPower $rxPower_
	        }
		# set idle power -- Chalermek
	        if [info exists idlePower_] {
			$netif setIdlePower $idlePower_
	        }
	#
		if [info exists sleepPower_] {
			$netif setSleepPower $sleepPower_
	        }
		if [info exists transitionPower_] {
			$netif setTransitionPower $transitionPower_
	        }
		if [info exists transitionTime_] {
			$netif setTransitionTime $transitionTime_
	        }
		
		#
		# Physical Channel
		#
		$channel addif $netif
		
	        # List-based improvement
		# For nodes talking to multiple channels this should
		# be called multiple times for each channel
		$channel add-node $self	;#FIXED need testing : (mac/channel.cc)could lead to a problem as it deals 
						;#with mobile nodes WirelessChannel::addNodeToList(MobileNode *mn)	
	
		# let topo keep handle of channel
		$topo channel $channel
		# ============================================================
#			set tracefd [$ns get-ns-traceall]
#	if {[Simulator set MacTrace_] == "ON" && $tracefd != "" } {
#		# puts "tracefile : $tracefd"
#		set sndT [bt-trace Send MAC $self]
#		$bnep_ down-target $sndT
#		$sndT target $phy_
#		$mac_ drop-target [bt-trace Drop MAC $self]
#		set rcvT [bt-trace Recv MAC $self]
#		$phy_ up-target $rcvT
#		$rcvT target $bb_
#
#		if {$namtracefd != "" } {
#		    $sndT namattach $namtracefd
#		    $rcvT namattach $namtracefd
#		}
#	}
#	###################log#######################

		if { [Simulator set MacTrace_] == "ON" } {
			#
			# Trace RTS/CTS/ACK Packets
			#
			if {$imepflag != ""} {
				set rcvT [$self mobility-trace Recv "MAC"]
			} else {
				set rcvT [bt-trace Recv "MAC" $self]
				#set rcvT [cmu-trace Recv "MAC" $self]
				#set rcvT [$self mobility-trace Recv "MAC"]
			}
			$mac log-target $rcvT
			if { $namfp != "" } {
				$rcvT namattach $namfp
			}
			#
			# Trace Sent Packets
			#
			if {$imepflag != ""} {
				set sndT [$self mobility-trace Send "MAC"]
			} else {
				set sndT [bt-trace Send "MAC" $self]
				#set sndT [cmu-trace Send "MAC" $self]
				#set sndT [$self mobility-trace Send "MAC"]
			}
			$sndT target [$mac down-target]
			$mac down-target $sndT
			if { $namfp != "" } {
				$sndT namattach $namfp
			}
			#
			# Trace Received Packets
			#
			if {$imepflag != ""} {
				set rcvT [$self mobility-trace Recv "MAC"]
			} else {
				set rcvT [bt-trace Recv "MAC" $self]
				#set rcvT [cmu-trace Recv "MAC" $self]
				#set rcvT [$self mobility-trace Recv "MAC"]
			}
			$rcvT target [$mac up-target]
			$mac up-target $rcvT
			if { $namfp != "" } {
				$rcvT namattach $namfp
			}
			#
			# Trace Dropped Packets
			#
			if {$imepflag != ""} {
				set drpT [$self mobility-trace Drop "MAC"]
			} else {
				set drpT [bt-trace Drop "MAC" $self]
				#set drpT [cmu-trace Drop "MAC" $self]
				#set drpT [$self mobility-trace Drop "MAC"]
			}
			$mac drop-target $drpT
			if { $namfp != "" } {
				$drpT namattach $namfp
			}
		} else {
			$mac log-target [$ns set nullAgent_]
			$mac drop-target [$ns set nullAgent_]
		}
	###################End log#######################
	# change wrt Mike's code
	       if { [Simulator set EotTrace_] == "ON" } {
	               #
	               # Also trace end of transmission time for packets
	               #
	
	               if {$imepflag != ""} {
	                       set eotT [$self mobility-trace EOT "MAC"]
	               } else {
	                       set eoT [bt-trace EOT "MAC" $self]
			       #set eoT [cmu-trace EOT "MAC" $self]
			       #set eotT [$self mobility-trace EOT "MAC"]
	               }
	               $mac eot-target $eotT
	       }
	
	
	
		# ============================================================
		
		#$pal_ addif $netif 	;#Test: add the interface to the 802_11PAL which extends mobilenode
		$self addif $netif	;#FIXED and checked : add the wireless phy to the node (mobilenode.cc) now added to bt-node.cc
		
		#$pal_ setup [AddrParams addr2id $args] $mac $l2cap_ $a2mp $self
		$pal_ _init
		
	} else	{
		error "Currently only these PAL values are supported (add-PAL): PAL/802_11"
	}



}

##############End Add 802.11 interface###################

################ modified monility-trace procedure for the btnode ########

Node/BTNode instproc mobility-trace { ttype atype } {
	set ns [Simulator instance]
        set tracefd [$ns get-ns-traceall]
        if { $tracefd == "" } {
	        puts "Warning: You have not defined you tracefile yet!"
	        puts "Please use trace-all command to define it."
		return ""
	}
	set T [new CMUTrace/$ttype $atype]
	$T newtrace [Simulator set WirelessNewTrace_]
	$T tagged [Simulator set TaggedTrace_]
	$T target [$ns nullagent]
	$T attach $tracefd
        $T set src_ [$self id]
        $T node $self
	return $T
}
###########################################################################


Node/BTNode instproc rt {rtagent} {
        $self instvar mac_ bnep_ sdp_ a2mp_ l2cap_ lmp_ bb_ ll_ arptable_ classifier_ dmux_ entry_ ragent_ 

	set addr [$self node-addr]

	switch -exact $rtagent {
	    DSDV {
		# puts "DSDV support is not complete!"
		# exit

		set rag [new Agent/DSDV/BT]
		$rag addr $addr
	        if [Simulator set mobile_ip_] {
                	$ragent port-dmux [$self demux]
        	}
	    }
	    AODV {
	 	set rag [new Agent/AODV/BT $addr]
		# $rag if-queue $ifq_
	    }
	    DumbAgent {
		puts "DumbAgent is no longer supported! Use Manual instead."
		exit
	    }
	    DSR {
		puts "DSR support is not complete!"
		exit
	    }
	    TORA {
		puts "TORA support is not complete!"
		exit
	    }
	    Manual {
	 	# set rag [new Agent/ManualBT $addr]
	 	set rag [new Agent/ManualBT ]
	    }
	    default {
		puts "Wrong routing agent!"
		exit
	    }
	}

	$rag node $self		;# connect in c++ space
	$self set ragent_ $rag
	# $self ragent $rag
	$self attach $rag [Node set rtagent_port_]
	# $ragent port-dmux [$self demux]

	set port [Node set rtagent_port_]
	$rag target $ll_
	$dmux_ install $port $rag
	$classifier_ defaulttarget $rag

##	set ns [Simulator instance]
##	set tracefd [$ns get-ns-traceall]
##	if {$tracefd != "" } {
##	    $self nodetrace $tracefd
##	    $self agenttrace $tracefd
##	}
##	set namtracefd [$ns get-nam-traceall]
##	if {$namtracefd != "" } {
##	    $self namattach $namtracefd
##	}
}


Node/BTNode instproc on { {rndSlotOffset yes} {clkdrift no} {drifval 0} } {
    $self instvar bb_ ragent_ randomizeSlotOffset_

    # Turn on immediately without the initial random delay up to 2 slots.
    # Use it if you want to control slots alignment yourself.
    if { $rndSlotOffset == "imm" || $clkdrift == "imm" } {
	set randomizeSlotOffset_ 0	
    }

    if { $rndSlotOffset == "drift" || $clkdrift == "drift" } {
	$bb_ set driftType_ 1	;# random
    }

    if { $rndSlotOffset == "drift-normal" || $clkdrift == "drift-normal" } {
	$bb_ set driftType_ 2	;# normal
    }

    if { $clkdrift == "drift-user" } {
	$bb_ set driftType_ 3	;# user defined.
	$bb_ set clkdrift_ $drifval
    }
    if { $rndSlotOffset == "drift-user" } {
	$bb_ set driftType_ 3   ;# user defined.
	$bb_ set clkdrift_ $clkdrift
    }

    # puts $rndSlotOffset 
    # puts $clkdrift
    # puts $drifval
    if { [$bb_ set clkdrift_] > 20 || [$bb_ set clkdrift_] < -20 } {
	puts "Clock drift should be in [-20, 20] ppm."
	exit
    }

    $self turn-it-on
    #$ragent_ start
}

Agent/ManualBT instproc start {} {
}

Agent/DSDV/BT instproc start {} {
    $self start-dsdv
}

Agent/DSDV/BT instproc if-queue {ifq} {
    # $self ll-queue $ifq
}

Node/BTNode instproc make-bnep-connection {dest {pktType DH1} {recvPktType DH1} {qos none} {ifq none} args} {
    if { $ifq == "none" } {
	$self bnep-connect $dest $pktType $recvPktType
    } else {
	$self bnep-connect $dest $pktType $recvPktType $ifq
    }
    
    if { [llength $qos] == 6 } {
	$self bnep-qos-setup $dest [set $qos]
    }

    if { [llength $args] > 0 } {
	#puts "$args";
	$self cmd-after-bnep-connect $dest "$args";
    }
}

Node/BTNode instproc make-br {br {pktType DH1} {recvPktType DH1} {qos none} {ifq none} args } {
    set debug_ 1
    $self instvar address_ bb_
    $br instvar address_
    set maddr [$self set address_]
    set baddr [$br set address_]
    #puts "$maddr try to connect br $baddr."
    #puts "$args"

    # $br pagescan 4096 4000
    $br pagescan 4096 4096
    [$self set bb_] set N_page_ 1
    if { [llength $args] > 0 } {
	# set cmd [join $args " "]
        # $self make-bnep-connection $br $pktType $recvPktType $qos "$cmd"
        $self make-bnep-connection $br $pktType $recvPktType $qos $ifq "$args"
    } else {
        $self make-bnep-connection $br $pktType $recvPktType $qos $ifq
    }
}

SDP instproc openSearch { } {
}

SDP instproc closeSearch { } {
}

# three senarioes are introduced in Profile 1.1 (p76)
# 1. collect_user_input -> inq -> foreach RemDev: (page) sdp_inq -> results
# 2. inq -> collect_user_input -> foreach RemDev: (page) sdp_inq -> results
# 3. inq -> collect_user_input -> conn all -> foreach RemDev: sdp_inq - results

#RemDevRelation : trusted/unknown/connected ??
SDP instproc serviceBrowse { remDev remDevRel browseGroup {getRemDevName no} } {
}

SDP instproc serviceSearch { remDev remDevRel srchPttn-attrLst {getRemDevName no} } {
}

SDP instproc enumerateRemDev { {classOfDev any} } {
}

SDP instproc terminatePrimitive { primitiveHandle } {
}

#####################################
source ../../bluetooth/ns-compund.tcl


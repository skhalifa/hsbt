set val(mac)            Mac/BNEP                 ;# MAC type
set val(nn)             2                        ;# number of mobilenodes
set val(numberOfMACs)   4                        ;# total number of MACs
set val(palType) PAL/802_11
set val(prop)   Propagation/TwoRayGround   ;# radio-propagation model
set val(chan)   Channel/WirelessChannel    ;# channel type


set StartTime [list 0.0 0.0006 0.1031 0.1134 0.3878 0.8531 0.6406 0.0627]

set ns_		[new Simulator]

set chan [new $val(chan)];#Create wireless channel
#Setup topography object
set topo       [new Topography]
$topo load_flatgrid 50 50

create-god $val(numberOfMACs)

set f [open a2mp.tr w]
$ns_ trace-all $f
set nf [open a2mp.nam w]
$ns_ namtrace-all-wireless $nf 7 7
$ns_ node-config -macType $val(mac) 	;# set node type to BTNode

Simulator set MacTrace_ ON

$ns_ node-config -macType $val(mac) 	;# set node type to BTNode

for {set i 0} {$i < $val(nn) } {incr i} {
	set node($i) [$ns_ node $i ]
        $node($i) rt AODV
	[$node($i) set bb_] set ver_ 11

	$ns_ at [lindex $StartTime $i] "$node($i) on"
}

	############# Add 802.11 PAL #####################
	#set a2mp [$node($i) set a2mp_]
	$node(0) add-PAL $val(palType) $topo $chan $val(prop)
$node(1) add-PAL $val(palType) $topo $chan $val(prop)
#$node(2) add-PAL $val(palType) $topo $chan $val(prop)
	##################################################
set a2mp0 [$node(0) set a2mp_]


#=========================================================================
# Configuration # of links, traffic and applications 
#=========================================================================
set tcp2 [new Agent/TCP] ;#Declaration of TCP traffic agent
$ns_ attach-agent $node(0) $tcp2 ;#Union agent with the node for (tx)
set ftp2 [new Application/FTP] ;#Declaration of new FTP application
$ftp2 attach-agent $tcp2 ;# union of the application agent Traffic


set null0 [new Agent/TCPSink] ;#Declaración del repositorio del agente de trafico TCP
$ns_ attach-agent $node(1) $null0 ;#Unión del repositorio con el nodo correspondiente (rx)
$ns_ connect $tcp2 $null0 ;#unión del agente de trafico con el repositorio
set ifq [new Queue/DropTail] ;#Declaration of the queue or buffer
$ifq set limit_ 20 ;#Limit the queue (packet)




#=========================================================================
# Event Organizer *
#=========================================================================
$ns_ at 0.000001 "$ns_ trace-annotate \" BEGIN SIMULATION \""
$ns_ at 0.1 "$node(0) make-hs-connection $node(1)"
#$ns_ at 0.1 "$node(0) make-bnep-connection $node(1) DH5 DH5 noqos $ifq"
#$ns_ at 0.1 "$node(0) make-bnep-connection $node(1)"

#$ns_ at 10.0 "$ftp2 send 4000000000"
$ns_ at 1.0 "$ftp2 start"
$ns_ at 50.0 "$ftp2 stop"

#===================================
#        Termination        
#===================================
#Define a 'finish' procedure
proc finish {} {
    global ns tracefile namfile
    $ns_ flush-trace
    close $tracefile
    close $namfile
    exec nam out.nam &
    exit 0
}

#$ns at $val(stop) "$ns nam-end-wireless $val(stop)"
#$ns at $val(stop) "finish"
#$ns at $val(stop) "puts \"done\" ; $ns halt"
#$ns run

#$ns_ at 1 "$node(0) make-hs-connection $node(1)"
#$ns_ at 1 "$a2mp0 discover $node(6)"
#$ns_ at 5 "$a2mp0 discover $node(1)"
#$ns_ at 10 "$a2mp0 discover $node(5)"
$ns_ at 60 "$ns_ halt"

$ns_ run


A Modified version of the UCBT to model bluetooth V3 + HS specification


The UCBT NS-2 Bluetooth extension (http://www.cs.uc.edu/~cdmc/ucbt/) is modified to the new Bluetooth V3.0 standard by added A2MP protocol, 802.11 PAL and UWB PAL. The new model supports discovering the remote devices alternative MAC/PHY capabilities and creating high speed connections over one of the alternative MAC/PHYs.

For the UWB MAC/PHY, EPFL (http://uwb.epfl.ch/ns-2/index.html) UWB NS-2 extension is used and integrated to the UCBT model. The EPFL model is based on Time Hopping (TH) Impules Radio (IR) Ultra Wide Band (UWB) and uses a MAC protocl based on Dynamic channel Coding (DCC) and Private MAC with interference mitigation schema based on erasures. DCC allows interference to occur and adopt to it by reducing the data rate where incremental redundant codes are used to code the data bits. DCC uses Time Hopping Sequence (THS) based on the MAC addresses of the nodes and a predefined THS for broadcast.

A stable version of HSBT has been released (http://code.google.com/p/hsbt/downloads/list) which supports adding Bluetooth nodes with one of the alternative MAC/PHYs (IEEE802.11b/g or TH IR-UWB).

A sample TCL script can be found in the wiki section (http://code.google.com/p/hsbt/w/list).

HSBT is compatible with tcl scenarios written for UCBT alone, EPFL alone, IEEE802.11b/g alone.

HSBT also provide an easy way to add your own alternative MAC/PHY for more details (http://code.google.com/p/hsbt/w/list).

For installation instructions, please visit the wiki section for more details (http://code.google.com/p/hsbt/w/list).

If there is an issue please report it to the issues section (http://code.google.com/p/hsbt/issues/list).

Publications:

Shady S. Khalifa, Hesham N. Elmahdy, Imane Aly Saroit and S.H. Ahmed, "An Assessment of Ultra Wide Band As an Alternative Controller for Bluetooth to Support High Rate Applications on Battery Powered Devices", CiiT International Journal of Wireless Communication, vol.3, no.7, pp.546-552, May 2011, DOI:WC052011012. 

Shady Khalifa

sh.khalifa@fci-cu.edu.eg

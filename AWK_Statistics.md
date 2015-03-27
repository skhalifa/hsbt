# AWK command to calculate the number of sent/received/dropped packets #

```
gawk 'BEGIN { print "begin";totsent=0; totsent2=0; toDrop=0; totrecv=0; } { if($4 == "AGT" && $7 == "tcp") { if ($1=="+") { totsent=totsent+$8; print "1";} if ($1=="s") { totsent2=totsent2+$8; print "2";} if ($1=="r") { totrecv=totrecv+$8; print "3";}} if ($4=="AGT" && $7=="cbr" && $1=="D") { toDrop=toDrop+$8; print "4";}  } END { print "Bytes sent+"totsent"\nBytes sents"totsent2"\nBytes received "totrecv"\nBytes Dropped "toDrop; } ' trace.csv
```

```
gawk 'BEGIN { print "begin";totsent=0; totsent2=0; toDrop=0; totrecv=0; } { if($4 == "AGT" && $7 == "cbr") { if ($1=="+") { totsent=totsent+$8; print "1";} if ($1=="s") { totsent2=totsent2+$8; print "2";} if ($1=="r") { totrecv=totrecv+$8; print "3";}} if ($7=="cbr" && $1=="D") { toDrop=toDrop+$8; print "4";}  } END { print "Bytes sent+"totsent"\nBytes sents"totsent2"\nBytes received "totrecv"\nBytes Dropped "toDrop; } ' trace.csv
```
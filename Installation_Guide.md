# Installation Guide #

### install from tar ball ###

1. get ns-allinone-2.29.tar.gz
> wget [ftp://ftp.isi.edu/nsnam/ns-allinone-2.29.tar.gz](ftp://ftp.isi.edu/nsnam/ns-allinone-2.29.tar.gz)

2. get HSBT-xx.tgz
> http://code.google.com/p/hsbt/downloads/list

3. tar zxvf ns-allinone-2.29.tar.gz

4. cd ns-allinone-2.29/ns-2.29/
> tar zxvf ../../HSBT-xx.tgz
> Assume HSBT-xx.tgz and ns-allinone-2.29.tar.gz are in the same
> directory.

5. cd ucbt-xx/
> ./install-bt
> or
> ./install-bt -d  # enable debug
> > or

> ./install-bt -t  # install tcl-debug

> ucbt-xx will be linked as bluetooth.

6. If you want debug enabled, while the debug option is not enable in the
> above step,
> cd ns-2.29
> ./enable-ns-debug.sh

7. try some tests:
> cd test/
> ../../ns test.tcl > test.out

8. to generate mobile scaniro:
> cd tools && make
> > then you can use the modified setdest program.

9. to recompile if you make some changes to the source code:

> run make in bluetooth/

> If you did a lot of changes, see file UPGRADE
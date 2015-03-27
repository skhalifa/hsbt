# Upgrade Process for different cases #

1. If the change is small, do a 'make' in ucbt-xxx/ is fine.
2. If new files are added or some files are deleted, you need to update
> the file Makefile.in, then in 'ns-allinone-xxx/ns-xxx', perform a
> './configure' to generate the new Makefile.  If you are using
> cygwin, you need to run
> './configure --x-libraries=/usr/X11R6/lib --x-includes=/usr/X11R6/include'
> instead.
3. If changes are extensive, espcially, header files are
> changed, you need to do a 'make clean' before 'make'. 'make clean'
> in dir 'ns-allinone-xxx/ns-xxx' will rebuild ns completely. 'make clean' in
> dir 'bluetooth/' or 'ucbt-xxx/' is just fine most of time.
4. If additional ns source files are changed, you need to update the
> script 'link-ns.sh' in bluetooth/ns-2.xxx/ and run it before 2.
5. A fresh re-installation should always works.
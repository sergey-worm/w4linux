####################################################################################################
#
#  Makefile for user application.
#  External vars my be:
#    arch      - target arch
#    dbg       - debug flag
#    cfgdir    - path to dir that contents sys-config.h
#    blddir    - path to dir that will content build result
#    wrmdir    - path to wrmos source dir
#
####################################################################################################

lnxdir = $(blddir)/br/output/build/linux-4.12.1

all:  Makefile
	mkdir -p $(blddir)
	make $(blddir)/linux.elf

clean:
	#make -C$(lnxdir) ARCH=w4sparc clean
	rm $(blddir)/linux.elf

distclean:  clean
	make -C$(blddir)/br clean

$(blddir)/linux.elf:  $(lnxdir)/arch/w4sparc/boot/image
	cp -f $(lnxdir)/arch/w4sparc/boot/image $(blddir)/linux.elf

$(lnxdir)/arch/w4sparc/boot/image:  $(lnxdir)/arch/w4sparc $(lnxdir)/cxx_support_patched
	make -C$(blddir)/br linux-rebuild ARCH=w4sparc -j16 WRMOS_SRC=$(wrmdir) WRMOS_BLD=$(blddir)/../..

$(lnxdir)/arch/w4sparc:  $(lnxdir) w4sparc
	cp -ur w4sparc $(lnxdir)/arch/

$(lnxdir)/cxx_support_patched:  $(lnxdir)
	-patch -d$(lnxdir) -N -p1 < cxx_support.patch
	touch $(lnxdir)/cxx_support_patched # mark as patched

$(lnxdir):  $(blddir)/br/configs/wrmos_sparc_defconfig $(blddir)/br/board/wrmos
	make -C$(blddir)/br wrmos_sparc_defconfig
	make -C$(blddir)/br -j16

$(blddir)/br/configs/wrmos_sparc_defconfig:  $(blddir)/br
	cp br/configs/wrmos_sparc_defconfig $(blddir)/br/configs/wrmos_sparc_defconfig

$(blddir)/br/board/wrmos:  $(blddir)/br
	cp -r br/board/wrmos $(blddir)/br/board/wrmos


$(blddir)/br:
	git clone git://git.buildroot.net/buildroot $(blddir)/br
	mkdir -p /tmp/buildroot_download_cache
	ln -s /tmp/buildroot_download_cache $(blddir)/br/dl
	cd $(blddir)/br;  git checkout 2017.11.2


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
libs = $(rtblddir)/lib/console/libconsole.a

.PHONY:  make-me clean distclean FORCE

# must be first target
make-me: $(libs)
	$(v)mkdir -p $(blddir)
	$(v)+make $(blddir)/linux.elf

FORCE:

clean-files::
	$(v)+if [ -d $(lnxdir) ]; then make -C$(lnxdir) ARCH=w4sparc W= clean; fi
	$(v)rm -f $(blddir)/linux.elf

distclean:  clean
	$(v)+make -C$(blddir)/br clean

$(blddir)/linux.elf:  $(lnxdir)/arch/w4sparc/boot/image
	$(v)cp -f $(lnxdir)/arch/w4sparc/boot/image $(blddir)/linux.elf

$(lnxdir)/arch/w4sparc/boot/image:  FORCE $(lnxdir)/arch/w4sparc $(lnxdir)/cxx_support_patched
	@echo "w4lx: Rebuild buildroot for w4sparc."
	$(v)+make -C$(blddir)/br linux-rebuild ARCH=w4sparc WRMOS_SRC=$(wrmdir) WRMOS_BLD=$(blddir)/../..

$(lnxdir)/arch/w4sparc:  FORCE | $(lnxdir)
	@echo "w4lx: Update w4sparc sources."
	$(v)cp -ur w4sparc $(lnxdir)/arch/

$(lnxdir)/cxx_support_patched:  | $(lnxdir)
	$(v)-patch -d$(lnxdir) -N -p1 < cxx_support.patch
	$(v)touch $(lnxdir)/cxx_support_patched # mark as patched

$(lnxdir):  $(blddir)/br/configs/wrmos_sparc_defconfig $(blddir)/br/board/wrmos
	@echo "w4lx: Build original buildroot."
	$(v)+make -C$(blddir)/br wrmos_sparc_defconfig
	$(v)+make -C$(blddir)/br

$(blddir)/br/configs/wrmos_sparc_defconfig:  br/configs/wrmos_sparc_defconfig | $(blddir)/br
	$(v)cp -u br/configs/wrmos_sparc_defconfig $(blddir)/br/configs/wrmos_sparc_defconfig

$(blddir)/br/board/wrmos:  | $(blddir)/br
	$(v)cp -ru br/board/wrmos $(blddir)/br/board/wrmos


$(blddir)/br:
	@echo "w4lx: Get buildroot sources."
	$(v)git clone git://git.buildroot.net/buildroot $(blddir)/br
	$(v)mkdir -p /tmp/buildroot_download_cache
	$(v)ln -s /tmp/buildroot_download_cache $(blddir)/br/dl
	$(v)cd $(blddir)/br;  git checkout 2017.11.2

include $(wrmdir)/mk/base.mk

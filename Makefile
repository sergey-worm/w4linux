####################################################################################################
#
#  Root makefile for w4linux project.
#
####################################################################################################

help:
	@echo "Use:"
	@echo "  make build   P=<path/*.prj> W=<wrmos-dir> B=<build-dir> [ V=1 ]"
	@echo "  make rebuild P=<path/*.prj> W=<wrmos-dir> B=<build-dir> [ V=1 ]"
	@echo "  make clean   P=<path/*.prj> W=<wrmos-dir> B=<build-dir> [ V=1 ]"
	@echo ""

build:
	mkdir -p $B
	+make -C $W build P=$(shell pwd)/$P B=$(abspath $B) E=$(shell pwd) V=$V

clean:
	mkdir -p $B
	+make -C $W clean P=$(shell pwd)/$P B=$(abspath $B) E=$(shell pwd) V=$V

rebuild:
	+make clean P=$P W=$W B=$B
	+make build P=$P W=$W B=$B

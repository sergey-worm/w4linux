####################################################################################################
#
#  ...
#
####################################################################################################

# undef MAKEFLAGS cause it contents W=<wrmdir> but linux use this wlag as warning level
override undefine MAKEFLAGS

help:
	@echo "Use:"
	@echo "  make build   P=<path/*.prj> W=<wrmos-dir> B=<build-dir> [ V=1 ]"
	@echo "  make rebuild P=<path/*.prj> W=<wrmos-dir> B=<build-dir> [ V=1 ]"
	@echo "  make clean   P=<path/*.prj> W=<wrmos-dir> B=<build-dir> [ V=1 ]"
	@echo ""

build:
	mkdir -p $B
	make -C $W build P=$$(pwd)/$P B=$$(realpath $B) E=$$(pwd) V=$V

clean:
	mkdir -p $B
	make -C $W clean P=$$(pwd)/$P B=$$(realpath $B) E=$$(pwd) V=$V

rebuild:
	make clean P=$P W=$W B=$B
	make build P=$P W=$W B=$B

	.file	"asm-offsets.c"
! GNU C89 (Buildroot 2016.08.1) version 6.1.0 (sparc-buildroot-linux-uclibc)
!	compiled by GNU C version 5.4.0 20160609, GMP version 6.1.1, MPFR version 3.1.4, MPC version 1.0.3, isl version none
! warning: GMP header version 6.1.1 differs from library version 6.1.0.
! GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
! options passed:  -nostdinc -I ./arch/w4sparc/include
! -I ./arch/w4sparc/include/generated/uapi
! -I ./arch/w4sparc/include/generated -I ./include
! -I ./arch/w4sparc/include/uapi -I ./include/uapi
! -I ./include/generated/uapi
! -iprefix /home/worm/Bin/Toolchains/sparc-buildroot.2016.08.1-linux-uclibc/usr/bin/../lib/gcc/sparc-buildroot-linux-uclibc/6.1.0/
! -isysroot /home/worm/Bin/Toolchains/sparc-buildroot.2016.08.1-linux-uclibc/usr/sparc-buildroot-linux-uclibc/sysroot
! -D__sparc_v8__ -D __KERNEL__ -D CC_HAVE_ASM_GOTO
! -D KBUILD_BASENAME="asm_offsets" -D KBUILD_MODNAME="asm_offsets"
! -isystem /home/worm/Bin/Toolchains/sparc-buildroot.2016.08.1-linux-uclibc/usr/bin/../lib/gcc/sparc-buildroot-linux-uclibc/6.1.0/include
! -include ./include/linux/kconfig.h
! -MD arch/w4sparc/kernel/.asm-offsets.s.d
! arch/w4sparc/kernel/asm-offsets.c -m32 -mcpu=v8 -mno-fpu
! -auxbase-strip arch/w4sparc/kernel/asm-offsets.s -O2 -Wall -Wundef
! -Wstrict-prototypes -Wno-trigraphs -Werror=implicit-function-declaration
! -Wno-format-security -Wno-frame-address -Wframe-larger-than=1024
! -Wno-unused-but-set-variable -Wunused-const-variable=0
! -Wdeclaration-after-statement -Wno-pointer-sign -Werror=implicit-int
! -Werror=strict-prototypes -Werror=date-time
! -Werror=incompatible-pointer-types -Werror=designated-init -std=gnu90
! -fno-strict-aliasing -fno-common -fno-PIE -fcall-used-g5 -fcall-used-g7
! -fno-delete-null-pointer-checks -fno-stack-protector -fomit-frame-pointer
! -fno-var-tracking-assignments -fno-strict-overflow -fconserve-stack
! -fverbose-asm --param allow-store-data-races=0
! options enabled:  -faggressive-loop-optimizations -falign-functions
! -falign-jumps -falign-labels -falign-loops -fauto-inc-dec
! -fbranch-count-reg -fcaller-saves -fchkp-check-incomplete-type
! -fchkp-check-read -fchkp-check-write -fchkp-instrument-calls
! -fchkp-narrow-bounds -fchkp-optimize -fchkp-store-bounds
! -fchkp-use-static-bounds -fchkp-use-static-const-bounds
! -fchkp-use-wrappers -fcombine-stack-adjustments -fcompare-elim
! -fcprop-registers -fcrossjumping -fcse-follow-jumps -fdefer-pop
! -fdelayed-branch -fdevirtualize -fdevirtualize-speculatively
! -fdwarf2-cfi-asm -fearly-inlining -feliminate-unused-debug-types
! -fexpensive-optimizations -fforward-propagate -ffunction-cse -fgcse
! -fgcse-lm -fgnu-runtime -fgnu-unique -fguess-branch-probability
! -fhoist-adjacent-loads -fident -fif-conversion -fif-conversion2
! -findirect-inlining -finline -finline-atomics
! -finline-functions-called-once -finline-small-functions -fipa-cp
! -fipa-cp-alignment -fipa-icf -fipa-icf-functions -fipa-icf-variables
! -fipa-profile -fipa-pure-const -fipa-ra -fipa-reference -fipa-sra
! -fira-hoist-pressure -fira-share-spill-slots
! -fisolate-erroneous-paths-dereference -fivopts -fkeep-static-consts
! -fleading-underscore -flifetime-dse -flra-remat -flto-odr-type-merging
! -fmath-errno -fmerge-constants -fmerge-debug-strings
! -fmove-loop-invariants -fomit-frame-pointer -foptimize-sibling-calls
! -foptimize-strlen -fpartial-inlining -fpcc-struct-return -fpeephole
! -fpeephole2 -fplt -fprefetch-loop-arrays -freorder-blocks
! -freorder-functions -frerun-cse-after-loop
! -fsched-critical-path-heuristic -fsched-dep-count-heuristic
! -fsched-group-heuristic -fsched-interblock -fsched-last-insn-heuristic
! -fsched-rank-heuristic -fsched-spec -fsched-spec-insn-heuristic
! -fsched-stalled-insns-dep -fschedule-fusion -fschedule-insns
! -fschedule-insns2 -fsemantic-interposition -fshow-column -fshrink-wrap
! -fsigned-zeros -fsplit-ivs-in-unroller -fsplit-wide-types -fssa-backprop
! -fssa-phiopt -fstdarg-opt -fstrict-volatile-bitfields -fsync-libcalls
! -fthread-jumps -ftoplevel-reorder -ftrapping-math -ftree-bit-ccp
! -ftree-builtin-call-dce -ftree-ccp -ftree-ch -ftree-coalesce-vars
! -ftree-copy-prop -ftree-cselim -ftree-dce -ftree-dominator-opts
! -ftree-dse -ftree-forwprop -ftree-fre -ftree-loop-if-convert
! -ftree-loop-im -ftree-loop-ivcanon -ftree-loop-optimize
! -ftree-parallelize-loops= -ftree-phiprop -ftree-pre -ftree-pta
! -ftree-reassoc -ftree-scev-cprop -ftree-sink -ftree-slsr -ftree-sra
! -ftree-switch-conversion -ftree-tail-merge -ftree-ter -ftree-vrp
! -funit-at-a-time -fverbose-asm -fzero-initialized-in-bss -m32 -mapp-regs
! -mlong-double-128 -mptr32 -msoft-quad-float -muclibc -muser-mode

	.section	".text"
	.align 4
	.global sparc32_foo
	.type	sparc32_foo, #function
	.proc	04
sparc32_foo:
#APP
! 23 "arch/w4sparc/kernel/asm-offsets.c" 1
	
.ascii "->AOFF_thread_fork_kpsr 8 offsetof(struct thread_struct, fork_kpsr)"	!
! 0 "" 2
#NO_APP
	jmp	%o7+8
	 mov	0, %o0	!, tmp24
	.size	sparc32_foo, .-sparc32_foo
	.align 4
	.global foo
	.type	foo, #function
	.proc	04
foo:
#APP
! 49 "arch/w4sparc/kernel/asm-offsets.c" 1
	
.ascii "->"
! 0 "" 2
! 50 "arch/w4sparc/kernel/asm-offsets.c" 1
	
.ascii "->AOFF_task_thread 1200 offsetof(struct task_struct, thread)"	!
! 0 "" 2
! 51 "arch/w4sparc/kernel/asm-offsets.c" 1
	
.ascii "->"
! 0 "" 2
! 52 "arch/w4sparc/kernel/asm-offsets.c" 1
	
.ascii "->AOFF_mm_context 348 offsetof(struct mm_struct, context)"	!
! 0 "" 2
! 53 "arch/w4sparc/kernel/asm-offsets.c" 1
	
.ascii "->"
! 0 "" 2
! 54 "arch/w4sparc/kernel/asm-offsets.c" 1
	
.ascii "->VMA_VM_MM 32 offsetof(struct vm_area_struct, vm_mm)"	!
! 0 "" 2
#NO_APP
	jmp	%o7+8
	 mov	0, %o0	!, tmp24
	.size	foo, .-foo
	.ident	"GCC: (Buildroot 2016.08.1) 6.1.0"
	.section	.note.GNU-stack,"",@progbits

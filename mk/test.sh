#!/bin/bash
####################################################################################################
#
#  Sanity check w4linux.
#
####################################################################################################

blddir=/tmp/wrm-test/w4linux

# CONFIG
w4linux_sparc_build=1
w4linux_sparc_exec=2
result[$w4linux_sparc_build]=-
result[$w4linux_sparc_exec]=-
result[$w4linux_arm_build]=-
result[$w4linux_arm_exec]=-
result[$w4linux_x86_build]=-
result[$w4linux_x86_exec]=-
result[$w4linux_x86_64_build]=-
result[$w4linux_x86_64_exec]=-

function get_result
{
	if [ $rc == 0 ]; then echo '\e[1;32m+\e[0m'; else echo '\e[1;31m-\e[0m'; fi
}

function do_build
{
	id=$1
	prj=$2
	arch=$3
	brd=$4
	make build  P=cfg/prj/${prj}-qemu-${brd}.prj W=../wrmos B=$blddir/$prj-qemu-${brd} -j
	rc=$?
	echo "Build:  rc=$rc."
	result[$id]=$(get_result $rc)
}

function do_exec
{
	id=$1
	prj=$2
	arch=$3
	brd=$4
	machine=$5

	qemu_args="-display none -serial stdio"
	run_qemu="qemu-system-$arch -M $machine $qemu_args -kernel $blddir/$prj-qemu-$brd/ldr/bootloader.elf"
	file=$blddir/$prj-qemu-$brd/ldr/bootloader.elf
	if [ $arch == x86 ]; then
		run_qemu="qemu-system-i386 $qemu_args -drive format=raw,file=$(realpath $blddir/$prj-qemu-$arch/ldr/bootloader.img)"
		file=$blddir/$prj-qemu-$brd/ldr/bootloader.img
	fi
	if [ $arch == x86_64 ]; then
		run_qemu="qemu-system-$arch $qemu_args -drive format=raw,file=$(realpath $blddir/$prj-qemu-$arch/ldr/bootloader.img)"
		file=$blddir/$prj-qemu-$brd/ldr/bootloader.img
	fi

	if [ -f $file ]; then
		expect -c "\
			set timeout 40; \
			if { [catch {spawn $run_qemu} reason] } { \
				puts \"failed to spawn qemu: $reason\r\"; exit 1 }; \
			expect \"buildroot login: \" { send \"root\r\" }  timeout { exit 2 }; \
			set timeout 10; \
			expect \"# \" { send \"cat /proc/cpuinfo\r\" }          timeout { exit 3 }; \
			expect \"# \" { send \"find / | wc -l | grep XXX\r\" }  timeout { exit 4 }; \
			set timeout 20; \
			expect \"# \" { send \"uptime\r\" }                     timeout { exit 5 }; \
			expect \"# \" { send \"halt\r\" }                       timeout { exit 6 }; \
			expect \"reboot: System halted\" { exit 0 }             timeout { exit 6 }; \
			exit 6"
		rc=$?
	fi

	echo -e "\nExecute:  rc=$rc."
	result[$id]=$(get_result $rc)
}

function do_all
{
	rm -fr $blddir

	# cmd     id                     prj      arch    brd     machine

	do_build  $w4linux_sparc_build   linux    sparc   leon3   leon3_generic
	do_exec   $w4linux_sparc_exec    linux    sparc   leon3   leon3_generic
	# now w4linux supports only arch=sparc
	#do_build  $w4linux_arm_build     linux    arm     veca9   vexpress-a9
	#do_exec   $w4linux_arm_exec      linux    arm     veca9   vexpress-a9
	#do_build  $w4linux_x86_build     linux    x86     x86     ""
	#do_exec   $w4linux_x86_exec      linux    x86     x86     ""
	#do_build  $w4linux_x86_64_build  linux    x86_64  x86_64  ""
	#do_exec   $w4linux_x86_64_exec   linux    x86_64  x86_64  ""
}

do_all

echo -e "---------------------------------------------"
echo -e "  REPORT:"
echo -e "---------------------------------------------"
echo -e "  project  arch    build  execute"
echo -e "- - - - - - - - - - - - - - - - - - - - - - -"
echo -e "  w4linux  sparc       ${result[$w4linux_sparc_build]}        ${result[$w4linux_sparc_exec]}"
echo -e "  w4linux  arm         ${result[$w4linux_arm_build]}        ${result[$w4linux_arm_exec]}"
echo -e "  w4linux  x86         ${result[$w4linux_x86_build]}        ${result[$w4linux_x86_exec]}"
echo -e "  w4linux  x86_64      ${result[$w4linux_x86_64_build]}        ${result[$w4linux_x86_64_exec]}"

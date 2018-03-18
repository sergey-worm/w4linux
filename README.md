# w4linux

Paravirtualized linux on top of WrmOS.

## Description

	w4linux - paravirtualized linux on top of WrmOS (https://github.com/sergey-worm/wrmos.git).

	w4linux project run paravirtualized linux as wrmos application.

	w4linux based on Buildroot project:

	* it uses rootfs for sparc arch;
	* it uses linux kernel for w4sparc arch.

## How to

	Build and run:

	qemu-sparc-leon3:
		make rebuild P=cfg/prj/linux-qemu-leon3.prj W=../wrmos B=../build/linux-qemu-leon3 -j V=1
		qemu-system-sparc -M leon3_generic -display none -serial stdio -kernel ../build/linux-qemu-leon3/ldr/bootloader.elf

	qemu-arm-vexpress-a9:
		TODO

	qemu-x86:
		TODO

	qemu-x86_64:
		TODO

## Project state

	Now supportes 1 archs:  SPARC (qemu and real hardware).

## Plans

	1. Support ARM.
	1. Support x86.
	1. Support x86 64-bit.

## Contacts

	Sergey Worm <sergey.worm@gmail.com>


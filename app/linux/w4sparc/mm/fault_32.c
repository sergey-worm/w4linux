/*
 * fault.c:  Page fault handlers for the Sparc.
 *
 * Copyright (C) 1995 David S. Miller (davem@caip.rutgers.edu)
 * Copyright (C) 1996 Eddie C. Dost (ecd@skynet.be)
 * Copyright (C) 1997 Jakub Jelinek (jj@sunsite.mff.cuni.cz)
 */

#include <asm/head.h>

#include <linux/string.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/threads.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/perf_event.h>
#include <linux/interrupt.h>
#include <linux/kdebug.h>
#include <linux/uaccess.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/openprom.h>
#include <asm/oplib.h>
#include <asm/setup.h>
#include <asm/smp.h>
#include <asm/traps.h>

#include "mm_32.h"

int show_unhandled_signals = 1;

static void __noreturn unhandled_fault(unsigned long address,
				       struct task_struct *tsk,
				       struct pt_regs *regs)
{
	if ((unsigned long) address < PAGE_SIZE) {
		printk(KERN_ALERT
		    "Unable to handle kernel NULL pointer dereference\n");
	} else {
		printk(KERN_ALERT "Unable to handle kernel paging request at virtual address %08lx\n",
		       address);
	}
	printk(KERN_ALERT "tsk->{mm,active_mm}->context = %08lx\n",
		(tsk->mm ? tsk->mm->context : tsk->active_mm->context));
	printk(KERN_ALERT "tsk->{mm,active_mm}->pgd = %08lx\n",
		(tsk->mm ? (unsigned long) tsk->mm->pgd :
			(unsigned long) tsk->active_mm->pgd));
	die_if_kernel("Oops", regs);
}

asmlinkage int lookup_fault(unsigned long pc, unsigned long ret_pc,
			    unsigned long address)
{
	struct pt_regs regs;
	unsigned long g2;
	unsigned int insn;
	int i;

	i = search_extables_range(ret_pc, &g2);
	switch (i) {
	case 3:
		/* load & store will be handled by fixup */
		return 3;

	case 1:
		/* store will be handled by fixup, load will bump out */
		/* for _to_ macros */
		insn = *((unsigned int *) pc);
		if ((insn >> 21) & 1)
			return 1;
		break;

	case 2:
		/* load will be handled by fixup, store will bump out */
		/* for _from_ macros */
		insn = *((unsigned int *) pc);
		if (!((insn >> 21) & 1) || ((insn>>19)&0x3f) == 15)
			return 2;
		break;

	default:
		break;
	}

	memset(&regs, 0, sizeof(regs));
	regs.pc = pc;
	regs.npc = pc + 4;
	__asm__ __volatile__(
		"rd %%psr, %0\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n" : "=r" (regs.psr));
	unhandled_fault(address, current, &regs);

	/* Not reached */
	return 0;
}

static inline void
show_signal_msg(struct pt_regs *regs, int sig, int code,
		unsigned long address, struct task_struct *tsk)
{
	if (!unhandled_signal(tsk, sig))
		return;

	if (!printk_ratelimit())
		return;

	printk("%s%s[%d]: segfault at %lx ip %p (rpc %p) sp %p error %x",
	       task_pid_nr(tsk) > 1 ? KERN_INFO : KERN_EMERG,
	       tsk->comm, task_pid_nr(tsk), address,
	       (void *)regs->pc, (void *)regs->u_regs[UREG_I7],
	       (void *)regs->u_regs[UREG_FP], code);

	print_vma_addr(KERN_CONT " in ", regs->pc);

	printk(KERN_CONT "\n");
}

static void __do_fault_siginfo(int code, int sig, struct pt_regs *regs,
			       unsigned long addr)
{
	siginfo_t info;

	info.si_signo = sig;
	info.si_code = code;
	info.si_errno = 0;
	info.si_addr = (void __user *) addr;
	info.si_trapno = 0;

	/*
	if (unlikely(show_unhandled_signals))
		show_signal_msg(regs, sig, info.si_code,
				addr, current);
	*/

	force_sig_info(sig, &info, current);
}

static unsigned long compute_si_addr(struct pt_regs *regs, int text_fault)
{
	unsigned int insn;

	if (text_fault)
		return regs->pc;

	if (regs->psr & PSR_PS)
		insn = *(unsigned int *) regs->pc;
	else
		__get_user(insn, (unsigned int *) regs->pc);

	return safe_compute_effective_address(regs, insn);
}

static noinline void do_fault_siginfo(int code, int sig, struct pt_regs *regs,
				      int text_fault)
{
	unsigned long addr = compute_si_addr(regs, text_fault);

	__do_fault_siginfo(code, sig, regs, addr);
}

asmlinkage void do_sparc_fault(struct pt_regs *regs, int text_fault, int write,
			       unsigned long address)
{
	struct vm_area_struct *vma;
	struct task_struct *tsk = current;
	struct mm_struct *mm = tsk->mm;
	unsigned int fixup;
	unsigned long g2;
	int from_user = !(regs->psr & PSR_PS);
	int fault, code;
	unsigned int flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_KILLABLE;

	if (text_fault)
		address = regs->pc;

	/*
	 * We fault-in kernel-space virtual memory on-demand. The
	 * 'reference' page table is init_mm.pgd.
	 *
	 * NOTE! We MUST NOT take any locks for this case. We may
	 * be in an interrupt or a critical region, and should
	 * only copy the information from the master page table,
	 * nothing more.
	 */
	code = SEGV_MAPERR;
	if (address >= TASK_SIZE)
		goto vmalloc_fault;

	/*
	 * If we're in an interrupt or have no user
	 * context, we must not take the fault..
	 */
	if (pagefault_disabled() || !mm)
		goto no_context;

	perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS, 1, regs, address);

retry:
	down_read(&mm->mmap_sem);

	if (!from_user && address >= PAGE_OFFSET)
		goto bad_area;

	vma = find_vma(mm, address);
	if (!vma)
		goto bad_area;
	if (vma->vm_start <= address)
		goto good_area;
	if (!(vma->vm_flags & VM_GROWSDOWN))
		goto bad_area;
	if (expand_stack(vma, address))
		goto bad_area;
	/*
	 * Ok, we have a good vm_area for this memory access, so
	 * we can handle it..
	 */
good_area:
	code = SEGV_ACCERR;
	if (write) {
		if (!(vma->vm_flags & VM_WRITE))
			goto bad_area;
	} else {
		/* Allow reads even for write-only mappings */
		if (!(vma->vm_flags & (VM_READ | VM_EXEC)))
			goto bad_area;
	}

	if (from_user)
		flags |= FAULT_FLAG_USER;
	if (write)
		flags |= FAULT_FLAG_WRITE;

	/*
	 * If for any reason at all we couldn't handle the fault,
	 * make sure we exit gracefully rather than endlessly redo
	 * the fault.
	 */
	fault = handle_mm_fault(vma, address, flags);

	if ((fault & VM_FAULT_RETRY) && fatal_signal_pending(current))
		return;

	if (unlikely(fault & VM_FAULT_ERROR)) {
		if (fault & VM_FAULT_OOM)
			goto out_of_memory;
		else if (fault & VM_FAULT_SIGSEGV)
			goto bad_area;
		else if (fault & VM_FAULT_SIGBUS)
			goto do_sigbus;
		BUG();
	}

	if (flags & FAULT_FLAG_ALLOW_RETRY) {
		if (fault & VM_FAULT_MAJOR) {
			current->maj_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MAJ,
				      1, regs, address);
		} else {
			current->min_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MIN,
				      1, regs, address);
		}
		if (fault & VM_FAULT_RETRY) {
			flags &= ~FAULT_FLAG_ALLOW_RETRY;
			flags |= FAULT_FLAG_TRIED;

			/* No need to up_read(&mm->mmap_sem) as we would
			 * have already released it in __lock_page_or_retry
			 * in mm/filemap.c.
			 */

			goto retry;
		}
	}

	up_read(&mm->mmap_sem);
	return;

	/*
	 * Something tried to access memory that isn't in our memory map..
	 * Fix it, but check if it's kernel or user first..
	 */
bad_area:
	up_read(&mm->mmap_sem);

bad_area_nosemaphore:
	/* User mode accesses just cause a SIGSEGV */
	if (from_user) {
		do_fault_siginfo(code, SIGSEGV, regs, text_fault);
		return;
	}

	/* Is this in ex_table? */
no_context:
	g2 = regs->u_regs[UREG_G2];
	if (!from_user) {
		fixup = search_extables_range(regs->pc, &g2);
		/* Values below 10 are reserved for other things */
		if (fixup > 10) {
			extern const unsigned int __memset_start[];
			extern const unsigned int __memset_end[];
			extern const unsigned int __csum_partial_copy_start[];
			extern const unsigned int __csum_partial_copy_end[];

#ifdef DEBUG_EXCEPTIONS
			printk("Exception: PC<%08lx> faddr<%08lx>\n",
			       regs->pc, address);
			printk("EX_TABLE: insn<%08lx> fixup<%08x> g2<%08lx>\n",
				regs->pc, fixup, g2);
#endif
			if ((regs->pc >= (unsigned long)__memset_start &&
			     regs->pc < (unsigned long)__memset_end) ||
			    (regs->pc >= (unsigned long)__csum_partial_copy_start &&
			     regs->pc < (unsigned long)__csum_partial_copy_end)) {
				regs->u_regs[UREG_I4] = address;
				regs->u_regs[UREG_I5] = regs->pc;
			}
			regs->u_regs[UREG_G2] = g2;
			regs->pc = fixup;
			regs->npc = regs->pc + 4;
			return;
		}
	}

	unhandled_fault(address, tsk, regs);
	do_exit(SIGKILL);

/*
 * We ran out of memory, or some other thing happened to us that made
 * us unable to handle the page fault gracefully.
 */
out_of_memory:
	up_read(&mm->mmap_sem);
	if (from_user) {
		pagefault_out_of_memory();
		return;
	}
	goto no_context;

do_sigbus:
	up_read(&mm->mmap_sem);
	do_fault_siginfo(BUS_ADRERR, SIGBUS, regs, text_fault);
	if (!from_user)
		goto no_context;

vmalloc_fault:
	{
		/*
		 * Synchronize this task's top level page-table
		 * with the 'reference' page table.
		 */
		int offset = pgd_index(address);
		pgd_t *pgd, *pgd_k;
		pmd_t *pmd, *pmd_k;

		pgd = tsk->active_mm->pgd + offset;
		pgd_k = init_mm.pgd + offset;

		if (!pgd_present(*pgd)) {
			if (!pgd_present(*pgd_k))
				goto bad_area_nosemaphore;
			pgd_val(*pgd) = pgd_val(*pgd_k);
			return;
		}

		pmd = pmd_offset(pgd, address);
		pmd_k = pmd_offset(pgd_k, address);

		if (pmd_present(*pmd) || !pmd_present(*pmd_k))
			goto bad_area_nosemaphore;

		*pmd = *pmd_k;
		return;
	}
}

void l4_kdb(const char*);
void print_vma(const struct mm_struct* mm);

// wrm
void reset_pte(struct mm_struct* mm, unsigned long addr)
{
	// clear pte
	pgd_t *pgd;
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep;

	pgd = pgd_offset(mm, addr);
	if (pgd_none(*pgd) || unlikely(pgd_bad(*pgd)))
		l4_kdb("pgd failed");

	p4d = p4d_offset(pgd, addr);
	if (p4d_none(*p4d) || unlikely(p4d_bad(*p4d)))
		l4_kdb("p4d failed");

	pud = pud_offset(p4d, addr);
	if (pud_none(*pud) || unlikely(pud_bad(*pud)))
		l4_kdb("pud failed");

	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd) || unlikely(pmd_bad(*pmd)))
		l4_kdb("pmd failed");

	ptep = pte_offset_map(pmd, addr);
	if (!ptep)
		l4_kdb("ptep failed");

	// set pte
	set_pte_at(mm, addr & ~0xfff, ptep, *ptep); // map inside
}

// regs may be NULL or not, if NULL and regs needs for processing - return 1, next call should be with valid regs.
int wrm_do_sparc_fault(struct pt_regs *regs, int from_user, int rwx, unsigned long address, unsigned long* inst)
{
	struct vm_area_struct *vma;
	struct task_struct *tsk = current;
	struct mm_struct *mm = tsk->mm;
	unsigned int fixup;
	unsigned long g2;
	//int from_user = !(regs->psr & PSR_PS);
	int fault, code;
	unsigned int flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_KILLABLE;

	int text_fault = rwx & 0x1;
	int write = rwx & 0x2;


	/* wrm commented
	if (text_fault)
		address = regs->pc;
	*/

	/*
	 * We fault-in kernel-space virtual memory on-demand. The
	 * 'reference' page table is init_mm.pgd.
	 *
	 * NOTE! We MUST NOT take any locks for this case. We may
	 * be in an interrupt or a critical region, and should
	 * only copy the information from the master page table,
	 * nothing more.
	 */
	code = SEGV_MAPERR;
	if (address >= TASK_SIZE)
		goto vmalloc_fault;

	/*
	 * If we're in an interrupt or have no user
	 * context, we must not take the fault..
	 */
	if (pagefault_disabled() || !mm)
		goto no_context;

	perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS, 1, regs, address);

retry:
	down_read(&mm->mmap_sem);

	if (!from_user && address >= PAGE_OFFSET)
		goto bad_area;

	vma = find_vma(mm, address);
	if (!vma)
		goto bad_area;
	if (vma->vm_start <= address)
		goto good_area;
	if (!(vma->vm_flags & VM_GROWSDOWN))
		goto bad_area;
	if (expand_stack(vma, address))
		goto bad_area;
	/*
	 * Ok, we have a good vm_area for this memory access, so
	 * we can handle it..
	 */
good_area:
	code = SEGV_ACCERR;
	if (write) {
		if (!(vma->vm_flags & VM_WRITE))
			goto bad_area;
	} else {
		/* Allow reads even for write-only mappings */
		if (!(vma->vm_flags & (VM_READ | VM_EXEC)))
			goto bad_area;
	}

	if (from_user)
		flags |= FAULT_FLAG_USER;
	if (write)
		flags |= FAULT_FLAG_WRITE;

	/*
	 * If for any reason at all we couldn't handle the fault,
	 * make sure we exit gracefully rather than endlessly redo
	 * the fault.
	 */
	fault = handle_mm_fault(vma, address, flags);

	// wrm
	if (!fault)
	{
		// kernel think already mapped, but in fact may be not mapped --> re-set pte
		reset_pte(mm, address);
	}
	// ~wrm

	if ((fault & VM_FAULT_RETRY) && fatal_signal_pending(current))
		return 0;

	if (unlikely(fault & VM_FAULT_ERROR)) {
		if (fault & VM_FAULT_OOM)
			goto out_of_memory;
		else if (fault & VM_FAULT_SIGSEGV)
			goto bad_area;
		else if (fault & VM_FAULT_SIGBUS)
			goto do_sigbus;
		BUG();
	}

	if (flags & FAULT_FLAG_ALLOW_RETRY) {
		if (fault & VM_FAULT_MAJOR) {
			current->maj_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MAJ,
				      1, regs, address);
		} else {
			current->min_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MIN,
				      1, regs, address);
		}
		if (fault & VM_FAULT_RETRY) {
			flags &= ~FAULT_FLAG_ALLOW_RETRY;
			flags |= FAULT_FLAG_TRIED;

			/* No need to up_read(&mm->mmap_sem) as we would
			 * have already released it in __lock_page_or_retry
			 * in mm/filemap.c.
			 */

			goto retry;
		}
	}

	up_read(&mm->mmap_sem);
	return 0;

	/*
	 * Something tried to access memory that isn't in our memory map..
	 * Fix it, but check if it's kernel or user first..
	 */
bad_area:
	up_read(&mm->mmap_sem);

bad_area_nosemaphore:
	/* User mode accesses just cause a SIGSEGV */
	if (from_user) {
		if (!regs)
			return 1;  // need retry with regs
		do_fault_siginfo(code, SIGSEGV, regs, text_fault);
		return 0;
	}

	/* Is this in ex_table? */
no_context:
	g2 = 0; //regs->u_regs[UREG_G2];
	if (!from_user) {
		fixup = search_extables_range(*inst/*regs->pc*/, &g2);
		/* Values below 10 are reserved for other things */
		if (fixup > 10) {
			extern const unsigned int __memset_start[];
			extern const unsigned int __memset_end[];
			extern const unsigned int __csum_partial_copy_start[];
			extern const unsigned int __csum_partial_copy_end[];

#ifdef DEBUG_EXCEPTIONS
			printk("Exception: PC<%08lx> faddr<%08lx>\n", *inst/*regs->pc*/, address);
			printk("EX_TABLE: insn<%08lx> fixup<%08x> g2<%08lx>\n", *inst/*regs->pc*/, fixup, g2);
#endif
			if ((*inst/*regs->pc*/ >= (unsigned long)__memset_start &&
			     *inst/*regs->pc*/ < (unsigned long)__memset_end) ||
			    (*inst/*regs->pc*/ >= (unsigned long)__csum_partial_copy_start &&
			     *inst/*regs->pc*/ < (unsigned long)__csum_partial_copy_end)) {
				printk("%s:  FIXME:  do work with regs->u_regs[UREG_I4/5].\n", __func__);
				regs->u_regs[UREG_I4] = address;
				regs->u_regs[UREG_I5] = regs->pc;
			}
			#if 1 // wrm
			*inst = fixup;
			#else // orig sparc code
			regs->u_regs[UREG_G2] = g2;
			regs->pc = fixup;
			regs->npc = regs->pc + 4;
			#endif
			return 0;
		}
	}

	if (!regs)
		return 1;  // need retry with regs
	unhandled_fault(address, tsk, regs);
	do_exit(SIGKILL);

/*
 * We ran out of memory, or some other thing happened to us that made
 * us unable to handle the page fault gracefully.
 */
out_of_memory:
	up_read(&mm->mmap_sem);
	if (from_user) {
		pagefault_out_of_memory();
		return 0;
	}
	goto no_context;

do_sigbus:
	up_read(&mm->mmap_sem);
	if (!regs)
		return 1;  // need retry with regs
	do_fault_siginfo(BUS_ADRERR, SIGBUS, regs, text_fault);
	if (!from_user)
		goto no_context;

vmalloc_fault:
	{
		/*
		 * Synchronize this task's top level page-table
		 * with the 'reference' page table.
		 */
		int offset = pgd_index(address);
		pgd_t *pgd, *pgd_k;
		pmd_t *pmd, *pmd_k;

		pgd = tsk->active_mm->pgd + offset;
		pgd_k = init_mm.pgd + offset;

		if (!pgd_present(*pgd)) {
			if (!pgd_present(*pgd_k))
				goto bad_area_nosemaphore;
			pgd_val(*pgd) = pgd_val(*pgd_k);
			return 0;
		}

		pmd = pmd_offset(pgd, address);
		pmd_k = pmd_offset(pgd_k, address);

		if (pmd_present(*pmd) || !pmd_present(*pmd_k))
			goto bad_area_nosemaphore;

		*pmd = *pmd_k;
		return 0;
	}

#if 0















	/*
	 * We fault-in kernel-space virtual memory on-demand. The
	 * 'reference' page table is init_mm.pgd.
	 *
	 * NOTE! We MUST NOT take any locks for this case. We may
	 * be in an interrupt or a critical region, and should
	 * only copy the information from the master page table,
	 * nothing more.
	 */
	//printk("%s:  2 - address=0x%lx.\n", __func__, address);
	code = SEGV_MAPERR;

	if (address >= TASK_SIZE)
	{
		printk("%s:  2.1 - vmalloc_fault:  address=0x%lx >= TASK_SIZE=0x%x, inst=0x%lx, comm=%s.\n", __func__, address, TASK_SIZE, *inst, current->comm);
		goto vmalloc_fault;
	}

	/*
	 * If we're in an interrupt or have no user
	 * context, we must not take the fault..
	 */
	//printk("%s:  3.\n", __func__);
	if (pagefault_disabled() || !mm)
	{
		#if 0
		// wrm:  ignore pagefault_disabled() - it may be BUG, but I don't known how to process it
		//printk("%s:  3.1 - pagefault_disabled=%d, mm=%p, addr=0x%lx.\n", __func__, pagefault_disabled(), mm, address);
		//l4_kdb("debug me");
		#else
		goto no_context;
		#endif
	}
//	perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS, 1, regs, address);

retry:
//printk("%s:  4.\n", __func__);
	down_read(&mm->mmap_sem);   // here may be context switch if CONFIG_PREEMPT_VOLUNTARY
//printk("%s:  4.1\n", __func__);

	if (!from_user && address >= PAGE_OFFSET)
		goto bad_area;

//printk("%s:  5 - before find_vma() - mm=0x%p.\n", __func__, mm);
	vma = find_vma(mm, address);
//printk("%s:  5.1 - after find_vma() - vma=0x%p.\n", __func__, vma);
if (!vma)
 printk("%s:  5.2 - find_vma() - failed, comm=%s, addr=0x%lx.\n", __func__, current->comm, address);
	if (!vma)
		goto bad_area;
//printk("%s:  7.\n", __func__);
	if (vma->vm_start <= address)
		goto good_area;
//printk("%s:  8 - check vm_flags, VM_GROWSDOWN=0x%x.\n", __func__, VM_GROWSDOWN);
	if (!(vma->vm_flags & VM_GROWSDOWN))
		goto bad_area;
//printk("%s:  9.\n", __func__);
	if (expand_stack(vma, address))
		goto bad_area;
//printk("%s:  10.\n", __func__);
	/*
	 * Ok, we have a good vm_area for this memory access, so
	 * we can handle it..
	 */
good_area:
//printk("%s:  11.\n", __func__);
	code = SEGV_ACCERR;
//printk("%s:  12.\n", __func__);
	if (write) {
//printk("%s:  13.\n", __func__);
		if (!(vma->vm_flags & VM_WRITE))
			goto bad_area;
	} else {
//printk("%s:  14.\n", __func__);
		/* Allow reads even for write-only mappings */
		if (!(vma->vm_flags & (VM_READ | VM_EXEC)))
			goto bad_area;
	}
//printk("%s:  15.\n", __func__);

	if (from_user)
		flags |= FAULT_FLAG_USER;
	if (write)
		flags |= FAULT_FLAG_WRITE;

	/*
	 * If for any reason at all we couldn't handle the fault,
	 * make sure we exit gracefully rather than endlessly redo
	 * the fault.
	 */
//printk("%s:  16 - vma=0x%x, address=0x%x, flags=0x%x.\n", __func__, (int)vma, (int)address, (int)flags);
	fault = handle_mm_fault(vma, address, flags);
//printk("%s:  17 - fault=%d.\n", __func__, fault);
if (!fault)
{
	// kernel think already mapped --> re-set pte
	// clear pte
	pgd_t *pgd;
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep;

	pgd = pgd_offset(mm, address);
	if (pgd_none(*pgd) || unlikely(pgd_bad(*pgd)))
		l4_kdb("pgd failed");

	p4d = p4d_offset(pgd, address);
	if (p4d_none(*p4d) || unlikely(p4d_bad(*p4d)))
		l4_kdb("p4d failed");

	pud = pud_offset(p4d, address);
	if (pud_none(*pud) || unlikely(pud_bad(*pud)))
		l4_kdb("pud failed");

	pmd = pmd_offset(pud, address);
	if (pmd_none(*pmd) || unlikely(pmd_bad(*pmd)))
		l4_kdb("pmd failed");

	ptep = pte_offset_map(pmd, address);
	if (!ptep)
		l4_kdb("ptep failed");

	//set_pte(ptep, *ptep);  // map inside
	set_pte_at(mm, address & ~0xfff, ptep, *ptep);
}

	if ((fault & VM_FAULT_RETRY) && fatal_signal_pending(current))
		return 0;

//printk("%s:  18.\n", __func__);
	if (unlikely(fault & VM_FAULT_ERROR)) {
		if (fault & VM_FAULT_OOM)
			goto out_of_memory;
		else if (fault & VM_FAULT_SIGSEGV)
			goto bad_area;
		else if (fault & VM_FAULT_SIGBUS)
			goto do_sigbus;
		BUG();
	}

//printk("%s:  19.\n", __func__);
	if (flags & FAULT_FLAG_ALLOW_RETRY) {
//printk("%s:  20.\n", __func__);
		if (fault & VM_FAULT_MAJOR) {
			current->maj_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MAJ,
				      1, regs, address);
		} else {
			current->min_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MIN,
				      1, regs, address);
		}
		if (fault & VM_FAULT_RETRY) {
			flags &= ~FAULT_FLAG_ALLOW_RETRY;
			flags |= FAULT_FLAG_TRIED;

			/* No need to up_read(&mm->mmap_sem) as we would
			 * have already released it in __lock_page_or_retry
			 * in mm/filemap.c.
			 */

			goto retry;
		}
	}

//printk("%s:  21.\n", __func__);
	up_read(&mm->mmap_sem);
//printk("%s:  22.\n", __func__);
	return;

	/*
	 * Something tried to access memory that isn't in our memory map..
	 * Fix it, but check if it's kernel or user first..
	 */
bad_area:
printk("%s:  23 - ERROR:  bad_area, address=0x%lx.\n", __func__, address);
//print_vma(mm);
//panic("bad area");
	up_read(&mm->mmap_sem);

bad_area_nosemaphore:
printk("%s:  24 - ERROR:  bad_area_nosemaphore.\n", __func__);
//print_vma(mm);
//panic("bad area nosemaphore");
	/* User mode accesses just cause a SIGSEGV */
	if (from_user)
	{
		printk("%s:  24.1 - bad_area_nosemaphore, from_user=%d, address=0x%lx.\n", __func__, from_user, address);
		do_fault_siginfo(code, SIGSEGV, regs, text_fault);
		return;
	}

printk("%s:  24.2 - ERROR:  bad_area_nosemaphore, from_user=%d, address=0x%lx.\n", __func__, from_user, address);
print_vma(mm);
//panic("bad area nosemaphore");

	/* Is this in ex_table? */
no_context:
printk("%s:  25 - ERROR:  no_context, from_user=%d, address=0x%lx.\n", __func__, from_user, address);

 //if (0)
 //{
	g2 = 0; //regs->u_regs[UREG_G2];
	if (!from_user) {
		fixup = search_extables_range(/*regs->pc*/*inst, &g2);
printk("%s:  25.1 - fixup_pc=%x, g2=%lx.\n", __func__, fixup, g2);
		// Values below 10 are reserved for other things
		if (fixup > 10) {
			extern const unsigned int __memset_start[];
			extern const unsigned int __memset_end[];
			extern const unsigned int __csum_partial_copy_start[];
			extern const unsigned int __csum_partial_copy_end[];

#ifdef DEBUG_EXCEPTIONS
			printk("Exception: PC<%08lx> faddr<%08lx>\n",
				/*regs->pc*/*inst, address);
			printk("EX_TABLE: insn<%08lx> fixup<%08x> g2<%08lx>\n",
				/*regs->pc*/*inst, fixup, g2);
#endif
			if ((/*regs->pc*/*inst >= (unsigned long)__memset_start &&
			     /*regs->pc*/*inst < (unsigned long)__memset_end) ||
			    (/*regs->pc*/*inst >= (unsigned long)__csum_partial_copy_start &&
			     /*regs->pc*/*inst < (unsigned long)__csum_partial_copy_end)) {
				printk("%s:  TODO:  inst inside memset() or csum_partial_copy().\n", __func__);
				regs->u_regs[UREG_I4] = address;
				regs->u_regs[UREG_I5] = regs->pc;
			}
			#if 1 // wrm
			*inst = fixup;
			#else
			regs->u_regs[UREG_G2] = g2;
			regs->pc = fixup;
			regs->npc = inst/*regs->pc*/ + 4;
			#endif
			return;
		}
	}
 //}
printk("%s:  26.\n", __func__);
//print_vma(mm);
//panic("no context");
	unhandled_fault(address, tsk, regs);
printk("%s:  27.\n", __func__);
	do_exit(SIGKILL);
printk("%s:  28.\n", __func__);

/*
 * We ran out of memory, or some other thing happened to us that made
 * us unable to handle the page fault gracefully.
 */
out_of_memory:
printk("%s:  29.\n", __func__);
	up_read(&mm->mmap_sem);
	if (from_user) {
		pagefault_out_of_memory();
		return;
	}
	goto no_context;

do_sigbus:
printk("%s:  30.\n", __func__);
	up_read(&mm->mmap_sem);
	do_fault_siginfo(BUS_ADRERR, SIGBUS, regs, text_fault);
	if (!from_user)
		goto no_context;

vmalloc_fault:
printk("%s:  31.\n", __func__);
	{
		/*
		 * Synchronize this task's top level page-table
		 * with the 'reference' page table.
		 */
		int offset = pgd_index(address);
		pgd_t *pgd, *pgd_k;
		pmd_t *pmd, *pmd_k;

		pgd = tsk->active_mm->pgd + offset;
		pgd_k = init_mm.pgd + offset;

		if (!pgd_present(*pgd)) {
			if (!pgd_present(*pgd_k))
				goto bad_area_nosemaphore;
			pgd_val(*pgd) = pgd_val(*pgd_k);
			return;
		}

		pmd = pmd_offset(pgd, address);
		pmd_k = pmd_offset(pgd_k, address);

		if (pmd_present(*pmd) || !pmd_present(*pmd_k))
			goto bad_area_nosemaphore;

		*pmd = *pmd_k;
printk("%s:  32.\n", __func__);
		return;
	}
printk("%s:  33.\n", __func__);
#endif // 0
}

/* This always deals with user addresses. */
static void force_user_fault(unsigned long address, int write)
{
	struct vm_area_struct *vma;
	struct task_struct *tsk = current;
	struct mm_struct *mm = tsk->mm;
	unsigned int flags = FAULT_FLAG_USER;
	int code;

	code = SEGV_MAPERR;

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, address);
	if (!vma)
		goto bad_area;
	if (vma->vm_start <= address)
		goto good_area;
	if (!(vma->vm_flags & VM_GROWSDOWN))
		goto bad_area;
	if (expand_stack(vma, address))
		goto bad_area;
good_area:
	code = SEGV_ACCERR;
	if (write) {
		if (!(vma->vm_flags & VM_WRITE))
			goto bad_area;
		flags |= FAULT_FLAG_WRITE;
	} else {
		if (!(vma->vm_flags & (VM_READ | VM_EXEC)))
			goto bad_area;
	}
	switch (handle_mm_fault(vma, address, flags)) {
	case VM_FAULT_SIGBUS:
	case VM_FAULT_OOM:
		goto do_sigbus;
	}
	up_read(&mm->mmap_sem);
	return;
bad_area:
	up_read(&mm->mmap_sem);
	__do_fault_siginfo(code, SIGSEGV, tsk->thread.kregs, address);
	return;

do_sigbus:
	up_read(&mm->mmap_sem);
	__do_fault_siginfo(BUS_ADRERR, SIGBUS, tsk->thread.kregs, address);
}

static void check_stack_aligned(unsigned long sp)
{
	if (sp & 0x7UL)
		force_sig(SIGILL, current);
}

void window_overflow_fault(void)
{
	unsigned long sp;

	sp = current_thread_info()->rwbuf_stkptrs[0];
	if (((sp + 0x38) & PAGE_MASK) != (sp & PAGE_MASK))
		force_user_fault(sp + 0x38, 1);
	force_user_fault(sp, 1);

	check_stack_aligned(sp);
}

void window_underflow_fault(unsigned long sp)
{
	if (((sp + 0x38) & PAGE_MASK) != (sp & PAGE_MASK))
		force_user_fault(sp + 0x38, 0);
	force_user_fault(sp, 0);

	check_stack_aligned(sp);
}

void window_ret_fault(struct pt_regs *regs)
{
	unsigned long sp;

	sp = regs->u_regs[UREG_FP];
	if (((sp + 0x38) & PAGE_MASK) != (sp & PAGE_MASK))
		force_user_fault(sp + 0x38, 0);
	force_user_fault(sp, 0);

	check_stack_aligned(sp);
}

//##################################################################################################
//
// ...
//
//##################################################################################################

#include <linux/start_kernel.h>
#include <../arch/w4sparc/kernel/kernel.h>
#include <linux/kernel.h>
#include "wrm_log.h"
#include "wlibc_cb.h"
#include "console.h"

#include "uapi/asm/unistd.h"
#include "asm/irqflags_32.h"

// full access memory to use linux kernel as free RAM memory (guest physical address)
asm (".section .free_mem_1,\"awx\",@nobits"); // WA:  to add @nobits
enum
{
	//Free_mem_sz = 16*0x100000  // max possible now
	//Free_mem_sz = 11*0x100000  // min possible now
	//Free_mem_sz = (int)(11.8*0x100000)  // min possible for find-test
	Free_mem_sz = 16*0x100000  // normal short work
};
char free_memory[Free_mem_sz] __attribute__((aligned(0x1000), section(".free_mem_1"))); // here I could not add @nobits
asm (".previous");

// use memory value instead of regiser %g6
struct thread_info* __current_thread_info_ptr;
struct thread_info* current_thread_info(void)   { return __current_thread_info_ptr; }
void set_current_thread_info(struct thread_info* v) { __current_thread_info_ptr = v; }

void cxx_start_native_threads(unsigned vcpus, unsigned aspaces_per_vcpu, int use_console);

unsigned sleep(unsigned sec);
int usleep(unsigned usec);

enum { VCPUS = 1, ASPACES_PER_VCPU = 8 };

extern union thread_union init_thread_union;

void cxx_unmap(unsigned long addr, unsigned long sz);
void unmap_kernel(unsigned long addr, unsigned long sz);
void unmap_kernel_array(unsigned long addrs[], unsigned long sizes[], unsigned arrsz);
void unmap_user(unsigned long addr, unsigned long sz);
void l4_kdb(const char* str);

int cur_eframe_syscall(void);

void w4sparc_local_irq_disable(void) { arch_local_irq_disable();       }
void w4sparc_local_irq_enable(void)  { arch_local_irq_enable();        }
int  w4sparc_local_save_flags(void)  { return arch_local_save_flags(); }

// print all area_struct
void print_vma(const struct mm_struct* mm)
{
	const struct rb_root* root = &mm->mm_rb;
	const struct rb_node* nd = NULL;

	wrm_logi("%s:  +++++++++++++++++++++++++++++++++++++++\n", __func__);
	wrm_logi("%s:  entry:  comm=%s, scall=%u, mm=0x%p:  %lx %lx %lx.\n", __func__,
		current_thread_info()->task->comm, cur_eframe_syscall(), mm, *((long*)mm + 0), *((long*)mm + 4), *((long*)mm + 8));

	for (nd=rb_first(root); nd; nd=rb_next(nd))
	{
		const struct vm_area_struct* vma;
		unsigned long sz;
		vma = rb_entry(nd, struct vm_area_struct, vm_rb);
		sz = vma->vm_end - vma->vm_start;
		printk("%s: --> vma=0x%p:  start=0x%lx  end=0x%lx, sz=0x%lx, flags=0x%lx.\n",
			__func__, vma, vma->vm_start, vma->vm_end, sz, vma->vm_flags);
	}
}

#if 0
// unmap all area_struct
void unmap_all_vma_OLD(const struct mm_struct* mm, int krn, int usr)
{
	const struct rb_root* root = &mm->mm_rb;
	const struct rb_node* nd = NULL;

	//wrm_logi("%s:  +++++++++++++++++++++++++++++++++++++++\n", __func__);
	//wrm_logi("%s:  entry:  comm=%s, scall=%u, mm=0x%x:  %lx %lx %lx, k/u=%d/%d.\n", __func__,
	//	current_thread_info()->task->comm, cur_eframe_syscall(), mm, *((long*)mm + 0), *((long*)mm + 4), *((long*)mm + 8),
	//		krn, usr);

	for (nd=rb_first(root); nd; nd=rb_next(nd))
	{
		const struct vm_area_struct* vma;
		unsigned long sz;
		unsigned long va;
		vma = rb_entry(nd, struct vm_area_struct, vm_rb);
		sz = vma->vm_end - vma->vm_start;
		//printk("%s: --> vma=0x%p:  start=0x%lx  end=0x%lx, sz=0x%lx, flags=0x%lx.\n",
		//	__func__, vma, vma->vm_start, vma->vm_end, sz, vma->vm_flags);

		for (va=vma->vm_start; va<vma->vm_end; va+=0x1000)
		{
			if (krn)
				unmap_kernel(va, 0x1000);
			if (usr)
				unmap_user(va, 0x1000);
		}
	}
}
#endif // 0

// unmap all area_struct
void unmap_all_vma(const struct mm_struct* mm, int krn, int usr)
{
	const struct rb_root* root = &mm->mm_rb;
	const struct rb_node* nd = NULL;
	enum { Sz = 256 };
	static unsigned long addrs[Sz];  // static to decrease stack frame
	static unsigned long sizes[Sz];  // static to decrease stack frame
	unsigned pos = 0;

	//wrm_logi("%s:  +++++++++++++++++++++++++++++++++++++++\n", __func__);
	//wrm_logi("%s:  entry:  comm=%s, scall=%u, mm=0x%x:  %lx %lx %lx, k/u=%d/%d.\n", __func__,
	//	current_thread_info()->task->comm, cur_eframe_syscall(), mm, *((long*)mm + 0),
	//	*((long*)mm + 4), *((long*)mm + 8), krn, usr);

	for (nd=rb_first(root); nd; nd=rb_next(nd))
	{
		const struct vm_area_struct* vma;
		unsigned long start;
		unsigned i;
		vma = rb_entry(nd, struct vm_area_struct, vm_rb);

		if ((long)vma & 0x3)
		{
			wrm_loge("%s: --> vma=0x%p.\n", __func__, vma);
			return;
			l4_kdb("unmap_all_vma:  bad vma");
		}

		if (vma->vm_start & 0xfff  ||  vma->vm_end & 0xfff)
		{
			//wrm_loge("%s: --> vma=0x%p:  start=0x%lx  end=0x%lx, sz=0x%lx, flags=0x%lx.\n", __func__,
			//	vma, vma->vm_start, vma->vm_end, vma->vm_end - vma->vm_start, vma->vm_flags);
			return;
			l4_kdb("unmap_all_vma:  bad vma record");
		}

		start = vma->vm_start;
		//printk("%s: --> vma=0x%p:  start=0x%lx  end=0x%lx, sz=0x%lx, flags=0x%lx.\n", __func__,
		//	vma, vma->vm_start, vma->vm_end, vma->vm_end - vma->vm_start, vma->vm_flags);

		// prepare records with size 1<<31, 1<<30, ... 1<<12
		while (start != vma->vm_end)
		{
			#define get_offset(n, a) ((n) & ((a)-1))
			#define is_aligned(n, a) !get_offset(n, a)
			unsigned long sz = vma->vm_end - start;
			for (i=31; i>=12; --i)
			{
				unsigned long s = 1 << i;
				if (s <= sz  &&  is_aligned(start, s))
				{
					//printk("%s:  a=0x%lx, s=0x%lx.\n", __func__, start, s);
					addrs[pos] = start;
					sizes[pos] = s;
					start += s;
					pos++;
					if (pos == Sz)
					{
						//printk("%s:  arr full.\n", __func__);
						if (krn)
							unmap_kernel_array(addrs, sizes, pos);
						pos = 0;
					}
					break;
				}
			}
		}
	}
	if (pos)
	{
		if (krn)
			unmap_kernel_array(addrs, sizes, pos);
	}
}

void wrm_unmap_range(unsigned long start, unsigned long end)
{
	unsigned long va;
	unsigned long sz = end - start;

return;

	printk("%s:  start=0x%lx  end=0x%lx, sz=0x%lx.\n", __func__, start, end, sz);

	for (va=start; va<end; va+=0x1000)
	{
		cxx_unmap(va, 0x1000);
	}
}

// instead of asm func
void srmmu_set_context(int context)
{
	//wrm_logi("%s:  set new context %d.\n", __func__, context);
}

void print_cur_thread_info(void)
{
	struct thread_info* ti = current_thread_info();

	wrm_logi("trap_ret:  thread_info:      0x%p.\n",  ti);
	wrm_logi("trap_ret:    uwinmask:       0x%lx.\n", ti->uwinmask);
	wrm_logi("trap_ret:    task:           0x%p.\n",  ti->task);
	wrm_logi("trap_ret:    flags:          0x%lx.\n", ti->flags);
	wrm_logi("trap_ret:    cpu:            0x%x.\n",  ti->cpu);
	wrm_logi("trap_ret:    preempt_count:  0x%x.\n",  ti->preempt_count);
	wrm_logi("trap_ret:    softirq_count:  0x%x.\n",  ti->softirq_count);
	wrm_logi("trap_ret:    hardirq_count:  0x%x.\n",  ti->hardirq_count);
	wrm_logi("trap_ret:    ksp:            0x%lx.\n", ti->ksp);
	wrm_logi("trap_ret:    kpc:            0x%lx.\n", ti->kpc);
	wrm_logi("trap_ret:    kpsr:           0x%lx.\n", ti->kpsr);
	wrm_logi("trap_ret:    kwim:           0x%lx.\n", ti->kwim);

	wrm_logi("--------------------------------\n");
	wrm_logi("trap_ret:    task:           0x%p.\n",  ti->task);
	wrm_logi("trap_ret:      comm:         %s.\n",    ti->task->comm);
	wrm_logi("trap_ret:      psr:          0x%lx.\n", ti->task->thread.kregs->psr);
	wrm_logi("trap_ret:      pc:           0x%lx.\n", ti->task->thread.kregs->pc);
	wrm_logi("trap_ret:      npc:          0x%lx.\n", ti->task->thread.kregs->npc);
	wrm_logi("trap_ret:      y:            0x%lx.\n", ti->task->thread.kregs->y);
	wrm_logi("trap_ret:      g0:           0x%lx.\n", ti->task->thread.kregs->u_regs[0]);
	wrm_logi("trap_ret:      g1:           0x%lx.\n", ti->task->thread.kregs->u_regs[1]);
	wrm_logi("trap_ret:      g2:           0x%lx.\n", ti->task->thread.kregs->u_regs[2]);
	wrm_logi("trap_ret:      g3:           0x%lx.\n", ti->task->thread.kregs->u_regs[3]);
	wrm_logi("trap_ret:      g4:           0x%lx.\n", ti->task->thread.kregs->u_regs[4]);
	wrm_logi("trap_ret:      g5:           0x%lx.\n", ti->task->thread.kregs->u_regs[5]);
	wrm_logi("trap_ret:      g6:           0x%lx.\n", ti->task->thread.kregs->u_regs[6]);
	wrm_logi("trap_ret:      g7:           0x%lx.\n", ti->task->thread.kregs->u_regs[7]);
	wrm_logi("trap_ret:      i0:           0x%lx.\n", ti->task->thread.kregs->u_regs[8]);
	wrm_logi("trap_ret:      i1:           0x%lx.\n", ti->task->thread.kregs->u_regs[9]);
	wrm_logi("trap_ret:      i2:           0x%lx.\n", ti->task->thread.kregs->u_regs[10]);
	wrm_logi("trap_ret:      i3:           0x%lx.\n", ti->task->thread.kregs->u_regs[11]);
	wrm_logi("trap_ret:      i4:           0x%lx.\n", ti->task->thread.kregs->u_regs[12]);
	wrm_logi("trap_ret:      i5:           0x%lx.\n", ti->task->thread.kregs->u_regs[13]);
	wrm_logi("trap_ret:      i6:           0x%lx.\n", ti->task->thread.kregs->u_regs[14]);
	wrm_logi("trap_ret:      i7:           0x%lx.\n", ti->task->thread.kregs->u_regs[15]);
}

const char* get_current_comm(void)      { return current_thread_info()->task->comm; }
unsigned long get_current_usr_pc(void)  { return current_thread_info()->task->thread.kregs->pc; }
unsigned long get_current_usr_npc(void) { return current_thread_info()->task->thread.kregs->npc; }
unsigned long get_current_usr_sp(void)  { return current_thread_info()->task->thread.kregs->u_regs[14]; }
unsigned long get_current_usr_i0(void)  { return current_thread_info()->task->thread.kregs->u_regs[8]; }
unsigned long get_current_usr_psr(void) { return current_thread_info()->task->thread.kregs->psr; }

unsigned long* get_current_usr_global(void){ return &current_thread_info()->task->thread.kregs->u_regs[0]; }
unsigned long* get_current_usr_inc(void)   { return &current_thread_info()->task->thread.kregs->u_regs[8]; }
unsigned long* get_current_usr_regwin(void){ return (long*)&current_thread_info()->reg_window[0]; }

struct pt_regs* get_current_kregs(void)    { return current_thread_info()->task->thread.kregs; }
unsigned long get_current_ti_flags(void)   { return current_thread_info()->flags; }

void set_current_usr_pc(unsigned long v)  { current_thread_info()->task->thread.kregs->pc = v; }
void set_current_usr_npc(unsigned long v) { current_thread_info()->task->thread.kregs->npc = v; }
void set_current_usr_sp(unsigned long v)  { current_thread_info()->task->thread.kregs->u_regs[14] = v; }
void set_current_usr_i0(unsigned long v)  { current_thread_info()->task->thread.kregs->u_regs[8] = v; }
void set_current_usr_psr(unsigned long v) { current_thread_info()->task->thread.kregs->psr = v; }

int w4linux_start_kernel(void)
{
	//asm volatile ("set init_thread_union, %g6");
	__current_thread_info_ptr = (struct thread_info*) &init_thread_union;

	sp_banks[0].base_addr = PAGE_OFFSET; // = 0xd0000000, fake paddr
	sp_banks[0].num_bytes = (unsigned long)free_memory - sp_banks[0].base_addr + sizeof(free_memory);

	wrm_logi("sp_bank[0].base_addr=0x%lx, sp_bank[0].num_bytes=0x%lx.\n", sp_banks[0].base_addr, sp_banks[0].num_bytes);

	start_kernel();

	return 0;
}

void process_flags(unsigned long orig_i0)
{
	if (get_current_ti_flags() & _TIF_NEED_RESCHED)
	{
		//wrm_loge("guest exc:  process_flags:  comm=%s:  resched start.\n", get_current_comm());
		schedule();
		//wrm_loge("guest exc:  process_flags:  comm=%s:  resched end.\n", get_current_comm());
	}

	while (get_current_ti_flags() & _TIF_DO_NOTIFY_RESUME_MASK)
	{
		struct pt_regs* regs = get_current_kregs();
		//wrm_loge("guest exc:  process_flags:  comm=%s:  do-notify-resume start.\n", get_current_comm());
		do_notify_resume(regs, orig_i0, get_current_ti_flags());
		//wrm_loge("guest exc:  process_flags:  comm=%s:  do-notify-resume end.\n", get_current_comm());
	}
}


void send_irq(int irq);
void handler_irq(unsigned int pil, struct pt_regs *regs);

void w4sparc_handle_irq(int irq)
{
	handler_irq(irq, get_current_kregs());
}

static int use_console = 0;

void attach_to_system_console(void)
{
	int rc = w4console_init();
	if (!rc)
		rc = w4console_open();
	if (rc)
	{
		wrm_logw("Failed to attach to system console, rc=%d. Use kdb output, no input.\n", rc);
	}
	else
	{
		wrm_logi("App attached to system console.\n");
		w4console_set_wlibc_cb();
		use_console = 1;
	}
}

extern const int wrm_timer_irq;

int main(int argc, const char* argv[])
{
	int i;

	wrm_logi("hello.\n");
	wrm_logi("argc=0x%x, argv=0x%p.\n", argc, argv);

	for (i=0; i<argc; i++)
	{
		wrm_logi("arg[%d] = %s.\n", i, argv[i]);
	}

	attach_to_system_console();

	// run native threads:  mapper, kernel, user.
	cxx_start_native_threads(VCPUS, ASPACES_PER_VCPU, use_console);

	// irq thread
	while (1)
	{
		const int tick_duration_usec = 1000*100;
		usleep(tick_duration_usec);
		send_irq(wrm_timer_irq);
	}

	return 0;
}

//--------------------------------------------------------------------------------------------------
//  WRM ADAPTATION
//--------------------------------------------------------------------------------------------------
void l4_kdb_putsn(const char* str, size_t len);

// GCC may use putchar() as builtin-printf
int putchar(int c)
{
	char ch = c;
	if (use_console)
		w4console_write(&ch, 1); // print via wrm-console-app
	else
		l4_kdb_putsn(&ch, 1);    // print via kdb syscall, this case putting in klog
	return 1;
}

// for wrm_logx() output
int vprintf(const char* format, va_list args)
{
	int res;
	char str[256];
	res = vsnprintf(str, sizeof(str), format, args);
	if (res > 0)
	{
		if (use_console)
			w4console_write(str, res); // print via wrm-console-app
		else
			l4_kdb_putsn(str, res);    // print via kdb syscall, this case putting in klog
	}
	return res;
}

// for wrm_logx() output
int printf(const char* format, ...)
{
	va_list args;
	int res;
	va_start(args, format);
	res = vprintf(format, args);
	va_end(args);
	return res;
}

int my_printf(const char* format, ...)
{
	va_list args;
	int res;
	va_start(args, format);
	res = vprintf(format, args);
	va_end(args);
	return res;
}

//--------------------------------------------------------------------------------------------------
//  ~ WRM ADAPTATION
//--------------------------------------------------------------------------------------------------

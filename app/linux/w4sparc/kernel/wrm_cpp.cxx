//##################################################################################################
//
// ...
//
//##################################################################################################

#include <stdint.h>
#include "wrmos.h"
#include "l4_api.h"
#include "l4_syscalls.h"
#include "console.h"
#include "cbuf.h"
#include "sys_eframe.h"

#include "linux/errno.h"

#include "uapi/asm/unistd.h"
extern const unsigned int sys_call_table[]; // from include/asm/syscall.h

extern "C" int panic(const char*, ...);
extern "C" int my_printf(const char* fmt, ...);
extern "C" unsigned sleep(unsigned sec);
extern "C" unsigned usleep(unsigned sec);
extern "C" void* memcpy(void* dst, const void* src, size_t sz);
extern "C" int wrm_do_sparc_fault(struct pt_regs *regs, int from_user, int rwx, unsigned long address, unsigned long* inst);
extern "C" void print_cur_thread_info();

const char* syscall_name(int nr);

enum
{
	Max_vcpus = 4,
	Max_vcpu_aspaces = 8,

	Exc_level_krn = 1,
	Exc_level_kexc = 2,
};

struct User_aspace_t
{
	L4_thrid_t  thrid_umap;        //
	L4_thrid_t  thrid_usr;         //
	long        owner;             //
	uint64_t    last_access_usec;  // activate time
	char        comm[18];          //
};

struct Vcpu_t
{
	L4_thrid_t thrid_kmap;
	L4_thrid_t thrid_kexc;
	L4_thrid_t thrid_krn;
	L4_thrid_t thrid_upgr;      // XXX: does it need?  may to use krn
	unsigned user_aspace_num;
	unsigned cur_user_aspace;
	User_aspace_t user_aspace[Max_vcpu_aspaces];
};
static Vcpu_t all_vcpus[Max_vcpus];
L4_thrid_t thrid_main;  // init and timer irq
L4_thrid_t thrid_cons;  // console input

Entry_frame_t* current_eframe_krn = 0;
Entry_frame_t* current_eframe_kexc = 0;

int cur_exc_type = 0;

Vcpu_t* cur_vcpu()
{
	int v = 0; // TODO:  support more vcpus
	return &all_vcpus[v];
}

User_aspace_t* cur_uaspace()
{
	Vcpu_t* vcpu = cur_vcpu();
	if (vcpu->cur_user_aspace >= vcpu->user_aspace_num)
		l4_kdb("bad cur_user_aspace");
	return &vcpu->user_aspace[vcpu->cur_user_aspace];
}

void cur_eframe(Entry_frame_t* v)
{
	L4_thrid_t id = l4_utcb()->global_id();
	Vcpu_t* vcpu = cur_vcpu();

	if (id == vcpu->thrid_krn)
		current_eframe_krn = v;
	else if (id == vcpu->thrid_kexc)
		current_eframe_kexc = v;
	else
		l4_kdb("set cur_eframe failed");
}

Entry_frame_t* cur_eframe(int k=0)
{
	L4_thrid_t id = l4_utcb()->global_id();
	Vcpu_t* vcpu = cur_vcpu();

	if (1)
		return current_eframe_krn;

	if (id == vcpu->thrid_krn)
		return current_eframe_krn;
	else if (id == vcpu->thrid_kexc)
		return current_eframe_kexc;

	l4_kdb("get cur_eframe failed");
	return 0;
}

int pending_irqs = 0;
int pending_frcexc = 0;

int inside_pfault_krn = 0;
int inside_pfault_kexc = 0;

extern "C" int cur_eframe_syscall()
{
	Entry_frame_t* e = cur_eframe();
	return e ? e->global_frame.g[1] : 0;
}

//static void unmap_user_from_kernel();
extern "C" void unmap_all_vma(const struct mm_struct* mm, int krn, int usr);
extern "C" void print_vma(const struct mm_struct* mm);
extern "C" void unmap_user(unsigned long addr, unsigned long sz);

extern "C" const char* get_current_comm(void);


void print_uaspaces()
{
	Vcpu_t* vcpu = cur_vcpu();
	for (unsigned i=0; i<vcpu->user_aspace_num; ++i)
	{
		wrm_loge("%s:    i=%u:  owner=%3ld, usrid=%u, %c comm=%s.\n", __func__, i,
			vcpu->user_aspace[i].owner, vcpu->user_aspace[i].thrid_usr.number(),
			i==vcpu->cur_user_aspace ? '*' : ' ', vcpu->user_aspace[i].comm);
	}
}

// find the same or not used for a long time user aspace
void choose_new_uaspace(long owner)
{
	Vcpu_t* vcpu = cur_vcpu();
	if (l4_utcb()->global_id() != vcpu->thrid_krn)
	{
		wrm_loge("%s:  old=%u, old_comm=%s, new_owner=%ld, new_comm=%s.\n", __func__,
			cur_uaspace()->thrid_usr.number(), cur_uaspace()->comm, owner, get_current_comm());
		l4_kdb("l4_utcb()->global_id() != vcpu->thrid_krn");
	}

	unsigned oldest = 0;
	//wrm_loge("%s:  old=%u, old_comm=%s, new_owner=%ld, new_comm=%s.\n", __func__,
	//	cur_uaspace()->thrid_usr.number(), cur_uaspace()->comm, owner, get_current_comm());
	//print_uaspaces();
	for (unsigned i=0; i<vcpu->user_aspace_num; ++i)
	{
		// find the same aspace
		if (vcpu->user_aspace[i].owner == owner)
		{
			vcpu->cur_user_aspace = i;
			vcpu->user_aspace[i].last_access_usec = l4_system_clock();
			memcpy(vcpu->user_aspace[i].comm, get_current_comm(), sizeof(cur_uaspace()->comm)-1);
			//wrm_loge("%s:  new=%u, comm=%s, same.\n", __func__, cur_uaspace()->thrid_usr.number(), cur_uaspace()->comm);
			//print_uaspaces();
			return;
		}

		// or find oldest aspace
		if (vcpu->user_aspace[i].last_access_usec < vcpu->user_aspace[oldest].last_access_usec)
			oldest = i;
	}
	vcpu->cur_user_aspace = oldest;
	vcpu->user_aspace[oldest].last_access_usec = l4_system_clock();
	vcpu->user_aspace[oldest].owner = owner;
	memcpy(vcpu->user_aspace[oldest].comm, get_current_comm(), sizeof(cur_uaspace()->comm)-1);
	//wrm_loge("%s:  new=%u, comm=%s, new.\n", __func__, cur_uaspace()->thrid_usr.number(), cur_uaspace()->comm);
	//print_uaspaces();
	unmap_user(0, 0);  // unmap all new aspace
}

static int process_flags_point = 0;
static int virt_kernel_entry_point[2] = {};

static void unwind_stack()
{
	unsigned long fp;
	unsigned long ra;
	asm volatile ("mov %%i6, %0" : "=r"(fp));
	asm volatile ("mov %%i7, %0" : "=r"(ra));

	wrm_logd("%s:  process_flags_point=%d, virt_kernel_entry_point=%d/%d.\n\n",
		__func__, process_flags_point, virt_kernel_entry_point[0], virt_kernel_entry_point[1]);

	sleep(1);
	cur_eframe(1)->dump(my_printf, false);

	sleep(1);
	wrm_logd("%s:        sp          ra\n", __func__);

	// flush windows to stack
	asm volatile ("save;    save;    save;    save;    save;    save;    save");
	asm volatile ("restore; restore; restore; restore; restore; restore; restore;");

	int n = 30;
	while (n--)
	{
		wrm_logd("%s:  %8lx    %8lx\n", __func__, fp, ra);

		for (int i=0; i<8; ++i)
			wrm_logd("%s:    l%d:  %8lx\n", __func__, i, ((long*)fp)[i]);
		for (int i=0; i<8; ++i)
			wrm_logd("%s:    i%d:  %8lx\n", __func__, i, ((long*)fp)[8+i]);

		ra = ((long*)fp)[8 + 7];
		fp = ((long*)fp)[8 + 6];
	}

	wrm_logd("%s:  end.\n", __func__);
	sleep(1);
	l4_kdb("aaa");
}

// choose usr aspace
extern "C" void w4linux_switch_mm(const struct mm_struct* old_mm, const struct mm_struct* new_mm, long new_ctx)
{
	//wrm_loge("%s:  switch  0x%p -> 0x%p, curctx=%ld, newctx=%ld, scall=%u/%s.\n", __func__, old_mm, new_mm,
	//	cur_uaspace()->owner, new_ctx, cur_eframe_syscall(), syscall_name(cur_eframe_syscall()));

	Vcpu_t* vcpu = cur_vcpu();
	if (l4_utcb()->global_id() != vcpu->thrid_krn)
	{
		wrm_loge("%s:  switch  0x%p -> 0x%p, curctx=%ld, newctx=%ld, scall=%u/%s.\n", __func__, old_mm, new_mm,
			cur_uaspace()->owner, new_ctx, cur_eframe_syscall(), syscall_name(cur_eframe_syscall()));
		wrm_loge("%s:  l4_utcb()->global_id()=%u, vcpu->thrid_krn=%u.\n", __func__,
			l4_utcb()->global_id().number(), vcpu->thrid_krn.number());

		unwind_stack();
		panic("l4_utcb()->global_id() != vcpu->thrid_krn");
		//sleep(1);
		//l4_kdb("l4_utcb()->global_id() != vcpu->thrid_krn");
	}

	// WA:  for next cases memory already unmaped and vma invalid:
	//        - syscall exit_group old_vma invalid;
	//        - unhandled user pfault.
	if ((cur_exc_type == L4_msgtag_t::System_exception   &&  cur_eframe_syscall() == __NR_exit_group) ||
		 cur_exc_type == L4_msgtag_t::Mmu_exception)
	{
		//wrm_logw("%s:  old task calls exit_group, do not touch mm.\n", __func__);
		cur_uaspace()->owner = -1;
	}
	else
	{
		// unmap old from kernel aspace
		#if 1
		//print_vma(old_mm);
		unmap_all_vma(old_mm, 1, 0);
		#else
		unmap_user_from_kernel();
		#endif
	}

	// find the same or not used for a long time user aspace
	choose_new_uaspace(new_ctx);

	//wrm_loge("%s:  done.\n", __func__);
}

extern "C" unsigned long get_current_usr_pc();
extern "C" unsigned long get_current_usr_npc();
extern "C" unsigned long get_current_usr_sp();
extern "C" unsigned long get_current_usr_i0();
extern "C" unsigned long get_current_usr_psr();

extern "C" unsigned long* get_current_usr_global(void);
extern "C" unsigned long* get_current_usr_inc(void);
extern "C" unsigned long* get_current_usr_regwin(void);

extern "C" struct pt_regs* get_current_kregs(void);
extern "C" unsigned long get_current_ti_flags(void);

extern "C" void set_current_usr_pc(unsigned long);
extern "C" void set_current_usr_npc(unsigned long);
extern "C" void set_current_usr_sp(unsigned long);
extern "C" void set_current_usr_i0(unsigned long);
extern "C" void set_current_usr_psr(unsigned long);

extern "C" void w4linux_start_kernel(void);
extern "C" void w4sparc_handle_irq(int irq);
extern "C" void process_flags(unsigned long orig_i0);
extern "C" void w4cons_insert_string(const char* str, unsigned sz);

typedef long (*syscall_func_t)(long, long, long, long, long, long);

void virt_kernel_entry(Entry_frame_t* eframe, int exc_type)
{
static int x = 0;
x = !x;
virt_kernel_entry_point[x] = 1;
	cur_exc_type = exc_type;
virt_kernel_entry_point[x] = 2;
	eframe->flush_regwins(true);
virt_kernel_entry_point[x] = 3;
	//eframe->dump(my_printf, true);
virt_kernel_entry_point[x] = 4;
	set_current_usr_pc(eframe->proc_status_frame.pc);
virt_kernel_entry_point[x] = 5;
	set_current_usr_npc(eframe->proc_status_frame.npc);
virt_kernel_entry_point[x] = 6;
	set_current_usr_psr(eframe->proc_status_frame.psr);
virt_kernel_entry_point[x] = 7;
	memcpy(get_current_usr_global(), &eframe->global_frame,  sizeof(Global_frame_t));
virt_kernel_entry_point[x] = 8;
	memcpy(get_current_usr_inc(),    &eframe->syscall_frame, sizeof(Syscall_frame_t));
virt_kernel_entry_point[x] = 9;
	unsigned last_uwin = 0;
virt_kernel_entry_point[x] = 10;
	memcpy(get_current_usr_regwin(), &eframe->regwin_frame[last_uwin], sizeof(Regwin_frame_t));
virt_kernel_entry_point[x] = 11;
	//print_cur_thread_info();
}

void virt_kernel_exit(Entry_frame_t* eframe, unsigned long orig_i0)
{
	//process_flags(orig_i0); // resched, signals
	//print_cur_thread_info();
	eframe->proc_status_frame.pc  = get_current_usr_pc();
	eframe->proc_status_frame.npc = get_current_usr_npc();
	eframe->proc_status_frame.psr = get_current_usr_psr();
	eframe->proc_status_frame.wim = 1 << ((get_current_usr_psr() + 2) % 8);  // one dirty win
	memcpy(&eframe->global_frame,  get_current_usr_global(), sizeof(eframe->global_frame));
	memcpy(&eframe->syscall_frame, get_current_usr_inc(),    sizeof(eframe->syscall_frame));
	unsigned last_uwin = 0;
	memcpy(&eframe->regwin_frame[last_uwin], get_current_usr_regwin(), sizeof(eframe->regwin_frame[0]));
	cur_exc_type = 0;
}

extern "C" void w4sparc_local_irq_disable(void);
extern "C" void w4sparc_local_irq_enable(void);
extern "C" int w4sparc_local_save_flags(void);

void set_user_force_exception(L4_thrid_t thrid, int frcexc);
void set_force_exception(L4_thrid_t thrid, int frcexc);

static void wait_msg_loop(int exch_level)
{
	while (1)
	{
		L4_thrid_t from = L4_thrid_t::Nil;
		L4_thrid_t from_spec = L4_thrid_t::Any;
		//wrm_logd("wait ipc message from=%d ...\n", from_spec.is_any() ? -1 : from_spec.number());
		L4_time_t tick = L4_time_t::Never;
		int rc = l4_receive(from_spec, tick, &from); // wait msg
		if (rc)
		{
			wrm_loge("%s:  received IPC msg, rc=%d.\n", __func__, rc);
			sleep(1);
			l4_kdb("debug me");
		}

		L4_utcb_t* utcb = l4_utcb();
		L4_msgtag_t tag = utcb->msgtag();
		word_t mr[140];
		if (1 + tag.untyped() + tag.typed() > sizeof(mr)/sizeof(mr[0]))
			l4_kdb("too big ipc");
		memcpy(mr, utcb->mr, (1 + tag.untyped() + tag.typed()) * sizeof(word_t));

		//wrm_logd("%s:  rx:  from=%u, tag=0x%lx, u=%u, t=%u.\n", __func__, from.number(), tag.raw(), tag.untyped(), tag.typed());

		// check sender thread
		//     allow:
		//         - msgs from current uaspace and first exc-msg from each usr-threads,
		//         - from krn-thread for kexc-thread
		//         - nop msg from irq's thread (timer irq thread (main) and cons thread)
		//         - second level pfaults from alpha
		Vcpu_t* vcpu = cur_vcpu();
		L4_thrid_t allowed_exc_thread = (exch_level == Exc_level_krn)  ? cur_uaspace()->thrid_usr :
		                                (exch_level == Exc_level_kexc) ? vcpu->thrid_krn :
		                                L4_thrid_t::Nil;
		static unsigned cnt = 0;
		if (from != allowed_exc_thread  &&  ++cnt > vcpu->user_aspace_num  &&
			from != thrid_main  &&  from != thrid_cons  &&  from != vcpu->thrid_upgr)
		{
			bool propagated = tag.propagated();
			unsigned vsnd = utcb->sender().number();
			wrm_loge("%s:  unexpected from=%u, ipc_label=%ld, u=%u, t=%u.\n", __func__,
				from.number(), tag.ipc_label(), tag.untyped(), tag.typed());
			wrm_loge("%s:  propagated=%d, virtual_sender=%u.\n", __func__, propagated, vsnd);
			sleep(1);
			l4_kdb("debug me");
		}

		//------------------------------------------------------------------------------------------
		// process pfault request
		//------------------------------------------------------------------------------------------
		if (tag.proto_label() == L4_msgtag_t::Pagefault  &&  exch_level == Exc_level_kexc)
		{
			if (exch_level == Exc_level_kexc)
				inside_pfault_kexc++;
			else if (exch_level == Exc_level_krn)  // XXX
				inside_pfault_krn++;

			//wrm_logd("guest pfault:  begin:  inside_pf=%d/%d.\n", inside_pfault_krn, inside_pfault_kexc);

			if (tag.untyped() != 2  ||  tag.typed() != 0)
			{
				wrm_loge("guest pfault:  wrong pfault msg format:  u=%u, t=%u.\n", tag.untyped(), tag.typed());
				continue;
			}

			acc_t  acc  = tag.pfault_access();  // rwx bits
			word_t addr = mr[1];
			word_t inst = mr[2];
			L4_thrid_t fault_thr = from;
			//wrm_logd("guest pfault:  thr=%u, addr=0x%lx, inst=0x%lx, acc=%u.\n",
			// fault_thr.number(), addr, inst, acc);

			L4_map_item_t item;

			if (addr >= 0xd0000000  &&  addr < 0xe0000000)
			{
				// kernel aspace - map 1:1
				//wrm_logd("guest pfault:  kernel aspace, map 1:1.\n");
				L4_fpage_t fpage = L4_fpage_t::create(addr & ~0xfff, 0x1000, acc);
				if (fpage.is_nil())
					l4_kdb("fpage is Nil");
				item = L4_map_item_t::create(fpage);
			}
			else
			{
				// user aspace - process pfault by linux
				//wrm_logd("guest pfault:  user aspace, process by linux...\n");
				rc = wrm_do_sparc_fault(0/*regs*/, 0/*from_user*/, acc, addr, &inst);
				//wrm_logd("guest pfault:  user aspace, process by linux, rc=%d.\n", rc);
				if (rc)
				{
					item = L4_map_item_t::create(L4_fpage_t::create_complete());  // to raise exception for fault thread
				}
				else
				{
					if (inst != mr[2])
					{
						//wrm_logd("guest pfault:  return inst was changed:  %lx -> %lx.\n", mr[2], inst);
						word_t pc[2] = { inst, inst + 4 };
						rc = l4_exreg_ip(fault_thr, pc, 0);  // write new pc
						if (rc)
						{
							wrm_loge("%s:  thr=%d, l4_exreg_ip() - rc=%d.\n", __func__, fault_thr.number(), rc);
							l4_kdb("failed to set ip");
						}
					}
					item = L4_map_item_t::create(L4_fpage_t::create_nil());  // to resume fault thread
				}
			}

			tag.ipc_label(0);
			tag.propagated(false);
			tag.untyped(0);
			tag.typed(2);
			utcb->mr[0] = tag.raw();
			utcb->mr[1] = item.word0();
			utcb->mr[2] = item.word1();
			rc = l4_send(fault_thr, L4_time_t::Zero);
			if (rc)
			{
				wrm_loge("guest pfault:  l4_send(map) failed, rc=%u.\n", rc);
				l4_kdb("sending grant/map item is failed");
			}

			//wrm_logd("guest pfault:  end:  inside_pf=%d/%d.\n", inside_pfault_krn, inside_pfault_kexc);

			if (exch_level == Exc_level_kexc)
				inside_pfault_kexc--;
			else if (exch_level == Exc_level_krn)
				inside_pfault_krn--;
		}
		//------------------------------------------------------------------------------------------
		// process exception request
		//------------------------------------------------------------------------------------------
		else if (tag.is_exc_request())
		{
			Entry_frame_t* eframe = (Entry_frame_t*) &mr[2];
			cur_eframe(eframe);

			//wrm_logd("guest exc:  begin, exch_level=%d, exctype=%d, pc/npc=%lx/%lx.\n",
			// exch_level, tag.proto_label(), eframe->proc_status_frame.pc, eframe->proc_status_frame.npc);

			bool user_exc = from == cur_uaspace()->thrid_usr;
			long pc = eframe->entry_pc();
			long opcode = *(long*) pc;
			//wrm_logd("guest exc:  pc=0x%lx, opcode=0x%lx.\n", pc, opcode);
			//eframe->dump(my_printf, true);

			if (tag.proto_label() == L4_msgtag_t::Force_exception)
			{
				//wrm_logw("irqs:  HELLO:  from=%u.\n", from.number());

				if (!pending_frcexc)
				{
					// possible fake catch of frcexc for user-thread,
					// this is becouse both (u/k) threads got Send_exception state,
					// and first catch krn-thread but impossible cancel exc for usr-thread
					if (from != cur_uaspace()->thrid_usr)
						wrm_loge("irqs:  ERROR:  pending_frcexc=%x(!), pending_irqs=%x, comm=%s.\n",
							pending_frcexc, pending_irqs, get_current_comm());
				}
				else
				{
					//wrm_logw("irqs:  pending_frcexc=1, from=%u.\n", from.number());

					pending_frcexc = 0;                                // cleare global frcexc-pending flag
					if (from == cur_uaspace()->thrid_usr)
						// now we are processing user exc -> clear kernel frcexc-flag
						set_force_exception(vcpu->thrid_krn, 0);
					else
						// now we are processing kernel exc -> clear user frcexc-flag
						set_user_force_exception(cur_uaspace()->thrid_usr, 0);
    
					//if (from != vcpu->thrid_krn)  // usr
					//	wrm_logw("irqs:  HELLO:  from=%u.\n", from.number());
    
					//wrm_logw("irqs:  clear pending vars, from=%u.\n", from.number());
    
					if (inside_pfault_krn || inside_pfault_kexc)
						l4_kdb("irq inside pf");
    
					int irq_on = w4sparc_local_save_flags();
					if (irq_on)
					{
						//wrm_loge("irqs start, comm=%s, pend=0x%x.\n", get_current_comm(), pending_irqs);
    
						//virt_kernel_entry(eframe);
						//unsigned long orig_i0 = eframe->syscall_frame.i[0];
						w4sparc_local_irq_disable();
						for (int i=0; pending_irqs && i<32; ++i)
						{
							if (pending_irqs & (1 << i))
							{
								pending_irqs &= ~(1 << i);
								//wrm_loge("irq start, comm=%s, irq=%u.\n", get_current_comm(), i);
								w4sparc_handle_irq(i);
								//wrm_loge("irq end, comm=%s, irq=%u.\n", get_current_comm(), i);
							}
						}
						w4sparc_local_irq_enable();
						//virt_kernel_exit(eframe, orig_i0);
						//wrm_loge("irqs end, comm=%s, pend=0x%x.\n", get_current_comm(), pending_irqs);
					}
					else
					{
						//wrm_logw("%s:  pend=0x%x, irqs are disabled, processed after enabling.\n", __func__, pending_irqs);
					}
				}
			}
			else if (tag.proto_label() == L4_msgtag_t::System_exception)
			{
				if ((opcode & 0xffffff00) == 0x91d02000)  // trap instruction "ta"
				{
					int tt = opcode & 0xff;

					// ta 0xf0 --> w4linux: suspend user thread
					if (tt == 0xf0)
					{
						// user thread is suspended and wait exc-reply
						// wait all usr-threads and start the kernel
						static unsigned c = 0;
						if (++c == vcpu->user_aspace_num)
						//if (from == cur_uaspace()->thrid_usr)
						{
							w4linux_start_kernel();  // never return here, will call w4sparc_kernel_exit()
						}
						else
						{
							wrm_logi("%s:  suspend usr thread id=%u.\n", __func__, from.number());
							continue;
						}
					}
 					// ta 0x3 --> user wants flush all windows
					else if (tt == 0x3  ||  tt == 0x83)
					{
						// skip trap instruction "ta X"
						eframe->proc_status_frame.pc  = eframe->proc_status_frame.npc;
						eframe->proc_status_frame.npc = eframe->proc_status_frame.npc + 4;

						//wrm_logd("do_flush_windows.\n");
						eframe->flush_regwins(true);
						//wrm_logd("do_flush_windows.\n");
					}
					// ta 0x10 --> linux syscall
					else if (tt == 0x10  ||  tt == 0x90)
					{
						// set return point, skip trap instruction "ta X"
						eframe->proc_status_frame.pc  = eframe->proc_status_frame.npc;
						eframe->proc_status_frame.npc = eframe->proc_status_frame.npc + 4;

						virt_kernel_entry(eframe, L4_msgtag_t::System_exception);
						unsigned long g1 = eframe->global_frame.g[1];
						//wrm_logd("guest exc:  comm=%s:  SYSCALL=%s, nr=0x%lx/%lu, handler=0x%x, inc0=0x%lx.\n",
						//	get_current_comm(), syscall_name(g1), g1, g1, sys_call_table[g1], eframe->syscall_frame.i[0]);
						if (g1 >= NR_syscalls)
							l4_kdb("wrong syscall number");
						unsigned long orig_i0 = eframe->syscall_frame.i[0];
						unsigned long* inc = eframe->syscall_frame.i;
						long res = ((syscall_func_t)(sys_call_table[g1]))(inc[0], inc[1], inc[2], inc[3], inc[4], inc[5]);
						//wrm_logd("guest exc:  comm=%s:  SYSCALL=%s, result 0x%lx/%ld.\n",
						//	get_current_comm(), syscall_name(g1), res, res);
						if ((unsigned long)res >= (unsigned long)-ERESTART_RESTARTBLOCK)
						{
							set_current_usr_psr(get_current_usr_psr() | (1 << 20));  // set carry bit if error
							res = -res;                                              // abs errno
						}
						else
						{
							set_current_usr_psr(get_current_usr_psr() & ~(1 << 20)); // clear carry bit if error
						}
						set_current_usr_i0(res);
process_flags_point=1;
						process_flags(orig_i0); // resched, signals
process_flags_point=2;
						virt_kernel_exit(eframe, orig_i0);
						//wrm_logd("guest exc:  comm=%s:  SYSCALL=%s, return pc/npc=%lx/%lx.\n",
						//	get_current_comm(), syscall_name(g1), eframe->proc_status_frame.pc, eframe->proc_status_frame.npc);
					}
				}
				else
				{
					wrm_loge("%s:  unknown exception instruction:  inst=0x%lx, opcode=0x%lx.\n", __func__, pc, opcode);
					l4_kdb("unknown exception instruction");
				}
			}
			else if (tag.proto_label() == L4_msgtag_t::Mmu_exception)
			{
				unsigned long addr = mr[1] & ~0xfff;
				int acc = mr[1] & 0x7;
				/**/wrm_loge("%s:  Mmu_exception:  inst=0x%lx, addr=0x%lx, acc=%d.\n", __func__, pc, addr, acc);
				virt_kernel_entry(eframe, L4_msgtag_t::Mmu_exception);
				struct pt_regs* regs = get_current_kregs();
				unsigned long orig_i0 = eframe->syscall_frame.i[0];
				if (addr >= 0xd0000000  &&  addr < 0xe0000000)
				{
					wrm_loge("%s:  unexpected addr=0x%lx for Mmu_exception.\n", __func__, addr);
					eframe->dump(my_printf, false);
					l4_kdb("unexpected addr for Mmu_exception");
				}
				word_t inst = eframe->proc_status_frame.pc;
				//wrm_logd("guest pfault:  user aspace, process by linux...\n");
				rc = wrm_do_sparc_fault(regs, 1/*from_user*/, acc, addr, &inst);
				//wrm_logd("guest pfault:  user aspace, process by linux, rc=%d.\n", rc);
				if (rc)
				{
					wrm_loge("%s:  unexpected wrm_do_sparc_fault() - rc=%d.\n", __func__, rc);
					l4_kdb("unexpected wrm_do_sparc_fault() - rc!=0");
				}
process_flags_point=3;
				process_flags(orig_i0); // resched, signals
process_flags_point=4;
				l4_kdb("DBGME:  unexpected exit from process_flags() for Mmu_exception");
			}
			else
			{
				wrm_loge("%s:  unknown exc_type=%ld.\n", __func__, tag.proto_label());
				l4_kdb("unknown exception type");
			}

			//wrm_logw("SEND EXC REPLY:\n");
			//eframe->dump(my_printf, false);

			if (user_exc  &&  from != cur_uaspace()->thrid_usr)
			{
				from = cur_uaspace()->thrid_usr;
			}

			// send exception reply
			tag.ipc_label(0);
			tag.propagated(false);
			tag.untyped(1+132);
			tag.typed(0);
			utcb->mr[0] = tag.raw();
			memcpy(utcb->mr+2, mr+2, sizeof(Entry_frame_t));
			int rc = l4_send(from, L4_time_t::Zero);
			//wrm_logi("guest exc:  send exc reply, rc=%d.\n", rc);
			if (rc)
			{
				wrm_loge("%s:  l4_send(exc) - rc=%u, to=%u.\n", __func__, rc, from.number());
				l4_kdb("sending exception reply failed");
			}
		}
		else
		{
			wrm_loge("%s:  unexpected msg format:  u=%u, t=%u.\n", __func__, tag.untyped(), tag.typed());
			sleep(1);
			l4_kdb("debug me");
		}
	}
}

extern "C" void w4sparc_kernel_exit()
{
	//unsigned long _sp;
	//asm volatile ("mov %%sp, %0" : "=r"(_sp));
	//wrm_loge("%s:  1 - cur_sp=0x%lx.\n", __func__, _sp);
	//print_cur_thread_info();

	L4_utcb_t* utcb = l4_utcb();
	Vcpu_t* vcpu = cur_vcpu();

	if (l4_utcb()->global_id() != vcpu->thrid_krn)
		l4_kdb("unexpected thread");

	Entry_frame_t reply;
	Entry_frame_t* eframe = &reply;

	eframe->proc_status_frame.pc  = get_current_usr_pc();
	eframe->proc_status_frame.npc = get_current_usr_npc();

	eframe->proc_status_frame.psr = get_current_usr_psr();
	eframe->proc_status_frame.wim = 1 << ((get_current_usr_psr() + 2) % 8);  // one dirty win

	memcpy(&eframe->global_frame,  get_current_usr_global(), sizeof(Global_frame_t));
	memcpy(&eframe->syscall_frame, get_current_usr_inc(),    sizeof(Syscall_frame_t));

	int last_uwin = 0;
	memcpy(&eframe->regwin_frame[last_uwin], get_current_usr_regwin(), sizeof(Regwin_frame_t));

	//wrm_loge("%s:  syscall=%s, pc=%lx, npc=%lx.\n", __func__, syscall_name(eframe->global_frame.g1),
	//	eframe->proc_status_frame.pc, eframe->proc_status_frame.npc);

	//eframe->dump(my_printf, false);

	// send exception reply
	L4_msgtag_t tag;
	tag.ipc_label(0);
	tag.propagated(false);
	tag.untyped(1+132);
	tag.typed(0);
	utcb->mr[0] = tag.raw();
	memcpy(utcb->mr+2, eframe, sizeof(Entry_frame_t));
	int rc = l4_send(cur_uaspace()->thrid_usr, L4_time_t::Zero);
	//wrm_logi("guest exc:  send exc reply, rc=%d.\n", rc);
	if (rc)
	{
		wrm_loge("%s:  l4_send(exc) - rc=%u, to=%u.\n", __func__, rc, cur_uaspace()->thrid_usr.number());
		l4_kdb("sending exception reply failed");
	}

	// wait new msg
	wait_msg_loop(Exc_level_krn);  // never return
}

// raise force-exc in current kernel aspace
void set_force_exception(L4_thrid_t thrid, int frcexc)
{
	//wrm_logw("%s:  thr=%d, frcexc=%d.\n", __func__, thrid.number(), frcexc);
	word_t flags = 0;
	int rc = l4_exreg_flags(thrid, 0, &flags); // read
	if (rc)
	{
		wrm_loge("%s:  thr=%d, l4_exreg_flags() - rc=%d.\n", __func__, thrid.number(), rc);
		l4_kdb("failed to get flags");
	}

	word_t new_flags = frcexc ? (flags | L4_flags_frcexc) : (flags & ~L4_flags_frcexc);

	if (flags != new_flags)
	{
		rc = l4_exreg_flags(thrid, &new_flags, 0); // write
		if (rc)
		{
			wrm_loge("%s:  thr=%d, l4_exreg_flags() - rc=%d.\n", __func__, thrid.number(), rc);
			l4_kdb("failed to set flags");
		}
	}

	//if (newval == frcexc)
	//	wrm_loge("%s:  thr=%d, frcexc:  prev=new=%d.\n", __func__, thrid.number(), frcexc);
}

// raise force-exc in alien user aspace
void set_user_force_exception(L4_thrid_t thrid, int frcexc)
{
	//wrm_logw("%s:  thr=%d, frcexc=%d.\n", __func__, thrid.number(), frcexc);
	L4_utcb_t* utcb = l4_utcb();
	L4_msgtag_t tag;
	tag.ipc_label(0x33);
	tag.propagated(false);
	tag.untyped(2);
	tag.typed(0);
	utcb->mr[0] = tag.raw();
	utcb->mr[1] = thrid.raw();
	utcb->mr[2] = frcexc;
	int rc = l4_send(cur_uaspace()->thrid_umap, L4_time_t::Never);
	if (rc)
	{
		wrm_loge("%s:  l4_send(map) - rc=%u.\n", __func__, rc);
		l4_kdb("sending force-exc msg is failed");
	}
}

// send nop-msg to krn-thread re-call IPC
void try_send_nop_msg_to_krn_thread()
{
	Vcpu_t* vcpu = cur_vcpu();
	L4_utcb_t* utcb = l4_utcb();
	L4_msgtag_t tag;
	tag.ipc_label(0);
	tag.propagated(false);
	tag.untyped(0);
	tag.typed(0);
	utcb->mr[0] = tag.raw();
	l4_send(vcpu->thrid_krn, L4_time_t::Zero);  // result does not impotant
}

// called from irq-threads or inside irq_enable
extern "C" void do_force_exception(void)
{
	if (pending_frcexc)
		return;  // already pending

	if (!w4sparc_local_save_flags())
		return;  // irqs disabled

	pending_frcexc = 1;

	Vcpu_t* vcpu = cur_vcpu();

	// interrupt krn-thread
	set_force_exception(vcpu->thrid_krn, 1);

#if 0
	// force-exc flag will be processed when krn-thread do wrm:krn-exit,
	// if usr-thread will work without syscalls/pfaults -> krn-thread hang in IPC,
	// so send nop-msg to re-call IPC to allow process force-exc flag for krn-thread
	try_send_nop_msg_to_krn_thread();
#else
	set_user_force_exception(cur_uaspace()->thrid_usr, 1);
#endif
}

extern "C" void send_irq(int irq)
{
	pending_irqs |= 1 << irq;
    do_force_exception();
}

// wait and process user pagefaults
static int upager_thread(int unused)
{
	wrm_logi("hello:  %s.\n", __func__);
	wrm_logi("my global_id=%u.\n", l4_utcb()->global_id().number());

	Vcpu_t* vcpu = cur_vcpu();
	l4_utcb()->exception_handler(vcpu->thrid_kexc);

	while (1)
	{
		L4_thrid_t from = L4_thrid_t::Nil;
		L4_thrid_t from_spec = L4_thrid_t::Any;
		//wrm_logd("wait ipc message from=%d ...\n", from_spec.is_any() ? -1 : from_spec.number());
		int rc = l4_receive(from_spec, L4_time_t::Never, &from); // wait msg
		if (rc)
		{
			wrm_loge("%s:  received msg, rc=%d.\n", __func__, rc);
			sleep(1);
			l4_kdb("debug me");
		}

		L4_utcb_t* utcb = l4_utcb();
		L4_msgtag_t tag = utcb->msgtag();
		word_t mr[4];
		if (1 + tag.untyped() + tag.typed() > sizeof(mr)/sizeof(mr[0]))
		{
			wrm_loge("%s:  too big ipc:   u=%u, t=%u.\n", __func__, tag.untyped(), tag.typed());
			l4_kdb("too big ipc");
		}
		memcpy(mr, utcb->mr, (1 + tag.untyped() + tag.typed()) * sizeof(word_t));

		//wrm_logd("%s:  rx:  from=%u, tag=0x%x, u=%u, t=%u.\n", __func__, from.number(), tag.raw(), tag.untyped(), tag.typed());

		// check pfault thread:
		//     allow pfaults from current uaspace
		//     and first 3 pfault msgs from each usr-threads
		static unsigned cnt = 0;
		if (from != cur_uaspace()->thrid_usr  &&  ++cnt > 3*vcpu->user_aspace_num)
		{
			wrm_loge("%s:  unexpected from=%u, allow=%u.\n", __func__, from.number(),
				cur_uaspace()->thrid_usr.number());
			sleep(1);
			l4_kdb("debug me");
		}

		if (tag.proto_label() == L4_msgtag_t::Pagefault)
		{
			//wrm_logd("usr pfault:  begin.\n");

			if (tag.untyped() != 2 ||  tag.typed() != 0)
			{
				wrm_loge("usr pfault:  wrong pfault msg format:  u=%u, t=%u.\n", tag.untyped(), tag.typed());
				continue;
			}

			acc_t  acc  = tag.pfault_access();  // rwx bits
			word_t addr = mr[1];
			word_t inst = mr[2];
			L4_thrid_t fault_thr = from;
			//wrm_logd("usr pfault:  thr=%u, addr=0x%lx, inst=0x%lx, opcode=0x%lx, acc=%u.\n",
			// fault_thr.number(), addr, inst, (acc&1 || inst>=0xf0000000) ? 0 : *(long*)inst, acc);

			L4_map_item_t item;

			if (!addr)
			{
				word_t inst = mr[2];
				wrm_loge("guest pfault:  thr=%u, addr=0x%lx, inst=0x%lx, opcode=0x%lx, acc=%u.\n",
					fault_thr.number(), addr, inst, (acc&1 || inst>=0xf0000000) ? 0 : *(long*)inst, acc);
				//l4_kdb("pfault at NULL address");
			}
			else if (addr >= 0xf0000000)
			{
				// wrm kernel aspace - error
				wrm_loge("usr pfault:  thr=%u, inst=0x%lx, acc=%d, addr=0x%lx >= 0xf0000000.\n",
					fault_thr.number(), inst, acc, addr);
				item = L4_map_item_t::create(L4_fpage_t::create_complete());  // to raise exception for fault thread
				//l4_kdb("usr pfault:  addr >= 0xf0000000");
			}
			else if (addr >= 0xd0000000)
			{
				// linux kernel aspace - allow first 3 pfault for linux kernel aspace, map 1:1
				static unsigned cnt = 0;
				cnt++;
				//wrm_logd("usr pfault:  thr=%u, inst=0x%lx, acc=%d, addr=0x%lx >= 0xd0000000, cnt=%u.\n",
				//	fault_thr.number(), inst, acc, addr, cnt);
				if (cnt > 3*vcpu->user_aspace_num)
				{
					wrm_loge("usr pfault:  thr=%u, inst=0x%lx, acc=%d, addr=0x%lx >= 0xd0000000, cnt=%u.\n",
						fault_thr.number(), inst, acc, addr, cnt++);
					sleep(1);
					l4_kdb("usr pfault:  addr >= 0xd0000000");
				}
				//wrm_logd("usr pfault:  kernel aspace, map 1:1.\n");
				L4_fpage_t fpage = L4_fpage_t::create(addr & ~0xfff, 0x1000, acc);
				if (fpage.is_nil())
					l4_kdb("fpage is Nil");
				item = L4_map_item_t::create(fpage);
			}
			else
			{
				// linux user aspace - process pfault by linux
				//wrm_logd("usr pfault:  user aspace, process by linux...\n");
				rc = wrm_do_sparc_fault(0/*regs*/, 1/*from user*/, acc, addr, &inst);
				//wrm_logd("usr pfault:  user aspace, process by linux, rc=%d.\n", rc);
				if (rc)
					item = L4_map_item_t::create(L4_fpage_t::create_complete());  // to raise exception for fault thread
				else
					item = L4_map_item_t::create(L4_fpage_t::create_nil());  // to resume fault thread
			}

			tag.ipc_label(0);
			tag.propagated(false);
			tag.untyped(0);
			tag.typed(2);
			utcb->mr[0] = tag.raw();
			utcb->mr[1] = item.word0();
			utcb->mr[2] = item.word1();
			rc = l4_send(fault_thr, L4_time_t::Zero);
			if (rc)
			{
				wrm_loge("usr pfault:  l4_send(map) failed, rc=%u.\n", rc);
				l4_kdb("sending grant/map item is failed");
			}
			//wrm_logd("usr pfault:  end:  inside_pf=%d/%d.\n", inside_pfault_krn, inside_pfault_kexc);
		}
		else
		{
			wrm_loge("usr pgfault:  unexpected msg format:  u=%u, t=%u.\n", tag.untyped(), tag.typed());
			sleep(1);
			l4_kdb("debug me");
		}
	}

	return 0;
}

// wait and process user exceptions
static int kernel_thread(int exch_level)
{
	wrm_logi("hello:  %s, exch_level=%d.\n", __func__, exch_level);
	wrm_logi("my global_id=%u.\n", l4_utcb()->global_id().number());

	// set exception-handler if need
	if (exch_level == Exc_level_krn)
		l4_utcb()->exception_handler(cur_vcpu()->thrid_kexc);

	wait_msg_loop(exch_level);  // never return

	return 0;
}

// kernel or user mapper thread
// possible:
//  - map operation (2 msg)
//  - unmap operation
//  - set frsexc flag
static int mapper_thread(int is_krn_mapper)
{
	wrm_logi("hello:  %s, krn=%d.\n", __func__, is_krn_mapper);
	wrm_logi("my global_id=%u.\n", l4_utcb()->global_id().number());

	Vcpu_t* vcpu = cur_vcpu();

	L4_thrid_t map_initiator = L4_thrid_t::Nil;

	while (1)
	{
		//wrm_logd("wait ipc message 1 ...\n");

		L4_thrid_t from = L4_thrid_t::Nil;
		L4_thrid_t from_spec = L4_thrid_t::Any; // msg 1 wait from Any
		int rc = l4_receive(from_spec, L4_time_t::Never, &from);
		if (rc)
		{
			wrm_loge("%s:  rx, rc=%d/%s.\n", __func__, rc, l4_ipcerr2str(rc));
			l4_kdb("l4_receive() failed");
		}
		L4_utcb_t* utcb = l4_utcb();
		L4_msgtag_t tag = utcb->msgtag();
		word_t mr[64];
		if (1 + tag.untyped() + tag.typed() > sizeof(mr)/sizeof(mr[0]))
		{
			wrm_logd("%s:  rx:  from=%u, ipc_label=0x%lx, u=%u, t=%u.\n",
				__func__, from.number(), tag.ipc_label(), tag.untyped(), tag.typed());
			l4_kdb("too big ipc msg");
		}
		memcpy(mr, utcb->mr, (1 + tag.untyped() + tag.typed()) * sizeof(word_t));

		//wrm_logd("%s:  rx:  from=%u, u=%u, t=%u.\n", __func__, from.number(), tag.untyped(), tag.typed());
		if (from != thrid_main       &&    // timer irq thread
			from != thrid_cons       &&    // console irq thread
		    from != vcpu->thrid_krn  &&    // map/unmap from krn-thread
			from != vcpu->thrid_kexc &&    // map/unmap from exc-thread
			from != vcpu->thrid_upgr)      // map-unmap from upgr-thread
		{
			wrm_loge("%s:  rx, unexpected sender=%u, allow: %u %u %u %u %u.\n", __func__,
				from.number(),  thrid_main.number(), thrid_cons.number(), vcpu->thrid_krn.number(),
				vcpu->thrid_kexc.number(), vcpu->thrid_upgr.number());
			l4_kdb("l4_receive() unexpected sender");
		}

		// process map request
		if (tag.ipc_label()==0x11 && tag.untyped()==1 && tag.typed()==0)
		{
			if (map_initiator != L4_thrid_t::Nil)
			{
				wrm_loge("%s:  map_initiator(%u) != Nil.\n", __func__, map_initiator.number());
				l4_kdb("from != map_initiator");
			}

			// acceptor request - set acceptor at incoming fpage
			L4_fpage_t fpage = L4_fpage_t::create(mr[1]);
			//wrm_logd("set acceptor at addr=0x%lx, sz=0x%lx, acc=%d.\n",
			//	fpage.addr(), fpage.size(), fpage.access());
			utcb->acceptor(L4_acceptor_t::create(fpage));
			map_initiator = from;
		}
		else if (tag.ipc_label()==0x11 && tag.untyped()==0 && tag.typed()==2)
		{
			if (from != map_initiator)
			{
				wrm_loge("%s:  from(%u) != map_initiator(%u).\n", __func__, from.number(), map_initiator.number());
				l4_kdb("from != map_initiator");
			}

			// map operation was done, clear acceptor and map_initiator
			utcb->acceptor(L4_acceptor_t::create(L4_fpage_t::create_nil()));
			map_initiator = L4_thrid_t::Nil;
			//L4_map_item_t item = L4_map_item_t::create(mr[1], mr[2]);
			//L4_fpage_t fpage = item.fpage();
			//wrm_logd("id=%u:  done mapping for addr=0x%lx, sz=0x%lx, acc=%d.\n",
			//	l4_utcb()->global_id().number(), fpage.addr(), fpage.size(), fpage.access());
		}
		// process unmap request - 1 region
		else if (tag.ipc_label()==0x22 && tag.untyped()==1 && tag.typed()==0)
		{
			// acceptor request - set acceptor at incaming fpage
			L4_fpage_t fpage = L4_fpage_t::create(mr[1]);
			//wrm_logd("id=%u:  unmap user:  addr=0x%lx, sz=0x%lx, acc=%d.\n",
			//	l4_utcb()->global_id().number(), fpage.addr(), fpage.size(), fpage.access());

			word_t control = 0; // 0 - one fpage
			utcb->mr[0] = fpage.raw();
			l4_unmap(control);
		}
		/**/
		// process force-exc request
		else if (tag.ipc_label()==0x33 && tag.untyped()==2 && tag.typed()==0)
		{
			// acceptor request - set acceptor at incaming fpage
			L4_thrid_t thrid(mr[1]);
			int frcexc = mr[2];
			//wrm_logd("frcexc user:  thrid=%u, frcexc=%d.\n", thrid.number(), frcexc);
			if (thrid != cur_uaspace()->thrid_usr)
				l4_kdb("bad thrid for user force-exc");
			set_force_exception(thrid, frcexc);
		}
		/*~*/
		else
		{
			l4_kdb("unexpected msg 1");
		}
	}
}

static int kernel_mapper_thread(int param)
{
	return mapper_thread(param);
}

static int user_mapper_thread(int param)
{
	return mapper_thread(param);
}

static int user_thread(int param)
{
	wrm_logi("hello:  %s, param=0x%x.\n", __func__, param);
	wrm_logi("my global_id=%u.\n", l4_utcb()->global_id().number());

	Vcpu_t* vcpu = cur_vcpu();

	// set exception-handler
	L4_thrid_t id = l4_utcb()->global_id();
	l4_utcb()->exception_handler(vcpu->thrid_krn);

	// to create exception, kernel thread will catch it and
	// change CPU context for execute user applications
	wrm_logi("switch to kernel.\n");
	asm volatile ("ta 0xf0");

	return 0;
}

static char console_input_buf[128];
static Cbuf_t console_input_cbuf;

// return char if success or 0 if error
extern "C" char console_get_input_char()
{
	char c;
	int rc = console_input_cbuf.read(&c, 1);
	return rc==1 ? c : 0;
}

static int console_input_thread(int param)
{
	wrm_logi("hello:  %s, param=0x%x.\n", __func__, param);
	wrm_logi("my global_id=%u.\n", l4_utcb()->global_id().number());

	console_input_cbuf.init(console_input_buf, sizeof(console_input_buf));

	while (1)
	{
		char buf[128];
		int rc = w4console_read(buf, sizeof(buf));
		if (rc < 0)
		{
			wrm_loge("%s:  w4console_read() - rc=%d.\n", __func__, rc);
		}
		else
		{
			console_input_cbuf.write(buf, rc);
			extern const int wrm_w4con_rx_irq;
			send_irq(wrm_w4con_rx_irq);
		}
	}

	return 0;
}

extern "C" void cxx_start_native_threads(unsigned vcpus, unsigned aspaces_per_vcpu)
{
	thrid_main = l4_utcb()->global_id();

	enum { Max_prio = 10 };  // TODO:  read cur prio

	// start vcpu threads
	vcpus = min(vcpus, Max_vcpus);
	for (unsigned v=0; v<vcpus; ++v)
	{
		wrm_logi("create_threads:  vcpu %d ...\n", v);

		Vcpu_t* vcpu = &all_vcpus[v];

		// name:  l-m0, l-k0, l-u0, l-m1, l-k1, l-u1 ...
		char name[4] = "l- ";
		name[3] = v + '0';

		// create mapper thread

		wrm_logi("create_thread:  kernel-mapper ...\n");
		name[2] = 'm';
		L4_fpage_t stack_fp = wrm_mpool_alloc(Cfg_page_sz);
		L4_fpage_t utcb_fp = wrm_mpool_alloc(Cfg_page_sz);
		if (stack_fp.is_nil() || utcb_fp.is_nil())
			l4_kdb("failed to alloc stack and utcb for kernel-mapper thread");
		wrm_logi("                stack_va=0x%lx, stack_sz=0x%lx.\n", stack_fp.addr(), stack_fp.size());

		int rc = wrm_thread_create(utcb_fp, kernel_mapper_thread, 1/*krn*/, stack_fp.addr(),
		                           stack_fp.size(), Max_prio-1, name, Wrm_thr_flag_no, &vcpu->thrid_kmap);

		wrm_logi("                rc=%d, id=%u.\n", rc, vcpu->thrid_kmap.number());
		if (rc)
			l4_kdb("failed to create kernel-mapper thread");

		// create kernel-exc thread

		wrm_logi("create_thread:  kernel-exc thread ...\n");
		name[2] = 'e';
		stack_fp = wrm_mpool_alloc(Cfg_page_sz);
		utcb_fp = wrm_mpool_alloc(Cfg_page_sz);
		if (stack_fp.is_nil() || utcb_fp.is_nil())
			l4_kdb("failed to alloc stack and utcb for kernel-exc thread");
		wrm_logi("                stack_va=0x%lx, stack_sz=0x%lx.\n", stack_fp.addr(), stack_fp.size());

		rc = wrm_thread_create(utcb_fp, kernel_thread, Exc_level_kexc, stack_fp.addr(),
		                       stack_fp.size(), Max_prio-2, name, Wrm_thr_flag_no, &vcpu->thrid_kexc);

		wrm_logi("                rc=%d, id=%u.\n", rc, vcpu->thrid_kexc.number());
		if (rc)
			l4_kdb("failed to create kernel-exc thread");

		// create kernel thread

		wrm_logi("create_thread:  kernel thread ...\n");
		name[2] = 'k';
		stack_fp = wrm_mpool_alloc(Cfg_page_sz);
		utcb_fp = wrm_mpool_alloc(Cfg_page_sz);
		if (stack_fp.is_nil() || utcb_fp.is_nil())
			l4_kdb("failed to alloc stack and utcb for kernel thread");
		wrm_logi("                stack_va=0x%lx, stack_sz=0x%lx.\n", stack_fp.addr(), stack_fp.size());

		rc = wrm_thread_create(utcb_fp, kernel_thread, Exc_level_krn, stack_fp.addr(),
		                       stack_fp.size(), Max_prio-2, name, Wrm_thr_flag_no, &vcpu->thrid_krn);

		wrm_logi("                rc=%d, id=%u.\n", rc, vcpu->thrid_krn.number());
		if (rc)
			l4_kdb("failed to create kernel thread");

		rc = l4_exreg_pager(vcpu->thrid_krn, &vcpu->thrid_kexc, 0);  // write new pager
		if (rc)
			l4_kdb("failed to set pager for kernel thread");

		// create usr-pager thread

		wrm_logi("create_thread:  pager thread ...\n");
		name[2] = 'p';
		stack_fp = wrm_mpool_alloc(Cfg_page_sz);
		utcb_fp = wrm_mpool_alloc(Cfg_page_sz);
		if (stack_fp.is_nil() || utcb_fp.is_nil())
			l4_kdb("failed to alloc stack and utcb for kernel thread");
		wrm_logi("                stack_va=0x%lx, stack_sz=0x%lx.\n", stack_fp.addr(), stack_fp.size());

		rc = wrm_thread_create(utcb_fp, upager_thread, 0, stack_fp.addr(),
		                       stack_fp.size(), Max_prio-2, name, Wrm_thr_flag_no, &vcpu->thrid_upgr);

		wrm_logi("                rc=%d, id=%u.\n", rc, vcpu->thrid_upgr.number());
		if (rc)
			l4_kdb("failed to create usr-pager thread");

		rc = l4_exreg_pager(vcpu->thrid_upgr, &vcpu->thrid_kexc, 0);  // write new pager
		if (rc)
			l4_kdb("failed to set pager for upager thread");

		// create user tasks
		vcpu->user_aspace_num = min(aspaces_per_vcpu, Max_vcpu_aspaces);
		vcpu->cur_user_aspace = 0;
		for (unsigned t=0; t<vcpu->user_aspace_num; ++t)
		{
			User_aspace_t* user = &vcpu->user_aspace[t];
			user->owner = -1;
			user->last_access_usec = 0;

			wrm_logi("create_task:    user task ...\n");
			name[2] = 'u';
			unsigned threads_max = 2;  // 2 user threads 'u' and 'U'
			stack_fp = wrm_mpool_alloc(Cfg_page_sz);
			L4_fpage_t utcbs_fp = wrm_mpool_alloc(threads_max * Cfg_page_sz);
			if (stack_fp.is_nil() || utcbs_fp.is_nil())
				l4_kdb("failed to alloc stack and utcb for user-task");
			wrm_logi("                stack_va=0x%lx, stack_sz=0x%lx.\n", stack_fp.addr(), stack_fp.size());

			L4_fpage_t kip_area = L4_fpage_t::create(0x3000, Cfg_page_sz, Acc_rx);
			//L4_thrid_t pager = vcpu->thrid_krn; // set kernel-thread as pager
			L4_thrid_t pager = vcpu->thrid_upgr;   // set pager-thread as pager
			rc = wrm_task_create(utcbs_fp, user_thread, 0, stack_fp.addr(), stack_fp.size(),
			                     Max_prio-2, name, Wrm_thr_flag_fpu, pager, kip_area,
			                     &user->thrid_usr);

			wrm_logi("                rc=%d, id=%u.\n", rc, user->thrid_usr.number());
			if (rc)
				l4_kdb("failed to create user task");

			// create user-mapper thread

			wrm_logi("create_thread:  user-mapper thread ...\n");
			name[2] = 'U';
			stack_fp = wrm_mpool_alloc(Cfg_page_sz);
			utcb_fp = wrm_mpool_alloc(Cfg_page_sz);
			if (stack_fp.is_nil() || utcb_fp.is_nil())
				l4_kdb("failed to alloc stack and utcb for user-mapper");
			wrm_logi("                stack_va=0x%lx, stack_sz=0x%lx.\n", stack_fp.addr(), stack_fp.size());

			L4_thrid_t space = user->thrid_usr;
			rc = wrm_thread_create(utcb_fp, user_mapper_thread, 0/*usr*/, stack_fp.addr(),
			                       stack_fp.size(), Max_prio-1, name, Wrm_thr_flag_no,
			                       &user->thrid_umap, space);

			wrm_logi("                rc=%d, id=%u.\n", rc, user->thrid_umap.number());
			if (rc)
				l4_kdb("failed to create user-mapper thread");
		}
	}

	// create console input thread

	wrm_logi("create_thread:  console input thread ...\n");
	L4_fpage_t stack_fp = wrm_mpool_alloc(Cfg_page_sz);
	L4_fpage_t utcb_fp = wrm_mpool_alloc(Cfg_page_sz);
	if (stack_fp.is_nil() || utcb_fp.is_nil())
		l4_kdb("failed to alloc stack and utcb for console input thread");
	wrm_logi("                stack_va=0x%lx, stack_sz=0x%lx.\n", stack_fp.addr(), stack_fp.size());
	int rc = wrm_thread_create(utcb_fp, console_input_thread, 0, stack_fp.addr(),
	                           stack_fp.size(), Max_prio-0, "l-in", Wrm_thr_flag_no, &thrid_cons);
	wrm_logi("                rc=%d, id=%u.\n", rc, thrid_cons.number());
	if (rc)
		l4_kdb("failed to create console input thread");
}

void map(int krn, unsigned long src_addr, unsigned long dst_addr, unsigned long sz, int acc)
{
	//wrm_logd("map:  krn=%d:  src=0x%lx, dst=0x%lx, sz=0x%lx, acc=%d.\n", krn, src_addr, dst_addr, sz, acc);

	Vcpu_t* vcpu = cur_vcpu();

	L4_thrid_t thrid = krn ? vcpu->thrid_kmap : cur_uaspace()->thrid_umap;

	// send acceptor with dst_addr
	//wrm_logd("map:  send acceptor.\n");
	L4_fpage_t fpage = L4_fpage_t::create(dst_addr, sz, acc);
	if (fpage.is_nil())
	{
		wrm_loge("map:  creation fpage is failed - dst_addr=0x%lx, sz=0x%lx, acc=%d.\n", dst_addr, sz, acc);
		l4_kdb("creation acceptor is failed");
	}
	L4_utcb_t* utcb = l4_utcb();
	L4_msgtag_t tag;
	tag.ipc_label(0x11);
	tag.propagated(false);
	tag.untyped(1);
	tag.typed(0);
	utcb->mr[0] = tag.raw();
	utcb->mr[1] = fpage.raw();
	int rc = l4_send(thrid, L4_time_t::Never);
	if (rc)
	{
		wrm_loge("map:  l4_send(map) - rc=%u.\n", rc);
		l4_kdb("sending grant/map item is failed");
	}

	// send map items src_addr
	//wrm_logd("map:  send map item.\n");
	L4_map_item_t item = L4_map_item_t::create(L4_fpage_t::create(src_addr, sz, acc));
	tag.ipc_label(0x11);
	tag.propagated(false);
	tag.untyped(0);
	tag.typed(2);
	utcb->mr[0] = tag.raw();
	utcb->mr[1] = item.word0();
	utcb->mr[2] = item.word1();
	rc = l4_send(thrid, L4_time_t::Never);
	if (rc)
	{
		wrm_loge("map:  l4_send(map) - rc=%u.\n", rc);
		l4_kdb("sending grant/map item is failed");
	}
}

void map_kernel(unsigned long src_addr, unsigned long dst_addr, unsigned long sz, int acc)
{
	//wrm_logd("map_krn:  src=0x%lx, dst=0x%lx, sz=0x%lx, acc=%d.\n", src_addr, dst_addr, sz, acc);
	map(1, src_addr, dst_addr, sz, acc);
}

void map_user(unsigned long src_addr, unsigned long dst_addr, unsigned long sz, int acc)
{
	//wrm_logd("map_user:  src=0x%lx, dst=0x%lx, sz=0x%lx, acc=%d.\n", src_addr, dst_addr, sz, acc);
	map(0, src_addr, dst_addr, sz, acc);
}

extern "C" void cxx_unmap(unsigned long addr, unsigned long sz);

extern "C" void cxx_map(unsigned long src_addr, unsigned long dst_addr, unsigned long sz, int acc)
{
	//wrm_logd("map:  src=0x%lx, dst=0x%lx, sz=0x%lx, acc=%d/%d.\n", src_addr, dst_addr, sz, (acc >> 3) & 7, acc & 7);
	cxx_unmap(dst_addr, sz); // unmap old mapping
	int krn_rwx = (acc >> 3) & 7;
	int usr_rwx = acc & 7;
	if (krn_rwx)
		map_kernel(src_addr, dst_addr, sz, krn_rwx);
	if (usr_rwx)
		map_user(src_addr, dst_addr, sz, usr_rwx);
}

extern "C" void unmap_kernel(unsigned long addr, unsigned long sz)
{
	//wrm_logd("unmap kernel:  va=0x%lx, sz=0x%lx.\n", addr, sz);
	L4_fpage_t fpage = L4_fpage_t::create(addr, sz, 7/*rwx*/);
	if (fpage.is_nil())
	{
		wrm_loge("%s:  creation fpage is failed - va=0x%lx, sz=0x%lx.\n", __func__, addr, sz);
		l4_kdb("creation fpage is failed");
	}
	word_t control = 0; // 0 - one fpage
	l4_utcb()->mr[0] = fpage.raw();
	l4_unmap(control);
}

extern "C" void unmap_kernel_array(unsigned long addrs[], unsigned long sizes[], unsigned arrsz)
{
	//wrm_logd("unmap kernel:  va=0x%lx, sz=0x%lx.\n", addr, sz);
	if (arrsz > L4_utcb_t::Mr_words)
	{
		wrm_loge("%s:  too big arrsz=%u, max=%u.\n", __func__, arrsz, L4_utcb_t::Mr_words);
		l4_kdb("creation fpage is failed");
	}
	L4_utcb_t* utcb = l4_utcb();
	for (unsigned i=0; i<arrsz; ++i)
	{
		L4_fpage_t fpage = L4_fpage_t::create(addrs[i], sizes[i], 7/*rwx*/);
		if (fpage.is_nil())
		{
			wrm_loge("%s:  creation fpage is failed - va=0x%lx, sz=0x%lx.\n", __func__, addrs[i], sizes[i]);
			l4_kdb("creation fpage is failed");
		}
		utcb->mr[i] = fpage.raw();
	}
	word_t control = arrsz - 1;
	l4_unmap(control);
}

extern "C" void unmap_user(unsigned long addr, unsigned long sz)
{
	//wrm_logd("unmap user:  va=0x%lx, sz=0x%lx.\n", addr, sz);
	L4_fpage_t fpage = (!addr && !sz) ? L4_fpage_t::create_complete() : L4_fpage_t::create(addr, sz, 7/*rwx*/);
	if (fpage.is_nil())
	{
		wrm_loge("%s:  creation fpage is failed - va=0x%lx, sz=0x%lx.\n", __func__, addr, sz);
		l4_kdb("creation fpage is failed");
	}
	L4_utcb_t* utcb = l4_utcb();
	L4_msgtag_t tag;
	tag.ipc_label(0x22);
	tag.propagated(false);
	tag.untyped(1);
	tag.typed(0);
	utcb->mr[0] = tag.raw();
	utcb->mr[1] = fpage.raw();
	int rc = l4_send(cur_uaspace()->thrid_umap, L4_time_t::Never);
	if (rc)
	{
		wrm_loge("%s:  l4_send(map) - rc=%u.\n", __func__, rc);
		l4_kdb("sending grant/map item is failed");
	}
}

extern "C" void cxx_unmap(unsigned long addr, unsigned long sz)
{
	//wrm_logd("unmap:  va=0x%lx, sz=0x%lx.\n", addr, sz);
	unmap_kernel(addr, sz);
	unmap_user(addr, sz);
}

extern "C" long do_fork(unsigned long clone_flags,
                        unsigned long stack_start,
                        unsigned long stack_size,
                        unsigned long parent_tidptr,
                        unsigned long child_tidptr);

// based on kernel/entry.S::sys_clone().
extern "C" int sys_clone(unsigned long flags, unsigned long stack, unsigned long ptid,
                         unsigned long tls, unsigned long ctid)
{
	//static unsigned cnt = 0;
	//wrm_logd("clone:  cnt=%u, flags=0x%lx, stack=0x%lx, ptid=0x%lx, tls=0x%lx, ctid=0x%lx.\n",
	//	cnt++, flags, stack, ptid, tls, ctid);

	stack = cur_eframe()->syscall_frame.i6;

	return do_fork(flags, stack, 0/*stack_size*/, ptid, ctid);
}

extern "C" int sparc_do_fork(unsigned long clone_flags,
                             unsigned long stack_start,
                             struct pt_regs *regs,
                             unsigned long stack_size);

// based on kernel/entry.S::sys_vfork().
extern "C" int sys_vfork(unsigned long flags, unsigned long stack, unsigned long ptid,
                          unsigned long tls, unsigned long ctid)
{
	//static unsigned cnt = 0;
	//wrm_logd("vfork:  cnt=%u, flags=0x%lx, stack=0x%lx, ptid=0x%lx, tls=0x%lx, ctid=0x%lx.\n",
	//	cnt++, flags, stack, ptid, tls, ctid);

	flags = 0x4000 | 0x100 | 0x14 /*SIGCHLD*/;
	stack = cur_eframe()->syscall_frame.i6;
	struct pt_regs* regs = get_current_kregs();

	return sparc_do_fork(flags, stack, regs, 0/*stack_size*/);
}

extern "C" long sparc_pipe(struct pt_regs *regs);

// based on kernel/entry.S::sys_vfork().
extern "C" int sys_sparc_pipe(int* fildes)
{
	//wrm_logd("pipe:  fildes=%p.\n", fildes);
	struct pt_regs* regs = get_current_kregs();
	return sparc_pipe(regs);
}

extern "C" void do_sigreturn(struct pt_regs *regs);

// based on kernel/entry.S::sys_sigreturn().
extern "C" int sys_sigreturn()
{
	//wrm_logd("sigreturn:  hi.\n");
	struct pt_regs* regs = get_current_kregs();
	do_sigreturn(regs);
	return get_current_usr_i0();
}

//just to linker be happy
extern "C" void flush_patch_three() { l4_kdb("DEBUGME:  flush_patch_three()"); }
extern "C" void flush_patch_four()  { l4_kdb("DEBUGME:  flush_patch_four()");  }


extern "C" void w4linux_idle(void)
{
	#if 0
	// allow to execute thread with prio >= current
	l4_thread_switch(L4_thrid_t::Nil);
	#else
	// allow to execute threads with prio < current
	usleep(1*1000); // 1 ms
	#endif
}

extern "C" void w4linux_check_wrm_thread(void)
{
	Vcpu_t* vcpu = cur_vcpu();
	if (l4_utcb()->global_id() != vcpu->thrid_krn)
	{
		wrm_loge("%s:  l4_utcb()->global_id()=%u, vcpu->thrid_krn=%u.\n", __func__,
			l4_utcb()->global_id().number(), vcpu->thrid_krn.number());
		panic("l4_utcb()->global_id() != vcpu->thrid_krn");
		//sleep(1);
		//l4_kdb("l4_utcb()->global_id() != vcpu->thrid_krn");
	}
}

const char* syscall_name(int nr)
{
	switch (nr)
	{
		case __NR_restart_syscall:        return "restart";
		case __NR_exit:                   return "exit";
		case __NR_fork:                   return "fork";
		case __NR_read:                   return "read";
		case __NR_write:                  return "write";
		case __NR_open:                   return "open";
		case __NR_close:                  return "close";
		case __NR_wait4:                  return "wait4";
		case __NR_creat:                  return "creat";
		case __NR_link:                   return "link";
		case __NR_unlink:                 return "unlink";
		case __NR_execv:                  return "execv";
		case __NR_chdir:                  return "chdir";
		case __NR_chown:                  return "chown";
		case __NR_mknod:                  return "mknod";
		case __NR_chmod:                  return "chmod";
		case __NR_lchown:                 return "lchown";
		case __NR_brk:                    return "brk";
		case __NR_perfctr:                return "perfctr";
		case __NR_lseek:                  return "lseek";
		case __NR_getpid:                 return "getpid";
		case __NR_capget:                 return "capget";
		case __NR_capset:                 return "capset";
		case __NR_setuid:                 return "setuid";
		case __NR_getuid:                 return "getuid";
		case __NR_vmsplice:               return "vmsplice";
		case __NR_ptrace:                 return "ptrace";
		case __NR_alarm:                  return "alarm";
		case __NR_sigaltstack:            return "sigaltstack";
		case __NR_pause:                  return "pause";
		case __NR_utime:                  return "utime";
		#ifdef __32bit_syscall_numbers__
		case __NR_lchown32:               return "lchown32";
		case __NR_fchown32:               return "fchown32";
		#endif
		case __NR_access:                 return "access";
		case __NR_nice:                   return "nice";
		#ifdef __32bit_syscall_numbers__
		case __NR_chown32:                return "chown32";
		#endif
		case __NR_sync:                   return "sync";
		case __NR_kill:                   return "kill";
		case __NR_stat:                   return "stat";
		case __NR_sendfile:               return "sendfile";
		case __NR_lstat:                  return "lstat";
		case __NR_dup:                    return "dup";
		case __NR_pipe:                   return "pipe";
		case __NR_times:                  return "times";
		#ifdef __32bit_syscall_numbers__
		case __NR_getuid32:               return "getuid32";
		#endif
		case __NR_umount2:                return "umount2";
		case __NR_setgid:                 return "setgid";
		case __NR_getgid:                 return "getgid";
		case __NR_signal:                 return "signal";
		case __NR_geteuid:                return "geteuid";
		case __NR_getegid:                return "getegid";
		case __NR_acct:                   return "acct";
		#ifdef __32bit_syscall_numbers__
		case __NR_getgid32:               return "getgid32";
		#else
		case __NR_memory_ordering:        return "memory_ordering";
		#endif
		case __NR_ioctl:                  return "ioctl";
		case __NR_reboot:                 return "reboot";
		#ifdef __32bit_syscall_numbers__
		case __NR_mmap2:                  return "mmap2";
		#endif
		case __NR_symlink:                return "symlink";
		case __NR_readlink:               return "readlink";
		case __NR_execve:                 return "execve";
		case __NR_umask:                  return "umask";
		case __NR_chroot:                 return "chroot";
		case __NR_fstat:                  return "fstat";
		case __NR_fstat64:                return "fstat64";
		case __NR_getpagesize:            return "getpagesize";
		case __NR_msync:                  return "msync";
		case __NR_vfork:                  return "vfork";
		case __NR_pread64:                return "pread64";
		case __NR_pwrite64:               return "pwrite64";
		#ifdef __32bit_syscall_numbers__
		case __NR_geteuid32:              return "geteuid32";
		case __NR_getegid32:              return "getegid32";
		#endif
		case __NR_mmap:                   return "mmap";
		#ifdef __32bit_syscall_numbers__
		case __NR_setreuid32:             return "setreuid32";
		#endif
		case __NR_munmap:                 return "munmap";
		case __NR_mprotect:               return "mprotect";
		case __NR_madvise:                return "madvise";
		case __NR_vhangup:                return "vhangup";
		#ifdef __32bit_syscall_numbers__
		case __NR_truncate64:             return "truncate64";
		#endif
		case __NR_mincore:                return "mincore";
		case __NR_getgroups:              return "getgroups";
		case __NR_setgroups:              return "setgroups";
		case __NR_getpgrp:                return "getpgrp";
		#ifdef __32bit_syscall_numbers__
		case __NR_setgroups32:            return "setgroups32";
		#endif
		case __NR_setitimer:              return "setitimer";
		#ifdef __32bit_syscall_numbers__
		case __NR_ftruncate64:            return "ftruncate64";
		#endif
		case __NR_swapon:                 return "swapon";
		case __NR_getitimer:              return "getitimer";
		#ifdef __32bit_syscall_numbers__
		case __NR_setuid32:               return "setuid32";
		#endif
		case __NR_sethostname:            return "sethostname";
		#ifdef __32bit_syscall_numbers__
		case __NR_setgid32:               return "setgid32";
		#endif
		case __NR_dup2:                   return "dup2";
		#ifdef __32bit_syscall_numbers__
		case __NR_setfsuid32:             return "setfsuid32";
		#endif
		case __NR_fcntl:                  return "fcntl";
		case __NR_select:                 return "select";
		#ifdef __32bit_syscall_numbers__
		case __NR_setfsgid32:             return "setfsgid32";
		#endif
		case __NR_fsync:                  return "fsync";
		case __NR_setpriority:            return "setpriority";
		case __NR_socket:                 return "socket";
		case __NR_connect:                return "connect";
		case __NR_accept:                 return "accept";
		case __NR_getpriority:            return "getpriority";
		case __NR_rt_sigreturn:           return "rt_sigreturn";
		case __NR_rt_sigaction:           return "rt_sigaction";
		case __NR_rt_sigprocmask:         return "rt_sigprocmask";
		case __NR_rt_sigpending:          return "rt_sigpending";
		case __NR_rt_sigtimedwait:        return "rt_sigtimedwait";
		case __NR_rt_sigqueueinfo:        return "rt_sigqueueinfo";
		case __NR_rt_sigsuspend:          return "rt_sigsuspend";
		#ifdef __32bit_syscall_numbers__
		case __NR_setresuid32:            return "setresuid32";
		case __NR_getresuid32:            return "getresuid32";
		case __NR_setresgid32:            return "setresgid32";
		case __NR_getresgid32:            return "getresgid32";
		case __NR_setregid32:             return "setregid32";
		#else
		case __NR_setresuid:              return "setresuid";
		case __NR_getresuid:              return "getresuid";
		case __NR_setresgid:              return "setresgid";
		case __NR_getresgid:              return "getresgid";
		#endif
		case __NR_recvmsg:                return "recvmsg";
		case __NR_sendmsg:                return "sendmsg";
		#ifdef __32bit_syscall_numbers__
		case __NR_getgroups32:            return "getgroups32";
		#endif
		case __NR_gettimeofday:           return "gettimeofday";
		case __NR_getrusage:              return "getrusage";
		case __NR_getsockopt:             return "getsockopt";
		case __NR_getcwd:                 return "getcwd";
		case __NR_readv:                  return "readv";
		case __NR_writev:                 return "writev";
		case __NR_settimeofday:           return "settimeofday";
		case __NR_fchown:                 return "fchown";
		case __NR_fchmod:                 return "fchmod";
		case __NR_recvfrom:               return "recvfrom";
		case __NR_setreuid:               return "setreuid";
		case __NR_setregid:               return "setregid";
		case __NR_rename:                 return "rename";
		case __NR_truncate:               return "truncate";
		case __NR_ftruncate:              return "ftruncate";
		case __NR_flock:                  return "flock";
		case __NR_lstat64:                return "lstat64";
		case __NR_sendto:                 return "sendto";
		case __NR_shutdown:               return "shutdown";
		case __NR_socketpair:             return "socketpair";
		case __NR_mkdir:                  return "mkdir";
		case __NR_rmdir:                  return "rmdir";
		case __NR_utimes:                 return "utimes";
		case __NR_stat64:                 return "stat64";
		case __NR_sendfile64:             return "sendfile64";
		case __NR_getpeername:            return "getpeername";
		case __NR_futex:                  return "futex";
		case __NR_gettid:                 return "gettid";
		case __NR_getrlimit:              return "getrlimit";
		case __NR_setrlimit:              return "setrlimit";
		case __NR_pivot_root:             return "pivot_root";
		case __NR_prctl:                  return "prctl";
		case __NR_pciconfig_read:         return "pciconfig_read";
		case __NR_pciconfig_write:        return "pciconfig_write";
		case __NR_getsockname:            return "getsockname";
		case __NR_inotify_init:           return "inotify_init";
		case __NR_inotify_add_watch:      return "inotify_add_watch";
		case __NR_poll:                   return "poll";
		case __NR_getdents64:             return "getdents64";
		#ifdef __32bit_syscall_numbers__
		case __NR_fcntl64:                return "fcntl64";
		#endif
		case __NR_inotify_rm_watch:       return "inotify_rm_watch";
		case __NR_statfs:                 return "statfs";
		case __NR_fstatfs:                return "fstatfs";
		case __NR_umount:                 return "umount";
		case __NR_sched_set_affinity:     return "sched_set_affinity";
		case __NR_sched_get_affinity:     return "sched_get_affinity";
		case __NR_getdomainname:          return "getdomainname";
		case __NR_setdomainname:          return "setdomainname";
		#ifndef __32bit_syscall_numbers__
		case __NR_utrap_install:          return "utrap_install";
		#endif
		case __NR_quotactl:               return "quotactl";
		case __NR_set_tid_address:        return "set_tid_address";
		case __NR_mount:                  return "mount";
		case __NR_ustat:                  return "ustat";
		case __NR_setxattr:               return "setxattr";
		case __NR_lsetxattr:              return "lsetxattr";
		case __NR_fsetxattr:              return "fsetxattr";
		case __NR_getxattr:               return "getxattr";
		case __NR_lgetxattr:              return "lgetxattr";
		case __NR_getdents:               return "getdents";
		case __NR_setsid:                 return "setsid";
		case __NR_fchdir:                 return "fchdir";
		case __NR_fgetxattr:              return "fgetxattr";
		case __NR_listxattr:              return "listxattr";
		case __NR_llistxattr:             return "llistxattr";
		case __NR_flistxattr:             return "flistxattr";
		case __NR_removexattr:            return "removexattr";
		case __NR_lremovexattr:           return "lremovexattr";
		case __NR_sigpending:             return "sigpending";
		case __NR_query_module:           return "query_module";
		case __NR_setpgid:                return "setpgid";
		case __NR_fremovexattr:           return "fremovexattr";
		case __NR_tkill:                  return "tkill";
		case __NR_exit_group:             return "exit_group";
		case __NR_uname:                  return "uname";
		case __NR_init_module:            return "init_module";
		case __NR_personality:            return "personality";
		case __NR_remap_file_pages:       return "remap_file_pages";
		case __NR_epoll_create:           return "epoll_create";
		case __NR_epoll_ctl:              return "epoll_ctl";
		case __NR_epoll_wait:             return "epoll_wait";
		case __NR_ioprio_set:             return "ioprio_set";
		case __NR_getppid:                return "getppid";
		case __NR_sigaction:              return "sigaction";
		case __NR_sgetmask:               return "sgetmask";
		case __NR_ssetmask:               return "ssetmask";
		case __NR_sigsuspend:             return "sigsuspend";
		case __NR_oldlstat:               return "oldlstat";
		case __NR_uselib:                 return "uselib";
		case __NR_readdir:                return "readdir";
		case __NR_readahead:              return "readahead";
		case __NR_socketcall:             return "socketcall";
		case __NR_syslog:                 return "syslog";
		case __NR_lookup_dcookie:         return "lookup_dcookie";
		case __NR_fadvise64:              return "fadvise64";
		case __NR_fadvise64_64:           return "fadvise64_64";
		case __NR_tgkill:                 return "tgkill";
		case __NR_waitpid:                return "waitpid";
		case __NR_swapoff:                return "swapoff";
		case __NR_sysinfo:                return "sysinfo";
		case __NR_ipc:                    return "ipc";
		case __NR_sigreturn:              return "sigreturn";
		case __NR_clone:                  return "clone";
		case __NR_ioprio_get:             return "ioprio_get";
		case __NR_adjtimex:               return "adjtimex";
		case __NR_sigprocmask:            return "sigprocmask";
		case __NR_create_module:          return "create_module";
		case __NR_delete_module:          return "delete_module";
		case __NR_get_kernel_syms:        return "get_kernel_syms";
		case __NR_getpgid:                return "getpgid";
		case __NR_bdflush:                return "bdflush";
		case __NR_sysfs:                  return "sysfs";
		case __NR_afs_syscall:            return "afs_syscall";
		case __NR_setfsuid:               return "setfsuid";
		case __NR_setfsgid:               return "setfsgid";
		case __NR__newselect:             return "_newselect";
		#ifdef __32bit_syscall_numbers__
		case __NR_time:                   return "time";
		#else
		#endif
		case __NR_splice:                 return "splice";
		case __NR_stime:                  return "stime";
		case __NR_statfs64:               return "statfs64";
		case __NR_fstatfs64:              return "fstatfs64";
		case __NR__llseek:                return "_llseek";
		case __NR_mlock:                  return "mlock";
		case __NR_munlock:                return "munlock";
		case __NR_mlockall:               return "mlockall";
		case __NR_munlockall:             return "munlockall";
		case __NR_sched_setparam:         return "sched_setparam";
		case __NR_sched_getparam:         return "sched_getparam";
		case __NR_sched_setscheduler:     return "sched_setscheduler";
		case __NR_sched_getscheduler:     return "sched_getscheduler";
		case __NR_sched_yield:            return "sched_yield";
		case __NR_sched_get_priority_max: return "sched_get_priority_max";
		case __NR_sched_get_priority_min: return "sched_get_priority_min";
		case __NR_sched_rr_get_interval:  return "sched_rr_get_interval";
		case __NR_nanosleep:              return "nanosleep";
		case __NR_mremap:                 return "mremap";
		case __NR__sysctl:                return "_sysctl";
		case __NR_getsid:                 return "getsid";
		case __NR_fdatasync:              return "fdatasync";
		case __NR_nfsservctl:             return "nfsservctl";
		case __NR_sync_file_range:        return "sync_file_range";
		case __NR_clock_settime:          return "clock_settime";
		case __NR_clock_gettime:          return "clock_gettime";
		case __NR_clock_getres:           return "clock_getres";
		case __NR_clock_nanosleep:        return "clock_nanosleep";
		case __NR_sched_getaffinity:      return "sched_getaffinity";
		case __NR_sched_setaffinity:      return "sched_setaffinity";
		case __NR_timer_settime:          return "timer_settime";
		case __NR_timer_gettime:          return "timer_gettime";
		case __NR_timer_getoverrun:       return "timer_getoverrun";
		case __NR_timer_delete:           return "timer_delete";
		case __NR_timer_create:           return "timer_create";
		/*case __NR_vserver:  267 Reserved for VSERVER */
		case __NR_io_setup:               return "io_setup";
		case __NR_io_destroy:             return "io_destroy";
		case __NR_io_submit:              return "io_submit";
		case __NR_io_cancel:              return "io_cancel";
		case __NR_io_getevents:           return "io_getevents";
		case __NR_mq_open:                return "mq_open";
		case __NR_mq_unlink:              return "mq_unlink";
		case __NR_mq_timedsend:           return "mq_timedsend";
		case __NR_mq_timedreceive:        return "mq_timedreceive";
		case __NR_mq_notify:              return "mq_notify";
		case __NR_mq_getsetattr:          return "mq_getsetattr";
		case __NR_waitid:                 return "waitid";
		case __NR_tee:                    return "tee";
		case __NR_add_key:                return "add_key";
		case __NR_request_key:            return "request_key";
		case __NR_keyctl:                 return "keyctl";
		case __NR_openat:                 return "openat";
		case __NR_mkdirat:                return "mkdirat";
		case __NR_mknodat:                return "mknodat";
		case __NR_fchownat:               return "fchownat";
		case __NR_futimesat:              return "futimesat";
		case __NR_fstatat64:              return "fstatat64";
		case __NR_unlinkat:               return "unlinkat";
		case __NR_renameat:               return "renameat";
		case __NR_linkat:                 return "linkat";
		case __NR_symlinkat:              return "symlinkat";
		case __NR_readlinkat:             return "readlinkat";
		case __NR_fchmodat:               return "fchmodat";
		case __NR_faccessat:              return "faccessat";
		case __NR_pselect6:               return "pselect6";
		case __NR_ppoll:                  return "ppoll";
		case __NR_unshare:                return "unshare";
		case __NR_set_robust_list:        return "set_robust_list";
		case __NR_get_robust_list:        return "get_robust_list";
		case __NR_migrate_pages:          return "migrate_pages";
		case __NR_mbind:                  return "mbind";
		case __NR_get_mempolicy:          return "get_mempolicy";
		case __NR_set_mempolicy:          return "set_mempolicy";
		case __NR_kexec_load:             return "kexec_load";
		case __NR_move_pages:             return "move_pages";
		case __NR_getcpu:                 return "getcpu";
		case __NR_epoll_pwait:            return "epoll_pwait";
		case __NR_utimensat:              return "utimensat";
		case __NR_signalfd:               return "signalfd";
		case __NR_timerfd_create:         return "timerfd_create";
		case __NR_eventfd:                return "eventfd";
		case __NR_fallocate:              return "fallocate";
		case __NR_timerfd_settime:        return "timerfd_settime";
		case __NR_timerfd_gettime:        return "timerfd_gettime";
		case __NR_signalfd4:              return "signalfd4";
		case __NR_eventfd2:               return "eventfd2";
		case __NR_epoll_create1:          return "epoll_create1";
		case __NR_dup3:                   return "dup3";
		case __NR_pipe2:                  return "pipe2";
		case __NR_inotify_init1:          return "inotify_init1";
		case __NR_accept4:                return "accept4";
		case __NR_preadv:                 return "preadv";
		case __NR_pwritev:                return "pwritev";
		case __NR_rt_tgsigqueueinfo:      return "rt_tgsigqueueinfo";
		case __NR_perf_event_open:        return "perf_event_open";
		case __NR_recvmmsg:               return "recvmmsg";
		case __NR_fanotify_init:          return "fanotify_init";
		case __NR_fanotify_mark:          return "fanotify_mark";
		case __NR_prlimit64:              return "prlimit64";
		case __NR_name_to_handle_at:      return "name_to_handle_at";
		case __NR_open_by_handle_at:      return "open_by_handle_at";
		case __NR_clock_adjtime:          return "clock_adjtime";
		case __NR_syncfs:                 return "syncfs";
		case __NR_sendmmsg:               return "sendmmsg";
		case __NR_setns:                  return "setns";
		case __NR_process_vm_readv:       return "process_vm_readv";
		case __NR_process_vm_writev:      return "process_vm_writev";
		case __NR_kern_features:          return "kern_features";
		case __NR_kcmp:                   return "kcmp";
		case __NR_finit_module:           return "finit_module";
		case __NR_sched_setattr:          return "sched_setattr";
		case __NR_sched_getattr:          return "sched_getattr";
		case __NR_renameat2:              return "renameat2";
		case __NR_seccomp:                return "seccomp";
		case __NR_getrandom:              return "getrandom";
		case __NR_memfd_create:           return "memfd_create";
		case __NR_bpf:                    return "bpf";
		case __NR_execveat:               return "execveat";
		case __NR_membarrier:             return "membarrier";
		case __NR_userfaultfd:            return "userfaultfd";
		case __NR_bind:                   return "bind";
		case __NR_listen:                 return "listen";
		case __NR_setsockopt:             return "setsockopt";
		case __NR_mlock2:                 return "mlock2";
		case __NR_copy_file_range:        return "copy_file_range";
		case __NR_preadv2:                return "preadv2";
		case __NR_pwritev2:               return "pwritev2";
		case __NR_statx:                  return "statx";
	}
	return "__unknown_syscall__";
}

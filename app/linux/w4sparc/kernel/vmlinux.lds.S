/* ld script for sparc32/sparc64 kernel */

#include <asm-generic/vmlinux.lds.h>

#include <asm/page.h>
#include <asm/thread_info.h>

#ifdef CONFIG_SPARC32
#define INITIAL_ADDRESS  0x10000 + SIZEOF_HEADERS

#if 0
 #define TEXTSTART	0xf0004000
#else
 #define TEXTSTART	0xd0000000
#endif

#define SMP_CACHE_BYTES_SHIFT 5

#else
#define SMP_CACHE_BYTES_SHIFT 6
#define INITIAL_ADDRESS 0x4000
#define TEXTSTART      0x0000000000404000

#endif

#define SMP_CACHE_BYTES (1 << SMP_CACHE_BYTES_SHIFT)

#ifdef CONFIG_SPARC32
OUTPUT_FORMAT("elf32-sparc", "elf32-sparc", "elf32-sparc")
OUTPUT_ARCH(sparc)
ENTRY(_start)
jiffies = jiffies_64 + 4;
#else
/* sparc64 */
OUTPUT_FORMAT("elf64-sparc", "elf64-sparc", "elf64-sparc")
OUTPUT_ARCH(sparc:v9a)
ENTRY(_start)
jiffies = jiffies_64;
#endif

#ifdef CONFIG_SPARC64
ASSERT((swapper_tsb == 0x0000000000408000), "Error: sparc64 early assembler too large")
#endif

SECTIONS
{
#ifdef CONFIG_SPARC64
	swapper_pg_dir = 0x0000000000402000;
#endif
	. = INITIAL_ADDRESS;
	.text TEXTSTART :
	{
		_text = .;
		HEAD_TEXT
		TEXT_TEXT
		SCHED_TEXT
		CPUIDLE_TEXT
		LOCK_TEXT
		KPROBES_TEXT
		IRQENTRY_TEXT
		SOFTIRQENTRY_TEXT
		*(.gnu.warning)

		/*wrm*/
		*(.text.*)
		_ctors_start = .;
		*(.ctors)
		_ctors_end = .;
		_dtors_start = .;
		_dtors_end = .;
		/*~wrm*/
	} = 0
	_etext = .;

	/*wrm*/

	. = ALIGN(PAGE_SIZE);

	__param : AT(ADDR(__param) - 0)
	{
		__start___param = .;
		KEEP(*(__param))
		__stop___param = .;
	}
	
	. = ALIGN(PAGE_SIZE);

	__modver : AT(ADDR(__modver) - 0)
	{
		__start___modver = .;
		KEEP(*(__modver))
		__stop___modver = .;
	}
	/*~wrm*/

	RO_DATA(PAGE_SIZE)

	/* Start of data section */
	_sdata = .;

	.data1 : {
		*(.data1)
	}
	RW_DATA_SECTION(SMP_CACHE_BYTES, 0, THREAD_SIZE)

	/* End of data section */
	_edata = .;

	. = ALIGN(PAGE_SIZE); /*wrm*/

	.fixup : {
		__start___fixup = .;
		*(.fixup)
		__stop___fixup = .;
	}
	/* wrm commented
	EXCEPTION_TABLE(16)
	NOTES
	*/
	EXCEPTION_TABLE(PAGE_SIZE) /*wrm*/

	. = ALIGN(PAGE_SIZE);
	__init_begin = ALIGN(PAGE_SIZE);
	INIT_TEXT_SECTION(PAGE_SIZE)
	__init_text_end = .;
	. = ALIGN(PAGE_SIZE); /*wrm*/
	INIT_DATA_SECTION(16)

	. = ALIGN(PAGE_SIZE); /*wrm*/

	. = ALIGN(4);
	.tsb_ldquad_phys_patch : {
		__tsb_ldquad_phys_patch = .;
		*(.tsb_ldquad_phys_patch)
		__tsb_ldquad_phys_patch_end = .;
	}

	.tsb_phys_patch : {
		__tsb_phys_patch = .;
		*(.tsb_phys_patch)
		__tsb_phys_patch_end = .;
	}

	.cpuid_patch : {
		__cpuid_patch = .;
		*(.cpuid_patch)
		__cpuid_patch_end = .;
	}

	.sun4v_1insn_patch : {
		__sun4v_1insn_patch = .;
		*(.sun4v_1insn_patch)
		__sun4v_1insn_patch_end = .;
	}
	.sun4v_2insn_patch : {
		__sun4v_2insn_patch = .;
		*(.sun4v_2insn_patch)
		__sun4v_2insn_patch_end = .;
	}
	.leon_1insn_patch : {
		__leon_1insn_patch = .;
		*(.leon_1insn_patch)
		__leon_1insn_patch_end = .;
	}
	.swapper_tsb_phys_patch : {
		__swapper_tsb_phys_patch = .;
		*(.swapper_tsb_phys_patch)
		__swapper_tsb_phys_patch_end = .;
	}
	.swapper_4m_tsb_phys_patch : {
		__swapper_4m_tsb_phys_patch = .;
		*(.swapper_4m_tsb_phys_patch)
		__swapper_4m_tsb_phys_patch_end = .;
	}
	.popc_3insn_patch : {
		__popc_3insn_patch = .;
		*(.popc_3insn_patch)
		__popc_3insn_patch_end = .;
	}
	.popc_6insn_patch : {
		__popc_6insn_patch = .;
		*(.popc_6insn_patch)
		__popc_6insn_patch_end = .;
	}
	.pause_3insn_patch : {
		__pause_3insn_patch = .;
		*(.pause_3insn_patch)
		__pause_3insn_patch_end = .;
	}
	.sun_m7_2insn_patch : {
		__sun_m7_2insn_patch = .;
		*(.sun_m7_2insn_patch)
		__sun_m7_2insn_patch_end = .;
	}
	PERCPU_SECTION(SMP_CACHE_BYTES)

#ifdef CONFIG_JUMP_LABEL
	. = ALIGN(PAGE_SIZE);
	.exit.text : {
		EXIT_TEXT
	}
#endif

	. = ALIGN(PAGE_SIZE);
	__init_end = .;
	_bss_start = .; /*wrm*/
	BSS_SECTION(0, 0, 0)
	_bss_end = .; /*wrm*/
	_end = . ;


	/*wrm,  may combine together */
	.free_memory : {
		KEEP(*(.free_mem_1))
	}
	.free_memory2 : {
		KEEP(*(.free_mem_2))
	}
	/*~wrm*/

	STABS_DEBUG
	DWARF_DEBUG

	DISCARDS
}

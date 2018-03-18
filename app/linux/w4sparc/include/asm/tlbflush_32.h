#ifndef _SPARC_TLBFLUSH_H
#define _SPARC_TLBFLUSH_H

#include <asm/cachetlb_32.h>

#if 0 // wrm
#define flush_tlb_all() \
	printk("%s:  call flush_tlb_all(), sparc32_cachetlb_ops=0x%lx.\n", __func__, (long)sparc32_cachetlb_ops) \
	/*sparc32_cachetlb_ops->tlb_all()*/
#else
#define flush_tlb_all() \
	sparc32_cachetlb_ops->tlb_all()
#endif


#if 0 // wrm
#define flush_tlb_mm(mm) \
	printk("%s:  call flush_tlb_mm(), sparc32_cachetlb_ops=0x%lx.\n", __func__, (long)sparc32_cachetlb_ops) \
	/*sparc32_cachetlb_ops->tlb_mm(mm)*/
#else
#define flush_tlb_mm(mm) \
	sparc32_cachetlb_ops->tlb_mm(mm)
#endif


#if 0 // wrm
#define flush_tlb_range(vma, start, end) \
	printk("%s:  call flush_tlb_range(), sparc32_cachetlb_ops=0x%lx.\n", __func__, (long)sparc32_cachetlb_ops) \
	/*sparc32_cachetlb_ops->cache_range(vma, start, end)*/
#else
#define flush_tlb_range(vma, start, end) \
	sparc32_cachetlb_ops->tlb_range(vma, start, end)
#endif


#if 0 // wrm
#define flush_tlb_page(vma, addr) \
	printk("%s:  call flush_tlb_page(), sparc32_cachetlb_ops=0x%lx.\n", __func__, (long)sparc32_cachetlb_ops) \
	/*sparc32_cachetlb_ops->tlb_page(vma, addr)*/
#else
#define flush_tlb_page(vma, addr) \
	sparc32_cachetlb_ops->tlb_page(vma, addr)
#endif

/*
 * This is a kludge, until I know better. --zaitcev XXX
 */
static inline void flush_tlb_kernel_range(unsigned long start,
					  unsigned long end)
{
	flush_tlb_all();
}

#endif /* _SPARC_TLBFLUSH_H */

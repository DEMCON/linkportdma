/*
 * Blackfin Linkport driver
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 *
 * 
 * Changed 2017-2020 by DEMCON.
 */

/*! 
 * \file
 * \brief \ref linkport implementation.
 * \author Jochem Rutgers (jochem.rutgers@demcon.com)
 */

/*!
 * \defgroup linkport Link port driver
 * \brief This unit implements the Linux kernel link port character device.
 *
 * As this port has to process much data, up to 32 MB/s input and output, performance of this driver is key.
 *
 * This unit is tested by bfin_lp_test.c.
 *
 * <b>DMA and buffers</b>
 *
 * DMA is used to copy data from/to the link port peripheral.
 * For this, the kernel has to allocate a block of memory, which is used by the DMA.
 * To do this, there are multiple options:
 *
 * - \c kmalloc() a buffer in DDR, and pass this to \c dma_map_single().
 *   This way, 'streaming DMA' can be used, which flushes the data cache before and after a read or write.
 *   For this, \c dma_sync_single_for_cpu() and \c dma_sync_single_for_device() are called.
 *   Flushing/invalidating caches takes significant amount of time at the intended throughput.
 * - Call \c dma_alloc_coherent() to allocate a buffer in DDR, which is called 'coherent DMA'.
 *   This buffer is mapped to kernel space with caches disabled.
 *   Therefore, flushing/invalidating the cache is not required, which saves some processor time in the driver.
 *   However, accessing the data is expensive, as the L1 cache is not used.
 *
 * DDR is relatively expensive, as it is off-chip memory.
 * The L2 SRAM can also be used (via the \c sram_alloc() API).
 * In theory, both solutions from above can be combined with L2 instead of DDR.
 * Instead of using \c dma_map_single() for streaming DMA, \c dma_map_resource() utilizes the L2 IO region.
 * However, \c dma_map_resources() is not available in the currently used kernel version, so L2 cannot be used in combination with streaming DMA at the moment.
 * To use the L2 for coherent DMA, the allocated L2 buffer can be passed to the coherent pool via \c dma_declare_coherent_memory().
 * This way, uncached access to the L2 buffer is realized.
 * Performance tests show that the DMA is significantly faster using L2 instead of DDR, but uncached access latency to either buffer by the application is equivalent.
 *
 * When #LP_DMA_SRAM is defined, coherent DMA utilizes the L2 instead of DDR.
 * This macro has no effect on streaming DMA.
 *
 * To get the data from/to the DMA buffer, the data has to be transferred between user space and kernel space.
 * For this, there are several options:
 *
 * - Given a user space buffer in the application, do a \c copy_from_user() (writes) or \c copy_to_user() (reads) to/from kernel space.
 *   There is overhead in copying the data, but the kernel has highly optimized methods for this.
 * - \c mmap() the DMA buffer from kernel space to user space, such that the application can access the buffer directly.
 *   In case of coherent DMA, the application accesses the buffer uncached, which may result in a significant performance hit when the application does this inefficiently.
 *
 * All permutations of the partial solutions above are valid, but with different performance.
 * The following approach seems to be the best.
 *
 * For writes, use an uncached L2 buffer \c mmap()ed to user space, in combination with coherent DMA.
 * This way, the DMA is fast, as it runs from L2; writes are uncached, so no cache flushes are required; no buffers are copied as \c mmap() is used.
 * Therefore, the driver defines #LP_DMA_COHERENT_WRITE.
 *
 * For reads, use a cached DDR buffer \c mmap()ed to user space, in combination with streaming DMA.
 * Reads must process/decode all data, so access from the application is important.
 * Uncached access to the buffer is too expensive.
 * This gives two solutions: uncached buffer (coherent DMA) and use \c copy_to_user(), or cached buffer (streaming DMA) with \c mmap() and flush the cache.
 * In both cases, the kernel has to iterate over the buffer, either for flush or copy, where the latter is assumed to be more efficient.
 * However, L2 cannot be used, due to the lack of \c dma_map_resource().
 * Therefore, the driver does not define \c LP_DMA_COHERENT_READ.
 *
 * <b>Double buffering</b>
 *
 * In case \c mmap() is used to access the DMA buffer, double buffering can be used.
 * Via a \c ioctl() call, both buffers (ping and pong) can be mapped to user space (see #bfin_lp_ioctl()).
 * When opened in non-blocking mode, reads and writes can be initiated on one buffer, while the other can be processed/prepared for the next read/write.
 * See #bfin_lp_read() and #bfin_lp_write() for examples.
 *
 * <b>Interrupts</b>
 *
 * A read works conceptually as follows:
 * 
 * -# Configure and start the DMA given the used buffer and buffer length.
 * -# Suspend process while waiting for the DMA to complete.
 * -# When the DMA completes, an interrupt is generated, which wakes the process.
 * -# Copy kernel space buffer to user space, when the supplied buffer was not \c mmap()ed.
 *
 * A write works conceptually as follows:
 *
 * -# When a previous write was not finished, enable interrupts, suspend the process, and wait for the DMA to finish and trigger an interrupt.
 * -# Copy user space buffer to kernel space, when the supplied buffer was not \c mmap()ed.
 * -# Disable DMA interrupt.
 * -# Configure and start the DMA given the used buffer and buffer length.
 *
 * When writes are sufficiently spaced in time, the DMA completes before the next write is initiated.
 * Then, it is checked whether the previous write was finished, without the overhead of interrupts, and sleeping and resuming the process.
 *
 * For non-blocking reads/writes, use \c poll() or \c select(), as exemplified by #bfin_lp_read() and #bfin_lp_write().
 *
 * \ingroup app
 */


//#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <mach/dma.h>
#include <mach/sc58x.h>
#include <mach/cpu.h>
#include <mach/irqs.h>
#include <asm/cacheflush.h>
#include <mach/sram.h>
#include <mach/hardware.h>


//////////////////////////////////////////////////////
// Configuration
//////////////////////////////////////////////////////

/*!
 * \brief When defined, use L2 SRAM for the DMA memory.
 * \ingroup linkport
 */
#define LP_DMA_SRAM

/*
 * \brief When defined, use coherent (uncached) dma for reads (either DDR or L2).
 * Otherwise, use streaming (cached) dma (DDR, never L2).
 * \ingroup linkport
 */
//#define LP_DMA_COHERENT_READ

/*!
 * \brief When defined, use coherent (uncached) dma for writes (either DDR or L2).
 * Otherwise, use streaming (cached) dma (DDR, never L2).
 * \ingroup linkport
 */
#define LP_DMA_COHERENT_WRITE

/*!
 * \brief Size of one ping (or pong) buffer.
 * \ingroup linkport
 */
#define LP_DMA_BUFSIZE_PINGPONG_	0x2000

/*!
 * \brief Default clock divider.
 * \ingroup linkport
 */
#define LP_DIV 0



//////////////////////////////////////////////////////
// Configuration
//////////////////////////////////////////////////////

#define LINKPORT_DRVNAME		"linkport-dma"

// Register definitions
#define LP_REG_SIZE				0x20

#define LP_CTL_EN				0x1
#define LP_CTL_TRAN				0x8
#define LP_CTL_TRQMSK			0x100
#define LP_CTL_RRQMSK			0x200
#define LP_CTL_ITMSK			0x800

#define LP_STAT_FREE			0x0
#define LP_STAT_INUSE			0x8000
#define LP_STAT_DMA				0x4000
#define LP_STAT_WNR				0x2000

#define LP_STAT_DONE			0x1000
#define LP_STAT_LTRQ 			0x1
#define LP_STAT_LRRQ 			0x2
#define LP_STAT_LPIT 			0x8
#define LP_STAT_FFST 			0x70
#define LP_STAT_LERR 			0x80
#define LP_STAT_LPBS 			0x100

#define LP_CTL_OFF				0x0
#define LP_STAT_OFF 			0x4
#define LP_DIV_OFF				0x8
#define LP_CNT_OFF				0xC
#define LP_TX_OFF				0x10
#define LP_RX_OFF				0x14
#define LP_TX_SHADOW_OFF		0x18
#define LP_RX_SHADOW_OFF		0x1C

#define DMA_CTL_PDRF			0x10000000

// Number of bytes to pad at the end of a write (rounded down to full words)
#define LP_TX_PADDING			3

/*!
 * \brief \c ioctl() magic number.
 * \ingroup linkport
 */
#define LP_IOCTL_MAGIC			15
/*!
 * \brief Set clock divider via \c ioctl().
 * \ingroup linkport
 */
#define LP_IOCTL_CLKDIV			_IOW(LP_IOCTL_MAGIC, 1, uint8_t)
/*!
 * \brief \c mmap() ping buffer and return a user space address via \c ioctl().
 * \ingroup linkport
 */
#define LP_IOCTL_BUFADDR_PING	_IOR(LP_IOCTL_MAGIC, 2, void*)
/*!
 * \brief \c mmap() pong buffer and return a user space address via \c ioctl().
 * \ingroup linkport
 */
#define LP_IOCTL_BUFADDR_PONG	_IOR(LP_IOCTL_MAGIC, 3, void*)
/*!
 * \brief Return the size of the ping and pong buffer \c ioctl().
 * \ingroup linkport
 */
#define LP_IOCTL_BUFSIZE		_IO(LP_IOCTL_MAGIC, 4)

#define LP_DMA_BUFSIZE_PINGPONG	((LP_DMA_BUFSIZE_PINGPONG_ + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1))
#define LP_DMA_BUFSIZE			(LP_DMA_BUFSIZE_PINGPONG * 2)
#define LP_DMA_PING_ADDR(dev)	((dev)->dma_handle)
#define LP_DMA_PONG_ADDR(dev)	((dev)->dma_handle + LP_DMA_BUFSIZE_PINGPONG)

#if    defined(LP_DMA_COHERENT_WRITE) && !defined(LP_DMA_COHERENT_READ)
#  define IS_DMA_COHERENT(filp)	((filp)->f_mode & FMODE_WRITE)
#elif !defined(LP_DMA_COHERENT_WRITE) &&  defined(LP_DMA_COHERENT_READ)
#  define IS_DMA_COHERENT(filp)	(!((filp)->f_mode & FMODE_WRITE))
#elif  defined(LP_DMA_COHERENT_WRITE) &&  defined(LP_DMA_COHERENT_READ)
#  define IS_DMA_COHERENT(filp) true
#else
#  define IS_DMA_COHERENT(filp) false
#endif



//////////////////////////////////////////////////////
// Data structures
//////////////////////////////////////////////////////

/*!
 * \brief Link port /dev file description.
 * \ingroup linkport
 */
struct bfin_linkport {
	struct list_head lp_dev;	//!< \brief The corresponding link port device list.
	struct class *class;		//!< \brief The class of the link port character device.
	int major;					//!< \brief The major number.
};

/*!
 * \brief Link port device description.
 * \ingroup linkport
 */
struct bfin_lp_dev {
	struct list_head list;		//!< \brief Device list administration.
	struct device *device;		//!< \brief The corresponding Linux device.

	phys_addr_t const preg_base;//!< \brief The physical address of the link port peripheral registers.
	int irq;					//!< \brief the IRQ for DMA.
	bool volatile irq_disabled;	//!< \brief Flag to indicate that the DMA interrupt is disabled.
	int status_irq;				//!< \brief The IRQ for transmit/receive requests (not used).
	int const dma_chan;			//!< \brief The DMA channel.

	int linkport_num;			//!< \brief Index of this link port (corresponds to /dev/linkport*).
	void __iomem *reg_base;		//!< \brief \c ioremap() of #preg_base.
	spinlock_t lock;			//!< \brief Lock for opening the device and checking its administration.
	struct mutex mutex;			//!< \brief Mutex to protect concurrent reads/writes to the same fd.
	struct completion complete;	//!< \brief Signal to indicate that the DMA has finished.
	struct pinctrl* pinctrl;	//!< \brief Administration of the link port pins.

	uint8_t clk_div;			//!< \brief Request clock divider (see LP_DIV register).
	int status;					//!< \brief Status of this device.
	bool nosleep;				//!< \brief Indicates whether the process had to sleep while waiting for the DMA.

	dma_addr_t dma_handle;		//!< \brief Physical address of #dma_buffer.
	dma_addr_t dma_start_addr;	//!< \brief Currently used start address of the DMA transfer.
	void* dma_buffer;			//!< \brief Virtual address of #dma_handle.
#ifdef LP_DMA_SRAM
	void* sram_buffer;			//!< \brief Allocated L2 memory, which is used as pool for coherent DMA.
#endif
	size_t dma_count;			//!< \brief Length of currently running DMA transfer.

	unsigned long ping_mmap;	//!< \brief User space address of ping buffer within #dma_buffer.
	unsigned long pong_mmap;	//!< \brief User space address of pong buffer within #dma_buffer.
};

/*!
 * \brief List of all link port devices.
 */
static struct bfin_linkport *linkport_dev;

/*!
 * \brief List of link port peripherals.
 */
static struct bfin_lp_dev lp_dev_info[2] = {
	{
		.preg_base = LP0_CTL,
		.irq = IRQ_LP0,
		.status_irq = IRQ_LP0_STAT,
		.dma_chan = CH_LP0,
	},
	{
		.preg_base = LP1_CTL,
		.irq = IRQ_LP1,
		.status_irq = IRQ_LP1_STAT,
		.dma_chan = CH_LP1,
	},
};

/*! \brief Number of link ports. */
#define LP_NUM (sizeof(lp_dev_info) / sizeof(lp_dev_info[0]))


//////////////////////////////////////////////////////
// Misc
//////////////////////////////////////////////////////

/*!
 * \brief Returns the current direction of the opened device, for streaming DMA functions.
 * \ingroup linkport
 */
static enum dma_data_direction bfin_lp_dma_direction(struct bfin_lp_dev* dev)
{
	return dev && (dev->status & LP_STAT_WNR) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
}

/*!
 * \brief Configure the link port peripheral.
 * \ingroup linkport
 */
static void bfin_lp_config_channel(struct bfin_lp_dev *dev)
{
	uint32_t reg;
	uint32_t cur;

	cur = readl(dev->reg_base + LP_CTL_OFF);

	if(dev->status & LP_STAT_WNR)
	{
		// read/receive
		reg = 0; //| LP_CTL_RRQMSK;
		if(!(cur & LP_CTL_TRAN) && (cur & LP_CTL_EN))
			// keep port enabled when already in receive mode
			reg |= LP_CTL_EN;
	}
	else
	{
		// write/transmit
		reg = LP_CTL_TRAN; //| LP_CTL_TRQMSK;
		if((cur & LP_CTL_TRAN) && (cur & LP_CTL_EN))
			// keep port enabled when already in transmit mode
			reg |= LP_CTL_EN;
		
		writel(dev->clk_div, dev->reg_base + LP_DIV_OFF);
	}
	
	writel(reg, dev->reg_base + LP_CTL_OFF);
}

/*!
 * \brief Enable a configured link port peripheral.
 * \details Call #bfin_lp_config_channel() first.
 * \ingroup linkport
 */
static void bfin_lp_enable(struct bfin_lp_dev *lpdev)
{
	uint32_t ctl;

	ctl = readl(lpdev->reg_base + LP_CTL_OFF);
	if(!(ctl & LP_CTL_EN))
		writel(ctl | LP_CTL_EN, lpdev->reg_base + LP_CTL_OFF);
}

/*!
 * \brief Disable a configured link port peripheral.
 * \ingroup linkport
 */
static void bfin_lp_disable(struct bfin_lp_dev *lpdev)
{
	uint32_t ctl;

	ctl = readl(lpdev->reg_base + LP_CTL_OFF);
	if(ctl & LP_CTL_EN)
		writel(ctl & ~LP_CTL_EN, lpdev->reg_base + LP_CTL_OFF);
}

/*!
 * \brief Resets a link port peripheral.
 * \ingroup linkport
 */
static int bfin_lp_reset(struct bfin_lp_dev *dev)
{
	writel(0, dev->reg_base + LP_CTL_OFF);
	writel(dev->clk_div, dev->reg_base + LP_DIV_OFF);
	// clear all (pending) interrupts
	writel(0xFF, dev->reg_base + LP_STAT_OFF);
	writel(0, dev->reg_base + LP_CTL_OFF);
	return 0;
}

#ifdef LP_DMA_SRAM
/*!
 * \brief Returns the physical address of an allocated L2 buffer address.
 * \ingroup linkport
 */
static inline dma_addr_t sram_dma_addr(void* sram_virt_addr)
{
	uintptr_t mmio_addr = (uintptr_t)sram_virt_addr;
	uintptr_t offs = mmio_addr - (uintptr_t)IO_ADDRESS(SYS_SRAM_BASE);
	return (dma_addr_t)(SYS_SRAM_BASE + offs);
}
#endif


//////////////////////////////////////////////////////
// IRQ handlers
//////////////////////////////////////////////////////

/*!
 * \brief IRQ handler for the status IRQ.
 * \ingroup linkport
 */
static irqreturn_t bfin_lp_irq(int irq, void *dev_id)
{
	struct bfin_lp_dev *dev = (struct bfin_lp_dev *)dev_id;
	uint32_t stat;

	if(unlikely(!dev))
		// can't do anything...
		return IRQ_HANDLED;

	stat = readl(dev->reg_base + LP_STAT_OFF);
	dev_dbg(dev->device, "lp irq %d stat %x dev %p status 0x%x\n", irq, stat, dev, dev->status);

	writel(stat, dev->reg_base + LP_STAT_OFF);
	return IRQ_HANDLED;
}

/*!
 * \brief Checks whether the DMA completed (whether it is currently in idle/stop state).
 * \details This function may be called from the IRQ context and from the normal context.
 * \return 0 when finished and \c -EINPROGRESS when it is still running
 * \ingroup linkport
 */
static int bfin_dma_check(struct bfin_lp_dev* dev)
{
	unsigned long dma_stat;

	if(unlikely(!(dev->status & LP_STAT_DMA)))
		return -EIO;

	dma_stat = get_dma_curr_irqstat(dev->dma_chan);
	dev_dbg(dev->device, "dmastat 0x%lx\n", dma_stat);

	if(unlikely(!(dma_stat & DMA_DONE) && (dma_stat & DMA_RUN_MASK)))
		return -EINPROGRESS;

	// For transmit, DMA_DONE means that the DMA finished reading memory,
	// but may not be finished writing to the peripheral.
	// So, the run status may not be idle/stop yet.
	
	if(__atomic_fetch_and(&dev->status, ~LP_STAT_DMA, __ATOMIC_RELAXED) & LP_STAT_DMA)
	{
		// dma finished
		complete(&dev->complete);
	}

	if(unlikely(dma_stat & DMA_ERR))
		dev_warn(dev->device, "dma error (stat: 0x%lx); ignored\n", dma_stat);

	return 0;
}

/*!
 * \brief IRQ handler when the DMA finishes.
 * \ingroup linkport
 */
static irqreturn_t bfin_dma_irq(int irq, void *dev_id)
{
	struct bfin_lp_dev *dev = dev_id;
	BUG_ON(!dev || irq != dev->irq);

	dev_dbg(dev->device, "dma irq %d dev %p status 0x%x\n", irq, dev, dev->status);

	switch(bfin_dma_check(dev))
	{
	case -EIO:
		// not sure where this irq came from
		disable_irq_nosync(irq);
		dev->irq_disabled = true;
		break;

	case 0:
		// done
		break;

	default:;
		// still waiting to complete...
	}

	clear_dma_irqstat(dev->dma_chan);
	return IRQ_HANDLED;
}



//////////////////////////////////////////////////////
// read/write/ioctl
//////////////////////////////////////////////////////

/*!
 * \brief Waits until a previous read/write finishes.
 * \details The caller must hold the <code>dev->mutex</code>.
 * \return 0 when a new read/write can be started, -EWOULDBLOCK when the DMA is currently busy and the file is opened as non-blocking
 * \ingroup linkport
 */
static int bfin_lp_complete(struct bfin_lp_dev* dev, bool blocking)
{
	unsigned long irq_flags;

	if(unlikely(!dev))
		return -EINVAL;

	if(unlikely(!(dev->status & (LP_STAT_LTRQ | LP_STAT_LRRQ))))
		// no outstanding reads/writes
		return 0;

	if(unlikely(dev->irq_disabled && bfin_dma_check(dev) == -EINPROGRESS && blocking))
	{
		// Someone is waiting for the DMA to finish, but IRQ was disabled.
		// Enable so we can properly sleep.
		dev->irq_disabled = false;
		enable_irq(dev->irq);
		// Flag that we have to sleep before the operation finishes.
		dev->nosleep = false;
	}

	// In case res == -EINPROGRESS, waiting for completion will sleep until the irq triggers.
	// In case res == 0, waiting for completion returns immediately and successfully.

	// transmit is/was in progress, wait for completion
	if(blocking)
	{
		if(try_wait_for_completion(&dev->complete)) {
			// finished immediately
			// nosleep is true when interrupts where enabled and the DMA already finished,
			// or nosleep is false in the race condition that it just finished between enabling the irq and now.
		} else if(wait_for_completion_interruptible(&dev->complete)) {
			// interrupted
			return -ERESTARTSYS;
		} else {
			// we did sleep for a while
			dev->nosleep = false;
		}
	}
	else
	{
		if((dev->status & LP_STAT_LTRQ) && (get_dma_curr_irqstat(dev->dma_chan) & DMA_RUN_MASK))
			// The done IRQ is triggered when the DMA finished reading memory, but may still
			// be busy transferring data to the peripheral.
			return -EWOULDBLOCK;

		if(!try_wait_for_completion(&dev->complete))
			// not completed yet
			return -EWOULDBLOCK;
	}


	// When we get here, a read/write was finished.
	//
	// We assume that reads/writes are called in a repetitive process.
	// When writes are sufficiently spaced in time, the DMA interrupt can be suppressed.
	// We have to detect whether this is the case.
	// When nosleep is true, it indicates that the DMA was finished before the call to bfin_lp_complete() and generated an interrupt.
	// However, when interrupts would have been disabled, the call to bfin_dma_check() would have noticed that DMA was finished,
	// without the need to generate and handle the interrupt.
	// So, in case of write() and nosleep, the DMA interrupt can be disabled, and it is assumed that it does not have to be reenabled
	// over and over again for every write.
	
	barrier();

	if(dev->status & LP_STAT_LRRQ)
	{
#ifndef LP_DMA_COHERENT_READ
		dma_sync_single_for_cpu(dev->device, dev->dma_start_addr, dev->dma_count, DMA_FROM_DEVICE);
#endif
		dev_dbg(dev->device, "finished read\n");
	}
	else if(dev->status & LP_STAT_LTRQ)
	{
		unsigned long u = 1;

#ifndef LP_DMA_COHERENT_WRITE
		dma_sync_single_for_cpu(dev->device, dev->dma_start_addr, dev->dma_count, DMA_TO_DEVICE);
#endif

		if(unlikely(dev->nosleep && !dev->irq_disabled))
		{
			// Disable DMA interrupt, as it does not seem to be required, as writes are not been done too quickly after each other.
			local_irq_save(irq_flags);
			if(!dev->irq_disabled)
			{
				disable_irq_nosync(dev->irq);
				dev->irq_disabled = true;
			}
			local_irq_restore(irq_flags);

			dev_dbg(dev->device, "nosleep; disable irq\n");
		}

		while(unlikely(get_dma_curr_irqstat(dev->dma_chan) & DMA_RUN_MASK))
		{
			// Wait until it is really stopped/idle, otherwise we cannot restart it.

			if(u <= 8)
			{
				// busy-wait, up to 15 us
				udelay(u);
			}
			else
			{
				// allow context switch
				usleep_range(u, u * 4);

				if(signal_pending(current))
					return -ERESTARTSYS;
			}

			if(u < 1024)
				// exponential back-off up to 1 ms
				u *= 2;
		}

		dev_dbg(dev->device, "finished write\n");
	}

	dev->status &= ~(LP_STAT_LTRQ | LP_STAT_LRRQ);

	return 0;
}

/*!
 * \brief \c fsync() implementation.
 * \details This is essentially a wrapper for #bfin_lp_complete() that always blocks, regardless whether the file was opened in non-blocking mode.
 * \ingroup linkport
 */
static int bfin_lp_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
	struct bfin_lp_dev *dev;
	int ret = 0;

	if(unlikely(!filp))
		return -EINVAL;

	dev = filp->private_data;

	if(unlikely(!dev))
		return -EINVAL;

	if(mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;
	
	ret = bfin_lp_complete(dev, true);

	mutex_unlock(&dev->mutex);
	return ret;
}

/*!
 * \brief Implementation of \c read().
 *
 * Simple example:
 *
 * \code
 * int fd = open("/dev/linkport0", O_RDONLY);
 * char buf[16];
 * int count = read(fd, buf, sizeof(buf));
 * // use buf...
 * \endcode
 *
 *
 * Non blocking example.
 * Note that the DMA must be initialized (so at least one \c read() must be made) before sleeping by \c poll() and friends.
 * Moreover, the supplied buffer size is the amount of data to wait for; read will not complete until all data was received.
 * This is not conforming to the normal non-blocking interface, but required for this specific driver.
 * The successive read returns the result of the previous DMA transfer, so one must supply the same buffer and the same buffer length to the second read as was supplied to the first one.
 *
 * \code
 * int fd = open("/dev/linkport0", O_RDONLY | O_NONBLOCK);
 * char buf[16];
 * int count = read(fd, buf, sizeof(buf)); // initiate DMA for given amount of data
 *
 * if(count < 0 && errno == EINPROGRESS)
 * {
 *   // a read must have been initiated before poll will mark the fd as readible
 *   poll(read fd...); // may use select/fsync too
 *   count = read(fd, buf, sizeof(buf));
 *   // use buf...
 * }
 * \endcode
 *
 *
 * DMA double buffer with \c mmap() (implicitly by \c ioctl()) example.
 *
 * \code
 * int fd = open("/dev/linkport0", O_RDONLY | O_NONBLOCK);
 * void* ping;
 * ioctl(fd, LP_IOCTL_BUFADDR_PING, &ping);
 * // ioctl() also mmap()s the buffer to user space.
 * void* pong;
 * ioctl(fd, LP_IOCTL_BUFADDR_PONG, &pong);
 * size_t bufsize = (size_t)ioctl(fd, LP_IOCTL_BUFSIZE);
 *
 * while(true)
 * {
 *   // start DMA for ping
 *   read(fd, ping, expected_data_size); // returns EINPROGRESS
 *
 *   // do something with pongbuf while ping is transferred
 *
 *   poll(read fd...) // wait for DMA to finish ping; may use select/fsync too
 *   read(fd, ping, expected_data_size); // actual read
 *
 *   // start DMA for pong
 *   read(fd, pong, expected_data_size); // returns EINPROGRESS
 *
 *   // do something with pingbuf while pong is transferred
 *
 *   poll(read fd...) // wait for DMA to finish pong; may use select/fsync too
 *   read(fd, pong, expected_data_size); // actual read
 * }
 * \endcode
 *
 *
 * If neither the \c mmap()ed ping and pong buffer is passed to \c read(), internally the ping buffer is
 * used to DMA the data into, before coping the data to userspace.
 * Be careful when mixing \c mmap()ed and normal reads.
 *
 * \ingroup linkport
 */
static ssize_t bfin_lp_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	struct bfin_lp_dev *dev;
	unsigned long irq_flags;
	int ret;
	
	if(unlikely(count == 0))
		return 0;
	if(unlikely(count <= 3))
		return -EINVAL;
	if(unlikely(!filp))
		return -EINVAL;
	if(unlikely(filp->f_mode & FMODE_WRITE))
		return -EPERM;

	dev = filp->private_data;

	if(unlikely(!dev || !dev->dma_buffer))
		return -EINVAL;

	if(!buf)
		buf = (char*)dev->ping_mmap;

	count = (count > LP_DMA_BUFSIZE_PINGPONG ? LP_DMA_BUFSIZE_PINGPONG : count) & ~3;

	if(filp->f_flags & O_NONBLOCK)
	{
		if (!mutex_trylock(&dev->mutex))
			return -EWOULDBLOCK;
	}
	else
	{
		if (mutex_lock_interruptible(&dev->mutex))
			return -ERESTARTSYS;
	}

	if(!(dev->status & LP_STAT_LRRQ))
	{
		// this is a new read operation

		dev_dbg(dev->device, "read 0x%x bytes\n", count);

		if(buf == (char*)dev->ping_mmap)
		{
			dev->dma_start_addr = LP_DMA_PING_ADDR(dev);
#ifdef DEBUG
			memset(dev->dma_buffer, 0xcd, LP_DMA_BUFSIZE_PINGPONG);
#endif
		}
		else if(buf == (char*)dev->pong_mmap)
		{
			dev->dma_start_addr = LP_DMA_PONG_ADDR(dev);
#ifdef DEBUG
			memset((void*)((uintptr_t)dev->dma_buffer + LP_DMA_BUFSIZE_PINGPONG), 0xcd, LP_DMA_BUFSIZE_PINGPONG);
#endif
		}
		else
		{
			dev->dma_start_addr = dev->dma_handle;
#ifdef DEBUG
			memset(dev->dma_buffer, 0xcd, LP_DMA_BUFSIZE_PINGPONG);
#endif
		}

		dev->dma_count = count;
#ifndef LP_DMA_COHERENT_READ
		dma_sync_single_for_device(dev->device, dev->dma_start_addr, dev->dma_count, DMA_FROM_DEVICE);
#endif

		// Configure dma
		set_dma_start_addr(dev->dma_chan, dev->dma_start_addr);
		set_dma_x_count(dev->dma_chan, dev->dma_count / 4);

		// start dma
		local_irq_save(irq_flags);
		dev->status |= LP_STAT_LRRQ | LP_STAT_DMA;
		mb();
		enable_dma(dev->dma_chan);
		if(dev->irq_disabled)
		{
			dev->irq_disabled = false;
			enable_irq(dev->irq);
		}
		local_irq_restore(irq_flags);

		// When the DMA completes, bfin_dma_irq() sets dev->complete.

		// In case the file is opened non-blocking, it is very unlikely
		// that the read finishes immediately. So, we guarantee that
		// the read that initiated a DMA transfer always returns
		// -EINPROGRESS.
		if(filp->f_flags & O_NONBLOCK)
		{
			count = -EINPROGRESS;
			goto done;
		}
	}
	else
	{
		dev_dbg(dev->device, "read already in progress\n");
		if(count < dev->dma_count)
		{
			// buf too small
			count = -EINVAL;
			goto done;
		}

		if(count > dev->dma_count)
			// do not read more than the dma was configured for
			count = dev->dma_count;
	}
	
	// try to complete this read right now
	if((ret = bfin_lp_complete(dev, !(filp->f_flags & O_NONBLOCK))))
	{
		// not possible right now
		dev_dbg(dev->device, "read would block");
		count = (ret == -EWOULDBLOCK ? -EINPROGRESS : ret);
		goto done;
	}

	// dma completed
	if(buf != (char*)dev->ping_mmap && buf != (char*)dev->pong_mmap)
	{
		if(unlikely(copy_to_user(buf, dev->dma_buffer, count)))
		{
			// data has been lost
			count = -EFAULT;
			goto done;
		}
	}

done:
	mutex_unlock(&dev->mutex);
	return count;
}

/*!
 * \brief Implementation of \c write().
 *
 * Simple example.
 * Even though the port is opened in blocking mode, \c write() returns before the data was actually transmitted.
 * A successive write will block until the previous write finishes.
 * Since the buffer was duplicated by the driver, \c buf can be reused safely meanwhile.
 *
 * \code
 * int fd = open("/dev/linkport0", O_WRONLY);
 * char buf[16];
 * // fill buf
 * write(fd, buf, sizeof(buf));
 * \endcode
 *
 *
 * Non blocking example:
 *
 * \code
 * int fd = open("/dev/linkport0", O_WRONLY | O_NONBLOCK);
 * char buf[16];
 * // fill buf
 * poll(write fd...); // in contrast to read+poll, no write is required before; may use select/fsync too
 * write(fd, buf, sizeof(buf)); // may still return EWOULDBLOCK in some cases, see bfin_lp_poll()
 * \endcode
 *
 *
 * DMA double buffer example.
 * In order to be able to \c mmap() the DMA buffer to user space, the file must be opened with read mode.
 * Hence, \c O_RDWR is interpreted as a write-only, but allows \c mmap().
 * Do not use O_RDWR when the file is intended to be \c read().
 *
 * \code
 * int fd = open("/dev/linkport0", O_RDWR | O_NONBLOCK);
 * void* ping;
 * ioctl(fd, LP_IOCTL_BUFADDR_PING, &ping);
 * void* pong;
 * ioctl(fd, LP_IOCTL_BUFADDR_PONG, &pong);
 * size_t bufsize = (size_t)ioctl(fd, LP_IOCTL_BUFSIZE);
 *
 * while(true)
 * {
 *   // fill pingbuf, up to bufsize bytes, while pong is being transferred
 *
 *   fsync(fd); // wait for previous DMA to complete; may use select/poll too
 *   write(fd, ping, length); // start DMA
 *
 *   // fill pongbuf, up to bufsize bytes, while ping is being transferred
 *
 *   fsync(fd); // wait for previous DMA to complete; may use select/poll too
 *   write(fd, pong, length); // start DMA
 * }
 * \endcode
 *
 *
 * If neither the \c mmap()ed ping and pong buffer is passed to \c write(), internally the ping buffer is
 * used to copy the data into from userspace.
 * Be careful when mixing \c mmap()ed and normal writes.
 *
 * \ingroup linkport
 */
static ssize_t bfin_lp_write(struct file *filp, const char *buf, size_t count, loff_t *pos)
{
	struct bfin_lp_dev *dev;
	unsigned long irq_flags;
	int ret;
	size_t padding;

	if(unlikely(count == 0))
		return 0;
	if(unlikely(!filp))
		return -EINVAL;
	if(unlikely(!(filp->f_mode & FMODE_WRITE)))
		return -EPERM;

	dev = filp->private_data;

	if(unlikely(!dev || !dev->dma_buffer))
		return -EINVAL;

	padding = ((count + LP_TX_PADDING) & ~3) - count;
	if(count + padding > LP_DMA_BUFSIZE_PINGPONG)
	{
		padding = LP_TX_PADDING & ~3;
		count = LP_DMA_BUFSIZE_PINGPONG - padding;
	}
	
	if(filp->f_flags & O_NONBLOCK)
	{
		if (!mutex_trylock(&dev->mutex))
			return -EWOULDBLOCK;
	}
	else
	{
		if (mutex_lock_interruptible(&dev->mutex))
			return -ERESTARTSYS;
	}

	if((ret = bfin_lp_complete(dev, !(filp->f_flags & O_NONBLOCK))))
	{
		dev_dbg(dev->device, "write would block");
		count = ret;
		goto done;
	}

	dev_dbg(dev->device, "write 0x%x bytes\n", count);

	if(!buf || buf == (char*)dev->ping_mmap)
	{
		dev->dma_start_addr = LP_DMA_PING_ADDR(dev);
	}
	else if(buf == (char*)dev->pong_mmap)
	{
		dev->dma_start_addr = LP_DMA_PONG_ADDR(dev);
	}
	else if(likely(copy_from_user(dev->dma_buffer, buf, count) == 0))
	{
		dev->dma_start_addr = dev->dma_handle;
	}
	else
	{
		count = -EFAULT;
		goto done;
	}

	memset(&((char*)dev->dma_buffer)[count], 0x00, padding);

	dev->dma_count = count + padding;
#ifndef LP_DMA_COHERENT_WRITE
	dma_sync_single_for_device(dev->device, dev->dma_start_addr, dev->dma_count, DMA_TO_DEVICE);
#endif

	// Configure dma
	set_dma_start_addr(dev->dma_chan, dev->dma_start_addr);
	set_dma_x_count(dev->dma_chan, dev->dma_count / 4);

	// start dma
	local_irq_save(irq_flags);
	dev->status |= LP_STAT_LTRQ | LP_STAT_DMA;
	mb();
	enable_dma(dev->dma_chan);
	local_irq_restore(irq_flags);

	dev->nosleep = true;
	
	// Return while dma is working.
	// Make sure to call bfin_lp_complete() before attempting another write
done:
	mutex_unlock(&dev->mutex);
	return count;
}

/*!
 * \brief Implementation of \c poll() and \c select().
 * \details There is a catch; for a writable port, \c poll() may return that it is writable before it really is...
 *          The DMA generates an interrupt when it has finished reading, but not when all data was transferred to the peripheral.
 *          \c poll() checks the state of the interrupt, but not of the actual state of this final data transfer.
 *          So, when \c poll() indicates that the device is writable, \c write() may still block (or return \c EWOULDBLOCK), as it does check the real DMA state.
 *          Unfortunately, \c poll() cannot give a more accurate indication, as the DMA state must be polled and does not generate (another) interrupt, which is expensive to implement.
 *          \c poll() does, however, properly indicate that the DMA buffer is writable, without intervening when a previous \c write().
 * \see #bfin_lp_read() and #bfin_lp_write() for examples
 * \ingroup linkport
 */
static unsigned int bfin_lp_poll(struct file *filp, poll_table *wait)
{
	struct bfin_lp_dev *dev;
	unsigned int ret = 0;
	unsigned long irq_flags;
	uint32_t dev_status;

	if(!filp)
		return POLLERR;

	dev = filp->private_data;

	if(!dev)
		return POLLERR;

	poll_wait(filp, &dev->complete.wait, wait);

	dev_status = dev->status;
	if(!(dev_status & LP_STAT_DMA))
	{
		if(filp->f_mode & FMODE_WRITE)
			ret |= POLLOUT | POLLWRNORM;

		if((dev_status & LP_STAT_LRRQ) || (!(filp->f_mode & FMODE_WRITE) && (filp->f_flags & O_NONBLOCK)))
			// a read has completed, or has not been started yet
			ret |= POLLIN | POLLRDNORM;
	}
	else
	{
		// make sure that the irq is enabled
		
		local_irq_save(irq_flags);
		if(dev->irq_disabled)
		{
			dev->irq_disabled = false;
			enable_irq(dev->irq);
		}
		local_irq_restore(irq_flags);

		dev->nosleep = false;
	}

	dev_dbg(dev->device, "poll() = 0x%x", ret);

	return ret;
}

/*!
 * \brief Implementation of \c mmap().
 * \details Maps the DMA buffer to user space.
 *          This requires the physical address of the allocated buffer.
 *          This cannot be requested from user space, so application should not try to \c mmap() themselves.
 *          Use \c ioctl() instead to request the ping and pong buffer.
 * \see #bfin_lp_read() and #bfin_lp_write() for examples
 * \ingroup linkport
 */
static int bfin_lp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct bfin_lp_dev *dev;
	unsigned long offset;
	unsigned long size;
	
	if(!filp || !vma)
		return -EINVAL;

	dev = filp->private_data;

	if(!dev)
		return -EINVAL;

	offset = vma->vm_pgoff << PAGE_SHIFT;
	size = vma->vm_end - vma->vm_start;

	if(	offset < (unsigned long)dev->dma_handle
		|| size > LP_DMA_BUFSIZE_PINGPONG
		|| offset + size > (unsigned long)dev->dma_handle + LP_DMA_BUFSIZE)
		return -EINVAL;

	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;

	if(IS_DMA_COHERENT(filp))
	{
		// dma_mmap_coherent() expects vma->vm_pgoff to be an offset within the previously alloc'ed DMA buffer.
		vma->vm_pgoff -= (unsigned long)dev->dma_handle >> PAGE_SHIFT;
		return dma_mmap_coherent(dev->device, vma, dev->dma_buffer, dev->dma_handle, LP_DMA_BUFSIZE);
	}
	else
		return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot);
}

/*!
 * \brief Helper function to \c mmap() a physical address to user space.
 */
static int bfin_lp_mmap_buf(struct file *filp, dma_addr_t phys, size_t len, unsigned long* addr)
{
	unsigned long populate;
	unsigned long a;
	struct bfin_lp_dev *dev = filp->private_data;

	down_write(&current->mm->mmap_sem);

	a = do_mmap_pgoff(filp, 0, len,
			(filp->f_mode & FMODE_READ ? PROT_READ : 0) |
			(filp->f_mode & FMODE_WRITE ? PROT_WRITE : 0),
			MAP_SHARED, (uintptr_t)phys >> PAGE_SHIFT, &populate);

	up_write(&current->mm->mmap_sem);

	if(IS_ERR_VALUE(a))
		return (int)a;

	dev_dbg(dev->device, "mmap buf 0x%lx -> 0x%lx\n", (unsigned long)phys, a);
	*addr = a;

	return 0;
}

/*!
 * \brief Implementation of \c ioctl().
 * \see #bfin_lp_read() and #bfin_lp_write() for examples
 * \ingroup linkport
 */
static long bfin_lp_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
	struct bfin_lp_dev *dev;
	void* a;
	int ret;

	if(!filp)
		return -EINVAL;

	dev = filp->private_data;

	if(!dev)
		return -EINVAL;

	switch(cmd)
	{
	case LP_IOCTL_CLKDIV:
		if(copy_from_user(&dev->clk_div, (void __user *)arg, sizeof(dev->clk_div)))
			return -EFAULT;
		
		bfin_lp_disable(dev);
		bfin_lp_config_channel(dev);
		bfin_lp_enable(dev);
		return 0;

	case LP_IOCTL_BUFADDR_PING:
		if(!dev->ping_mmap)
			if((ret = bfin_lp_mmap_buf(filp, LP_DMA_PING_ADDR(dev), LP_DMA_BUFSIZE_PINGPONG, &dev->ping_mmap)))
				return ret;

		a = (void*)dev->ping_mmap;
		return copy_to_user((void __user *)arg, &a, sizeof(a)) ? -EFAULT : 0;
	
	case LP_IOCTL_BUFADDR_PONG:
		if(!dev->pong_mmap)
			if((ret = bfin_lp_mmap_buf(filp, LP_DMA_PONG_ADDR(dev), LP_DMA_BUFSIZE_PINGPONG, &dev->pong_mmap)))
				return ret;

		a = (void*)dev->pong_mmap;
		return copy_to_user((void __user *)arg, &a, sizeof(a)) ? -EFAULT : 0;
	
	case LP_IOCTL_BUFSIZE:
		return (long)LP_DMA_BUFSIZE_PINGPONG;

	default:
		return -EINVAL;
	}
}


//////////////////////////////////////////////////////
// Open/release
//////////////////////////////////////////////////////

/*!
 * \brief Implementation of \c open().
 * \details If a read/receive from the link port is intended, use \c O_RDONLY.
 *			If a write/transmit to the link port is intended, use \c O_RDWR when an \c mmap() ping/pong is to be used, or O_WRONLY otherwise.
 *			Do not mix reads and writes to the same file descriptor.
 * \ingroup linkport
 */
static int bfin_lp_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct bfin_lp_dev *dev;
	unsigned int index = iminor(inode);
	int ret = 0;

	dev = &lp_dev_info[index];

	if(!filp)
		return -EINVAL;
	if(!(filp->f_mode & FMODE_WRITE) && !(filp->f_mode & FMODE_READ))
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);

	if(dev->status & LP_STAT_INUSE)
	{
		ret = -EBUSY;
	}
	else
	{
		filp->private_data = dev;
		dev->status = LP_STAT_INUSE;
		if(!(filp->f_mode & FMODE_WRITE))
			dev->status |= LP_STAT_WNR;
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	if(ret)
		goto done;
	
	init_completion(&dev->complete);

	if((ret = request_dma(dev->dma_chan, LINKPORT_DRVNAME ":dma")))
	{
		dev_err(dev->device, "unable to request linkport rx dma channel\n");
		goto error_dma;
	}

	if(IS_DMA_COHERENT(filp))
	{
		// Allocate uncached memory to use for DMA.
		// When LP_DMA_SRAM, use the SRAM as memory pool.

		if(!dev->dma_buffer)
		{
#ifdef LP_DMA_SRAM
			if(!dev->sram_buffer)
			{
				if(!(dev->sram_buffer = sram_alloc(LP_DMA_BUFSIZE)))
				{
					dev_err(dev->device, "cannot allocate L2 memory\n");
					ret = -ENOMEM;
					goto error_buffer;
				}

				dev_dbg(dev->device, "using L2 %p (phys %p)\n", dev->sram_buffer,
					(void*)sram_dma_addr(dev->sram_buffer));
			}

			if((ret = dma_declare_coherent_memory(dev->device,
				sram_dma_addr(dev->sram_buffer), sram_dma_addr(dev->sram_buffer), LP_DMA_BUFSIZE, DMA_MEMORY_EXCLUSIVE | DMA_MEMORY_MAP)) < 0)
			{
				dev_err(dev->device, "cannot declare coherent\n");
				goto error_buffer;
			}
#endif

			if(!(dev->dma_buffer = dma_alloc_coherent(dev->device, LP_DMA_BUFSIZE, &dev->dma_handle, GFP_KERNEL | GFP_DMA)))
			{
				dev_err(dev->device, "cannot allocate dma memory\n");
				ret = -ENOMEM;
				goto error_buffer;
			}

			dev_dbg(dev->device, "using coherent dma at %p (phys %p)\n", dev->dma_buffer, (void*)dev->dma_handle);
		}
	}
	else
	{
		// Allocate cached memory to use for DMA.
		// Never uses SRAM.

		if(!dev->dma_buffer)
		{
			if(!(dev->dma_buffer = kmalloc(LP_DMA_BUFSIZE, GFP_KERNEL)))
			{
				ret = -ENOMEM;
				goto error_buffer;
			}
		}

		dev->dma_handle = dma_map_single(dev->device, dev->dma_buffer, LP_DMA_BUFSIZE, bfin_lp_dma_direction(dev));
		
		if(dma_mapping_error(dev->device, dev->dma_handle))
		{
			dev_err(dev->device, "unable to map dma\n");
			ret = -ENOMEM;
			goto error_buffer;
		}
		
		dev_dbg(dev->device, "using streaming dma at %p (phys %p)\n", dev->dma_buffer, (void*)dev->dma_handle);
	}

	// Check alignment of returned addresses.
	if(((uintptr_t)dev->dma_buffer & (PAGE_SIZE - 1)) || ((uintptr_t)dev->dma_handle & (PAGE_SIZE - 1)))
	{
		dev_err(dev->device, "dma buffer not page aligned; vm %p / phys %p\n", dev->dma_buffer, (void*)dev->dma_handle);
		ret = -ENOMEM;
		goto error_buffer;
	}

	// Initialize DMA buffer.
	memset(dev->dma_buffer, 0, LP_DMA_BUFSIZE);
	dev->ping_mmap = 0;
	dev->pong_mmap = 0;

	// Initialize LP port.
	dev->clk_div = LP_DIV; // set default clock divider
	bfin_lp_reset(dev);

	// Initialize DMA.
	// set_dma_callback will (re)enable the irq
	dev->irq_disabled = false;
	clear_dma_irqstat(dev->dma_chan);
	set_dma_callback(dev->dma_chan, bfin_dma_irq, dev);

	set_dma_start_addr(dev->dma_chan, dev->dma_handle);
	set_dma_x_modify(dev->dma_chan, 4);

	if(dev->status & LP_STAT_WNR)
	{
		// read from port
		set_dma_config(dev->dma_chan, WNR | DI_EN | DMAFLOW_STOP | WDSIZE_32 | PSIZE_32);
	}
	else
	{
		// write to port
		set_dma_config(dev->dma_chan, DI_EN | DMAFLOW_STOP | WDSIZE_32 | PSIZE_32);
	}
	
	bfin_lp_config_channel(dev);

	// Ready
	dev_dbg(dev->device, "bfin lp open %d%s%s%s\n", index,
		filp->f_mode & FMODE_READ ? " read" : "",
		filp->f_mode & FMODE_WRITE ? " write" : "",
		filp->f_flags & O_NONBLOCK ? " nonblock" : "");

	bfin_lp_enable(dev);

	return 0;

error_buffer:
	if(dev->dma_buffer)
	{
		if(IS_DMA_COHERENT(filp))
		{
			dma_free_coherent(dev->device, LP_DMA_BUFSIZE, dev->dma_buffer, dev->dma_handle);

#ifdef LP_DMA_SRAM
			dma_release_declared_memory(dev->device);
#endif
		}
		else
		{
			kfree(dev->dma_buffer);
		}

		dev->dma_buffer = NULL;
	}

#ifdef LP_DMA_SRAM
	if(dev->sram_buffer)
	{
		sram_free(dev->sram_buffer);
		dev->sram_buffer = NULL;
	}
#endif

	free_dma(dev->dma_chan);
error_dma:
	mb();
	dev->status = LP_STAT_FREE;
done:
	return ret;
}

/*!
 * \brief Implementation of \c close().
 * \details When a read or write was in progress, it is terminated immediately.
 *          In that case, data will be lost.
 *          If this is not intended, call fsync() first.
 * \ingroup linkport
 */
static int bfin_lp_release(struct inode *inode, struct file *filp)
{
	struct bfin_lp_dev *dev;
	unsigned int index = iminor(inode);

	if(!filp)
		return -EINVAL;

	dev = filp->private_data;

	if(!dev)
		return -EINVAL;

	mutex_lock(&dev->mutex);
	bfin_lp_complete(dev, false);
	
	dev_dbg(dev->device, "bfin lp release %d\n", index);
	
	bfin_lp_reset(dev);
	disable_dma(dev->dma_chan);

	if(current->mm)
	{
		down_write(&current->mm->mmap_sem);
		if(dev->ping_mmap)
		{
			do_munmap(current->mm, dev->ping_mmap, LP_DMA_BUFSIZE_PINGPONG);
			dev->ping_mmap = 0;
		}
		if(dev->pong_mmap)
		{
			do_munmap(current->mm, dev->pong_mmap, LP_DMA_BUFSIZE_PINGPONG);
			dev->pong_mmap = 0;
		}
		up_write(&current->mm->mmap_sem);
	}

	if(dev->dma_buffer)
	{
		if(IS_DMA_COHERENT(filp))
		{
			dma_free_coherent(dev->device, LP_DMA_BUFSIZE, dev->dma_buffer, dev->dma_handle);
#ifdef LP_DMA_SRAM
			dma_release_declared_memory(dev->device);
#endif
		}
		else
		{
			dma_unmap_single(dev->device, dev->dma_handle, LP_DMA_BUFSIZE, bfin_lp_dma_direction(dev));
			kfree(dev->dma_buffer);
		}

		dev->dma_buffer = NULL;
	}

#ifdef LP_DMA_SRAM
	if(dev->sram_buffer)
	{
		sram_free(dev->sram_buffer);
		dev->sram_buffer = NULL;
	}
#endif
	
	free_dma(dev->dma_chan);

	mb();
	dev->status = LP_STAT_FREE;

	mutex_unlock(&dev->mutex);
	return 0;
}


static const struct file_operations linkport_fops = {
	.owner = THIS_MODULE,
	.read = bfin_lp_read,
	.write = bfin_lp_write,
	.unlocked_ioctl = bfin_lp_ioctl,
	.open = bfin_lp_open,
	.release = bfin_lp_release,
	.fsync = bfin_lp_fsync,
	.poll = bfin_lp_poll,
	.mmap = bfin_lp_mmap,
};




//////////////////////////////////////////////////////
// /sys/class interface
//////////////////////////////////////////////////////

/*!
 * \brief Implementation of /sys/class/linkport-dma/status.
 * \ingroup linkport
 */
static ssize_t bfin_lp_status_show(struct class *class, struct class_attribute *attr, char *buf)
{
	char *p = buf;
	struct bfin_lp_dev *dev;

	p += sprintf(p, "linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d\n", dev->linkport_num);
		p += sprintf(p, "\t status 0x%08x\n", dev->status);
	}
	return p - buf;
}

/*!
 * \brief Implementation of /sys/class/linkport-dma/reg.
 * \ingroup linkport
 */
static ssize_t bfin_lp_reg_show(struct class *class, struct class_attribute *attr, char *buf)
{
	char *p = buf;
	struct bfin_lp_dev *dev;

	p += sprintf(p, "linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d, base 0x%x\n", dev->linkport_num, dev->preg_base);
		p += sprintf(p, "\t ctl  0x%08x\n", readl(dev->reg_base + LP_CTL_OFF));
		p += sprintf(p, "\t stat 0x%08x\n", readl(dev->reg_base + LP_STAT_OFF));
	}
	return p - buf;
}

/*!
 * \brief Implementation of /sys/class/linkport-dma/reg write operations.
 * 
 * This interface allows to read/write link port peripheral registers directly.
 * The commands to be written to this file have the following format:
 * \verbatim <cmd><bitsize> <phys addr> <data>\endverbatim
 * To read a register, write \verbatim r(8|16|32) *[0-9a-fA-F]{1,8}\endverbatim
 * To write a register, write \verbatim w(8|16|32) *[0-9a-fA-F]{1,8} *[0-9a-fA-F]{1,8}\endverbatim
 *
 * Writing to registers is only allowed when the driver has been compiled with \c DEBUG defined.
 *
 * \ingroup linkport
 */
static ssize_t bfin_lp_reg_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	char *p;
	int rw = 0;
	uint32_t value = 0;
	char buffer[64] = {};
	uint32_t access_size;
	unsigned long paddr;
	void __iomem *addr;
	struct bfin_lp_dev *dev;

	if(count >= sizeof(buffer))
		count = sizeof(buffer) - 1;

	memcpy(buffer, buf, count);

	p = buffer;

	if(count == 0)
		goto done;

	while(true)
	{
		switch(*p)
		{
		case ' ':
		case '\t':
		case '\r':
		case '\n':
			break;
		default:
			goto skipped_first;
		}

		p++;
	}
skipped_first:
	
	switch(*p++)
	{
	case 'r':
		rw = 0;
		break;
#ifdef DEBUG
	case 'w':
		rw = 1;
		break;
#endif
	default:
		pr_debug(LINKPORT_DRVNAME " -EINVAL\n");
		goto done;
	}

	if (p[0] < '0' && p[0] > '9')
	{
		pr_debug(LINKPORT_DRVNAME " -EINVAL2\n");
		goto done;
	}

	switch((access_size = simple_strtoul(p, &p, 10)))
	{
	case 8:
	case 16:
	case 32:
		break;
	default:
		pr_debug(LINKPORT_DRVNAME " -EINVAL3\n");
		goto done;
	}

	while (*p == ' ')
		p++;

	paddr = simple_strtoul(p, &p, 16);
	addr = NULL;

	list_for_each_entry(dev, &linkport_dev->lp_dev, list)
	{
		if((uintptr_t)paddr >= (uintptr_t)dev->preg_base && (uintptr_t)paddr + access_size / 8 <= (uintptr_t)dev->preg_base + LP_REG_SIZE)
		{
			addr = (void __iomem *)((uintptr_t)dev->reg_base + (uintptr_t)paddr - (uintptr_t)dev->preg_base);
			break;
		}
	}

	if(addr == NULL)
	{
		pr_debug(LINKPORT_DRVNAME " -EINVAL4\n");
		goto done;
	}

	if (rw) {
		while (*p == ' ')
			p++;

		value = simple_strtoul(p, &p, 16);
		pr_debug(LINKPORT_DRVNAME ": write addr 0x%lx reg %08x\n", paddr, value);
		switch (access_size) {
		case 8:
			writeb((uint8_t)value, addr);
			value = readb(addr);
			break;
		case 16:
			writew((uint16_t)value, addr);
			value = readw(addr);
			break;
		case 32:
			writel((uint32_t)value, addr);
			value = readl(addr);
			break;
		}
	} else {
		switch (access_size) {
		case 8:
			value = readb(addr);
			break;
		case 16:
			value = readw(addr);
			break;
		case 32:
			value = readl(addr);
			break;
		}
		pr_info(LINKPORT_DRVNAME ": read addr 0x%lx reg %08x\n", paddr, value);
	}

done:
	return p - buffer;
}

static CLASS_ATTR(status, S_IRUSR, &bfin_lp_status_show, NULL);
static CLASS_ATTR(reg, S_IRUSR | S_IWUSR, &bfin_lp_reg_show, &bfin_lp_reg_store);



//////////////////////////////////////////////////////
// Init
//////////////////////////////////////////////////////

/*! 
 * \brief Driver module cleanup code.
 * \ingroup linkport
 */
static void bfin_linkport_cleanup(void)
{
	int i;
	dev_t lp_dev;

	if(!linkport_dev)
		// nothing to be done
		return;

	if(!linkport_dev->class)
		goto free_dev;
	
	lp_dev = MKDEV(linkport_dev->major, 0);

	for (i = LP_NUM - 1; i >= 0; i--)
	{
		struct bfin_lp_dev * const lpdev = &lp_dev_info[i];
		if(!lpdev->device)
			// skip this device
			continue;

		if(lpdev->pinctrl)
			pinctrl_put(lpdev->pinctrl);

		if(lpdev->status_irq > 0)
			free_irq(lpdev->status_irq, lpdev);

		if(lpdev->irq > 0)
			free_irq(lpdev->irq, lpdev);

		mutex_destroy(&lpdev->mutex);

		if(lpdev->reg_base)
			iounmap(lpdev->reg_base);

		device_destroy(linkport_dev->class, lp_dev + i);
	}

	class_remove_file(linkport_dev->class, &class_attr_reg);
	class_remove_file(linkport_dev->class, &class_attr_status);
	class_destroy(linkport_dev->class);

free_dev:
	unregister_chrdev(linkport_dev->major, LINKPORT_DRVNAME);
	kfree(linkport_dev);
}

/*! 
 * \brief Driver module exit.
 * \ingroup linkport
 */
static void __exit bfin_linkport_exit(void)
{
	bfin_linkport_cleanup();
}

/*!
 * \brief Enable cycle counter access from user space.
 * \details This counter is used to get accurate and low-cost time stamps.
 */
static void enable_ccnt_read(void* data)
{
	// PMUSERENR = 1
	asm volatile ("mcr p15, 0, %0, c9, c14, 0" :: "r"(1));

	// PMCR.E (bit 0) = 1
	asm volatile ("mcr p15, 0, %0, c9, c12, 0" :: "r"(1));

	// PMCNTENSET.C (bit 31) = 1
	asm volatile ("mcr p15, 0, %0, c9, c12, 1" :: "r"(1 << 31));
}

/*! 
 * \brief Driver module init.
 * \ingroup linkport
 */
static int __init bfin_linkport_init(void)
{
	struct device_node *np;
	int res;
	dev_t lp_dev;
	int i;
	
	on_each_cpu(enable_ccnt_read, NULL, 1);

	linkport_dev = kzalloc(sizeof(*linkport_dev), GFP_KERNEL);
	if (!linkport_dev) {
		res = -ENOMEM;
		goto done;
	}

	linkport_dev->major = register_chrdev(0, LINKPORT_DRVNAME, &linkport_fops);
	if (linkport_dev->major < 0) {
		res = linkport_dev->major;
		pr_err("Error %d registering chrdev for device\n", res);
		goto free;
	}

	lp_dev = MKDEV(linkport_dev->major, 0);

	linkport_dev->class = class_create(THIS_MODULE, LINKPORT_DRVNAME);
	if(!linkport_dev->class) {
		pr_err(LINKPORT_DRVNAME ": error while creating class\n");
		res = -ENOMEM;
		goto free_chrdev;
	}

	res = class_create_file(linkport_dev->class, &class_attr_status);
	if (res) {
		pr_err(LINKPORT_DRVNAME ": error %d registering class device\n", res);
		goto free_class;
	}

	res = class_create_file(linkport_dev->class, &class_attr_reg);
	if (res) {
		pr_err(LINKPORT_DRVNAME ": error %d registering class device\n", res);
		class_remove_file(linkport_dev->class, &class_attr_status);
		goto free_class;
	}

	INIT_LIST_HEAD(&linkport_dev->lp_dev);

	for (i = 0; i < LP_NUM; i++) {
		struct bfin_lp_dev * const lpdev = &lp_dev_info[i];
		struct device *dev;

		lpdev->linkport_num = i;
		lpdev->status = LP_STAT_FREE;

		lpdev->device = dev = device_create(linkport_dev->class, NULL, lp_dev + i, lpdev, "linkport%d", i);
		if (!dev)
		{
			pr_err(LINKPORT_DRVNAME ": cannot create device %d", i);
			res = -ENOMEM;
			goto rollback;
		}

		np = of_find_compatible_node(NULL, NULL, dev_name(dev));
		if (np) {
			pr_devel(LINKPORT_DRVNAME ": find dt node %s\n", np->name);
			dev->of_node = of_node_get(np);
			lpdev->irq = irq_of_parse_and_map(dev->of_node, 0);
			if (lpdev->irq <= 0)
				panic("Can't parse IRQ");
			lpdev->status_irq = irq_of_parse_and_map(dev->of_node, 1);
			if (lpdev->status_irq <= 0)
				panic("Can't parse IRQ");
			pr_devel(LINKPORT_DRVNAME ": of parse irq %d status irq %d\n", lpdev->irq, lp_dev_info[i].status_irq);
			// mark the linkport DMA channels to generate secure transactions
			set_spu_securep_msec(5, true);
			set_spu_securep_msec(6, true);
		}
		else
		{
			pr_warn(LINKPORT_DRVNAME ": not found dt node %s\n", dev_name(dev));
			device_destroy(linkport_dev->class, lp_dev + i);
			lpdev->device = 0;
			continue;
		}

		lpdev->reg_base = ioremap(lpdev->preg_base, LP_REG_SIZE);
		spin_lock_init(&lpdev->lock);
		INIT_LIST_HEAD(&lpdev->list);
		init_completion(&lpdev->complete);
		list_add(&lpdev->list, &linkport_dev->lp_dev);
		mutex_init(&lpdev->mutex);
		lpdev->irq_disabled = false;

#if 0
		if (request_irq(lpdev->irq, bfin_lp_irq, 0, LINKPORT_DRVNAME ":irq", lpdev)) {
			pr_err(LINKPORT_DRVNAME ": requesting irq %d failed\n", lpdev->irq);
			res = -ENODEV;
			goto rollback;
		}
#endif

		if (request_irq(lpdev->status_irq, bfin_lp_irq, 0, LINKPORT_DRVNAME ":status", lpdev)) {
			pr_err(LINKPORT_DRVNAME ": requesting status irq %d failed\n", lpdev->status_irq);
			res = -ENODEV;
			goto rollback;
		}

		lpdev->pinctrl = devm_pinctrl_get_select_default(dev);
		if (IS_ERR(lpdev->pinctrl)) {
			pr_err(LINKPORT_DRVNAME ": requesting Peripheral for %s failed.\n", dev_name(dev));
			lpdev->pinctrl = NULL;
			res = -EINVAL;
			goto rollback;
		}
		
		dev->dma_mask = &dev->coherent_dma_mask;

		if((res = dma_set_mask_and_coherent(dev, (u64)DMA_BIT_MASK(32))))
		{
			pr_err(LINKPORT_DRVNAME ": cannot set dma mask\n");
			goto rollback;
		}

		bfin_lp_reset(lpdev);
		pr_info(LINKPORT_DRVNAME ": initialized linkport%d\n", lpdev->linkport_num);
	}

	return 0;

rollback:
	bfin_linkport_cleanup();
	goto done;

free_class:
	class_destroy(linkport_dev->class);
free_chrdev:
	unregister_chrdev(linkport_dev->major, LINKPORT_DRVNAME);
free:
	kfree(linkport_dev);
done:
	return res;
}


module_init(bfin_linkport_init);
module_exit(bfin_linkport_exit);


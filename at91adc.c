/******************************************************************************
| File name : at91adc.c |
| Description : Source code for the EDAQ ADC driver. This driver supports two |
| : adc channesl with minor numbers 0 and 1. When the module is |
| : is inserted to kernel, TC0 module is setup to generate interr-|
| : upt every 1 msec. Both ADC channels are sampled in the ISR |
| : and the samples are copied into a ring buffer. Sampling rate |
| : can be changed by changing SAMPLE_INTERVAL_MS. In response to |
| : the read from user space, latest requested number of samples |
| : are returned as an unsigned short array. Channel 0/1 data is |
| : returned depending on the minor number of the device opened. |
| History : |
| ### Date Author Comments |
| 001 30-Jul-08 S. Thalore Initial creation |
| Copyright 2008 |
| This program is free software: you can redistribute it and/or modify |
| it under the terms of the GNU General Public License as published by |
| the Free Software Foundation, either version 3 of the License, or |
| (at your option) any later version. |
| |
| This program is distributed in the hope that it will be useful, |
| but WITHOUT ANY WARRANTY; without even the implied warranty of |
| MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the |
| GNU General Public License for more details. |
| |
| You should have received a copy of the GNU General Public License |
| along with this program. If not, see <http://www.gnu.org/licenses/>. |
******************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <mach/at91_tc.h>
#include <mach/at91_adc.h>
#include <asm/uaccess.h>

#define MAX_ADCSAMPLES 512

int at91adc_devno;
struct cdev at91adc_cdev;
struct file_operations at91adc_fops;

unsigned short *at91adc_pbuf0, *at91adc_pbuf1, *at91adc_pbuf2, *at91adc_pbuf3, at91adc_appidx;
void __iomem *at91tc0_base;
void __iomem *at91adc_base;
struct clk *at91adc_clk;
struct clk *at91tc0_clk;

/*****************************************************************************************
| Timer counter 0 ISR: Sample both ADC channels and copy the data to module ring buffer. |
*****************************************************************************************/
static irqreturn_t at91tc0_isr (int irq, void *dev_id)
{
	int status;

	// Read TC0 status register to reset RC compare status.

	status = ioread32(at91tc0_base + AT91_TC_SR);

	if (at91adc_appidx < MAX_ADCSAMPLES)
	{
		// Trigger the ADC (this will be done using TIOA automatically eventually).
		iowrite32(AT91_ADC_START, (at91adc_base + AT91_ADC_CR));

		// Wait for conversion to be complete.
		while ((ioread32(at91adc_base + AT91_ADC_SR) & AT91_ADC_DRDY) == 0) cpu_relax();

		// Copy converted data to module ring buffer.
		at91adc_pbuf0[at91adc_appidx] = ioread32(at91adc_base + AT91_ADC_CHR(0)) & 0x3FF;
		at91adc_pbuf1[at91adc_appidx] = ioread32(at91adc_base + AT91_ADC_CHR(1)) & 0x3FF;
		at91adc_pbuf2[at91adc_appidx] = ioread32(at91adc_base + AT91_ADC_CHR(2)) & 0x3FF;
		at91adc_pbuf3[at91adc_appidx] = ioread32(at91adc_base + AT91_ADC_CHR(3)) & 0x3FF;

		// Increment the ring buffer index and check for wrap around.

		at91adc_appidx++;
	}

	return IRQ_HANDLED;
}

/*****************************************************************************************
| Module initialization: Allocate device numbers, register device, setup ADC and timer |
| counter registers for 100 msec periodic sampling. |
*****************************************************************************************/
static int __init at91adc_init (void)
{
	int ret;

	// Dynamically allocate major number and minor number
	ret = alloc_chrdev_region(&at91adc_devno, // pointer to where the device number to be stored
	0, // first minor number requested
	1, // number of devices
	"at91adc"); // device name

	if (ret < 0)
	{
		printk(KERN_INFO "at91adc: Device number allocation failed\n");
		ret = -ENODEV;
		goto exit_1;
	}

	// Initialize cdev structure.
	cdev_init(&at91adc_cdev, // pointer to the cdev structure
	&at91adc_fops); // pointer to the file operations structure.
	at91adc_cdev.owner = THIS_MODULE;
	at91adc_cdev.ops = &at91adc_fops;

	// Register the device with kernel
	ret = cdev_add(&at91adc_cdev, // pointer to the initialized cdev structure
	at91adc_devno, // device number allocated
	1); // number of devices
	
	if (ret != 0)
	{
		printk(KERN_INFO "at91adc: Device registration failed\n");
		ret = -ECANCELED;
		goto exit_2;
	}

	// Character device driver initialization complete. Do device specific initialization now.

	// Allocate ring buffer memory for storing ADC values for both channels.
	at91adc_pbuf0 = (unsigned short *)kmalloc((MAX_ADCSAMPLES * sizeof(unsigned short)), // Number of bytes
	GFP_KERNEL); // Flags
	at91adc_pbuf1 = (unsigned short *)kmalloc((MAX_ADCSAMPLES * sizeof(unsigned short)), // Number of bytes
	GFP_KERNEL); // Flags
	at91adc_pbuf2 = (unsigned short *)kmalloc((MAX_ADCSAMPLES * sizeof(unsigned short)), // Number of bytes
	GFP_KERNEL); // Flags
	at91adc_pbuf3 = (unsigned short *)kmalloc((MAX_ADCSAMPLES * sizeof(unsigned short)), // Number of bytes
	GFP_KERNEL); // Flags
	if ((at91adc_pbuf0 == NULL) || (at91adc_pbuf1 == NULL) || (at91adc_pbuf2 == NULL) || (at91adc_pbuf3 == NULL))
	{
		printk(KERN_INFO "at91adc: Memory allocation failed\n");
		ret = -ECANCELED;
		goto exit_3;
	}

	// Initialize the ring buffer and append index.
	at91adc_appidx = MAX_ADCSAMPLES;
	for (ret = 0; ret < MAX_ADCSAMPLES; ret++)
	{
		at91adc_pbuf0[ret] = 0;
		at91adc_pbuf1[ret] = 0;
		at91adc_pbuf2[ret] = 0;
		at91adc_pbuf3[ret] = 0;
	}

	// Initialize ADC. The following two lines set the appropriate PMC bit
	// for the ADC. Easier than mapping PMC registers and then setting the bit.
	at91adc_clk = clk_get(NULL, // Device pointer - not required.
	"adc_clk"); // Clock name.
	clk_enable(at91adc_clk);

	// Map ADC registers to the current address space.
	at91adc_base = ioremap_nocache(AT91SAM9260_BASE_ADC, // Physical address
	64); // Number of bytes to be mapped.
	
	if (at91adc_base == NULL)
	{
		printk(KERN_INFO "at91adc: ADC memory mapping failed\n");
		ret = -EACCES;
		goto exit_4;
	}

	// MUX GPIO pins for ADC (peripheral A) operation
	at91_set_A_periph(AT91_PIN_PC0, 0);
	at91_set_A_periph(AT91_PIN_PC1, 0);
	at91_set_A_periph(AT91_PIN_PC2, 0);
	at91_set_A_periph(AT91_PIN_PC3, 0);

	// Reset the ADC
	iowrite32(AT91_ADC_SWRST, (at91adc_base + AT91_ADC_CR));

	// Enable all ADC channels
	iowrite32((AT91_ADC_CH(3) | AT91_ADC_CH(2) | AT91_ADC_CH(1) | AT91_ADC_CH(0)), (at91adc_base + AT91_ADC_CHER));

	// Configure ADC mode register.
	// From table 43-31 in page #775 and page#741 of AT91SAM9260 user manual:
	// Maximum ADC clock frequency = 5MHz = MCK / ((PRESCAL+1) * 2)
	// PRESCAL = ((MCK / 5MHz) / 2) -1 = ((100MHz / 5MHz)/2)-1) = 9
	// Maximum startup time = 15uS = (STARTUP+1)*8/ADC_CLOCK
	// STARTUP = ((15uS*ADC_CLOK)/8)-1 = ((15uS*5MHz)/8)-1 = 9
	// Minimum hold time = 1.2uS = (SHTIM+1)/ADC_CLOCK
	// SHTIM = (1.2uS*ADC_CLOCK)-1 = (1.2uS*5MHz)-1 = 5, Use 9 to ensure 2uS hold time.
	// Enable sleep mode and hardware trigger from TIOA output from TC0.
	iowrite32((AT91_ADC_TRGSEL_TC0 | AT91_ADC_SHTIM_(9) | AT91_ADC_STARTUP_(9) | AT91_ADC_PRESCAL_(9) |
	/*AT91_ADC_SLEEP |*/ AT91_ADC_TRGEN), (at91adc_base + AT91_ADC_MR));

	// Initialize Timer Counter module 0. The following two lines set the appropriate
	// PMC bit for TC0. Easier than mapping PMC registers and then setting the bit.
	at91tc0_clk = clk_get(NULL, // Device pointer - not required.
	"tc0_clk"); // Clock name.
	clk_enable(at91tc0_clk);

	// Map TC0 registers to the current address space.
	at91tc0_base = ioremap_nocache(AT91SAM9260_BASE_TC0, // Physical address
	64); // Number of bytes to be mapped.
	if (at91tc0_base == NULL)
	{
		printk(KERN_INFO "at91adc: TC0 memory mapping failed\n");
		ret = -EACCES;
		goto exit_5;
	}

	// Configure TC0 in waveform mode, TIMER_CLK1 and to generate interrupt on RC compare.
	// Load 50000 to RC so that with TIMER_CLK1 = MCK/2 = 50MHz, the interrupt will be
	// generated every 1/50MHz * 50000 = 20nS * 50000 = 1 milli second.
	// NOTE: Even though AT91_TC_RC is a 32-bit register, only 16-bits are programmble.

	//printk(KERN_INFO "RC: %08X\n",ioread32(at91tc0_base + AT91_TC_RC));
	//printk(KERN_INFO "CMR: %08X\n",ioread32(at91tc0_base + AT91_TC_CMR));
	//printk(KERN_INFO "IMR: %08X\n",ioread32(at91tc0_base + AT91_TC_IMR));
	//printk(KERN_INFO "BMR: %08X\n",ioread32(at91tc0_base + AT91_TC_BMR));
	//printk(KERN_INFO "SR: %08X\n",ioread32(at91tc0_base + AT91_TC_SR));

	iowrite32(1134 /*50000*/, (at91tc0_base + AT91_TC_RC));  /// konfiguracja timera 44khz jak zmiana to zmieniæ wartoœæ w rej AT91_TC_RC  // skopiowac do ioctl z odpowiednimi case'ami od argumentu arg. 
	iowrite32((AT91_TC_WAVE | AT91_TC_WAVESEL_UP_AUTO), (at91tc0_base + AT91_TC_CMR));
	iowrite32(AT91_TC_CPCS, (at91tc0_base + AT91_TC_IDR)); // wy³¹czane przerwanie od timera  
	iowrite32((AT91_TC_SWTRG | AT91_TC_CLKEN), (at91tc0_base + AT91_TC_CCR));

	// Install interrupt for TC0.
	ret = request_irq(AT91SAM9260_ID_TC0, // Interrupt number
	at91tc0_isr, // Pointer to the interrupt sub-routine
	IRQF_DISABLED | IRQF_NOBALANCING | IRQF_IRQPOLL, // Flags - fast, shared or contributing to entropy pool
	"at91adc", // Device name to show as owner in /proc/interrupts
	NULL); // Private data for shared interrupts

	ret = 0;
	
	if (ret != 0)
	{
		printk(KERN_INFO "at91adc: Timer interrupt request failed\n");
		ret = -EBUSY;
		goto exit_6;
	}

	printk(KERN_INFO "at91adc: Loaded module\n");
	return 0;

exit_6:
	iounmap(at91tc0_base);

exit_5:
	clk_disable(at91tc0_clk);
	iounmap(at91adc_base);

exit_4:
	clk_disable(at91adc_clk);

exit_3:
	kfree(at91adc_pbuf0);
	kfree(at91adc_pbuf1);
	kfree(at91adc_pbuf2);
	kfree(at91adc_pbuf3);

exit_2:
	// Free device number allocated.
	unregister_chrdev_region(at91adc_devno, // allocated device number
	1); // number of devices

exit_1:
	return ret;
}

static void __exit at91adc_exit (void)
{
	// Reset PMC bit for ADC and TC0
	clk_disable(at91adc_clk);
	clk_disable(at91tc0_clk);

	// Free TC0 IRQ.
	free_irq(AT91SAM9260_ID_TC0, // Interrupt number
	NULL); // Private data for shared interrupts

	// Unmap ADC and TC0 register map.
	iounmap(at91adc_base);
	iounmap(at91tc0_base);

	// Free kernel memory allocated
	kfree(at91adc_pbuf0);
	kfree(at91adc_pbuf1);
	kfree(at91adc_pbuf2);
	kfree(at91adc_pbuf3);

	// Free device number allocated.
	unregister_chrdev_region(at91adc_devno, // allocated device number
	1); // number of devices

	printk(KERN_INFO "at91adc: Unloaded module\n");
}

/*****************************************************************************************
| Module open: |
*****************************************************************************************/
static int at91adc_open (struct inode *inode, struct file *filp)
{
	return 0;
}

/*****************************************************************************************
| Module close: |
*****************************************************************************************/
static int at91adc_release (struct inode *inode, struct file *filp)
{
	return 0;
}

/*****************************************************************************************
| Module read: Return last READ_SAMPLES samples from ADC chan 0 or 1 depending on the |
| minor number. |
*****************************************************************************************/
static ssize_t at91adc_read (struct file *filp, char __iomem *buf, size_t bufsize, loff_t *f_pos)
{
	unsigned int minor;

	minor = iminor(filp->f_dentry->d_inode);
	if (minor!=0) return 0;

	if (bufsize < (sizeof(unsigned short) * 4 * MAX_ADCSAMPLES)) return 0;

	if (at91adc_appidx < MAX_ADCSAMPLES) return 0;

	iowrite32(AT91_TC_CPCS, (at91tc0_base + AT91_TC_IDR)); // wy³¹cza przerwanie timera

	copy_to_user(buf + (sizeof(unsigned short) * 0 * MAX_ADCSAMPLES),at91adc_pbuf0,sizeof(unsigned short) * MAX_ADCSAMPLES);  // 
	copy_to_user(buf + (sizeof(unsigned short) * 1 * MAX_ADCSAMPLES),at91adc_pbuf1,sizeof(unsigned short) * MAX_ADCSAMPLES);
	copy_to_user(buf + (sizeof(unsigned short) * 2 * MAX_ADCSAMPLES),at91adc_pbuf2,sizeof(unsigned short) * MAX_ADCSAMPLES);
	copy_to_user(buf + (sizeof(unsigned short) * 3 * MAX_ADCSAMPLES),at91adc_pbuf3,sizeof(unsigned short) * MAX_ADCSAMPLES);

	return (sizeof(unsigned short) * 4 * MAX_ADCSAMPLES);
}

static int at91adc_ioctl(struct inode * i, struct file * f, unsigned int cmd, unsigned long arg)
{
	//printk(KERN_INFO "adc ioctl %d,%d\n",(int)cmd,(int)arg);

	switch (cmd)
	{
	case 0: at91adc_appidx = 0; iowrite32(AT91_TC_CPCS, (at91tc0_base + AT91_TC_IER)); break;
	}

	return -ENOIOCTLCMD;
}

struct file_operations at91adc_fops = {
.owner = THIS_MODULE,
.open = at91adc_open,
.read = at91adc_read,
.ioctl = at91adc_ioctl,
.release = at91adc_release,
};

module_init(at91adc_init);
module_exit(at91adc_exit);

MODULE_AUTHOR("Sudhir Thalore");
MODULE_DESCRIPTION("Initialize and read AT91SAM9260 ADC channels");
MODULE_LICENSE("GPL");

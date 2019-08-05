#ifdef CONFIG_BKSV
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <asm/irq.h>
#include <linux/platform_data/serial-imx.h>
#include <linux/platform_data/dma-imx.h>

#include "serial_mctrl_gpio.h"

/* Register definitions */
#define URXD0 0x0  /* Receiver Register */
#define URTX0 0x40 /* Transmitter Register */
#define UCR1  0x80 /* Control Register 1 */
#define UCR2  0x84 /* Control Register 2 */
#define UCR3  0x88 /* Control Register 3 */
#define UCR4  0x8c /* Control Register 4 */
#define UFCR  0x90 /* FIFO Control Register */
#define USR1  0x94 /* Status Register 1 */
#define USR2  0x98 /* Status Register 2 */
#define UESC  0x9c /* Escape Character Register */
#define UTIM  0xa0 /* Escape Timer Register */
#define UBIR  0xa4 /* BRM Incremental Register */
#define UBMR  0xa8 /* BRM Modulator Register */
#define UBRC  0xac /* Baud Rate Count Register */
#define IMX21_ONEMS 0xb0 /* One Millisecond register */
#define IMX1_UTS 0xd0 /* UART Test Register on i.mx1 */
#define IMX21_UTS 0xb4 /* UART Test Register on all other i.mx*/

/* UART Control Register Bit Fields.*/
#define URXD_DUMMY_READ (1<<16)
#define URXD_CHARRDY	(1<<15)
#define URXD_ERR	(1<<14)
#define URXD_OVRRUN	(1<<13)
#define URXD_FRMERR	(1<<12)
#define URXD_BRK	(1<<11)
#define URXD_PRERR	(1<<10)
#define URXD_RX_DATA	(0xFF<<0)
#define UCR1_ADEN	(1<<15) /* Auto detect interrupt */
#define UCR1_ADBR	(1<<14) /* Auto detect baud rate */
#define UCR1_TRDYEN	(1<<13) /* Transmitter ready interrupt enable */
#define UCR1_IDEN	(1<<12) /* Idle condition interrupt */
#define UCR1_ICD_REG(x) (((x) & 3) << 10) /* idle condition detect */
#define UCR1_RRDYEN	(1<<9)	/* Recv ready interrupt enable */
#define UCR1_RDMAEN	(1<<8)	/* Recv ready DMA enable */
#define UCR1_IREN	(1<<7)	/* Infrared interface enable */
#define UCR1_TXMPTYEN	(1<<6)	/* Transimitter empty interrupt enable */
#define UCR1_RTSDEN	(1<<5)	/* RTS delta interrupt enable */
#define UCR1_SNDBRK	(1<<4)	/* Send break */
#define UCR1_TDMAEN	(1<<3)	/* Transmitter ready DMA enable */
#define IMX1_UCR1_UARTCLKEN (1<<2) /* UART clock enabled, i.mx1 only */
#define UCR1_ATDMAEN    (1<<2)  /* Aging DMA Timer Enable */
#define UCR1_DOZE	(1<<1)	/* Doze */
#define UCR1_UARTEN	(1<<0)	/* UART enabled */
#define UCR2_ESCI	(1<<15)	/* Escape seq interrupt enable */
#define UCR2_IRTS	(1<<14)	/* Ignore RTS pin */
#define UCR2_CTSC	(1<<13)	/* CTS pin control */
#define UCR2_CTS	(1<<12)	/* Clear to send */
#define UCR2_ESCEN	(1<<11)	/* Escape enable */
#define UCR2_PREN	(1<<8)	/* Parity enable */
#define UCR2_PROE	(1<<7)	/* Parity odd/even */
#define UCR2_STPB	(1<<6)	/* Stop */
#define UCR2_WS		(1<<5)	/* Word size */
#define UCR2_RTSEN	(1<<4)	/* Request to send interrupt enable */
#define UCR2_ATEN	(1<<3)	/* Aging Timer Enable */
#define UCR2_TXEN	(1<<2)	/* Transmitter enabled */
#define UCR2_RXEN	(1<<1)	/* Receiver enabled */
#define UCR2_SRST	(1<<0)	/* SW reset */
#define UCR3_DTREN	(1<<13) /* DTR interrupt enable */
#define UCR3_PARERREN	(1<<12) /* Parity enable */
#define UCR3_FRAERREN	(1<<11) /* Frame error interrupt enable */
#define UCR3_DSR	(1<<10) /* Data set ready */
#define UCR3_DCD	(1<<9)	/* Data carrier detect */
#define UCR3_RI		(1<<8)	/* Ring indicator */
#define UCR3_ADNIMP	(1<<7)	/* Autobaud Detection Not Improved */
#define UCR3_RXDSEN	(1<<6)	/* Receive status interrupt enable */
#define UCR3_AIRINTEN	(1<<5)	/* Async IR wake interrupt enable */
#define UCR3_AWAKEN	(1<<4)	/* Async wake interrupt enable */
#define UCR3_DTRDEN	(1<<3)	/* Data Terminal Ready Delta Enable. */
#define IMX21_UCR3_RXDMUXSEL	(1<<2)	/* RXD Muxed Input Select */
#define UCR3_INVT	(1<<1)	/* Inverted Infrared transmission */
#define UCR3_BPEN	(1<<0)	/* Preset registers enable */
#define UCR4_CTSTL_SHF	10	/* CTS trigger level shift */
#define UCR4_CTSTL_MASK	0x3F	/* CTS trigger is 6 bits wide */
#define UCR4_INVR	(1<<9)	/* Inverted infrared reception */
#define UCR4_ENIRI	(1<<8)	/* Serial infrared interrupt enable */
#define UCR4_WKEN	(1<<7)	/* Wake interrupt enable */
#define UCR4_REF16	(1<<6)	/* Ref freq 16 MHz */
#define UCR4_IDDMAEN    (1<<6)  /* DMA IDLE Condition Detected */
#define UCR4_IRSC	(1<<5)	/* IR special case */
#define UCR4_TCEN	(1<<3)	/* Transmit complete interrupt enable */
#define UCR4_BKEN	(1<<2)	/* Break condition interrupt enable */
#define UCR4_OREN	(1<<1)	/* Receiver overrun interrupt enable */
#define UCR4_DREN	(1<<0)	/* Recv data ready interrupt enable */
#define UFCR_RXTL_SHF	0	/* Receiver trigger level shift */
#define UFCR_DCEDTE	(1<<6)	/* DCE/DTE mode select */
#define UFCR_RFDIV	(7<<7)	/* Reference freq divider mask */
#define UFCR_RFDIV_REG(x)	(((x) < 7 ? 6 - (x) : 6) << 7)
#define UFCR_TXTL_SHF	10	/* Transmitter trigger level shift */
#define USR1_PARITYERR	(1<<15) /* Parity error interrupt flag */
#define USR1_RTSS	(1<<14) /* RTS pin status */
#define USR1_TRDY	(1<<13) /* Transmitter ready interrupt/dma flag */
#define USR1_RTSD	(1<<12) /* RTS delta */
#define USR1_ESCF	(1<<11) /* Escape seq interrupt flag */
#define USR1_FRAMERR	(1<<10) /* Frame error interrupt flag */
#define USR1_RRDY	(1<<9)	 /* Receiver ready interrupt/dma flag */
#define USR1_AGTIM	(1<<8)	 /* Ageing timer interrupt flag */
#define USR1_DTRD	(1<<7)	 /* DTR Delta */
#define USR1_RXDS	 (1<<6)	 /* Receiver idle interrupt flag */
#define USR1_AIRINT	 (1<<5)	 /* Async IR wake interrupt flag */
#define USR1_AWAKE	 (1<<4)	 /* Aysnc wake interrupt flag */
#define USR2_ADET	 (1<<15) /* Auto baud rate detect complete */
#define USR2_TXFE	 (1<<14) /* Transmit buffer FIFO empty */
#define USR2_DTRF	 (1<<13) /* DTR edge interrupt flag */
#define USR2_IDLE	 (1<<12) /* Idle condition */
#define USR2_RIDELT	 (1<<10) /* Ring Interrupt Delta */
#define USR2_RIIN	 (1<<9)	 /* Ring Indicator Input */
#define USR2_IRINT	 (1<<8)	 /* Serial infrared interrupt flag */
#define USR2_WAKE	 (1<<7)	 /* Wake */
#define USR2_DCDIN	 (1<<5)	 /* Data Carrier Detect Input */
#define USR2_RTSF	 (1<<4)	 /* RTS edge interrupt flag */
#define USR2_TXDC	 (1<<3)	 /* Transmitter complete */
#define USR2_BRCD	 (1<<2)	 /* Break condition */
#define USR2_ORE	(1<<1)	 /* Overrun error */
#define USR2_RDR	(1<<0)	 /* Recv data ready */
#define UTS_FRCPERR	(1<<13) /* Force parity error */
#define UTS_LOOP	(1<<12)	 /* Loop tx and rx */
#define UTS_TXEMPTY	 (1<<6)	 /* TxFIFO empty */
#define UTS_RXEMPTY	 (1<<5)	 /* RxFIFO empty */
#define UTS_TXFULL	 (1<<4)	 /* TxFIFO full */
#define UTS_RXFULL	 (1<<3)	 /* RxFIFO full */
#define UTS_SOFTRST	 (1<<0)	 /* Software reset */

#define IMX_RXBD_NUM 20

//////////////////////////////////////////////////////////

/* defines common for all i.MX */
#define GPT_CR 		0x00
#define GPT_PR 		0x04
#define GPT_SR   	0x08
#define GPT_IR		0x0c
#define GPT_OCR1 	0x10
#define GPT_OCR2 	0x14
#define GPT_OCR3 	0x18
#define GPT_ICR1 	0x1C
#define GPT_ICR2 	0x20
#define GPT_CNT	 	0x24

//////////////////////////////////////////////////////////
typedef struct
{
    int valid;
    uint32_t ppstime;
    int length;
    uint32_t cnt1;
    uint32_t cnt2;
    unsigned char message[4*1024];
} gps_pps_data_t;

gps_pps_data_t gps_pps_data;  //global vaiable
extern spinlock_t gps_pps_lock;
/////////////////////////////////////////////////////////////////
struct imx_dma_bufinfo {
	bool filled;
	unsigned int rx_bytes;
};

struct imx_dma_rxbuf {
	unsigned int		periods;
	unsigned int		period_len;
	unsigned int		buf_len;

	void			*buf;
	dma_addr_t		dmaaddr;
	unsigned int		cur_idx;
	unsigned int		last_completed_idx;
	dma_cookie_t		cookie;
	struct imx_dma_bufinfo	buf_info[IMX_RXBD_NUM];
};

struct imx_port {
	struct uart_port	port;
	struct timer_list	timer;
	unsigned int		old_status;
	unsigned int		have_rtscts:1;
	unsigned int		dte_mode:1;
	unsigned int		irda_inv_rx:1;
	unsigned int		irda_inv_tx:1;
	unsigned short		trcv_delay; /* transceiver delay */
	struct clk		*clk_ipg;
	struct clk		*clk_per;
	const struct imx_uart_data *devdata;

	struct mctrl_gpios *gpios;

	/* DMA fields */
	unsigned int		dma_is_inited:1;
	unsigned int		dma_is_enabled:1;
	unsigned int		dma_is_rxing:1;
	unsigned int		dma_is_txing:1;
	struct dma_chan		*dma_chan_rx, *dma_chan_tx;
	struct scatterlist	tx_sgl[2];
	struct imx_dma_rxbuf	rx_buf;
	unsigned int		tx_bytes;
	unsigned int		dma_tx_nents;
	struct work_struct	tsk_dma_tx;
	wait_queue_head_t	dma_wait;
	unsigned int            saved_reg[10];
	bool			context_saved;
#define DMA_TX_IS_WORKING 1
	unsigned long		flags;
};

void gps_pps(struct imx_port *sport)
{
    int i;
    int mempil = 0;
    int diff;
    static int gpt_valid = 0;
    static void __iomem volatile *gpt = NULL;    //General Purpose Timer
    uint32_t icr1;
    static uint32_t old_icr1=0;
    uint32_t cnt;
    uint64_t cnt64;
    unsigned long flags;

//	struct tty_struct *tty = sport->port.state->port.tty;
	unsigned int cur_idx = sport->rx_buf.cur_idx;
    if (gpt_valid == 0)
    {//map gpt
        gpt=(uint32_t *)ioremap_nocache(0x302e0000,4096);
        if (gpt==NULL)
        {
            printk("gps_pps: request_mem_region gpt failed\n");
            return;
        }
        gpt_valid = 1;
    }
 	spin_lock_irqsave(&gps_pps_lock, flags);
    gps_pps_data.cnt1=__raw_readl(gpt + GPT_CNT);
    icr1 = __raw_readl(gpt + GPT_ICR1);
    if (icr1 == old_icr1)
    {
        gps_pps_data.valid = 0;
     	spin_unlock_irqrestore(&gps_pps_lock, flags);
        return;
    }
    old_icr1 = icr1;
    cnt  = __raw_readl(gpt + GPT_CNT);
    if (icr1 > cnt)
    {
        cnt64 = cnt | 0x100000000ll;
    }
    else
    {
        cnt64 = cnt;
    }
    diff = cnt64-icr1;    
    if (diff > 16777216)
    {
        gps_pps_data.valid = 0;
     	spin_unlock_irqrestore(&gps_pps_lock, flags);
        return;
    }
//    printk("%s %d %d pps=%llx\n",__func__,sport->rx_buf.last_completed_idx,cur_idx,gps_pps_data.ppstime);
    gps_pps_data.ppstime = icr1;

	if (sport->rx_buf.last_completed_idx < cur_idx)
    {
        for (i=sport->rx_buf.last_completed_idx + 1;i<cur_idx;i++)
        {
//            printk("1 %d %d\n",i,sport->rx_buf.buf_info[i].rx_bytes);
            memcpy(&gps_pps_data.message[mempil],
                   &(((unsigned char *)sport->rx_buf.dmaaddr)[sport->rx_buf.period_len*i]),
                   sport->rx_buf.buf_info[i].rx_bytes);
            mempil += sport->rx_buf.buf_info[i].rx_bytes;
            if (mempil > (4*1024))
                break;
        }
	}
    else
    if (sport->rx_buf.last_completed_idx == (IMX_RXBD_NUM - 1))
    {
        for (i=0;i<cur_idx;i++)
        {
//            printk("2 %d %d\n",i,sport->rx_buf.buf_info[i].rx_bytes);
            memcpy(&gps_pps_data.message[mempil],
                   &(((unsigned char *)sport->rx_buf.dmaaddr)[sport->rx_buf.period_len*i]),
                   sport->rx_buf.buf_info[i].rx_bytes);
            mempil += sport->rx_buf.buf_info[i].rx_bytes;
            if (mempil > (4*1024))
                break;
        }
	}
    else
    {
        for (i=sport->rx_buf.last_completed_idx + 1;i<IMX_RXBD_NUM;i++)
        {
//            printk("3 %d %d\n",i,sport->rx_buf.buf_info[i].rx_bytes);
            memcpy(&gps_pps_data.message[mempil],
                   &(((unsigned char *)sport->rx_buf.dmaaddr)[sport->rx_buf.period_len*i]),
                   sport->rx_buf.buf_info[i].rx_bytes);
            mempil += sport->rx_buf.buf_info[i].rx_bytes;
            if (mempil > (4*1024))
                break;
        }
        for (i=0;i<cur_idx;i++)
        {
//            printk("4 %d %d\n",i,sport->rx_buf.buf_info[i].rx_bytes);
            memcpy(&gps_pps_data.message[mempil],
                   &(((unsigned char *)sport->rx_buf.dmaaddr)[sport->rx_buf.period_len*i]),
                   sport->rx_buf.buf_info[i].rx_bytes);
            mempil += sport->rx_buf.buf_info[i].rx_bytes;
            if (mempil > (4*1024))
                break;
        }
	}
    gps_pps_data.length=mempil;
    gps_pps_data.valid = 1;
    gps_pps_data.cnt2=__raw_readl(gpt + GPT_CNT);
	spin_unlock_irqrestore(&gps_pps_lock, flags);
/*    
    i=0;
    do
    {
        if (gps_pps_data.message[i]==0xb5)
        {
            short int *len =(short int *)&gps_pps_data.message[i+4];
            printk("i=%d %02x %02x %02x %02x len=%d \n",i,gps_pps_data.message[i+0],gps_pps_data.message[i+1],gps_pps_data.message[i+2],gps_pps_data.message[i+3],*len);
            i +=  2 + 1 + 1 + 2 + *len + 2;
            if(i >= gps_pps_data.length)
                break;
        }
        if (gps_pps_data.message[i]!=0xb5)
        if (i < gps_pps_data.length)
            printk("i=%d %02x %02x %02x %02x\n",i,gps_pps_data.message[i+0],gps_pps_data.message[i+1],gps_pps_data.message[i+2],gps_pps_data.message[i+3]);
        
    } while (gps_pps_data.message[i]==0xb5);
*/
}
#endif //CONFIG_BKSV
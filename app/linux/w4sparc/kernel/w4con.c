/*
 * w4con - virtual serial port driver for work on top of WrmOS.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/clk.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/delay.h>


void wrm_loge(const char*, ...);
void wrm_logd(const char*, ...);
void l4_kdb_putsn(const char* str, size_t len);
int w4console_write(const char* str, size_t len);

/* struct w4con_sport - w4con serial port descriptor
 * @port: uart port descriptor
 * @idx: port index
 * @irq_rx: virtual rx interrupt number
 * @irqflags_rx: flags related to rx irq
 * @irq_rx_name: irq rx name
 * @dev: device descriptor
 **/
struct w4con_sport
{
        struct uart_port port;
        int idx;
        int irq_rx;
        int irqflags_rx;
        const char *irq_rx_name;
        struct device *dev;
};
#define to_w4con_sport(c) container_of(c, struct w4con_sport, port)
#define w4con_get_port(sport) (&sport->port)
#define w4con_get_opt(sport) (&sport->opt)
#define tx_irq_enabled(sport) (sport->enable_tx_irq)

// UART name and device definitions
#define W4CON_DEV_NAME  "wrmos-uart"
#define W4CON_MAX_UARTS 1
#define W4CON_SDEV_NAME "ttyWrm"
#define PORT_W4CON 0xabcd

// w4con_sport pointer for console use
static struct w4con_sport *w4con_sports[W4CON_MAX_UARTS];

// serial core request to check if uart tx buffer is empty
static unsigned int w4con_uart_tx_empty(struct uart_port *port)
{
	return 1;  // always ready for tx
}

// serial core request to set UART outputs
static void w4con_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	// nothing
}

// serial core request to return the state of misc UART input pins
static unsigned int w4con_uart_get_mctrl(struct uart_port *port)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
	return 0;
}

// serial core request to disable tx ASAP (used for flow control)
static void w4con_uart_stop_tx(struct uart_port *port)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
}

static void insert_char(struct uart_port* port, const char ch)
{
	struct tty_port* tty = &port->state->port;
	spin_lock(&port->lock);
	port->icount.rx++;
	if (!uart_handle_sysrq_char(port, ch))
		tty_insert_flip_char(tty, ch, TTY_NORMAL);
	spin_unlock(&port->lock);
	tty_flip_buffer_push(tty);
}

// serial core request to (re)enable tx
static void w4con_uart_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	//wrm_logd("%s:  entry:  port->x_char=%d, port->icount.tx=%d.\n",
	// __func__, port->x_char, port->icount.tx);

	if (uart_tx_stopped(port))
	{
		wrm_loge("%s:  uart_tx_stopped.\n", __func__);
		return;
	}

	while (xmit->head != xmit->tail)
	{
		if (xmit->head > xmit->tail)  // +---t====h---+
		{
			unsigned sz = xmit->head - xmit->tail;
			int rc = w4console_write(xmit->buf + xmit->tail, sz);
			if (rc < 0)
			{
				wrm_loge("%s:  w4console_write() - rc=%d, %u bytes was lost.\n", __func__, rc, sz);
				rc = sz;
			}
			port->icount.tx += rc;
			xmit->tail = xmit->head;
		}
		else                          // +===h----t===+
		{
			unsigned sz = UART_XMIT_SIZE - xmit->tail;
			int rc = w4console_write(xmit->buf + xmit->tail, sz);
			if (rc < 0)
			{
				wrm_loge("%s:  w4console_write() - rc=%d, %u bytes was lost.\n", __func__, rc, sz);
				rc = sz;
			}
			port->icount.tx += rc;
			xmit->tail = 0;
		}
	}
}

// serial core request to stop rx, called before port shutdown
static void w4con_uart_stop_rx(struct uart_port *port)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
}

// serial core request to start/stop emitting break char
static void w4con_uart_break_ctl(struct uart_port *port, int ctl)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
}

// get port type in string format
static const char *w4con_uart_type(struct uart_port *port)
{
	return (port->type == PORT_W4CON) ? W4CON_DEV_NAME : NULL;
}

char console_get_input_char(void);

// read all chars in rx fifo and send them to core
static void w4con_uart_do_rx(struct uart_port *port)
{
	while (1)
	{
		char c = console_get_input_char();
		if (!c)
			break;
		insert_char(port, c);
	}
}

// RX interrupt handler
static irqreturn_t w4con_uart_rx_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	w4con_uart_do_rx(port);
	return IRQ_HANDLED;
}

// serial core request to initialize uart and start rx operation
static int w4con_uart_startup(struct uart_port *port)
{
	struct w4con_sport *sport = to_w4con_sport(port);
	int ret;

	sport->irq_rx_name = kasprintf(GFP_KERNEL, "%s%d-rx", w4con_uart_type(port), sport->idx);
	if (!sport->irq_rx_name)
	{
		dev_err(port->dev, "%s: kasprintf err!", __func__);
		ret = -ENOMEM;
		return ret;
	}
	irq_set_status_flags(sport->irq_rx, 0/*IRQ_NOAUTOEN*/);
	ret = request_irq(sport->irq_rx, w4con_uart_rx_interrupt, sport->irqflags_rx, sport->irq_rx_name, port);
	if (ret)
	{
		dev_err(port->dev, "%s: request irq(%d) err! ret:%d name:%s\n", __func__, sport->irq_rx, ret, w4con_uart_type(port));
		return ret;
	}
	return 0;
}

// serial core request to flush & disable uart
static void w4con_uart_shutdown(struct uart_port *port)
{
	struct w4con_sport *sport = to_w4con_sport(port);

	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");

	free_irq(sport->irq_rx, port);
}

// serial core request to change current uart setting
static void w4con_uart_set_termios(struct uart_port *port, struct ktermios *new, struct ktermios *old)
{
	// nothing
}

// serial core request to claim uart iomem
static int w4con_uart_request_port(struct uart_port *port)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
	return 0;
}

// serial core request to release uart iomem
static void w4con_uart_release_port(struct uart_port *port)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsigned");
}

// serial core request to do any port required auto-configuration
static void w4con_uart_config_port(struct uart_port *port, int flags)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
}

// serial core request to check that port information in serinfo are suitable
static int w4con_uart_verify_port(struct uart_port *port, struct serial_struct *serinfo)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
	return 0;
}

// serial core callbacks
static const struct uart_ops w4con_uart_ops =
{
	.tx_empty     = w4con_uart_tx_empty,
	.get_mctrl    = w4con_uart_get_mctrl,
	.set_mctrl    = w4con_uart_set_mctrl,
	.start_tx     = w4con_uart_start_tx,
	.stop_tx      = w4con_uart_stop_tx,
	.stop_rx      = w4con_uart_stop_rx,
	.break_ctl    = w4con_uart_break_ctl,
	.startup      = w4con_uart_startup,
	.shutdown     = w4con_uart_shutdown,
	.set_termios  = w4con_uart_set_termios,
	.type         = w4con_uart_type,
	.release_port = w4con_uart_release_port,
	.request_port = w4con_uart_request_port,
	.config_port  = w4con_uart_config_port,
	.verify_port  = w4con_uart_verify_port,
};

// console core request to output given string
static void w4con_console_write(struct console *co, const char *str, unsigned sz)
{
	//wrm_logd("%s:  entry:  sz=%u.\n",  __func__, sz);
	int rc = w4console_write(str, sz);
	if ((unsigned)rc != sz)
		wrm_loge("%s:  w4console_write() - rc=%d, sz=%u, %u bytes was lost.\n", __func__, rc, sz);
}

static struct uart_driver w4con_uart_driver;

// console core request to setup given console
static int w4con_console_setup(struct console *co, char *options)
{
	// nothing
	return 0;
}

// static struct uart_driver w4con_uart_driver;
static struct console w4con_console =
{
	.name   = W4CON_SDEV_NAME,
	.write  = w4con_console_write,
	.device = uart_console_device,
	.setup  = w4con_console_setup,
	.flags  = CON_PRINTBUFFER,
	.index  = -1,
	.data   = &w4con_uart_driver
};

static int __init w4con_console_init(void)
{
	register_console(&w4con_console);
	return 0;
}
console_initcall(w4con_console_init);

static inline bool is_w4con_console_port(struct uart_port *port)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");
	return port->cons && port->cons->index == port->line;
}

// late console initialization
static int __init w4con_late_console_init(void)
{
	if (!(w4con_console.flags & CON_ENABLED))
		register_console(&w4con_console);
	return 0;
}

core_initcall(w4con_late_console_init);

static struct uart_driver w4con_uart_driver =
{
	.owner       = THIS_MODULE,
	.driver_name = W4CON_DEV_NAME,
	.dev_name    = W4CON_SDEV_NAME,
	.nr          = W4CON_MAX_UARTS,
	.cons        = &w4con_console,
};

static int w4con_uart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct w4con_sport *sport;
	struct uart_port *port;
	int uart_idx = 0;
	int ret;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	sport->idx         = uart_idx;
	sport->irq_rx      = irq_of_parse_and_map(np, 0);
	sport->irqflags_rx = IRQF_NO_THREAD;
	sport->dev         = &pdev->dev;

	w4con_sports[uart_idx] = sport;
	port                 = &sport->port;
	memset(port, 0, sizeof(*port));
	port->iotype         = UPIO_MEM;
	port->mapbase        = 0;
	port->ops            = &w4con_uart_ops;
	port->flags          = UPF_BOOT_AUTOCONF;
	port->dev            = &pdev->dev;
	port->fifosize       = 0;
	port->uartclk        = 0;
	port->line           = uart_idx;

	port->type = PORT_W4CON; // wrm add

	ret = uart_add_one_port(&w4con_uart_driver, port);
	if (ret)
	{
		dev_err(port->dev, "%s: uart add port error!\n", __func__);
		goto err;
	}

	platform_set_drvdata(pdev, port);
	dev_info(&pdev->dev, "%s: uart(%d) driver initialized.\n", __func__, uart_idx);

	return 0;
err:
	return ret;
}

static int w4con_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");

	uart_remove_one_port(&w4con_uart_driver, port);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id w4con_serial_dt_ids[] =
{
	{ .compatible = "WrmLab,w4con" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, w4con_serial_dt_ids);

static struct platform_driver w4con_uart_platform_driver =
{
	.probe  = w4con_uart_probe,
	.remove = w4con_uart_remove,
	.driver =
	{
		.name           = W4CON_DEV_NAME,
		.of_match_table = of_match_ptr(w4con_serial_dt_ids),
	},
};

static int __init w4con_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&w4con_uart_driver);
	if (ret)
	{
		pr_err("failed to register %s:%d\n", w4con_uart_driver.driver_name, ret);
		panic("failed to register %s:%d\n", w4con_uart_driver.driver_name, ret);
		return ret;
	}

	ret = platform_driver_register(&w4con_uart_platform_driver);
	if (ret)
	{
		pr_err("fail to register w4con uart\n");
		panic("fail to register w4con uart\n");
		uart_unregister_driver(&w4con_uart_driver);
	}

	return ret;
}
arch_initcall(w4con_uart_init);

static void __exit w4con_uart_exit(void)
{
	wrm_loge("%s:  entry.\n", __func__);
	panic("unsupported");

	unregister_console(&w4con_console);
	platform_driver_unregister(&w4con_uart_platform_driver);
	uart_unregister_driver(&w4con_uart_driver);
}
module_exit(w4con_uart_exit);

MODULE_AUTHOR("Sergey Worm <sergey.worm@gmail.com>");
MODULE_DESCRIPTION("Virtual serial port driver for work on top of WrmOS");
MODULE_LICENSE("MIT");

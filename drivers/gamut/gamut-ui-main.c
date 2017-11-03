/*
 * iZotope Gamut UI driver
 *
 * Copyright (C) 2016-2017, iZotope inc.
 *
 * Authors: Matthew Campbell <mcampbell@izotope.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include "izspi.h"
#include "gamut-ui-main.h"

/* TODO: in this driver
 Done * create and register an input device that works for buttons
 Done  * gpio interrupt for the int line
 * create and register an input device that works for touch
 * create a custom Char device to expose the LEDs
 * Iron out the SPI comms to atmel (flow control, full duplex, etc)
*/

/* IOCTL stuff, should be moved into a sared header that user sapce can include*/
/* Right now it's just here as a nop so we can add IOCTLs eaisy later */
#define GAMUTLED_IOC_MAGIC 'i'
#define GAMUTLED_IOC_BASE 0x20

#define GAMUTLED_IOC_NOP _IO(GAMUTLED_IOC_MAGIC, GAMUTLED_IOC_BASE + 0)

#define DRV_NAME "gamut-ui"
#define LED_DEVNAME "gamutfpmsg"

/* Base backoff time for SPI comms errors. In seconds */
#define SPI_BASE_BACKOFF (0.050)

struct gamut_ui {
	/* pointers to useful things */
	struct device *dev;
	/* resources used by the driver */
	struct input_dev *input;
	int gpio_int; /* gpio used for interrupt from Atmel */
	int irq;
	/* workqueu related */
	struct workqueue_struct *xfer_wq;
	struct delayed_work xfer_work;
	int shutting_down;
	/* chrdev related */
	struct cdev cdev;
	int major;
	int minor;
	wait_queue_head_t rxq;
	wait_queue_head_t txq;
	/* Handling for the spi communications */
	struct izspi izspi;
	int atmel_can_rx;
	unsigned long spi_backoff; /* in jiffies */
	/* for debug */
	struct dentry *dbg_dir;
	int32_t txfrm_cnt;
	int32_t rxfrm_cnt;
};

static struct class * g_gamutfpmsg_class;

/**
 * gamut_ui_spi_xfer - do an spi transactino to the Atmel
 * @izspi: pointer to izspi which is embedded in the gamut_ui structure
 * @tx: pointer to tx buffer. Null for nothing
 * @rx: pointer to rx buffer. Null for nothing
 * @size: number of bytes in the transtion
 * @hold_ss: if 0 slave select is lowered after transaction
 *	     if 1 slave select is held after the transaction
 *
 * Return:
 *	0 = success
 *	-E* = error returned from spi transaction
 */
int gamut_ui_spi_xfer(struct izspi *izspi, char *tx, char *rx, int size, int hold_ss)
{
	struct spi_transfer xfers[] = {
		{
			.len = size,
			.tx_buf = tx,
			.rx_buf = rx,
			.cs_change = hold_ss != 0,
		},
	};

	return spi_sync_transfer(izspi->spi, xfers, ARRAY_SIZE(xfers));
}

static void xfer_work(struct work_struct *work)
{
	struct gamut_ui *prv = container_of(to_delayed_work(work),
						struct gamut_ui, xfer_work);
	int res;
	int gpio_int_state;
	int can_rx;
	int can_tx;
	int atmel_can_rx;

	res = izspi_xfer_frame(&prv->izspi);
	/* don't allow transmission if the atmel cleared the 'canrx' bit until
	 * the Atmel reasserts the int line */
	prv->atmel_can_rx = !(res & IZSPI_ATMEL_RX_BUF_FULL);

	if (res & IZSPI_DATA_TXED) {
		prv->txfrm_cnt++;
		wake_up_interruptible(&prv->txq);
	}
	if (res & IZSPI_DATA_RXED) {
		prv->rxfrm_cnt++;
		wake_up_interruptible(&prv->rxq);
	}

	/* Do not ever requeue work if we are shutting down */
	if (prv->shutting_down) {
		return;
	}

	gpio_int_state = gpio_get_value(prv->gpio_int);
	can_rx = izspi_can_rx_frame(&prv->izspi);
	can_tx = izspi_can_tx_frame(&prv->izspi);
	atmel_can_rx = prv->atmel_can_rx;

	/* clear backoff before requeue if no errors to avoid unneeded delay */
	if (!(res & (IZSPI_SPI_ERROR | IZSPI_ATMEL_RX_ERROR))) {
		prv->spi_backoff = 0;
	}

	/* if there is more to send, enqueue more work */
	if ((gpio_int_state == 0 && can_rx) || (can_tx && atmel_can_rx)) {
		queue_delayed_work(prv->xfer_wq, &prv->xfer_work,
					prv->spi_backoff);
	}

	/* backoff retries in error cases. Do after requeue so first requeue is
	 * instant. Start at 50ms and double every time until over 1000ms
	 */
	if (res & IZSPI_ATMEL_RX_ERROR || res & IZSPI_SPI_ERROR) {
		if (prv->spi_backoff == 0) {
			prv->spi_backoff = (SPI_BASE_BACKOFF * HZ);
			/* if we have a really werid tick rate, make sure that
			 * backoff gets set to atlease 1.
			 */
			if (prv->spi_backoff == 0)
				prv->spi_backoff = 1;
		} else if (prv->spi_backoff < HZ){
			prv->spi_backoff *= 2;
		}
	}
}

static irqreturn_t gamut_ui_irq(int irq, void *data)
{
	struct gamut_ui *prv = data;

	/* If backoff  delay is active and work is already queued and waiting,
	 * this will not change anything */
	queue_delayed_work(prv->xfer_wq, &prv->xfer_work, 0);

	return IRQ_HANDLED;
}

static const struct of_device_id gamut_ui_of_match[] = {
	{ .compatible = "izotope,gamut-ui", },
	{ }
};
MODULE_DEVICE_TABLE(of, gamut_ui_of_match);

/*
 * parse the device dree entry into the private data
 * dev - pointer to device to parse for
 * prv - pointer to private data struct to populate
 */
static int gamut_ui_parse_dt(struct device *dev, struct gamut_ui *prv)
{
	const struct of_device_id *of_id =
				of_match_device(gamut_ui_of_match, dev);
	struct device_node *np = dev->of_node;

	if (!of_id || !np) {
		dev_err(dev, "Couldn't find devicetree entry\n");
		return -EINVAL;
	}

	prv->gpio_int = of_get_named_gpio(np, "gpio-int", 0);
	if (prv->gpio_int < 0) {
		dev_err(dev, "Couldn't parse 'gpio-int' property from devicetree\n");
		return prv->gpio_int;
	}

	return 0;
}

static int gamutfpmsg_open(struct inode *inode, struct file *filp)
{
	struct gamut_ui *prv;
	prv = container_of(inode->i_cdev, struct gamut_ui, cdev);
	filp->private_data = prv;

	return 0;
}

static ssize_t gamutfpmsg_read(struct file *filp, char __user *buf, size_t count,loff_t *f_pos)
{
	struct gamut_ui *prv = filp->private_data;
	u8 tmp_buf[FRM_SZ];
	int err;
	int pkt_sz;
	int gpio_int_state;

	if (count < ATMEL_PAYLOAD_SZ) {
		return -EINVAL;
	}

	do {
		pkt_sz = izspi_read_packet(&prv->izspi, tmp_buf);
		if (!pkt_sz && (filp->f_flags & O_NONBLOCK)) {
			return -EAGAIN;
		}
		if (pkt_sz > 0) {
			break;
		}
		// TODO: check for error condition from read_packet, and bail?
		if (wait_event_interruptible(prv->rxq,
				izspi_can_read_packet(&prv->izspi))) {
			/* we got a signal: tell the fs layer to handle it */
			return -ERESTARTSYS;
		}
	} while (pkt_sz == 0);

	wake_up_interruptible(&prv->rxq);

	/* if the Atmel still wants a transaction and we freed up enough space,
	   queue up another transaction */
	gpio_int_state = gpio_get_value(prv->gpio_int);
	if ((gpio_int_state == 0) && izspi_can_rx_frame(&prv->izspi)) {
		queue_delayed_work(prv->xfer_wq, &prv->xfer_work, 0);
	}

	err = copy_to_user(buf, tmp_buf, pkt_sz);
	if (err) {
		return -EFAULT;
	}

	return pkt_sz;
}

static ssize_t gamutfpmsg_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct gamut_ui *prv = filp->private_data;
	u8 tmp_buf[FRM_SZ];
	int err;

	if (count > IMX_PAYLOAD_SZ + 1) {
		return -EFBIG;
	}

	err = copy_from_user(tmp_buf, buf, count);
	if (err) {
		return -EFAULT;
	}

	do {
		err = izspi_write_packet(&prv->izspi, tmp_buf, count);
		if (err < 0 && (filp->f_flags & O_NONBLOCK)) {
			return err;
		}
		if (err == count) {
			break;
		}
		// TODO: check for other error conditions?
		if (wait_event_interruptible(prv->txq,
				izspi_can_write_packet(&prv->izspi))) {
			/* we got a signal: tell the fs layer to handle it */
			return -ERESTARTSYS;
		}
	} while (err <= 0);

	wake_up_interruptible(&prv->txq);

	if (prv->atmel_can_rx) {
		queue_delayed_work(prv->xfer_wq, &prv->xfer_work, 0);
	}

	return count;
}

static unsigned int gamutfpmsg_poll(struct file *filp, poll_table *wait)
{
	struct gamut_ui *prv = filp->private_data;
	int mask = 0;

	poll_wait(filp, &prv->rxq, wait);
	poll_wait(filp, &prv->txq, wait);
	/* are we readable (any packet data) */
	if (cbuf_cnt(&prv->izspi.rxbuf)) {
		mask |= POLLIN | POLLRDNORM;
	}
	/* are we writeable (full packet)*/
	if (cbuf_space(&prv->izspi.txbuf) >= (IMX_PAYLOAD_SZ + 1)) {
		mask |= POLLOUT |POLLWRNORM;
	}

	return mask;
}

long gamutfpmsg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gamut_ui *prv = filp->private_data;
	int ret = 0;

	switch (cmd) {
	case GAMUTLED_IOC_NOP:
		dev_info(prv->dev, "Got an IOCTL NOP\n");
		break;
	}

	return ret;
}

static struct file_operations gamutfpmsg_fops = {
	.owner = THIS_MODULE,
	.open = gamutfpmsg_open,
	.read = gamutfpmsg_read,
	.write = gamutfpmsg_write,
	.poll = gamutfpmsg_poll,
	.unlocked_ioctl = gamutfpmsg_ioctl,
};

static int setup_chrdev(struct gamut_ui *prv)
{
	int err;
	dev_t devnum = 0;
	struct device *device;

	err = alloc_chrdev_region(&devnum, 0, 1, DRV_NAME);
	if (err) {
		dev_err(prv->dev, "Failed to obtain major/minors\n");
		goto exit;
	}

	prv->major = MAJOR(devnum);
	prv->minor = MINOR(devnum);
	dev_info(prv->dev, "Registered Maj/Min: %d/%d\n", prv->major, prv->minor);

	cdev_init(&prv->cdev, &gamutfpmsg_fops);
	prv->cdev.owner = THIS_MODULE;
	err = cdev_add(&prv->cdev, MKDEV(prv->major, prv->minor), 1);
	if (err) {
		dev_err(prv->dev, "Error %d while trying to add %s%d\n",
				err, LED_DEVNAME, prv->minor);
		goto exit_unreg_cdev_region;
	}

	device = device_create(g_gamutfpmsg_class,
				NULL,
				MKDEV(prv->major, prv->minor),
				NULL,
				"%s%d", LED_DEVNAME, 0);
	if (IS_ERR(device)) {
		dev_err(prv->dev, "Failed to create %s%d device.\n",
				LED_DEVNAME, prv->minor);
		err = PTR_ERR(device);
		goto exit_destroy_dev;
	}

	dev_info(prv->dev, "Created chrdev %s%d\n", LED_DEVNAME, prv->minor);

	// setup the wait_queues here as well
	init_waitqueue_head(&prv->rxq);
	init_waitqueue_head(&prv->txq);

	return 0;

exit_destroy_dev:
	device_destroy(g_gamutfpmsg_class, MKDEV(prv->major, prv->minor));
	cdev_del(&prv->cdev);
exit_unreg_cdev_region:
	unregister_chrdev_region(MKDEV(prv->major, prv->minor), 1);
exit:
	return err;
}

static void cleanup_chrdev(struct gamut_ui *prv)
{
	device_destroy(g_gamutfpmsg_class, MKDEV(prv->major, prv->minor));
	cdev_del(&prv->cdev);
	unregister_chrdev_region(MKDEV(prv->major, prv->minor), 1);
}

static int gamut_ui_spi_probe(struct spi_device *spi)
{
	struct input_dev *input;
	struct gamut_ui *prv;
	int err;

	//TODO: querry the Atmel to make sure it exists before setting up everything else

	dev_info(&spi->dev, "Probing the gamut UI\n");

	prv = devm_kzalloc(&spi->dev, sizeof(*prv), GFP_KERNEL);
	input = input_allocate_device();
	if (!prv || !input) {
		err = -ENOMEM;
		goto exit_free_mem;
	}

	prv->input = input;
	prv->izspi.spi = spi;
	prv->dev = prv->izspi.dev = &spi->dev;

	err = gamut_ui_parse_dt(&spi->dev, prv);
	if (err < 0) {
		dev_err(&spi->dev, "Couldn't parse devicetree");
		goto exit_free_mem;
	}

	/* setup the GPIO and interrupt for that line */
	err = gpio_request_one(prv->gpio_int, GPIOF_IN, dev_name(prv->dev));
	if (err) {
		dev_err(prv->dev, "Unable to request GPIO %d\n", prv->gpio_int);
		goto exit_free_mem;
	}

	prv->irq = gpio_to_irq(prv->gpio_int);
	err = request_irq(prv->irq, gamut_ui_irq, IRQF_TRIGGER_FALLING,
				DRV_NAME, prv);
	if (err) {
		dev_err(prv->dev, "unable to request IRQ %d\n", prv->irq);
		goto exit_free_gpio_int;
	}

	//TODO: revisit the flags we use
	prv->xfer_wq = alloc_workqueue("gamut_ui_xfer", 0, 0);
	if(!prv->xfer_wq) {
		err = -ENOMEM;
		dev_err(prv->dev, "Coulnd't create workqueue\n");
		goto exit_free_gpio_irq;
	}
	INIT_DELAYED_WORK(&prv->xfer_work, xfer_work);
	prv->shutting_down = 0;

	/* setup the char device */
	err = setup_chrdev(prv);
	if (err) {
		goto exit_free_workq;
	}

	/* init everything for the SPI comms */
	err = cbuf_devm_init(&prv->izspi.txbuf, prv->dev, BUF_SZ);
	if (err) {
		err = -ENOMEM;
		goto exit_free_workq;
	}
	err = cbuf_devm_init(&prv->izspi.rxbuf, prv->dev, BUF_SZ);
	if (err) {
		err = -ENOMEM;
		goto exit_free_workq;
	}

	prv->atmel_can_rx = 1;

	/* setup the input device */
	/* this is the last thing we should do that can fail and need cleanup */
	input->name = "gamut_buttons";
	input->id.bustype = BUS_SPI;
	input->dev.parent = prv->dev;

	input->evbit[0] = BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(KEY_F1)] |= BIT_MASK(KEY_F1);
	input->keybit[BIT_WORD(KEY_F2)] |= BIT_MASK(KEY_F2);
	input->keybit[BIT_WORD(KEY_F3)] |= BIT_MASK(KEY_F3);
	input->keybit[BIT_WORD(KEY_F4)] |= BIT_MASK(KEY_F4);
	input->keybit[BIT_WORD(KEY_F5)] |= BIT_MASK(KEY_F5);
	input->keybit[BIT_WORD(KEY_F6)] |= BIT_MASK(KEY_F6);
	input->keybit[BIT_WORD(KEY_F7)] |= BIT_MASK(KEY_F7);

	err = input_register_device(input);
	if (err) {
		dev_err(prv->dev, "failed to register input device\n");
		goto exit_free_cdev;
	}

	/* setup debugfs enteries, if fails, just plow on */
	prv->dbg_dir = debugfs_create_dir("gamut", NULL);
	if(IS_ERR(prv->dbg_dir)) {
		dev_err(prv->dev, "Couldn't create debugfs dir: %ld\n",
			PTR_ERR(prv->dbg_dir));
		prv->dbg_dir = NULL;
		goto skip_debugfs_setup;
	}

	/* do any other debugfs setup here */
	debugfs_create_u32("rxbuf_head", 0444, prv->dbg_dir, &prv->izspi.rxbuf.head);
	debugfs_create_u32("rxbuf_tail", 0444, prv->dbg_dir, &prv->izspi.rxbuf.tail);
	debugfs_create_u32("txbuf_head", 0444, prv->dbg_dir, &prv->izspi.txbuf.head);
	debugfs_create_u32("txbuf_tail", 0444, prv->dbg_dir, &prv->izspi.txbuf.tail);
	prv->txfrm_cnt = prv->rxfrm_cnt = 0;
	debugfs_create_u32("txfrm_cnt", 0444, prv->dbg_dir, &prv->txfrm_cnt);
	debugfs_create_u32("rxfrm_cnt", 0444, prv->dbg_dir, &prv->rxfrm_cnt);
	prv->izspi.txpkt_cnt = prv->izspi.rxpkt_cnt = 0;
	debugfs_create_u32("txpkt_cnt", 0444, prv->dbg_dir, &prv->izspi.txpkt_cnt);
	debugfs_create_u32("rxpkt_cnt", 0444, prv->dbg_dir, &prv->izspi.rxpkt_cnt);
	debugfs_create_bool("atmel_can_rx", 0444, prv->dbg_dir, &prv->atmel_can_rx);

skip_debugfs_setup:
	spi_set_drvdata(spi, prv);

	/* finally, we enque an xfer to make sure the Atmel and us are in sync*/
	queue_delayed_work(prv->xfer_wq, &prv->xfer_work, 0);

	return 0;

exit_free_cdev:
	cleanup_chrdev(prv);
exit_free_workq:
	destroy_workqueue(prv->xfer_wq);
exit_free_gpio_irq:
	free_irq(prv->irq, prv);
exit_free_gpio_int:
	gpio_free(prv->gpio_int);
exit_free_mem:
	input_free_device(input);

	return err;
}

static int gamut_ui_spi_remove( struct spi_device *spi)
{
	struct gamut_ui *prv = spi_get_drvdata(spi);

	dev_info(prv->dev, "Removing the gamut UI\n");

	debugfs_remove_recursive(prv->dbg_dir);

	/* kill chardev and gpio first so no new SPI transactions can start */
	cleanup_chrdev(prv);
	free_irq(prv->irq, prv);
	gpio_free(prv->gpio_int);

	/* now flush the workqueue before destroying */
	prv->shutting_down = 1;
	flush_workqueue(prv->xfer_wq);
	destroy_workqueue(prv->xfer_wq);


	input_unregister_device(prv->input);

	return 0;
}

static const struct spi_device_id gamut_ui_id_table[] = {
	{ "gamut-ui", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, gamut_ui_id_table);

static struct spi_driver gamut_ui_spi_driver = {
	.driver  = {
		.name   = "gamut-ui",
		.owner  = THIS_MODULE,
		.of_match_table = gamut_ui_of_match,
	},
	.id_table = gamut_ui_id_table,
	.probe   = gamut_ui_spi_probe,
	.remove  = gamut_ui_spi_remove,
};

static int __init gamut_ui_spi_driver_init(void)
{
	g_gamutfpmsg_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(g_gamutfpmsg_class)) {
		pr_err(DRV_NAME ": Failed to register class\n");
		return PTR_ERR(g_gamutfpmsg_class);
	}

	return spi_register_driver(&(gamut_ui_spi_driver));
}
module_init(gamut_ui_spi_driver_init);

static void __exit gamut_ui_spi_driver_exit(void)
{
	spi_unregister_driver(&(gamut_ui_spi_driver));

	class_destroy(g_gamutfpmsg_class);
}
module_exit(gamut_ui_spi_driver_exit);


MODULE_AUTHOR("Matthew Campbell <mcampbell@izotope.com>");
MODULE_DESCRIPTION("Driver for Atmel based UI interface for iZotope Gamut");
MODULE_LICENSE("GPL v2");

/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/input/et310.h>

#ifdef FP_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...)	pr_err(fmt, ## args)
#else
/* Don't do anything in release builds */
#define DEBUG_PRINT(fmt, args...)
#endif

/*lws add beg*/
static int egis_reset_gpio;
static int egis_int_gpio;
unsigned int irq_gpio;
/*lws add end*/
/* add for fingerprint module info. */
#include <linux/productinfo.h>
#define PRODUCTINFO_EGISTECH "dolfa ET310"

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/* ------------------------------ Interrupt -----------------------------*/

/*
 * Interrupt description
 */

#define FP_INT_DETECTION_PERIOD  10
#define FP_DETECTION_THRESHOLD	10

/*
 * FPS interrupt table
 */

struct interrupt_desc fps_ints[] = {
	{0, 0, "BUT0" , 0} /* TINY4412CON15 XEINT12 pin */
};
int fps_ints_size = ARRAY_SIZE(fps_ints);

static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);

/*
 *	FUNCTION NAME.
 *		interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

void interrupt_timer_routine(
	unsigned long _data
)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;
	printk("FPS interrupt count = %d" , bdata->int_count);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		printk("FPS triggered !!!!!!!\n");
	} else
		printk("FPS not triggered !!!!!!!\n");
	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*
 *	FUNCTION NAME.
 *		fingerprint_interrupt
 *
 *	FUNCTIONAL DESCRIPTION.
 *		finger print interrupt callback routine
 *
 *	ENTRY PARAMETERS.
 *		irq
 *		dev_id
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

static irqreturn_t fingerprint_interrupt(
	int irq,
	void *dev_id
)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)dev_id;
	if (!bdata->int_count)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->detect_period));
	bdata->int_count++;
	return IRQ_HANDLED;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Init(int int_num, int detect_period, int detect_threshold)
{
	int i, irq, err = 0;

	pr_info("FP %s int_num = %d detect_period = %d detect_threshold = %d\n",
				__func__,
				int_num,
				detect_period,
				detect_threshold);

	for (i = 0; i < ARRAY_SIZE(fps_ints); i++) {
		fps_ints[i].int_count = 0;
		fps_ints[i].finger_on = 0;
		if (i == int_num) {
			fps_ints[i].detect_period = detect_period;
			fps_ints[i].detect_threshold = detect_threshold;
		} else {
			fps_ints[i].detect_period = FP_INT_DETECTION_PERIOD;
			fps_ints[i].detect_threshold = FP_DETECTION_THRESHOLD;
		}
		if (!fps_ints[i].gpio)
			continue;

		/*
		 * set the IRQ function and GPIO.
		 * then setting the interrupt trigger type
		 */
		irq = gpio_to_irq(egis_int_gpio);
		err = request_irq(
				irq, fingerprint_interrupt,
				IRQ_TYPE_EDGE_RISING,
				fps_ints[i].name, (void *)&fps_ints[i]);
		if (err){
			printk("%s:request_irq failed!!\n",__func__);
			break;
		}else
			printk("%s:request_irq success!!\n",__func__);


	}

	if (err) {
		i--;
		for (; i >= 0; i--) {
			if (!fps_ints[i].gpio)
				continue;
			irq = gpio_to_irq(egis_int_gpio);
			disable_irq(irq);
			free_irq(irq, (void *)&fps_ints[i]);
			del_timer_sync(&fps_ints[i].timer);
		}
		return -EBUSY;
	}
	return 0;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Free(void)
{
	int irq, i;

	for (i = 0; i < ARRAY_SIZE(fps_ints); i++) {
		if (!fps_ints[i].gpio)
			continue;
		irq = gpio_to_irq(egis_int_gpio);
		free_irq(irq, (void *)&fps_ints[i]);

		del_timer_sync(&fps_ints[i].timer);
	}

	return 0;
}

/*
 *	FUNCTION NAME.
 *		fps_interrupt_re d
 *
 *	FUNCTIONAL DESCRIPTION.
 *		FPS interrupt read status
 *
 *	ENTRY PARAMETERS.
 *		wait poll table structure
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

unsigned int fps_interrupt_poll(
	struct file *file,
	struct poll_table_struct *wait
)
{
	unsigned int mask = 0;
	int i = 0;

	pr_info("%s\n", __func__);

	for (i = 0; i < fps_ints_size; i++)
		fps_ints[i].int_count = 0;
	poll_wait(file, &interrupt_waitq, wait);
	for (i = 0; i < fps_ints_size; i++) {
		if (fps_ints[i].finger_on) {
			mask |= POLLIN | POLLRDNORM;
			fps_ints[i].finger_on = 0;
		}
	}
	return mask;
}

void fps_interrupt_abort(void)
{
	int i = 0;
	for (i = 0; i < fps_ints_size; i++)
		fps_ints[i].finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*-------------------------------------------------------------------------*/

static void fp_reset(void)
{
	pr_info("%s\n", __func__);
	gpio_set_value(egis_reset_gpio, 0);
	msleep(30);
	gpio_set_value(egis_reset_gpio, 1);
	msleep(20);
}

static void fp_reset_set(int enable)
{
	pr_info("%s\n", __func__);
	pr_info("%s enable %d\n", __func__, enable);
	if (enable == 0) {
		gpio_set_value(egis_reset_gpio, 0);
		msleep(30);
	} else {
		gpio_set_value(egis_reset_gpio, 1);
		msleep(20);
	}
}

static ssize_t fp_read(struct file *filp,
						char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t fp_write(struct file *filp,
						const char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static long fp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct fp_data *fp;
	//struct device *dev;
	u32 tmp;
	struct egis_ioc_transfer *ioc = NULL;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != EGIS_IOC_MAGIC) {
		pr_err("%s _IOC_TYPE(cmd) != EGIS_IOC_MAGIC", __func__);
		return -ENOTTY;
	}

	/*
	 * Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err) {
		pr_err("%s err", __func__);
		return -EFAULT;
	}

	/*
	 * guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	fp = filp->private_data;
	spin_lock_irq(&fp->spi_lock);
	//spi = spi_dev_get(fp->spi);
	spin_unlock_irq(&fp->spi_lock);

	if (fp->device == NULL) {
		pr_err("%s device == NULL", __func__);
		return -ESHUTDOWN;
	}

	mutex_lock(&fp->buf_lock);

	/* segmented and/or full-duplex I/O request */
	if (_IOC_NR(cmd) != _IOC_NR(EGIS_IOC_MESSAGE(0))
					|| _IOC_DIR(cmd) != _IOC_WRITE) {
		retval = -ENOTTY;
		goto out;
	}

	tmp = _IOC_SIZE(cmd);
	if ((tmp == 0) || (tmp % sizeof(struct egis_ioc_transfer)) != 0) {
		retval = -EINVAL;
		goto out;
	}

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (ioc == NULL) {
		retval = -ENOMEM;
		goto out;
	}
	if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
		retval = -EFAULT;
		goto out;
	}


	if (ioc->opcode == FP_SENSOR_RESET)
		fp_reset();

	if (ioc->opcode == FP_RESET_SET) {
		pr_info("%s FP_SENSOR_RESET\n", __func__);
		pr_info("%s status = %d\n", __func__, ioc->len);
		fp_reset_set(ioc->len);
	}
	if (ioc->opcode == FP_POWER_ONOFF)
		pr_info("power control status = %d\n", ioc->len);

	/*
	 * Trigger inital routine
	 */
	if (ioc->opcode == INT_TRIGGER_INIT) {
		pr_info(">>> fp Trigger function init\n");
		retval = Interrupt_Init(
				(int)ioc->pad[0],
				(int)ioc->pad[1],
				(int)ioc->pad[2]);
		pr_info("trigger init = %d\n", retval);
	}

	/*
	 * trigger
	 */
	if (ioc->opcode == INT_TRIGGER_CLOSE) {
		pr_info("<<< fp Trigger function close\n");
		retval = Interrupt_Free();
		pr_info("trigger close = %d\n", retval);
	}

	/*
	 * read interrupt status
	 */
	if (ioc->opcode == INT_TRIGGER_ABORT)
		fps_interrupt_abort();

out:
	if (ioc != NULL)
		kfree(ioc);

	mutex_unlock(&fp->buf_lock);
	//spi_dev_put(spi);
	if (retval < 0)
		pr_err("%s retval = %d", __func__, retval);
	return retval;
}

#ifdef CONFIG_COMPAT
static long fp_compat_ioctl(struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return fp_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define fp_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int fp_open(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;
	int			status = -ENXIO;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);

	list_for_each_entry(fp, &device_list, device_entry)
	{
		if (fp->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (fp->buffer == NULL) {
			fp->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (fp->buffer == NULL) {
				//dev_dbg(&fp ->device, "open/ENOMEM\n");
				printk("open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			fp->users++;
			filp->private_data = fp;
			nonseekable_open(inode, filp);
		}
	} else
		printk("fp: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int fp_release(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);
	fp = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	fp->users--;
	if (fp->users == 0) {
		int	dofree;

		kfree(fp->buffer);
		fp->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&fp->spi_lock);
		dofree = (fp->device == NULL);
		spin_unlock_irq(&fp->spi_lock);

		if (dofree)
			kfree(fp);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations fp_fops = {
	.owner = THIS_MODULE,
	.write = fp_write,
	.read = fp_read,
	.unlocked_ioctl = fp_ioctl,
	.compat_ioctl = fp_compat_ioctl,
	.open = fp_open,
	.release = fp_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll
};

/*-------------------------------------------------------------------------*/


static int et310_vreg_setup(struct fp_data*fp, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = fp->device;

	for (i = 0; i < ARRAY_SIZE(vreg_conf); i++) {
		const char *n = vreg_conf[i].name;
		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
found:
	vreg = fp->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (!vreg) {
				dev_err(dev, "Unable to get  %s\n", name);
				return -ENODEV;
			}
		}
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fp->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			fp->vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

static int egis_parse_dt(struct device *dev)
{
    struct device_node *np = dev->of_node;
	unsigned int egis_reset_gpio_flags;
	unsigned int egis_int_gpio_flags;
	
	DEBUG_PRINT("%s\n", __func__);
	
    egis_reset_gpio = of_get_named_gpio_flags(np, "egis,reset-gpio",    0, &egis_reset_gpio_flags);
	if (egis_reset_gpio < 0)
		DEBUG_PRINT("egis_reset_gpio request error,%s\n", __func__);
	else
	{
  		if(gpio_request(egis_reset_gpio, "fpreset"))
			DEBUG_PRINT("egis_reset_gpio request error,%s\n", __func__);
		else			
			gpio_direction_output(egis_reset_gpio, 1);
	}
	
	egis_int_gpio = of_get_named_gpio_flags(np, "egis,int-gpio",    0, &egis_int_gpio_flags);
	if (egis_int_gpio < 0)
		DEBUG_PRINT("egis_int_gpio request error,%s\n", __func__);
    else
	{
  		if(gpio_request(egis_int_gpio, "fpint"))
			DEBUG_PRINT("egis_int_gpio request error,%s\n", __func__);
		else	
			DEBUG_PRINT("egis_int_gpio request ok,%s\n", __func__);
	}
	
    return 0;
}


static struct class *fp_class;

/*-------------------------------------------------------------------------*/

static int  fp_probe(struct platform_device *device)
{
	struct device *dev = &device->dev;
	struct fp_data *fp;
	int status;
	unsigned long minor;
	int i;

	DEBUG_PRINT("%s\n", __func__);


	/* Allocate driver data */
	fp = kzalloc(sizeof(*fp), GFP_KERNEL);
	if (fp == NULL)
		return -ENOMEM;

	/* Initialize the driver data */
	fp->device = dev;
	spin_lock_init(&fp->spi_lock);
	mutex_init(&fp->buf_lock);

	INIT_LIST_HEAD(&fp->device_entry);

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		fp->devt = MKDEV(FP_MAJOR, minor);
		dev = device_create(fp_class, &device->dev, fp->devt,
					fp, "esfp0");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else{
		dev_dbg(&device->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&fp->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

/*lws add beg*/
	egis_parse_dt(&device->dev);	
/*lws add end*/
	fps_ints[0].gpio = egis_int_gpio;

	status = et310_vreg_setup(fp, "vdd_ana", true);
	printk("%s:vdd_ana status= %d\n",__func__,status);
	status = et310_vreg_setup(fp, "vdd_dig", true);	
	printk("%s:vdd_dig status= %d\n",__func__,status);
	msleep(2);
	fp_reset();

/*lws add for test beg
	status = irq_gpio = gpio_to_irq(egis_int_gpio);
	if (status < 0)
		DEBUG_PRINT("gpio_to_irq failed==========%s,%d\n", __func__,__LINE__);
	status = request_irq(irq_gpio, egis_irq_handler, IRQF_TRIGGER_RISING , "egis-irq", NULL);
lws add for test end*/

	for (i = 0; i < fps_ints_size; i++) {
		setup_timer(&fps_ints[i].timer, interrupt_timer_routine,
					(unsigned long)&fps_ints[i]);
	}
	printk("%s:OK!!!\n",__func__);
	return status;
}

static int  fp_remove(struct platform_device *device)
{
	struct fp_data *fp = dev_get_drvdata(&device->dev);
	DEBUG_PRINT("%s\n", __func__);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&fp->device_entry);
	device_destroy(fp_class, fp->devt);
	clear_bit(MINOR(fp->devt), minors);
	if (fp->users == 0)
		kfree(fp);
	mutex_unlock(&device_list_lock);

	return 0;
}

//lws add begin
static struct of_device_id et310_of_match[] = {
	{ .compatible = "egis,et310", },
	{}
};
MODULE_DEVICE_TABLE(of, et310_of_match);


static struct platform_driver et310_driver = {
	.driver = {
		.name = "et310",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(et310_of_match),
	},
	.probe = fp_probe,
	.remove = fp_remove,
};

/*-------------------------------------------------------------------------*/

static int __init fp_init(void)
{
	int status;
	DEBUG_PRINT("%s\n", __func__);

	/*
	 * Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(FP_MAJOR, et310_driver.driver.name, &fp_fops);
	if (status < 0)
		return status;
	fp_class = class_create(THIS_MODULE, "fp");
	if (IS_ERR(fp_class)) {
		unregister_chrdev(FP_MAJOR, et310_driver.driver.name);
		return PTR_ERR(fp_class);
	}

	status = platform_driver_register(&et310_driver);
	if (status < 0) {
		class_destroy(fp_class);
		unregister_chrdev(FP_MAJOR, et310_driver.driver.name);
	}
	return status;
}

static void __exit fp_exit(void)
{
	DEBUG_PRINT("%s\n", __func__);
	gpio_free(egis_reset_gpio);
	platform_driver_unregister(&et310_driver);
	class_destroy(fp_class);
	unregister_chrdev(FP_MAJOR, et310_driver.driver.name);
}

//lws add end

/*-------------------------------------------------------------------------*/
module_init(fp_init);
module_exit(fp_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("ET310 Fingerprint sensor device driver.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:egis");

/*
 * Fairchild FAN54063 Battery Charger Driver
 *
 * Copyright (C) 2016, iZotope inc.
 *
 * Authors: Matthew Campbell <mcampbell@izotope.com>
 *          Sam Baxter <sbaxter@izotope.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/*
 * Register addresses
 */
#define REG_CONTROL0			(0x00)
#define REG_CONTROL0_EN_STAT_MASK	(0x40)
#define REG_CONTROL0_EN_STAT_SHIFT	(6)
#define REG_CONTROL0_STAT_MASK		(0x30)
#define REG_CONTROL0_TMR_RST_MASK	(0x80)
#define REG_CONTROL0_TMR_RST_SHIFT	(7)
#define REG_CONTROL1			(0x01)
#define REG_CONTROL1_IBUSLIM_MASK	(0xC0)
#define REG_CONTROL1_IBUSLIM_SHIFT	(6)
#define REG_CONTROL1_VLOWV_MASK		(0x30)
#define REG_CONTROL1_VLOWV_SHIFT	(4)
#define REG_CONTROL1_TE_MASK		(0x08)
#define REG_CONTROL1_TE_SHIFT		(3)
#define REG_CONTROL1_CE_MASK		(0x04)
#define REG_CONTROL1_CE_SHIFT		(2)
#define REG_OREG			(0x02)
#define REG_OREG_OREG_MASK		(0xFC)
#define REG_OREG_OREG_SHIFT		(2)
#define REG_OREG_DBAT_B_MASK		(0x02)
#define REG_OREG_EOC_MASK		(0x01)
#define REG_OREG_EOC_SHIFT		(0)
#define REG_IC_INFO			(0x03)
#define REG_IBAT			(0x04)
#define REG_IBAT_IOCHARGE_MASK		(0xF8)
#define REG_IBAT_IOCHARGE_SHIFT		(3)
#define REG_IBAT_ITERM_MASK		(0x87)
#define REG_IBAT_ITERM_SHIFT		(0)
#define REG_VBUS_CONTROL		(0x05)
#define REG_VBUS_CONTROL_IO_LEVEL_MASK	(0x20)
#define REG_VBUS_CONTROL_IO_LEVEL_SHIFT	(5)
#define REG_VBUS_CONTROL_VBUS_CON_MASK	(0x10)
#define REG_SAFETY			(0x06)
#define REG_SAFETY_MASK			(0xFF)
#define REG_SAFETY_VALUE		(0x70)
#define REG_SAFETY_ISAFE_MASK		(0xF0)
#define REG_SAFETY_VSAFE_MASK		(0x0F)
#define REG_POST_CHARGING		(0x07)
#define REG_MONITOR0			(0x10)
#define REG_MONITOR1			(0x11)
#define REG_MONITOR1_NOBAT_MASK		(0x08)
#define REG_NTC				(0x12)
#define REG_WD_CONTROL			(0x13)
#define REG_RESTART			(0xFA)
#define REG_LASTREG			REG_RESTART

#define REG_IC_INFO_VENDOR_CODE_MASK	(0xC0)
#define REG_IC_INFO_VENDOR_CODE_SHIFT	(6)
#define REG_IC_INFO_PN_MASK		(0x38)
#define REG_IC_INFO_PN_SHIFT		(3)
#define REG_IC_INFO_REV_MASK		(0x07)
#define REG_IC_INFO_REV_SHIFT		(0)
#define IC_INFO_VENDOR_CODE		(0x2)
#define IC_INFO_PN			(0x2)

/* STAT Bits States */
#define STAT_STANDBY		(0x00)
#define STAT_CHARGING		(0x10)
#define STAT_FULLY_CHARGED	(0x20)
#define STAT_FAULT		(0x30)

/* STAT Interrupt Fault Statuses */
#define IRQSTAT_NORMAL			(0)
#define IRQSTAT_VBUS_OVP		(1)
#define IRQSTAT_SLEEP			(2)
#define IRQSTAT_POOR_INPUT_SOURCE	(3)
#define IRQSTAT_BATTERY_OVP		(4)
#define IRQSTAT_THERMAL_SHUTDOWN	(5)
#define IRQSTAT_TIMER_FAULT		(6)
#define IRQSTAT_NO_BATTERY		(7)

#define DRV_NAME "fan54063-charger"

struct fan54063 {
	struct mutex lock;

	struct i2c_client *client;
	struct regmap *regmap;

	struct power_supply *mains;
	struct power_supply *battery;
	struct tasklet_struct stat_tasklet;

	int gpio_stat;
	int irq_stat;

	int fault_cache;

	bool mains_online;
};

static const unsigned int max_cur_tbl[] = {
	550000,
	650000,
	750000,
	850000,
	950000,
	1050000,
	1050000,
	1250000,
	1350000,
	1450000,
	1550000,
};

static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
{
	if (val >= size)
		return -EINVAL;
	return tbl[val];
}

/*
 * Returns true if input power source is connected. Note that this refers
 * to the wall wart charger only, since we do not support usb power sources
 */
static bool fan54063_is_ps_online(struct fan54063 *fan54063)
{
	bool ret;

	mutex_lock(&fan54063->lock);
	ret = fan54063->mains_online;
	mutex_unlock(&fan54063->lock);

	return ret;
}

static void fan54063_set_ps_online(struct fan54063 *fan54063, bool online)
{
	mutex_lock(&fan54063->lock);
	fan54063->mains_online = online;
	mutex_unlock(&fan54063->lock);
}

static void register_dump(struct fan54063 *fan54063)
{
	struct regmap *regmap = fan54063->regmap;
	struct device *dev = &fan54063->client->dev;
#define print_reg(X) \
{ \
	int val, ret; \
	ret = regmap_read(regmap, X, &val); \
	if (ret) \
		dev_info(dev, "Couldn't read reg " __stringify(X) " 0x%02X\n",X); \
	else \
		dev_info(dev, __stringify(X) " 0x%02X : 0x%02X. Ret=%d\n", X, val, ret); \
}
	dev_info(dev, "Dumping FAN54062 registers:\n");
	print_reg(REG_CONTROL0);
	print_reg(REG_CONTROL1);
	print_reg(REG_OREG);
	print_reg(REG_IC_INFO);
	print_reg(REG_IBAT);
	print_reg(REG_VBUS_CONTROL);
	print_reg(REG_SAFETY);
	print_reg(REG_POST_CHARGING);
	print_reg(REG_MONITOR0);
	print_reg(REG_MONITOR1);
	print_reg(REG_NTC);
	print_reg(REG_WD_CONTROL);
	print_reg(REG_RESTART);
#undef print_reg
}

static int get_const_charge_current_max(struct fan54063 *fan54063)
{
	int ret, intval;
	unsigned int i;

	if (!fan54063_is_ps_online(fan54063))
		return -ENODATA;

	ret = regmap_read(fan54063->regmap, REG_SAFETY, &i);
	if (ret)
		return ret;

	intval = hw_to_current(max_cur_tbl, ARRAY_SIZE(max_cur_tbl),
			       (i & REG_SAFETY_ISAFE_MASK) >> 4);

	return intval;
}

static int get_const_charge_voltage_max(struct fan54063* fan54063)
{
	int ret, intval;
	unsigned int v;

	if (!fan54063_is_ps_online(fan54063))
		return -ENODATA;

	ret = regmap_read(fan54063->regmap, REG_SAFETY, &v);
	if (ret)
		return ret;

	v &= REG_SAFETY_VSAFE_MASK;
	/*
	 * Above 0x0c, VSAFE is limited to 4.45V
	 */
	if (v > 0x0c)
		v = 0x0c;

	intval = 4210000 + v * 20000;

	return intval;
}

static int fan54063_mains_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct fan54063 *fan54063 = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = fan54063->mains_online;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_const_charge_current_max(fan54063);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = get_const_charge_voltage_max(fan54063);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property fan54063_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
};

static int fan54063_get_charging_status(struct fan54063 *fan54063)
{
	int ret, status;
	unsigned int val;

	if (!fan54063_is_ps_online(fan54063))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = regmap_read(fan54063->regmap, REG_CONTROL0, &val);
	val &= REG_CONTROL0_STAT_MASK;
	if (ret)
		return ret;

	if (val == STAT_FAULT) {
		/*
		 * Set to NOT_CHARGING upon charger error
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else if (val == STAT_STANDBY) {
		/*
		 * Set to unknown if power source is online, but we are
		 * neither charging, nor fully charged.
		 */
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	} else if (val == STAT_CHARGING) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_FULL;
	}

	return status;
}

static int fan54063_battery_get_property(struct power_supply *psy,
					 enum power_supply_property prop,
					 union power_supply_propval *val)
{
	struct fan54063 *fan54063 = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = fan54063_get_charging_status(fan54063);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!fan54063_is_ps_online(fan54063))
			return -ENODATA;

		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3400000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4210000;
		break;

	default:
		return -EINVAL;
	};

	return 0;
}

static enum power_supply_property fan54063_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static bool fan54063_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_CONTROL0:
	case REG_OREG:
	case REG_VBUS_CONTROL:
	case REG_SAFETY:
	case REG_MONITOR0:
	case REG_MONITOR1:
	case REG_NTC:
		return true;
	}
	return false;
}

static bool fan54063_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_CONTROL1:
	case REG_IC_INFO:
	case REG_IBAT:
	case REG_POST_CHARGING:
	case REG_WD_CONTROL:
		return true;
	}
	return fan54063_volatile_reg(dev, reg);
}

static bool fan54063_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_CONTROL0:
	case REG_CONTROL1:
	case REG_OREG:
	case REG_IBAT:
	case REG_VBUS_CONTROL:
	case REG_SAFETY:
	case REG_POST_CHARGING:
	case REG_MONITOR1:
	case REG_NTC:
	case REG_WD_CONTROL:
	case REG_RESTART:
		return true;
	}
	return false;
}

static struct regmap_config fan54063_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_LASTREG,
	.volatile_reg = fan54063_volatile_reg,
	.readable_reg = fan54063_readable_reg,
	.writeable_reg = fan54063_writeable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int fan54063_register_init(struct fan54063 *fan54063)
{
	int ret;

	/*
	 * Set the entire SAFETY register at once, twice, to lock the
	 * values in the register. They cannot be set again until
	 * VBAT falls below VSHORT.
	 */
	ret = regmap_update_bits(fan54063->regmap, REG_SAFETY,
				 REG_SAFETY_MASK, REG_SAFETY_VALUE);
	ret = regmap_update_bits(fan54063->regmap, REG_SAFETY,
				 REG_SAFETY_MASK, REG_SAFETY_VALUE);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_CONTROL0,
				 REG_CONTROL0_EN_STAT_MASK,
				 1 << REG_CONTROL0_EN_STAT_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
				 REG_CONTROL1_CE_MASK,
				 0 << REG_CONTROL1_CE_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
				 REG_CONTROL1_TE_MASK,
				 1 << REG_CONTROL1_TE_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_VBUS_CONTROL,
				 REG_VBUS_CONTROL_IO_LEVEL_MASK,
				 0 << REG_VBUS_CONTROL_IO_LEVEL_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
				  REG_CONTROL1_IBUSLIM_MASK,
				  3 << REG_CONTROL1_IBUSLIM_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
				  REG_CONTROL1_VLOWV_MASK,
				  0 << REG_CONTROL1_VLOWV_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_OREG,
				  REG_OREG_OREG_MASK,
				  32 << REG_OREG_OREG_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_OREG,
				 REG_OREG_EOC_MASK,
				 1 << REG_OREG_EOC_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_IBAT,
				  REG_IBAT_IOCHARGE_MASK,
				  5 << REG_IBAT_IOCHARGE_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(fan54063->regmap, REG_IBAT,
				  REG_IBAT_ITERM_MASK,
				  4 << REG_IBAT_ITERM_SHIFT);
	if (ret)
		return ret;

	return ret;
}

static int fan54063_hw_init(struct fan54063 *fan54063)
{
	int ret;
	int vbus_con;
	int no_bat;

	ret = regmap_read(fan54063->regmap, REG_VBUS_CONTROL, &vbus_con);
	vbus_con &= REG_VBUS_CONTROL_VBUS_CON_MASK;
	if (ret) {
		dev_err(&fan54063->client->dev, "reading VBUS_CONTROL failed\n");
		return ret;
	}

	ret = regmap_read(fan54063->regmap, REG_MONITOR1, &no_bat);
	no_bat &= REG_MONITOR1_NOBAT_MASK;
	if (ret) {
		dev_err(&fan54063->client->dev, "reading REG_MONITOR1 failed\n");
		return ret;
	}

	printk("fan54063 VBUS_CON:\t%d\n", vbus_con);
	/*
	 * If we are running off of batter power then set our state active if
	 * the battery can supply enough power to run the system.
	 */
	if (!vbus_con && !no_bat) {
		dev_info(&fan54063->client->dev, "Running off battery\n");
		fan54063->fault_cache = IRQSTAT_SLEEP;
		fan54063_set_ps_online(fan54063, false);

		ret = fan54063_register_init(fan54063);
		if (ret) {
			dev_err(&fan54063->client->dev,
				"Failed to initialize registers\n");
			return ret;
		}

		goto exit_end;
	}

	fan54063_set_ps_online(fan54063, true);

	ret = fan54063_register_init(fan54063);
	if (ret) {
		dev_err(&fan54063->client->dev,
			"Failed to initialize registers\n");
		return ret;
	}

	if (!no_bat) {
		dev_info(&fan54063->client->dev, "Wallwart and battery\n");
		goto exit_end;
	} else {
		fan54063->fault_cache = IRQSTAT_NO_BATTERY;
		dev_info(&fan54063->client->dev, "No battery\n");
		goto exit_disable_charging;
	}

exit_disable_charging:
	ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
				 REG_CONTROL1_CE_MASK,
				 1 << REG_CONTROL1_CE_SHIFT);
	if (ret) {
		dev_err(&fan54063->client->dev,
			"Failed to update CE# bit in REG_CONTROL1\n");
		return ret;
	}

	ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
				 REG_CONTROL1_TE_MASK,
				 0 << REG_CONTROL1_TE_SHIFT);
	if (ret)
		dev_err(&fan54063->client->dev,
			"Failed to update TE bit in REG_CONTROL1\n");
exit_end:
	return ret;
}

static int parse_gpio(struct device *dev, struct device_node *np,
		      const char *name, int *val)
{
	*val = of_get_named_gpio(np, name, 0);
	if (*val < 0) {
		dev_err(dev, "Couldn't parse '%s' property from devicetree\n",
				name);
		return *val;
	}

	return 0;
}

static void fan54063_stat_tasklet(unsigned long data)
{
	struct fan54063 *fan54063 = (struct fan54063 *) data;
	int irqstat_fault;
	int ret;

	ret = regmap_read(fan54063->regmap, REG_CONTROL0, &irqstat_fault);
	irqstat_fault &= IRQSTAT_NO_BATTERY;
	if (ret) {
		dev_err(&fan54063->client->dev, "reading CONTROL0 failed\n");
		return;
	}

	/*
	 * If we transition to running off of battery power, set the
	 * battery state active if it can power the system
	 */
	if (irqstat_fault == IRQSTAT_SLEEP) {
		power_supply_changed(fan54063->battery);
		fan54063_set_ps_online(fan54063, false);
	} else if (!irqstat_fault && (fan54063->fault_cache == IRQSTAT_SLEEP)) {
		/*
		 * If we transition from battery to wall wart,
		 * we write to the timer reset bit to reset it.
		 */
		ret = regmap_update_bits(fan54063->regmap, REG_CONTROL0,
					 REG_CONTROL0_TMR_RST_MASK,
					 1 << REG_CONTROL0_TMR_RST_SHIFT);
		if (ret)
			dev_err(&fan54063->client->dev,
				"resetting CONTROL0 timer failed\n");

		power_supply_changed(fan54063->mains);
		fan54063_set_ps_online(fan54063, true);
	} else if (!irqstat_fault) {
		power_supply_changed(fan54063->mains);
	}

	if (fan54063->fault_cache == IRQSTAT_NO_BATTERY) {
		ret = fan54063_register_init(fan54063);
		if (ret)  {
			dev_err(&fan54063->client->dev,
				"Failed to reinitialize HW registers\n");
		}
	}

	if (irqstat_fault == IRQSTAT_NO_BATTERY) {
		dev_info(&fan54063->client->dev, "Disabling charging\n");
		ret = fan54063_register_init(fan54063);
		if (ret)  {
			dev_err(&fan54063->client->dev,
				"Failed to reinitialize HW registers\n");
		}

		ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
					 REG_CONTROL1_CE_MASK,
					 1 << REG_CONTROL1_CE_SHIFT);
		if (ret)
			dev_err(&fan54063->client->dev,
				"Failed to update CE# bit in REG_CONTROL1\n");

		ret = regmap_update_bits(fan54063->regmap, REG_CONTROL1,
					 REG_CONTROL1_TE_MASK,
					 0 << REG_CONTROL1_TE_SHIFT);
		if (ret)
			dev_err(&fan54063->client->dev,
				"Failed to update TE bit in REG_CONTROL1\n");
	}

	fan54063->fault_cache = irqstat_fault;
}

static irqreturn_t fan54063_interrupt(int irq, void *data)
{
	struct fan54063 *fan54063 = data;
	tasklet_schedule(&fan54063->stat_tasklet);

	return IRQ_HANDLED;
}

static const struct of_device_id fan54063_of_match[] = {
	{ .compatible = "fairchild,fan54063", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fan54063_of_match);

/*
 * parse the device dree entry into the private data
 * dev - pointer to device to parse for
 * fan54063 - pointer to private data struct to populate
 */
static int fan54063_parse_dt(struct device *dev, struct fan54063* fan54063)
{
	const struct of_device_id *of_id =
				of_match_device(fan54063_of_match, dev);
	struct device_node *np = dev->of_node;
	int ret;

	if (!of_id || !np) {
		dev_err(dev, "Couldn't find devicetree entry\n");
		return -EINVAL;
	}

	ret = parse_gpio(dev, np, "gpio-stat", &fan54063->gpio_stat);
	if (ret)
		return ret;

	return 0;
}

static bool fan54063_valid_info(int ic_info)
{
	int vendor_code;
	int pn;

	vendor_code = (ic_info & REG_IC_INFO_VENDOR_CODE_MASK)
				>> REG_IC_INFO_VENDOR_CODE_SHIFT;
	pn = (ic_info & REG_IC_INFO_PN_MASK) >> REG_IC_INFO_PN_SHIFT;

	if ((vendor_code != IC_INFO_VENDOR_CODE) || (pn != IC_INFO_PN))
		return false;
	return true;
}

static const struct power_supply_desc desc_main = {
	.name = "fan54063-mains",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.get_property = fan54063_mains_get_property,
	.properties = fan54063_mains_properties,
	.num_properties = ARRAY_SIZE(fan54063_mains_properties),
};

static const struct power_supply_desc desc_battery = {
	.name = "fan54063-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = fan54063_battery_get_property,
	.properties = fan54063_battery_properties,
	.num_properties = ARRAY_SIZE(fan54063_battery_properties),
};

static int fan54063_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	static char *battery[] = { "fan54063-battery" };
	struct fan54063 *fan54063;
	int ret;
	int val;
	struct power_supply_config main_psy_cfg = {},
				   battery_psy_cfg = {};


	dev_info(&client->dev, "Probing FAN54063-charger module\n");

	fan54063 = devm_kzalloc(&client->dev, sizeof(*fan54063), GFP_KERNEL);
	if (!fan54063)
		return -ENOMEM;
	mutex_init(&fan54063->lock);

	fan54063->client = client;
	fan54063->fault_cache = 0;
	fan54063->regmap = devm_regmap_init_i2c(client,
						&fan54063_regmap_config);
	if (IS_ERR(fan54063->regmap)) {
		dev_err(&client->dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	ret = regmap_read(fan54063->regmap, REG_IC_INFO, &val);
	if (ret) {
		dev_err(&client->dev, "Couldn't read from device\n");
		goto exit_probe;
	}

	if (fan54063_valid_info(val)) {
		val = (val & REG_IC_INFO_REV_MASK) >> REG_IC_INFO_REV_SHIFT;
		dev_info(&client->dev, "FAN54063 REV 0x%0X found\n", val);
	} else {
		dev_err(&client->dev, "No FAN54063 found\n");
		return -ENODEV;
	}

	ret = fan54063_hw_init(fan54063);
	if (ret) {
		dev_err(&client->dev,
			"Failed to initialize HW registers\n");
		goto exit_probe;
	}

	main_psy_cfg.drv_data = fan54063;
	main_psy_cfg.supplied_to = battery;
	main_psy_cfg.num_supplicants = ARRAY_SIZE(battery);
	fan54063->mains = power_supply_register(&client->dev,
						&desc_main,
						&main_psy_cfg);
	if (IS_ERR(fan54063->mains)) {
		ret = PTR_ERR(fan54063->mains);
		goto exit_probe;
	}

	battery_psy_cfg.drv_data = fan54063;
	fan54063->battery = power_supply_register(&client->dev,
						&desc_battery,
						&battery_psy_cfg);
	if (IS_ERR(fan54063->battery)) {
		ret = PTR_ERR(fan54063->battery);
		goto exit_unreg_mains;
	}

	tasklet_init(&fan54063->stat_tasklet, fan54063_stat_tasklet,
		     (unsigned long) fan54063);

	ret = fan54063_parse_dt(&client->dev, fan54063);
	if (ret)
		goto exit_unreg_battery;

	/* setup the GPIO and interrupt for STAT line */
	fan54063->irq_stat = gpio_to_irq(fan54063->gpio_stat);
	ret = gpio_request_one(fan54063->gpio_stat, GPIOF_IN,
			       dev_name(&client->dev));
	if (ret) {
		dev_err(&client->dev, "Unable to request GPIO %d\n",
			fan54063->gpio_stat);
		goto exit_irq_init;
	}

	ret = request_irq(fan54063->irq_stat, fan54063_interrupt,
			  IRQF_TRIGGER_RISING, DRV_NAME, fan54063);
	if (ret) {
		dev_err(&client->dev, "Unable to request IRQ %d\n",
			fan54063->irq_stat);
		goto exit_free_gpio_stat;
	}

	i2c_set_clientdata(client, fan54063);

	register_dump(fan54063);

	return 0;

exit_free_gpio_stat:
	gpio_free(fan54063->gpio_stat);
exit_irq_init:
	fan54063->irq_stat = 0;
exit_unreg_battery:
	power_supply_unregister(fan54063->battery);
exit_unreg_mains:
	power_supply_unregister(fan54063->mains);
exit_probe:
	return ret;
}

static int fan54063_i2c_remove(struct i2c_client *client)
{
	struct fan54063 *fan54063 = i2c_get_clientdata(client);

	dev_info(&fan54063->client->dev, "Removing FAN54063 spi\n");

	free_irq(fan54063->irq_stat, fan54063);
	gpio_free(fan54063->gpio_stat);

	tasklet_kill(&fan54063->stat_tasklet);

	power_supply_unregister(fan54063->battery);
	power_supply_unregister(fan54063->mains);

	dev_info(&fan54063->client->dev, "FAN54063 spi removed\n");

	return 0;
}

static const struct i2c_device_id fan54063_id_table[] = {
	{ "fan54063", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fan54063_id_table);

static struct i2c_driver fan54063_driver = {
	.driver = {
		.name	= "fan54063",
		.owner	= THIS_MODULE,
		.of_match_table = fan54063_of_match,
	},
	.id_table = fan54063_id_table,
	.probe	= fan54063_i2c_probe,
	.remove	= fan54063_i2c_remove,
};
module_i2c_driver(fan54063_driver);

MODULE_AUTHOR("Matthew Campbell <mcampbell@izotope.com>");
MODULE_AUTHOR("Sam Baxter <sbaxter@izotope.com>");
MODULE_DESCRIPTION("FAN54063 battery charger driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:fan54063");

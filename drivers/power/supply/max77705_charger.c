// SPDX-License-Identifier: GPL-2.0
/*
 * Based on max77650-charger.c
 *
 * Copyright (C) 2025 Dzmitry Sankouski <dsankouski@gmail.org>
 *
 * Battery charger driver for MAXIM 77705 charger/power-supply.
 */

#include <linux/backlight.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77705-private.h>
#include <linux/power/max77705_charger.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

static const char *max77705_charger_model		= "max77705";
static const char *max77705_charger_manufacturer	= "Maxim Integrated";

/*
 * For tests only: a lower charge-stop / resume temperature for this boot
 * (milli-degC, 0 = stock's 50.0 / 48.0 C). Never above stock's.
 */
static int test_stop_mc;
module_param(test_stop_mc, int, 0644);
MODULE_PARM_DESC(test_stop_mc, "test only: stop charging from this battery temperature (mC)");
static int test_resume_mc;
module_param(test_resume_mc, int, 0644);
MODULE_PARM_DESC(test_resume_mc, "test only: charge again from this battery temperature (mC)");

static const struct regmap_config max77705_chg_regmap_config = {
	.reg_base = MAX77705_CHG_REG_BASE,
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77705_CHG_REG_SAFEOUT_CTRL,
};

static enum power_supply_property max77705_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int max77705_chgin_lim_ma(unsigned int regval)
{
	return regval <= 3 ? MAX77705_CURRENT_CHGIN_MIN / 1000 :
			     (regval + 1) * MAX77705_CURRENT_CHGIN_STEP / 1000;
}

static void max77705_aicl_irq_on(struct max77705_charger_data *chg)
{
	lockdep_assert_held(&chg->aicl_lock);
	if (chg->aicl_irq_off) {
		enable_irq(chg->aicl_irq);
		chg->aicl_irq_off = false;
	}
}

/*
 * AICL. AICL_OK low means the charger draws less than its input limit to hold
 * VBUS at VCHGIN_REG: the source sags at that current. As the stock kernel
 * does, step the limit down until it fits, MAX77705_AICL_STEP every 100 ms
 * and to no less than MAX77705_AICL_FLOOR. Stock then keeps it until the
 * cable changes. Here, MAX77705_AICL_RERUN_MS after it fits, it goes back up
 * a step every MAX77705_AICL_CLIMB_MS while AICL stays quiet, up to the limit
 * asked for: a source that sagged for a while (a hub whose other ports were
 * busy) is used in full again, and one that cannot do more costs a step up
 * and back down every MAX77705_AICL_RERUN_MS.
 *
 * All of it runs in this one work item, not in the interrupt handler: that
 * runs in the one thread all MAX77705 interrupts share.
 */
static void max77705_aicl_work(struct work_struct *work)
{
	struct max77705_charger_data *chg =
		container_of(to_delayed_work(work), struct max77705_charger_data, aicl_work);
	unsigned long now = jiffies, delay = 0;
	unsigned int int_ok, lim;
	bool changed = false;

	mutex_lock(&chg->aicl_lock);
	if (chg->aicl_stopped ||
	    regmap_read(chg->regmap, MAX77705_CHG_REG_INT_OK, &int_ok) ||
	    regmap_field_read(chg->rfield[MAX77705_CHG_CHGIN_LIM], &lim))
		goto out;

	/* no input to fit: look again when it is back (CHGIN_I), or in a while */
	if (!(int_ok & MAX77705_CHGIN_OK)) {
		if (lim < chg->ilim_req)
			delay = msecs_to_jiffies(MAX77705_AICL_RERUN_MS);
		goto out;
	}

	if (!(int_ok & MAX77705_AICL_OK)) {
		chg->aicl_stepping = true;
		if (lim > MAX77705_AICL_FLOOR) {
			lim = max(lim - MAX77705_AICL_STEP, MAX77705_AICL_FLOOR);
			regmap_field_write(chg->rfield[MAX77705_CHG_CHGIN_LIM], lim);
			delay = msecs_to_jiffies(AICL_WORK_DELAY_MS);
			goto out;
		}
		/* at the floor: no more AICL interrupts until the limit goes up */
		if (!chg->aicl_irq_off) {
			disable_irq_nosync(chg->aicl_irq);
			chg->aicl_irq_off = true;
		}
	} else if (!chg->aicl_stepping && lim < chg->ilim_req &&
		   time_after_eq(now, chg->rerun_at)) {
		lim = min(lim + MAX77705_AICL_STEP, chg->ilim_req);
		regmap_field_write(chg->rfield[MAX77705_CHG_CHGIN_LIM], lim);
		max77705_aicl_irq_on(chg);
		chg->aicl_climbing = true;
		chg->rerun_at = now + msecs_to_jiffies(MAX77705_AICL_CLIMB_MS);
		/* if the source sags at this one, step back down at once */
		delay = msecs_to_jiffies(AICL_WORK_DELAY_MS);
		goto out;
	} else if (chg->aicl_climbing && time_after_eq(now, chg->rerun_at)) {
		/* the last step up, to the limit asked for, held */
		chg->aicl_climbing = false;
	}

	if (chg->aicl_stepping) {
		/* it fits, or is as low as it goes: wait before going up again */
		chg->aicl_stepping = false;
		chg->aicl_climbing = false;
		chg->rerun_at = now + msecs_to_jiffies(MAX77705_AICL_RERUN_MS);
	}

	/* tell where it settles, not each step on the way */
	if (!chg->aicl_climbing && lim != chg->ilim_shown) {
		dev_info(chg->dev, "AICL: input limit %d -> %d mA%s\n",
			 max77705_chgin_lim_ma(chg->ilim_shown), max77705_chgin_lim_ma(lim),
			 int_ok & MAX77705_AICL_OK ? "" : ", VBUS still low");
		chg->ilim_shown = lim;
		changed = true;
	}

	if (lim < chg->ilim_req || chg->aicl_climbing)
		delay = time_after(chg->rerun_at, now) ? chg->rerun_at - now : 1;
out:
	if (delay)
		mod_delayed_work(chg->wqueue, &chg->aicl_work, delay);
	mutex_unlock(&chg->aicl_lock);
	if (changed)
		power_supply_changed(chg->psy_chg);
}

/* Before the AICL interrupt goes: the work must not touch it any more. */
static void max77705_aicl_stop(void *data)
{
	struct max77705_charger_data *chg = data;

	mutex_lock(&chg->aicl_lock);
	chg->aicl_stopped = true;
	mutex_unlock(&chg->aicl_lock);
	cancel_delayed_work_sync(&chg->aicl_work);
}

static irqreturn_t max77705_aicl_irq(int irq, void *irq_drv_data)
{
	struct max77705_charger_data *chg = irq_drv_data;

	/* it fires when AICL ends, too: the work reads which it is */
	mod_delayed_work(chg->wqueue, &chg->aicl_work,
			 msecs_to_jiffies(AICL_WORK_DELAY_MS));

	return IRQ_HANDLED;
}

static irqreturn_t max77705_chgin_irq(int irq, void *irq_drv_data)
{
	struct max77705_charger_data *chg = irq_drv_data;

	queue_work(chg->wqueue, &chg->chgin_work);

	return IRQ_HANDLED;
}

static irqreturn_t max77705_wcin_irq(int irq, void *irq_drv_data)
{
	struct max77705_charger_data *chg = irq_drv_data;

	mod_delayed_work(chg->wqueue, &chg->wcin_work, 0);
	mod_delayed_work(chg->wqueue, &chg->temp_work, 0);

	return IRQ_HANDLED;
}

static const struct regmap_irq max77705_charger_irqs[] = {
	REGMAP_IRQ_REG_LINE(MAX77705_BYP_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_INP_LIMIT_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_BATP_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_BAT_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_CHG_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_WCIN_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_CHGIN_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_AICL_I, BITS_PER_BYTE),
};

static const struct regmap_irq_chip max77705_charger_irq_chip = {
	.name			= "max77705-charger",
	.status_base		= MAX77705_CHG_REG_INT,
	.mask_base		= MAX77705_CHG_REG_INT_MASK,
	.num_regs		= 1,
	.irqs			= max77705_charger_irqs,
	.num_irqs		= ARRAY_SIZE(max77705_charger_irqs),
};

static int max77705_charger_enable(struct max77705_charger_data *chg)
{
	int rv;

	rv = regmap_field_write(chg->rfield[MAX77705_CHG_EN], 1);
	if (rv)
		dev_err(chg->dev, "unable to enable the charger: %d\n", rv);

	return rv;
}

static void max77705_charger_disable(void *data)
{
	struct max77705_charger_data *chg = data;
	int rv;

	rv = regmap_field_write(chg->rfield[MAX77705_CHG_EN], MAX77705_CHG_DISABLE);
	if (rv)
		dev_err(chg->dev, "unable to disable the charger: %d\n", rv);
}

/* unbind: no temperature check may enable the charger after this */
static void max77705_charger_stop(void *data)
{
	struct max77705_charger_data *chg = data;

	WRITE_ONCE(chg->unbinding, true);
	cancel_delayed_work_sync(&chg->temp_work);
	max77705_charger_disable(chg);
}

static int max77705_get_online(struct regmap *regmap, int *val)
{
	unsigned int data;
	int ret;

	ret = regmap_read(regmap, MAX77705_CHG_REG_INT_OK, &data);
	if (ret < 0)
		return ret;

	*val = !!(data & MAX77705_CHGIN_OK);

	return 0;
}

static int max77705_set_integer(struct max77705_charger_data *chg, enum max77705_field_idx fidx,
				unsigned int clamp_min, unsigned int clamp_max,
				unsigned int div, int val)
{
	unsigned int regval;

	regval = clamp_val(val, clamp_min, clamp_max) / div;

	return regmap_field_write(chg->rfield[fidx], regval);
}

static int max77705_check_battery(struct max77705_charger_data *chg, int *val)
{
	unsigned int reg_data;
	unsigned int reg_data2;
	struct regmap *regmap = chg->regmap;

	regmap_read(regmap, MAX77705_CHG_REG_INT_OK, &reg_data);

	dev_dbg(chg->dev, "CHG_INT_OK(0x%x)\n", reg_data);

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_00, &reg_data2);

	dev_dbg(chg->dev, "CHG_DETAILS00(0x%x)\n", reg_data2);

	if ((reg_data & MAX77705_BATP_OK) || !(reg_data2 & MAX77705_BATP_DTLS))
		*val = true;
	else
		*val = false;

	return 0;
}

static int max77705_get_charge_type(struct max77705_charger_data *chg, int *val)
{
	struct regmap *regmap = chg->regmap;
	unsigned int reg_data, chg_en;

	regmap_field_read(chg->rfield[MAX77705_CHG_EN], &chg_en);
	if (!chg_en) {
		*val = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	}

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_01, &reg_data);
	reg_data &= MAX77705_CHG_DTLS;

	switch (reg_data) {
	case 0x0:
	case MAX77705_CHARGER_CONSTANT_CURRENT:
	case MAX77705_CHARGER_CONSTANT_VOLTAGE:
		*val = POWER_SUPPLY_CHARGE_TYPE_FAST;
		return 0;
	default:
		*val = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	}

	return 0;
}

static int max77705_get_status(struct max77705_charger_data *chg, int *val)
{
	struct regmap *regmap = chg->regmap;
	unsigned int reg_data, chg_en;

	regmap_field_read(chg->rfield[MAX77705_CHG_EN], &chg_en);
	if (!chg_en) {
		/* stopped (a hot battery): "not charging" only with an input */
		regmap_read(regmap, MAX77705_CHG_REG_INT_OK, &reg_data);
		*val = reg_data & (MAX77705_CHGIN_OK | MAX77705_WCIN_OK) ?
		       POWER_SUPPLY_STATUS_NOT_CHARGING : POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_01, &reg_data);
	reg_data &= MAX77705_CHG_DTLS;

	switch (reg_data) {
	case 0x0:
	case MAX77705_CHARGER_CONSTANT_CURRENT:
	case MAX77705_CHARGER_CONSTANT_VOLTAGE:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	case MAX77705_CHARGER_END_OF_CHARGE:
	case MAX77705_CHARGER_DONE:
		*val = POWER_SUPPLY_STATUS_FULL;
		return 0;
	/* those values hard coded as in vendor kernel, because of */
	/* failure to determine it's actual meaning. */
	case 0x05:
	case 0x06:
	case 0x07:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case 0x08:
	case 0xA:
	case 0xB:
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}

	return 0;
}

static int max77705_get_vbus_state(struct regmap *regmap, int *value)
{
	int ret;
	unsigned int charge_dtls;

	ret = regmap_read(regmap, MAX77705_CHG_REG_DETAILS_00, &charge_dtls);
	if (ret)
		return ret;

	charge_dtls = ((charge_dtls & MAX77705_CHGIN_DTLS) >>
			MAX77705_CHGIN_DTLS_SHIFT);

	switch (charge_dtls) {
	case 0x00:
		*value = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
		break;
	case 0x01:
		*value = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
		break;
	case 0x02:
		*value = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	case 0x03:
		*value = POWER_SUPPLY_HEALTH_GOOD;
		break;
	default:
		return 0;
	}
	return 0;
}

static int max77705_get_battery_health(struct max77705_charger_data *chg,
					int *value)
{
	struct regmap *regmap = chg->regmap;
	unsigned int bat_dtls;

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_01, &bat_dtls);
	bat_dtls = ((bat_dtls & MAX77705_BAT_DTLS) >> MAX77705_BAT_DTLS_SHIFT);

	switch (bat_dtls) {
	case MAX77705_BATTERY_NOBAT:
		dev_dbg(chg->dev, "%s: No battery and the chg is suspended\n",
			__func__);
		*value = POWER_SUPPLY_HEALTH_NO_BATTERY;
		break;
	case MAX77705_BATTERY_PREQUALIFICATION:
		dev_dbg(chg->dev, "%s: battery is okay but its voltage is low(~VPQLB)\n",
			__func__);
		break;
	case MAX77705_BATTERY_DEAD:
		dev_dbg(chg->dev, "%s: battery dead\n", __func__);
		*value = POWER_SUPPLY_HEALTH_DEAD;
		break;
	case MAX77705_BATTERY_GOOD:
	case MAX77705_BATTERY_LOWVOLTAGE:
		*value = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case MAX77705_BATTERY_OVERVOLTAGE:
		dev_dbg(chg->dev, "%s: battery ovp\n", __func__);
		*value = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	default:
		dev_dbg(chg->dev, "%s: battery unknown\n", __func__);
		*value = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	}

	return 0;
}

static int max77705_get_health(struct max77705_charger_data *chg, int *val)
{
	struct regmap *regmap = chg->regmap;
	int ret, is_online = 0;

	if (chg->bat_hot) {
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	}

	ret = max77705_get_online(regmap, &is_online);
	if (ret)
		return ret;
	if (is_online) {
		ret = max77705_get_vbus_state(regmap, val);
		if (ret || (*val != POWER_SUPPLY_HEALTH_GOOD))
			return ret;
	}
	return max77705_get_battery_health(chg, val);
}

static int max77705_get_input_current(struct max77705_charger_data *chg,
					int *val)
{
	unsigned int reg_data;
	int get_current = 0;

	regmap_field_read(chg->rfield[MAX77705_CHG_CHGIN_LIM], &reg_data);

	if (reg_data <= 3)
		get_current = MAX77705_CURRENT_CHGIN_MIN;
	else
		get_current = (reg_data + 1) * MAX77705_CURRENT_CHGIN_STEP;

	*val = get_current;

	return 0;
}

static int max77705_get_charge_current(struct max77705_charger_data *chg,
					int *val)
{
	unsigned int reg_data;

	regmap_field_read(chg->rfield[MAX77705_CHG_CC_LIM], &reg_data);

	*val = reg_data <= 0x2 ? MAX77705_CURRENT_CHGIN_MIN : reg_data * MAX77705_CURRENT_CHG_STEP;

	return 0;
}

static int max77705_set_float_voltage(struct max77705_charger_data *chg,
					int float_voltage)
{
	int float_voltage_mv;
	unsigned int reg_data = 0;

	float_voltage_mv = float_voltage / 1000;
	reg_data = float_voltage_mv <= 4000 ? 0x0 :
		float_voltage_mv >= 4500 ? 0x23 :
		(float_voltage_mv <= 4200) ? (float_voltage_mv - 4000) / 50 :
		(((float_voltage_mv - 4200) / 10) + 0x04);

	return regmap_field_write(chg->rfield[MAX77705_CHG_CV_PRM], reg_data);
}

static int max77705_get_float_voltage(struct max77705_charger_data *chg,
					int *val)
{
	unsigned int reg_data = 0;
	int voltage_mv;

	regmap_field_read(chg->rfield[MAX77705_CHG_CV_PRM], &reg_data);
	voltage_mv = reg_data <= 0x04 ? reg_data * 50 + 4000 :
					(reg_data - 4) * 10 + 4200;
	*val = voltage_mv * 1000;

	return 0;
}

static int max77705_chg_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct max77705_charger_data *chg = power_supply_get_drvdata(psy);
	struct regmap *regmap = chg->regmap;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return max77705_get_online(regmap, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		return max77705_check_battery(chg, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return max77705_get_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return max77705_get_charge_type(chg, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return max77705_get_health(chg, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return max77705_get_input_current(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return max77705_get_charge_current(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return max77705_get_float_voltage(chg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chg->bat_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = max77705_charger_model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = max77705_charger_manufacturer;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * A new input limit: AICL starts over from it. The same one again (USB-C
 * writes it along with each new charge current) leaves AICL where it is.
 */
static int max77705_set_input_current(struct max77705_charger_data *chg, int ua)
{
	/* code n is (n + 1) x 25 mA, as max77705_get_input_current() reads it */
	unsigned int lim = clamp_val(ua, MAX77705_CURRENT_CHGIN_MIN,
				     MAX77705_CURRENT_CHGIN_MAX) / MAX77705_CURRENT_CHGIN_STEP - 1;
	int err = 0;

	mutex_lock(&chg->aicl_lock);
	if (lim == chg->ilim_req)
		goto out;
	err = regmap_field_write(chg->rfield[MAX77705_CHG_CHGIN_LIM], lim);
	if (err)
		goto out;
	chg->ilim_req = lim;
	chg->ilim_shown = lim;
	chg->aicl_stepping = false;
	chg->aicl_climbing = false;
	chg->rerun_at = jiffies + msecs_to_jiffies(MAX77705_AICL_RERUN_MS);
	if (!chg->aicl_stopped) {
		max77705_aicl_irq_on(chg);
		/* AICL_OK may be low already, with no new edge to tell */
		mod_delayed_work(chg->wqueue, &chg->aicl_work,
				 msecs_to_jiffies(AICL_WORK_DELAY_MS));
	}
out:
	mutex_unlock(&chg->aicl_lock);

	return err;
}

static int max77705_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct max77705_charger_data *chg = power_supply_get_drvdata(psy);
	int err = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		err = max77705_set_integer(chg, MAX77705_CHG_CC_LIM,
					   MAX77705_CURRENT_CHGIN_MIN,
					   MAX77705_CURRENT_CHGIN_MAX,
					   MAX77705_CURRENT_CHG_STEP,
					   val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		err = max77705_set_input_current(chg, val->intval);
		break;
	default:
		err = -EINVAL;
	}

	return err;
};

static int max77705_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return true;
	default:
		return false;
	}
}

/*
 * The wireless input as a supply of its own, so that userspace can tell a
 * pad from a cable: the charger's supply stays CHGIN's.
 */
static enum power_supply_property max77705_wcin_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int max77705_wcin_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct max77705_charger_data *chg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, MAX77705_CHG_REG_INT_OK, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & MAX77705_WCIN_OK);
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/* WCIN_LIM code n limits the input to (n + 1) x 25 mA, 100 mA at least */
		ret = regmap_read(chg->regmap, MAX77705_CHG_REG_CNFG_10, &reg);
		if (ret)
			return ret;
		reg &= MAX77705_WCIN_LIM_MASK;
		val->intval = reg <= 3 ? 100000 : (reg + 1) * 25000;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc max77705_wcin_psy_desc = {
	.name = "max77705-wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = max77705_wcin_props,
	.num_properties = ARRAY_SIZE(max77705_wcin_props),
	.get_property = max77705_wcin_get_property,
};

static const struct power_supply_desc max77705_charger_psy_desc = {
	.name = "max77705-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77705_charger_props,
	.property_is_writeable = max77705_property_is_writeable,
	.num_properties = ARRAY_SIZE(max77705_charger_props),
	.get_property = max77705_chg_get_property,
	.set_property = max77705_set_property,
};

static void max77705_chgin_isr_work(struct work_struct *work)
{
	struct max77705_charger_data *chg =
		container_of(work, struct max77705_charger_data, chgin_work);

	mod_delayed_work(chg->wqueue, &chg->temp_work, 0);
	/* an input that comes back gets its limit fitted again */
	mod_delayed_work(chg->wqueue, &chg->aicl_work, msecs_to_jiffies(AICL_WORK_DELAY_MS));
	power_supply_changed(chg->psy_chg);
}

/* the battery thermistor (the SoC ADC's NTC zone on these phones), in mC */
static int max77705_battery_mc(int *mc)
{
	struct thermal_zone_device *tz;

	tz = thermal_zone_get_zone_by_name(MAX77705_BAT_ZONE);
	if (IS_ERR(tz))
		return PTR_ERR(tz);

	return thermal_zone_get_temp(tz, mc);
}

/*
 * Stock stops charging from every input once the battery thermistor reads
 * 50.0 C and starts again at 48.0 C (battery,temp_high_threshold_normal and
 * battery,temp_high_recovery_normal). Charging stops through CHG_EN only:
 * the input keeps running the phone. Looked at every 10 s while an input is
 * online; the input interrupts start it again. A board without the zone
 * gets no stop, and one warning.
 */
static void max77705_temp_work(struct work_struct *work)
{
	struct max77705_charger_data *chg =
		container_of(to_delayed_work(work), struct max77705_charger_data, temp_work);
	int stop_mc = MAX77705_BAT_STOP_MC, resume_mc = MAX77705_BAT_RESUME_MC;
	unsigned int ok;
	int mc;

	if (READ_ONCE(chg->unbinding) ||
	    regmap_read(chg->regmap, MAX77705_CHG_REG_INT_OK, &ok) ||
	    !(ok & (MAX77705_CHGIN_OK | MAX77705_WCIN_OK)))
		return;

	if (test_stop_mc > 0)
		stop_mc = min(test_stop_mc, MAX77705_BAT_STOP_MC);
	resume_mc = min(test_resume_mc > 0 ? test_resume_mc : resume_mc,
			stop_mc - MAX77705_BAT_MIN_HYST_MC);

	if (max77705_battery_mc(&mc)) {
		if (chg->zone_misses < MAX77705_BAT_ZONE_TRIES &&
		    ++chg->zone_misses == MAX77705_BAT_ZONE_TRIES)
			dev_warn(chg->dev, "no %s thermal zone: charging does not stop on a hot battery\n",
				 MAX77705_BAT_ZONE);
	} else if (!chg->bat_hot && mc >= stop_mc) {
		chg->bat_hot = true;
		max77705_charger_disable(chg);
		dev_warn(chg->dev, "battery at %d.%d degC: charging stopped until %d.%d degC\n",
			 mc / 1000, mc % 1000 / 100, resume_mc / 1000, resume_mc % 1000 / 100);
		power_supply_changed(chg->psy_chg);
	} else if (chg->bat_hot && mc <= resume_mc) {
		chg->bat_hot = false;
		max77705_charger_enable(chg);
		dev_info(chg->dev, "battery back at %d.%d degC: charging again\n",
			 mc / 1000, mc % 1000 / 100);
		power_supply_changed(chg->psy_chg);
	}

	queue_delayed_work(chg->wqueue, &chg->temp_work,
			   msecs_to_jiffies(MAX77705_BAT_POLL_MS));
}

/*
 * Where the wireless input is held down and let go, on the battery: the
 * board's maxim,wcin-derate-millicelsius = <hot cool>, else the S20 FE's
 * stock 41.0 / 38.0 C. Read each time, so that a DT overlay applied after
 * probe counts too.
 */
static void max77705_wcin_derate_temps(struct max77705_charger_data *chg,
				       int *hot_mc, int *cool_mc)
{
	u32 t[2];

	*hot_mc = MAX77705_WCIN_HOT_MC;
	*cool_mc = MAX77705_WCIN_COOL_MC;
	if (!device_property_read_u32_array(chg->dev, "maxim,wcin-derate-millicelsius", t, 2) &&
	    t[1] < t[0] && t[0] <= MAX77705_BAT_STOP_MC) {
		*hot_mc = t[0];
		*cool_mc = t[1];
	}
}

/*
 * Stock's wireless thermal policy (battery,wpc_high_temp and the rest): from
 * 41.0 C on the battery thermistor (the S20 FE; 38.0 C on the S10e) the input
 * is held at 600 mA, 450 mA with the screen on, until the battery is back at
 * 38.0 C (36.0 C). A board without the battery thermistor's zone gets no
 * derating.
 */
static int max77705_wcin_ceiling(struct max77705_charger_data *chg)
{
	struct backlight_device *bd = NULL;
	int mc, ma, hot_mc, cool_mc;

	if (max77705_battery_mc(&mc))
		return chg->wcin_max_ma;

	max77705_wcin_derate_temps(chg, &hot_mc, &cool_mc);
	if (!chg->wcin_hot && mc >= hot_mc) {
		chg->wcin_hot = true;
		dev_info(chg->dev, "battery at %d.%d degC: wireless input held down\n",
			 mc / 1000, mc % 1000 / 100);
	} else if (chg->wcin_hot && mc <= cool_mc) {
		chg->wcin_hot = false;
		dev_info(chg->dev, "battery back at %d.%d degC: wireless input released\n",
			 mc / 1000, mc % 1000 / 100);
	}
	if (!chg->wcin_hot)
		return chg->wcin_max_ma;

#if IS_REACHABLE(CONFIG_BACKLIGHT_CLASS_DEVICE)
	bd = backlight_device_get_by_type(BACKLIGHT_RAW);
#endif
	ma = bd && !backlight_is_blank(bd) ? MAX77705_WCIN_HOT_LCD_MA : MAX77705_WCIN_HOT_MA;

	return min(ma, chg->wcin_max_ma);
}

/*
 * Wireless (WCIN, fed by the IDT receiver): the receiver's own firmware runs
 * the Qi link at 5 V, the charger only limits what it takes. Stock starts at
 * 480 mA and steps up, so a weak pad is not overloaded the moment it starts;
 * the ceiling is the board's (maxim,wcin-current-limit-microamp), else 820 mA,
 * and lower on a hot battery. On a pad the battery is looked at every 10 s.
 */
static void max77705_wcin_work(struct work_struct *work)
{
	struct max77705_charger_data *chg =
		container_of(to_delayed_work(work), struct max77705_charger_data, wcin_work);
	unsigned int ok;
	int ma, ceiling = 0;
	bool changed;

	if (regmap_read(chg->regmap, MAX77705_CHG_REG_INT_OK, &ok))
		return;

	if (ok & MAX77705_WCIN_OK)
		ceiling = max77705_wcin_ceiling(chg);

	if (!(ok & MAX77705_WCIN_OK))
		ma = MAX77705_WCIN_START_MA;
	else if (!chg->wcin_ma)
		ma = min(MAX77705_WCIN_START_MA, ceiling);
	else if (chg->wcin_ma > ceiling)
		ma = ceiling;
	else
		ma = min(chg->wcin_ma + MAX77705_WCIN_STEP_MA, ceiling);

	/* without a pad this runs from the interrupt only */
	changed = ma != chg->wcin_ma || !(ok & MAX77705_WCIN_OK);
	if (changed)
		regmap_update_bits(chg->regmap, MAX77705_CHG_REG_CNFG_10,
				   MAX77705_WCIN_LIM_MASK, MAX77705_WCIN_LIM(ma));

	if (!(ok & MAX77705_WCIN_OK)) {
		if (chg->wcin_ma)
			dev_info(chg->dev, "wireless input gone\n");
		chg->wcin_ma = 0;
	} else {
		if (!chg->wcin_ma)
			dev_info(chg->dev, "wireless input present\n");
		chg->wcin_ma = ma;
		queue_delayed_work(chg->wqueue, &chg->wcin_work,
				   msecs_to_jiffies(ma < ceiling ? MAX77705_WCIN_STEP_MS :
						    MAX77705_WCIN_POLL_MS));
	}

	if (changed) {
		power_supply_changed(chg->psy_chg);
		power_supply_changed(chg->psy_wcin);
	}
}

static int max77705_charger_initialize(struct max77705_charger_data *chg)
{
	struct power_supply_battery_info *info;
	struct regmap *regmap = chg->regmap;
	int err;

	err = power_supply_get_battery_info(chg->psy_chg, &info);
	if (err)
		return dev_err_probe(chg->dev, err, "error on getting battery info");

	chg->bat_info = info;

	/* unlock charger setting protect */
	/* slowest LX slope */
	err = regmap_field_write(chg->rfield[MAX77705_CHGPROT], MAX77705_CHGPROT_UNLOCKED);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_LX_SLOPE], MAX77705_SLOWEST_LX_SLOPE);
	if (err)
		goto err;

	/* fast charge timer disable */
	/* restart threshold disable */
	/* pre-qual charge disable */
	err = regmap_field_write(chg->rfield[MAX77705_FCHGTIME], MAX77705_FCHGTIME_DISABLE);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_CHG_RSTRT], MAX77705_CHG_RSTRT_DISABLE);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_CHG_PQEN], MAX77705_CHG_PQEN_DISABLE);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_MODE],
			MAX77705_CHG_MASK | MAX77705_BUCK_MASK);
	if (err)
		goto err;

	/* charge current 450mA(default) */
	/* otg current limit 900mA */
	err = regmap_field_write(chg->rfield[MAX77705_OTG_ILIM], MAX77705_OTG_ILIM_900);
	if (err)
		goto err;

	/* BAT to SYS OCP 4.80A */
	err = regmap_field_write(chg->rfield[MAX77705_REG_B2SOVRC], MAX77705_B2SOVRC_4_8A);
	if (err)
		goto err;

	/* top off current 150mA */
	/* top off timer 30min */
	err = regmap_field_write(chg->rfield[MAX77705_TO], MAX77705_TO_ITH_150MA);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_TO_TIME], MAX77705_TO_TIME_30M);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_SYS_TRACK], MAX77705_SYS_TRACK_DISABLE);
	if (err)
		goto err;

	/* cv voltage 4.2V or 4.35V */
	/* MINVSYS 3.6V(default) */
	if (info->voltage_max_design_uv < 0) {
		dev_warn(chg->dev, "missing battery:voltage-max-design-microvolt\n");
		max77705_set_float_voltage(chg, 4200000);
	} else {
		max77705_set_float_voltage(chg, info->voltage_max_design_uv);
	}

	err = regmap_field_write(chg->rfield[MAX77705_VCHGIN], MAX77705_VCHGIN_4_5);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_WCIN], MAX77705_WCIN_4_5);
	if (err)
		goto err;

	/* take both inputs (stock keeps WCINSEL and CHGINSEL set) */
	err = regmap_update_bits(regmap, MAX77705_CHG_REG_CNFG_12,
				 MAX77705_WCINSEL | MAX77705_CHGINSEL,
				 MAX77705_WCINSEL | MAX77705_CHGINSEL);
	if (err)
		goto err;

	err = regmap_update_bits(regmap, MAX77705_CHG_REG_CNFG_10, MAX77705_WCIN_LIM_MASK,
				 MAX77705_WCIN_LIM(MAX77705_WCIN_START_MA));
	if (err)
		goto err;

	/* Watchdog timer */
	regmap_update_bits(regmap, MAX77705_CHG_REG_CNFG_00,
				MAX77705_WDTEN_MASK, 0);

	/* VBYPSET=5.0V */
	err = regmap_field_write(chg->rfield[MAX77705_VBYPSET], 0);
	if (err)
		goto err;

	/* Switching Frequency : 1.5MHz */
	err = regmap_field_write(chg->rfield[MAX77705_REG_FSW], MAX77705_CHG_FSW_1_5MHz);
	if (err)
		goto err;

	/* Auto skip mode */
	err = regmap_field_write(chg->rfield[MAX77705_REG_DISKIP], MAX77705_AUTO_SKIP);
	if (err)
		goto err;

	return 0;

err:
	return dev_err_probe(chg->dev, err, "error while configuring");

}

static int max77705_charger_probe(struct i2c_client *i2c)
{
	struct power_supply_config pscfg = {};
	struct max77705_charger_data *chg;
	struct regmap_irq_chip *chip_desc;
	struct device *dev;
	struct regmap_irq_chip_data *irq_data;
	u32 wcin_ua;
	int ret;

	dev = &i2c->dev;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	i2c_set_clientdata(i2c, chg);

	chip_desc = devm_kmemdup(dev, &max77705_charger_irq_chip,
				 sizeof(max77705_charger_irq_chip),
				 GFP_KERNEL);
	if (!chip_desc)
		return -ENOMEM;
	chip_desc->irq_drv_data = chg;

	chg->regmap = devm_regmap_init_i2c(i2c, &max77705_chg_regmap_config);
	if (IS_ERR(chg->regmap))
		return PTR_ERR(chg->regmap);

	for (int i = 0; i < MAX77705_N_REGMAP_FIELDS; i++) {
		chg->rfield[i] = devm_regmap_field_alloc(dev, chg->regmap,
							 max77705_reg_field[i]);
		if (IS_ERR(chg->rfield[i]))
			return dev_err_probe(dev, PTR_ERR(chg->rfield[i]),
					     "cannot allocate regmap field\n");
	}

	/*
	 * The input limit can be set as soon as the power supply is there:
	 * what AICL needs comes first, and goes last.
	 */
	chg->wqueue = devm_alloc_ordered_workqueue(dev, "%s", 0, dev_name(dev));
	if (!chg->wqueue)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &chg->aicl_lock);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(dev, &chg->aicl_work, max77705_aicl_work);
	if (ret)
		return ret;

	/* AICL fits what the boot loader left, until a limit is asked for */
	ret = regmap_field_read(chg->rfield[MAX77705_CHG_CHGIN_LIM], &chg->ilim_req);
	if (ret)
		return ret;
	chg->ilim_shown = chg->ilim_req;
	chg->rerun_at = jiffies;

	pscfg.fwnode = dev_fwnode(dev);
	pscfg.drv_data = chg;

	chg->psy_chg = devm_power_supply_register(dev, &max77705_charger_psy_desc, &pscfg);
	if (IS_ERR(chg->psy_chg))
		return PTR_ERR(chg->psy_chg);

	/* nothing in the DT names the wireless input: no node of its own */
	pscfg.fwnode = NULL;
	chg->psy_wcin = devm_power_supply_register(dev, &max77705_wcin_psy_desc, &pscfg);
	if (IS_ERR(chg->psy_wcin))
		return PTR_ERR(chg->psy_wcin);

	ret = devm_regmap_add_irq_chip(chg->dev, chg->regmap, i2c->irq,
					IRQF_ONESHOT, 0,
					chip_desc, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add irq chip\n");

	ret = devm_work_autocancel(dev, &chg->chgin_work, max77705_chgin_isr_work);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize interrupt work\n");

	ret = devm_delayed_work_autocancel(dev, &chg->temp_work, max77705_temp_work);
	if (ret)
		return ret;

	ret = max77705_charger_initialize(chg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize charger IC\n");

	ret = devm_request_threaded_irq(dev, regmap_irq_get_virq(irq_data, MAX77705_CHGIN_I),
					NULL, max77705_chgin_irq,
					IRQF_TRIGGER_NONE,
					"chgin-irq", chg);
	if (ret)
		return ret;

	chg->wcin_max_ma = MAX77705_WCIN_MAX_MA;
	if (!device_property_read_u32(dev, "maxim,wcin-current-limit-microamp", &wcin_ua))
		chg->wcin_max_ma = clamp_val(wcin_ua / 1000, MAX77705_WCIN_START_MA,
					     MAX77705_WCIN_LIM_MAX_MA);

	ret = devm_delayed_work_autocancel(dev, &chg->wcin_work, max77705_wcin_work);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, regmap_irq_get_virq(irq_data, MAX77705_WCIN_I),
					NULL, max77705_wcin_irq,
					IRQF_TRIGGER_NONE,
					"wcin-irq", chg);
	if (ret)
		return ret;
	/* a pad the phone was already lying on */
	queue_delayed_work(chg->wqueue, &chg->wcin_work, 0);

	chg->aicl_irq = regmap_irq_get_virq(irq_data, MAX77705_AICL_I);
	ret = devm_request_threaded_irq(dev, chg->aicl_irq,
					NULL, max77705_aicl_irq,
					IRQF_TRIGGER_NONE,
					"aicl-irq", chg);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, max77705_aicl_stop, chg);
	if (ret)
		return ret;

	/* AICL may be on already, with no edge to tell */
	mod_delayed_work(chg->wqueue, &chg->aicl_work, msecs_to_jiffies(AICL_WORK_DELAY_MS));

	ret = max77705_charger_enable(chg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable charge\n");

	ret = devm_add_action_or_reset(dev, max77705_charger_stop, chg);
	if (ret)
		return ret;

	queue_delayed_work(chg->wqueue, &chg->temp_work, 0);

	return 0;
}

static const struct of_device_id max77705_charger_of_match[] = {
	{ .compatible = "maxim,max77705-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77705_charger_of_match);

static struct i2c_driver max77705_charger_driver = {
	.driver = {
		.name = "max77705-charger",
		.of_match_table = max77705_charger_of_match,
	},
	.probe = max77705_charger_probe,
};
module_i2c_driver(max77705_charger_driver);

MODULE_AUTHOR("Dzmitry Sankouski <dsankouski@gmail.com>");
MODULE_DESCRIPTION("Maxim MAX77705 charger driver");
MODULE_LICENSE("GPL");

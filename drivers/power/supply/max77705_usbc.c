// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Maxim MAX77705 USB Type-C / USB PD controller: the power a sink is offered.
 *
 * The MAX77705's USB-C block sits at its own I2C address (0x25) and runs the
 * Type-C state machine and the USB PD policy engine in firmware. The host
 * reads its status registers and talks to that firmware through opcodes: the
 * opcode and its data are written from 0x21 on, a write to 0x41 runs a
 * command shorter than 32 bytes, and APCmdResI (UIC_INT bit 7) says that the
 * answer is waiting from 0x51 on.
 *
 * This driver reports what the attached source offers - a USB PD contract, a
 * Type-C Rp current or a BC1.2 port type - as a USB power supply, asks a PD
 * source for its best fixed PDO up to the sink limits of the connector node
 * (9 V and 15 W without one, as the stock kernel of the Galaxy S20 FE), and
 * sets the MAX77705 charger's input and charge current from it, with the
 * stock kernel's battery temperature limits on the charge current.
 *
 * The status registers are polled every 2 s when the device has no
 * interrupt; with one (the MAX77705 interrupt source USBC), every change is
 * handled at once and the battery temperature is checked every 30 s.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/unaligned.h>
#include <linux/usb/pd.h>
#include <linux/workqueue.h>

#define MAX77705_USBC_HW_REV		0x00
#define MAX77705_USBC_FW_REV		0x01
#define MAX77705_USBC_UIC_INT		0x02	/* 0x02-0x05 clear on read */
#define MAX77705_USBC_STATUS1		0x06	/* 0x06-0x0d: status block */
#define MAX77705_USBC_OPCODE_W		0x21
#define MAX77705_USBC_OPCODE_END	0x41
#define MAX77705_USBC_OPCODE_R		0x51

#define UIC_INT_APCMDRESI		BIT(7)

/* offsets in the status block read from 0x06 */
#define ST_BC		(0x08 - MAX77705_USBC_STATUS1)
#define ST_CC0		(0x0a - MAX77705_USBC_STATUS1)
#define ST_PD1		(0x0d - MAX77705_USBC_STATUS1)
#define ST_LEN		8

#define BC_VBUSDET			BIT(7)
#define BC_CHGTYP			GENMASK(1, 0)
#define CC0_ISTAT			GENMASK(5, 4)
#define CC0_STAT			GENMASK(2, 0)
#define PD1_PSRDY			BIT(4)

enum { CHGTYP_NONE, CHGTYP_SDP, CHGTYP_CDP, CHGTYP_DCP };
enum { CCSTAT_NONE, CCSTAT_SINK, CCSTAT_SOURCE };
enum { CCI_NOT_UFP, CCI_DEFAULT, CCI_1_5A, CCI_3_0A };

#define OP_CURRENT_SRCCAP		0x30
#define OP_SRCCAP_REQUEST		0x32
#define OP_DATA_MAX			32
#define MAX_SRC_PDOS			7

#define POLL_MS				2000
#define RECHECK_MS			500	/* after asking for a new PDO */
#define EVENT_MS			20	/* an event came in during an update */
#define TEMP_POLL_MS			30000

/* the input current per BC1.2 port type when Rp asks for nothing more */
#define SDP_UA				500000
#define CDP_UA				1500000
#define DCP_UA				1800000

/* MAX77705 charger range */
#define CHGIN_MIN_UA			100000
#define CHGIN_MAX_UA			3200000
#define CHG_MIN_UA			100000

/*
 * Charge current: the stock kernel's per-input values (battery node of the
 * G780F device tree: 2100 mA from a 5 V adapter, 3150 mA from 9 V), never
 * more than 90 % of the input power can carry at 4.2 V.
 */
#define CC_5V_UA			2100000
#define CC_HV_UA			3150000
#define CC_BAT_UV			4200000

/*
 * Battery temperature zones (0.1 degC) with the stock limits: 437 mA below
 * 5 degC, 1048 mA below 15 degC, 1573 mA from 42 degC, and no charging below
 * 0 degC or from 50 degC. A zone is left 2 degC past its edge.
 */
enum { BAT_COLD, BAT_COOL2, BAT_COOL1, BAT_NORMAL, BAT_WARM, BAT_HOT };

static const struct {
	const char *name;
	int limit_ua;
} max77705_usbc_zones[] = {
	[BAT_COLD]	= { "below 0 degC, not charging", 0 },
	[BAT_COOL2]	= { "below 5 degC", 437000 },
	[BAT_COOL1]	= { "below 15 degC", 1048000 },
	[BAT_NORMAL]	= { "normal", INT_MAX },
	[BAT_WARM]	= { "42 degC or warmer", 1573000 },
	[BAT_HOT]	= { "50 degC or warmer, not charging", 0 },
};

struct max77705_usbc {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *charger;
	struct delayed_work work;
	struct mutex lock;		/* the chip's opcode channel, and this state */
	int irq;

	/* the sink's limits */
	unsigned int max_mv;
	unsigned int max_mw;

	/* what the source offers */
	bool online;
	enum power_supply_usb_type usb_type;
	unsigned int mv;		/* VBUS now */
	unsigned int max_ma;		/* the input current we may draw */
	unsigned int contract_mv;	/* highest fixed PDO we would take */
	u32 pdos[MAX_SRC_PDOS];
	unsigned int npdos;
	unsigned int sel;		/* 1-based PDO in the contract, 0 = none */
	bool requested;			/* asked for the best PDO in this attach */
	bool recheck;			/* the answer to that is due */
	bool event;			/* a status change not handled yet */
	char why[48];

	/* what the charger was given */
	int zone;
	bool temp_warned;
	const char *temp_src;		/* where the temperature came from last */
	int ilim_ua, cc_ua;
	int saved_ilim_ua, saved_cc_ua;
};

static char *battery_zone = "battery-thermal";
module_param(battery_zone, charp, 0444);
MODULE_PARM_DESC(battery_zone, "thermal zone of the battery thermistor (else the battery's TEMP)");

static const struct regmap_config max77705_usbc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x71,
};

/*
 * Reads and clears the four interrupt registers. APCmdResI only answers an
 * opcode; anything else is a status change, which the next update handles.
 */
static int max77705_usbc_read_ints(struct max77705_usbc *u, u8 *ints)
{
	int ret;

	lockdep_assert_held(&u->lock);
	ret = regmap_bulk_read(u->regmap, MAX77705_USBC_UIC_INT, ints, 4);
	if (!ret && ((ints[0] & ~UIC_INT_APCMDRESI) | ints[1] | ints[2] | ints[3]))
		u->event = true;
	return ret;
}

/*
 * Runs one firmware opcode and reads its answer: resp[0] = the opcode,
 * resp[1..rlen] its data.
 */
static int max77705_usbc_opcode(struct max77705_usbc *u, u8 op, const u8 *data,
				unsigned int wlen, u8 *resp, unsigned int rlen)
{
	u8 buf[OP_DATA_MAX + 1], ints[4];
	int ret, i;

	lockdep_assert_held(&u->lock);
	if (wlen > OP_DATA_MAX || rlen > OP_DATA_MAX)
		return -EINVAL;

	/* drop stale flags: an old APCmdResI would end the wait at once */
	ret = max77705_usbc_read_ints(u, ints);
	if (ret)
		return ret;

	buf[0] = op;
	memcpy(&buf[1], data, wlen);
	ret = regmap_bulk_write(u->regmap, MAX77705_USBC_OPCODE_W, buf, wlen + 1);
	if (ret)
		return ret;
	if (wlen < OP_DATA_MAX) {
		ret = regmap_write(u->regmap, MAX77705_USBC_OPCODE_END, 0);
		if (ret)
			return ret;
	}

	for (i = 0; i < 100; i++) {
		msleep(10);
		ret = max77705_usbc_read_ints(u, ints);
		if (ret)
			return ret;
		if (ints[0] & UIC_INT_APCMDRESI)
			break;
	}
	if (i == 100) {
		dev_warn(u->dev, "opcode %#04x: no answer\n", op);
		return -ETIMEDOUT;
	}

	ret = regmap_bulk_read(u->regmap, MAX77705_USBC_OPCODE_R, resp, rlen + 1);
	if (ret)
		return ret;
	if (resp[0] != op) {
		dev_warn(u->dev, "opcode %#04x answered as %#04x\n", op, resp[0]);
		return -EIO;
	}
	return 0;
}

/* Reads the source's PDOs and which one the contract is on. */
static int max77705_usbc_read_srccap(struct max77705_usbc *u)
{
	u8 resp[2 + 4 * MAX_SRC_PDOS];
	unsigned int i;
	int ret;

	ret = max77705_usbc_opcode(u, OP_CURRENT_SRCCAP, NULL, 0, resp, sizeof(resp) - 1);
	if (ret)
		return ret;

	u->sel = (resp[1] >> 3) & 7;
	u->npdos = min_t(unsigned int, resp[1] & 7, MAX_SRC_PDOS);
	for (i = 0; i < u->npdos; i++)
		u->pdos[i] = get_unaligned_le32(&resp[2 + 4 * i]);
	if (u->sel > u->npdos)
		u->sel = 0;
	return 0;
}

/*
 * The fixed PDO with the most power we can take (capped at max_mw, at most
 * max_mv); of equal power the higher voltage, for less current in the cable.
 */
static unsigned int max77705_usbc_best_pdo(struct max77705_usbc *u)
{
	unsigned int i, best = 0, best_mw = 0;

	for (i = 0; i < u->npdos; i++) {
		u32 pdo = u->pdos[i];
		unsigned int mv, mw;

		if (pdo_type(pdo) != PDO_TYPE_FIXED)
			continue;
		mv = pdo_fixed_voltage(pdo);
		if (mv > u->max_mv)
			continue;
		mw = min(mv * pdo_max_current(pdo) / 1000, u->max_mw);
		if (mw >= best_mw) {
			best = i + 1;
			best_mw = mw;
		}
	}
	return best;
}

static unsigned int max77705_usbc_pdo_ma(struct max77705_usbc *u, u32 pdo)
{
	unsigned int mv = pdo_fixed_voltage(pdo);

	return min(pdo_max_current(pdo), mv ? u->max_mw * 1000 / mv : 0);
}

static bool max77705_usbc_has_pps(struct max77705_usbc *u)
{
	unsigned int i;

	for (i = 0; i < u->npdos; i++)
		if (pdo_type(u->pdos[i]) == PDO_TYPE_APDO &&
		    pdo_apdo_type(u->pdos[i]) == APDO_TYPE_PPS)
			return true;
	return false;
}

static int max77705_usbc_charger_set(struct max77705_usbc *u,
				     enum power_supply_property psp, int ua)
{
	union power_supply_propval val = { .intval = ua };

	return power_supply_set_property(u->charger, psp, &val);
}

/* The battery the charger feeds: its temperature, in 0.1 degC. */
static int max77705_usbc_find_battery(struct power_supply *psy, void *data)
{
	struct power_supply **found = data;
	struct max77705_usbc *u = power_supply_get_drvdata(*found);
	size_t i;

	if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
		return 0;
	for (i = 0; i < psy->num_supplies; i++) {
		if (!strcmp(psy->supplied_from[i], u->charger->desc->name)) {
			*found = psy;
			return 1;
		}
	}
	return 0;
}

/*
 * The battery thermistor's thermal zone first: a fuel gauge without a
 * thermistor of its own (the MAX77705's) only reports what was last written
 * to its TEMP register, by the boot loader for all we know.
 */
static int max77705_usbc_battery_temp(struct max77705_usbc *u, int *temp)
{
	union power_supply_propval val;
	struct power_supply *bat = u->psy;
	struct thermal_zone_device *tz;
	const char *src;
	int ret, mc;

	tz = battery_zone && *battery_zone ? thermal_zone_get_zone_by_name(battery_zone) :
					     ERR_PTR(-ENODEV);
	if (!IS_ERR(tz) && !thermal_zone_get_temp(tz, &mc)) {
		*temp = mc / 100;
		src = battery_zone;
	} else {
		if (!power_supply_for_each_psy(&bat, max77705_usbc_find_battery) || bat == u->psy)
			return -ENODEV;
		ret = power_supply_get_property(bat, POWER_SUPPLY_PROP_TEMP, &val);
		if (ret)
			return ret;
		*temp = val.intval;
		src = bat->desc->name;
	}
	if (src != u->temp_src) {
		dev_info(u->dev, "battery temperature from %s: %d.%d degC\n", src,
			 *temp / 10, abs(*temp % 10));
		u->temp_src = src;
	}
	return 0;
}

static int max77705_usbc_zone(int t, int prev)
{
	int z;

	if (t >= 500)
		z = BAT_HOT;
	else if (t >= 420)
		z = BAT_WARM;
	else if (t < 0)
		z = BAT_COLD;
	else if (t < 50)
		z = BAT_COOL2;
	else if (t < 150)
		z = BAT_COOL1;
	else
		z = BAT_NORMAL;

	/* hysteresis: a stricter zone is left only 2 degC past its edge */
	if (prev == BAT_HOT && z == BAT_WARM && t > 480)
		z = BAT_HOT;
	else if (prev == BAT_WARM && z == BAT_NORMAL && t > 400)
		z = BAT_WARM;
	else if (prev == BAT_COLD && z == BAT_COOL2 && t < 20)
		z = BAT_COLD;
	else if (prev == BAT_COOL2 && z == BAT_COOL1 && t < 70)
		z = BAT_COOL2;
	else if (prev == BAT_COOL1 && z == BAT_NORMAL && t < 170)
		z = BAT_COOL1;
	return z;
}

/* Gives the charger the input and charge current the source and the battery allow. */
static void max77705_usbc_apply(struct max77705_usbc *u)
{
	int ilim, cc, temp, zone = u->zone;

	if (!u->charger)
		return;

	if (!u->online) {
		/* a new plug starts low until it says more */
		ilim = SDP_UA;
		cc = SDP_UA;
	} else {
		ilim = clamp_t(int, u->max_ma * 1000, CHGIN_MIN_UA, CHGIN_MAX_UA);
		cc = u->mv > 6000 ? CC_HV_UA : CC_5V_UA;
		cc = min_t(int, cc, div_u64((u64)ilim * u->mv * 1000 * 9, CC_BAT_UV * 10ULL));
		if (!max77705_usbc_battery_temp(u, &temp)) {
			zone = max77705_usbc_zone(temp, u->zone);
			if (zone != u->zone)
				dev_info(u->dev, "battery %d.%d degC: %s\n", temp / 10,
					 abs(temp % 10), max77705_usbc_zones[zone].name);
			u->zone = zone;
		} else if (!u->temp_warned) {
			dev_warn(u->dev, "no battery temperature: no temperature limits\n");
			u->temp_warned = true;
		}
		cc = min(cc, max77705_usbc_zones[zone].limit_ua);
		/* the charger's least is 100 mA: as close to off as it goes */
		cc = max(cc, CHG_MIN_UA);
	}

	if (ilim == u->ilim_ua && cc == u->cc_ua)
		return;
	if (!max77705_usbc_charger_set(u, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, ilim))
		u->ilim_ua = ilim;
	if (!max77705_usbc_charger_set(u, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, cc))
		u->cc_ua = cc;
	dev_info(u->dev, "charger: input %d mA, charge %d mA\n", u->ilim_ua / 1000, u->cc_ua / 1000);
}

static void max77705_usbc_update(struct max77705_usbc *u)
{
	bool online, was_online = u->online;
	enum power_supply_usb_type type;
	unsigned int mv = 5000, ma, contract_mv = 5000;
	u8 st[ST_LEN], chgtyp, cci, ccstat;
	char why[sizeof(u->why)];
	int ret;

	ret = regmap_bulk_read(u->regmap, MAX77705_USBC_STATUS1, st, sizeof(st));
	if (ret) {
		dev_warn_ratelimited(u->dev, "status read: %d\n", ret);
		return;
	}
	chgtyp = FIELD_GET(BC_CHGTYP, st[ST_BC]);
	cci = FIELD_GET(CC0_ISTAT, st[ST_CC0]);
	ccstat = FIELD_GET(CC0_STAT, st[ST_CC0]);

	/* VBUS from a source; as a source ourselves we are not supplied */
	online = (st[ST_BC] & BC_VBUSDET) && ccstat != CCSTAT_SOURCE;
	if (!online) {
		u->npdos = 0;
		u->sel = 0;
		u->requested = false;
		u->online = false;
		u->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		u->mv = 0;
		u->max_ma = 0;
		u->contract_mv = 0;
		if (was_online) {
			dev_info(u->dev, "source gone\n");
			u->why[0] = 0;
			power_supply_changed(u->psy);
		}
		max77705_usbc_apply(u);
		return;
	}

	if (ccstat == CCSTAT_SINK && (st[ST_PD1] & PD1_PSRDY) &&
	    !max77705_usbc_read_srccap(u) && u->sel) {
		unsigned int best = max77705_usbc_best_pdo(u);
		u32 pdo = u->pdos[u->sel - 1];

		if (pdo_type(pdo) == PDO_TYPE_FIXED) {
			mv = pdo_fixed_voltage(pdo);
			ma = max77705_usbc_pdo_ma(u, pdo);
		} else {
			/* an APDO we never asked for: assume the PD floor */
			ma = 1500;
		}
		if (best)
			contract_mv = pdo_fixed_voltage(u->pdos[best - 1]);
		type = max77705_usbc_has_pps(u) ? POWER_SUPPLY_USB_TYPE_PD_PPS :
						  POWER_SUPPLY_USB_TYPE_PD;
		snprintf(why, sizeof(why), "PD %u mV %u mA (PDO %u of %u)", mv,
			 pdo_type(pdo) == PDO_TYPE_FIXED ? pdo_max_current(pdo) : 0,
			 u->sel, u->npdos);

		if (best && best != u->sel && !u->requested) {
			u32 bpdo = u->pdos[best - 1];
			u8 num = best, res = 0xff, resp[2];

			u->requested = true;
			/*
			 * Lower the input limit to what the new PDO allows
			 * first: the charger must not pull the old current at
			 * the new voltage before the next update.
			 */
			u->max_ma = min(ma, max77705_usbc_pdo_ma(u, bpdo));
			u->mv = mv;
			max77705_usbc_apply(u);
			if (!max77705_usbc_opcode(u, OP_SRCCAP_REQUEST, &num, 1, resp, 1))
				res = resp[1];
			dev_info(u->dev, "asked for PDO %u (%u mV %u mA): %s\n", best,
				 pdo_fixed_voltage(bpdo), pdo_max_current(bpdo),
				 res == 0 ? "sent" : res == 0xfe ? "SinkTxNg" :
				 res == 0xff ? "not ready" : "error");
			if (res) {
				u->requested = false;
			} else {
				/* keep the lowered limit until the new contract shows */
				u->recheck = true;
				return;
			}
		}
	} else if (cci == CCI_3_0A || cci == CCI_1_5A) {
		ma = cci == CCI_3_0A ? 3000 : 1500;
		type = POWER_SUPPLY_USB_TYPE_C;
		snprintf(why, sizeof(why), "Type-C Rp %s A", cci == CCI_3_0A ? "3.0" : "1.5");
	} else {
		switch (chgtyp) {
		case CHGTYP_DCP:
			ma = DCP_UA / 1000;
			type = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case CHGTYP_CDP:
			ma = CDP_UA / 1000;
			type = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		default:
			ma = SDP_UA / 1000;
			type = chgtyp == CHGTYP_SDP ? POWER_SUPPLY_USB_TYPE_SDP :
						      POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		}
		snprintf(why, sizeof(why), "BC1.2 %s",
			 (const char *[]){ "none", "SDP", "CDP", "DCP" }[chgtyp]);
	}

	u->online = true;
	u->contract_mv = contract_mv;
	if (u->usb_type != type || u->mv != mv || u->max_ma != ma || !was_online ||
	    strcmp(u->why, why)) {
		u->usb_type = type;
		u->mv = mv;
		u->max_ma = ma;
		strscpy(u->why, why);
		dev_info(u->dev, "%s: may draw %u mA at %u mV\n", why, ma, mv);
		power_supply_changed(u->psy);
	}
	max77705_usbc_apply(u);
}

static void max77705_usbc_work(struct work_struct *work)
{
	struct max77705_usbc *u = container_of(to_delayed_work(work),
					       struct max77705_usbc, work);
	unsigned long next;

	mutex_lock(&u->lock);
	u->event = false;
	max77705_usbc_update(u);
	next = u->irq > 0 ? (u->online ? TEMP_POLL_MS : 0) : POLL_MS;
	if (u->recheck) {
		u->recheck = false;
		next = RECHECK_MS;
	} else if (u->event) {
		next = EVENT_MS;
	}
	mutex_unlock(&u->lock);

	if (next)
		queue_delayed_work(system_freezable_power_efficient_wq, &u->work,
				   msecs_to_jiffies(next));
}

static irqreturn_t max77705_usbc_irq(int irq, void *data)
{
	struct max77705_usbc *u = data;
	bool event;
	u8 ints[4];

	/*
	 * An opcode's answer (APCmdResI) raises the interrupt too: the opcode
	 * has read it by the time the lock is free, and it is not an event.
	 * An update for it would read the source caps with an opcode again.
	 */
	mutex_lock(&u->lock);
	max77705_usbc_read_ints(u, ints);
	event = u->event;
	mutex_unlock(&u->lock);
	if (event)
		mod_delayed_work(system_freezable_power_efficient_wq, &u->work, 0);
	return IRQ_HANDLED;
}

static const enum power_supply_property max77705_usbc_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int max77705_usbc_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct max77705_usbc *u = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = u->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = u->usb_type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = u->mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = u->contract_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = u->max_ma * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *max77705_usbc_supplied_to[] = { "max77705-charger" };

static const struct power_supply_desc max77705_usbc_psy_desc = {
	.name = "max77705-usbc",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) | BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) | BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_C) | BIT(POWER_SUPPLY_USB_TYPE_PD) |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS),
	.properties = max77705_usbc_props,
	.num_properties = ARRAY_SIZE(max77705_usbc_props),
	.get_property = max77705_usbc_get_property,
};

/* Sink limits from the usb-c-connector child, if there is one. */
static void max77705_usbc_parse_connector(struct max77705_usbc *u)
{
	struct fwnode_handle *conn;
	u32 pdos[PDO_MAX_OBJECTS], uw;
	int n, i;

	u->max_mv = 9000;
	u->max_mw = 15000;

	conn = device_get_named_child_node(u->dev, "connector");
	if (!conn)
		return;
	if (!fwnode_property_read_u32(conn, "op-sink-microwatt", &uw))
		u->max_mw = uw / 1000;
	n = fwnode_property_count_u32(conn, "sink-pdos");
	if (n > 0 && n <= PDO_MAX_OBJECTS &&
	    !fwnode_property_read_u32_array(conn, "sink-pdos", pdos, n)) {
		u->max_mv = 0;
		for (i = 0; i < n; i++)
			if (pdo_type(pdos[i]) == PDO_TYPE_FIXED)
				u->max_mv = max(u->max_mv, pdo_fixed_voltage(pdos[i]));
		if (!u->max_mv)
			u->max_mv = 5000;
	}
	fwnode_handle_put(conn);
}

static void max77705_usbc_restore(void *data)
{
	struct max77705_usbc *u = data;

	if (u->saved_ilim_ua > 0)
		max77705_usbc_charger_set(u, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, u->saved_ilim_ua);
	if (u->saved_cc_ua > 0)
		max77705_usbc_charger_set(u, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, u->saved_cc_ua);
	power_supply_put(u->charger);
}

static int max77705_usbc_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct device *dev = &client->dev;
	union power_supply_propval val;
	struct max77705_usbc *u;
	unsigned int hw, fw;
	int ret;

	u = devm_kzalloc(dev, sizeof(*u), GFP_KERNEL);
	if (!u)
		return -ENOMEM;
	u->dev = dev;
	u->zone = BAT_NORMAL;
	u->ilim_ua = -1;
	u->cc_ua = -1;
	ret = devm_mutex_init(dev, &u->lock);
	if (ret)
		return ret;

	u->regmap = devm_regmap_init_i2c(client, &max77705_usbc_regmap_config);
	if (IS_ERR(u->regmap))
		return PTR_ERR(u->regmap);
	ret = regmap_read(u->regmap, MAX77705_USBC_HW_REV, &hw);
	if (!ret)
		ret = regmap_read(u->regmap, MAX77705_USBC_FW_REV, &fw);
	if (ret)
		return dev_err_probe(dev, ret, "no answer\n");

	max77705_usbc_parse_connector(u);

	/* the charger; its settings come back on unbind */
	u->charger = power_supply_get_by_name(max77705_usbc_supplied_to[0]);
	if (!u->charger)
		return dev_err_probe(dev, -EPROBE_DEFER, "waiting for %s\n",
				     max77705_usbc_supplied_to[0]);
	if (!power_supply_get_property(u->charger, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val))
		u->saved_ilim_ua = val.intval;
	if (!power_supply_get_property(u->charger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val))
		u->saved_cc_ua = val.intval;
	ret = devm_add_action_or_reset(dev, max77705_usbc_restore, u);
	if (ret)
		return ret;

	cfg.drv_data = u;
	cfg.fwnode = dev_fwnode(dev);
	cfg.supplied_to = max77705_usbc_supplied_to;
	cfg.num_supplicants = ARRAY_SIZE(max77705_usbc_supplied_to);
	u->psy = devm_power_supply_register(dev, &max77705_usbc_psy_desc, &cfg);
	if (IS_ERR(u->psy))
		return dev_err_probe(dev, PTR_ERR(u->psy), "power supply\n");

	ret = devm_delayed_work_autocancel(dev, &u->work, max77705_usbc_work);
	if (ret)
		return ret;

	u->irq = client->irq;
	if (u->irq > 0) {
		ret = devm_request_threaded_irq(dev, u->irq, NULL, max77705_usbc_irq,
						IRQF_ONESHOT, dev_name(dev), u);
		if (ret)
			return dev_err_probe(dev, ret, "irq %d\n", u->irq);
	}

	dev_info(dev, "hw %#04x fw %#04x, sink up to %u mV / %u mW, %s\n", hw, fw,
		 u->max_mv, u->max_mw, u->irq > 0 ? "interrupt" : "polled");
	queue_delayed_work(system_freezable_power_efficient_wq, &u->work, 0);
	return 0;
}

static const struct of_device_id max77705_usbc_of_match[] = {
	{ .compatible = "maxim,max77705-usbc" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77705_usbc_of_match);

static const struct i2c_device_id max77705_usbc_id[] = {
	{ "max77705-usbc" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77705_usbc_id);

static struct i2c_driver max77705_usbc_driver = {
	.driver = {
		.name = "max77705-usbc",
		.of_match_table = max77705_usbc_of_match,
	},
	.probe = max77705_usbc_probe,
	.id_table = max77705_usbc_id,
};
module_i2c_driver(max77705_usbc_driver);

MODULE_DESCRIPTION("Maxim MAX77705 USB Type-C / PD sink power");
MODULE_LICENSE("GPL");

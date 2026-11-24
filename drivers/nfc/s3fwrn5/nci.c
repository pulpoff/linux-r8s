// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * NCI based driver for Samsung S3FWRN5 NFC chip
 *
 * Copyright (C) 2015 Samsung Electronics
 * Robert Baldyga <r.baldyga@samsung.com>
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "s3fwrn5.h"
#include "nci.h"

static int s3fwrn5_nci_prop_rsp(struct nci_dev *ndev, struct sk_buff *skb)
{
	__u8 status = skb->data[0];

	nci_req_complete(ndev, status);
	return 0;
}

/* Keeps the payload: a GET_VER answer carries the versions, not a status. */
static int s3fwrn5_nci_dual_rsp(struct nci_dev *ndev, struct sk_buff *skb)
{
	struct s3fwrn5_info *info = nci_get_drvdata(ndev);

	info->dual_rsp_len = min_t(unsigned int, skb->len,
				   sizeof(info->dual_rsp));
	memcpy(info->dual_rsp, skb->data, info->dual_rsp_len);

	nci_req_complete(ndev, skb->len ? skb->data[0] : NCI_STATUS_SYNTAX_ERROR);
	return 0;
}

const struct nci_driver_ops s3fwrn5_nci_prop_ops[5] = {
	{
		.opcode = nci_opcode_pack(NCI_GID_PROPRIETARY,
				NCI_PROP_SET_RFREG),
		.rsp = s3fwrn5_nci_prop_rsp,
	},
	{
		.opcode = nci_opcode_pack(NCI_GID_PROPRIETARY,
				NCI_PROP_START_RFREG),
		.rsp = s3fwrn5_nci_prop_rsp,
	},
	{
		.opcode = nci_opcode_pack(NCI_GID_PROPRIETARY,
				NCI_PROP_STOP_RFREG),
		.rsp = s3fwrn5_nci_prop_rsp,
	},
	{
		.opcode = nci_opcode_pack(NCI_GID_PROPRIETARY,
				NCI_PROP_FW_CFG),
		.rsp = s3fwrn5_nci_prop_rsp,
	},
	{
		.opcode = nci_opcode_pack(NCI_GID_PROPRIETARY,
				NCI_PROP_DUAL_OPTION),
		.rsp = s3fwrn5_nci_dual_rsp,
	},
};

#define S3FWRN5_RFREG_SECTION_SIZE 252

int s3fwrn5_nci_rf_configure(struct s3fwrn5_info *info, const char *fw_name)
{
	struct device *dev = &info->ndev->nfc_dev->dev;
	const struct firmware *fw;
	struct nci_prop_fw_cfg_cmd fw_cfg;
	struct nci_prop_set_rfreg_cmd set_rfreg;
	struct nci_prop_stop_rfreg_cmd stop_rfreg;
	u32 checksum;
	int i, len;
	int ret;

	ret = request_firmware(&fw, fw_name, dev);
	if (ret < 0)
		return ret;

	/* Compute rfreg checksum */

	checksum = 0;
	for (i = 0; i < fw->size; i += 4)
		checksum += *((u32 *)(fw->data+i));

	/* Set default clock configuration for external crystal */

	fw_cfg.clk_type = 0x01;
	fw_cfg.clk_speed = 0xff;
	fw_cfg.clk_req = 0xff;
	ret = nci_prop_cmd(info->ndev, NCI_PROP_FW_CFG,
		sizeof(fw_cfg), (__u8 *)&fw_cfg);
	if (ret < 0)
		goto out;

	/* Start rfreg configuration */

	dev_info(dev, "rfreg configuration update: %s\n", fw_name);

	ret = nci_prop_cmd(info->ndev, NCI_PROP_START_RFREG, 0, NULL);
	if (ret < 0) {
		dev_err(dev, "Unable to start rfreg update\n");
		goto out;
	}

	/* Update rfreg */

	set_rfreg.index = 0;
	for (i = 0; i < fw->size; i += S3FWRN5_RFREG_SECTION_SIZE) {
		len = (fw->size - i < S3FWRN5_RFREG_SECTION_SIZE) ?
			(fw->size - i) : S3FWRN5_RFREG_SECTION_SIZE;
		memcpy(set_rfreg.data, fw->data+i, len);
		ret = nci_prop_cmd(info->ndev, NCI_PROP_SET_RFREG,
			len+1, (__u8 *)&set_rfreg);
		if (ret < 0) {
			dev_err(dev, "rfreg update error (code=%d)\n", ret);
			goto out;
		}
		set_rfreg.index++;
	}

	/* Finish rfreg configuration */

	stop_rfreg.checksum = checksum & 0xffff;
	ret = nci_prop_cmd(info->ndev, NCI_PROP_STOP_RFREG,
		sizeof(stop_rfreg), (__u8 *)&stop_rfreg);
	if (ret < 0) {
		dev_err(dev, "Unable to stop rfreg update\n");
		goto out;
	}

	dev_info(dev, "rfreg configuration update: success\n");
out:
	release_firmware(fw);
	return ret;
}

/*
 * Configure the reference clock. The S3NRN4V expects the single-byte FW_CFG
 * form (just the clock-speed selector). The downstream stack sends this in the
 * bootloader before CORE_RESET; the earliest the mainline NCI core lets us in
 * is the ->setup hook (after CORE_RESET, before CORE_INIT), which works.
 */
int s3fwrn5_nci_clk_cfg(struct s3fwrn5_info *info)
{
	u8 clk_speed = NCI_PROP_FW_CFG_CLK_SPEED;
	struct clk *clk;

	/* a 26 MHz reference (the Exynos boards' TCXO) has its own selector */
	clk = clk_get_optional(info->pdev, NULL);
	if (!IS_ERR_OR_NULL(clk)) {
		if (clk_get_rate(clk) == 26000000)
			clk_speed = NCI_PROP_FW_CFG_CLK_SPEED_26MHZ;
		clk_put(clk);
	}

	return nci_prop_cmd(info->ndev, NCI_PROP_FW_CFG, 1, &clk_speed);
}

/*
 * S3NRN4V RF calibration data update. The HW and SW calibration blobs are
 * merged into a single stream (HW first) and pushed via the DUAL_OPTION
 * command: START_UPDATE, one SET_OPTION per 252-byte section, then STOP_UPDATE
 * carrying a 16-bit checksum (running sum of the merged stream as 32-bit
 * words).
 */
int s3fwrn5_nci_rf_configure_dual(struct s3fwrn5_info *info,
				  const char *hw_name, const char *sw_name)
{
	const struct firmware *hw_fw = NULL, *sw_fw = NULL;
	struct nci_prop_dual_set_option_cmd set_option;
	struct device *dev = &info->ndev->nfc_dev->dev;
	size_t merged_size, i, len;
	u8 *merged = NULL;
	u8 stop_cmd[3];
	u32 checksum;
	u8 sub_oid;
	int ret;

	ret = firmware_request_nowarn(&hw_fw, hw_name, dev);
	if (ret < 0)
		return ret;
	ret = firmware_request_nowarn(&sw_fw, sw_name, dev);
	if (ret < 0)
		goto out_hw;

	merged_size = hw_fw->size + sw_fw->size;

	/*
	 * The stream is checksummed as 32-bit words and pushed in at most 256
	 * sections (the section index is a single byte); reject blobs that
	 * would silently break either.
	 */
	if (merged_size % 4 ||
	    merged_size > 256 * NCI_PROP_DUAL_SECTION_SIZE) {
		dev_err(dev, "invalid calibration data size (%zu)\n", merged_size);
		ret = -EINVAL;
		goto out;
	}

	merged = kmalloc(merged_size, GFP_KERNEL);
	if (!merged) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy(merged, hw_fw->data, hw_fw->size);
	memcpy(merged + hw_fw->size, sw_fw->data, sw_fw->size);

	/* Running sum of the merged stream as little-endian 32-bit words. */
	checksum = 0;
	for (i = 0; i + 4 <= merged_size; i += 4)
		checksum += get_unaligned_le32(merged + i);

	dev_dbg(dev, "calibration data update: %s + %s\n", hw_name, sw_name);

	/* START_UPDATE */
	sub_oid = NCI_PROP_DUAL_SUB_START_UPDATE;
	ret = nci_prop_cmd(info->ndev, NCI_PROP_DUAL_OPTION, 1, &sub_oid);
	if (ret < 0) {
		dev_err(dev, "Unable to start calibration data update\n");
		goto out;
	}

	/* SET_OPTION per section */
	set_option.sub_oid = NCI_PROP_DUAL_SUB_SET_OPTION;
	set_option.index = 0;
	for (i = 0; i < merged_size; i += NCI_PROP_DUAL_SECTION_SIZE) {
		len = min_t(size_t, merged_size - i, NCI_PROP_DUAL_SECTION_SIZE);
		memcpy(set_option.data, merged + i, len);
		ret = nci_prop_cmd(info->ndev, NCI_PROP_DUAL_OPTION,
				   len + 2, (__u8 *)&set_option);
		if (ret < 0) {
			dev_err(dev, "calibration data update error (code=%d)\n",
				ret);
			goto out;
		}
		set_option.index++;
	}

	/* STOP_UPDATE with checksum */
	stop_cmd[0] = NCI_PROP_DUAL_SUB_STOP_UPDATE;
	put_unaligned_le16(checksum, &stop_cmd[1]);
	ret = nci_prop_cmd(info->ndev, NCI_PROP_DUAL_OPTION, 3, stop_cmd);
	if (ret < 0) {
		dev_err(dev, "Unable to stop calibration data update\n");
		goto out;
	}

	dev_dbg(dev, "calibration data update: success\n");
out:
	kfree(merged);
	release_firmware(sw_fw);
out_hw:
	release_firmware(hw_fw);
	return ret;
}

/*
 * Ask the S3NRN4V which RF (hwreg) and SW (swreg) calibration versions it
 * holds. Stock's userspace sends this after CORE_INIT and uploads the files only
 * when their trailing version differs, so the chip keeps them between
 * power-ups. Read-only; the answer is logged raw for comparison with the
 * last 16 bytes of each file.
 */
void s3fwrn5_nci_dual_get_ver(struct s3fwrn5_info *info)
{
	struct device *dev = &info->ndev->nfc_dev->dev;
	u8 sub_oid = NCI_PROP_DUAL_SUB_GET_VER;
	int ret;

	info->dual_rsp_len = 0;
	ret = nci_prop_cmd(info->ndev, NCI_PROP_DUAL_OPTION, 1, &sub_oid);
	if (!info->dual_rsp_len) {
		dev_warn(dev, "calibration versions: no answer (%d)\n", ret);
		return;
	}
	dev_info(dev, "calibration versions in the chip (%u bytes): %*ph\n",
		 info->dual_rsp_len, info->dual_rsp_len, info->dual_rsp);
}

#include <linux/types.h>
#include <linux/kstrtox.h>

#include "ad9088.h"

static int ad9088_fsrc_apply(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	int ret;

	if (phy->trig_sync_en) {
		if (phy->trig_req_gpio) {
			// TODO: route trigger to the axi_fsrc_tx ip core as well.
			gpiod_set_value(phy->trig_req_gpio, 1);
			udelay(1);
			gpiod_set_value(phy->trig_req_gpio, 0);
		}

		ret = adi_apollo_hal_bf_wait_to_set(&phy->ad9088, BF_TRIGGER_SYNC_DONE_A0_INFO(MCS_SYNC_MCSTOP0), 1000000, 100);
		if (ret) {
			dev_err(&phy->spi->dev, "Error in adi_apollo_hal_bf_wait_to_set %d\n", ret);
			return ret;
		}
	}
	return adi_apollo_clk_mcs_man_reconfig_sync(&phy->ad9088);
}

ssize_t ad9088_ext_info_read_fsrc(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	u8 cddc_num, fddc_num, side;
	u32 cddc_mask, fddc_mask;
	long long val;

	guard(mutex)(&phy->lock);
	ad9088_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
				    &fddc_mask, &cddc_num, &cddc_mask, &side);

	switch (private) {
	case FSRC_N_M:
		return sprintf(buf, "%u %u\n",
			       chan->output ? phy->fsrc.tx_n : phy->fsrc.rx_n,
			       chan->output ? phy->fsrc.tx_m : phy->fsrc.rx_m);
	case FSRC_EN:
		return sprintf(buf, "%u\n",
			       chan->output ? phy->fsrc.tx_en : phy->fsrc.rx_en);
	default:
		return -EINVAL;
	}
	return sprintf(buf, "%lld\n", val);
}

ssize_t ad9088_ext_info_write_fsrc(struct iio_dev *indio_dev, uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	u8 dir = chan->output ? ADI_APOLLO_TX : ADI_APOLLO_RX;
	struct ad9088_phy *phy = conv->phy;
	u8 cddc_num, fddc_num, side;
	u32 cddc_mask, fddc_mask;
	u32 n, m, reg;
	bool enable;
	u16 links_;
	int ret;

	guard(mutex)(&phy->lock);
	ad9088_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
				    &fddc_mask, &cddc_num, &cddc_mask, &side);

	switch (private) {
	case FSRC_N_M:
		ret = sscanf(buf, "%d %d", &n, &m);
		if (ret != 2)
			return -EINVAL;

		if (n != m) {
			ret = adi_apollo_fsrc_ratio_set(&phy->ad9088, dir, ADI_APOLLO_FSRC_ALL, n, m);
			if (ret)
				ret = -EINVAL;
		}

		ret = adi_apollo_fsrc_mode_1x_enable_set(&phy->ad9088, dir, ADI_APOLLO_FSRC_ALL, (n == m));
		if (ret)
			return -EFAULT;

		if (chan->output) {
			phy->fsrc.tx_n = n;
			phy->fsrc.tx_m = m;
			if (phy->fsrc.tx_en) {
				ret = ad9088_fsrc_apply(indio_dev, chan);
				if (ret)
					return ret;
			}

			ret = iio_write_channel_ext_info(phy->iio_axi_fsrc, "tx_ratio_set", buf, len);
			if (ret != len) {
				dev_err(&phy->spi->dev, "Failed to set axi_fsrc tx ratio\n");
				return -EINVAL;
			}
		} else {
			phy->fsrc.rx_n = n;
			phy->fsrc.rx_m = m;
			if (phy->fsrc.rx_en) {
				ret = ad9088_fsrc_apply(indio_dev, chan);
				if (ret)
					return ret;
			}
		}
		break;
	case FSRC_EN:
		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;

		if (phy->iio_axi_fsrc) {
			char buf [] = "rx_enable";
			buf[0] += dir*2;
			ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc, buf, enable);
			if (ret < 0) {
				dev_err(&phy->spi->dev, "Failed to enable/disable axi_fsrc enable\n");
				return ret;
			}
		}
		ret = adi_apollo_clk_mcs_trig_reset_dsp_enable(&phy->ad9088);
		if (ret)
			return ret;

		links_ = ADI_APOLLO_LINK_ALL;
		for (int i = 0; i < ADI_APOLLO_NUM_JTX_LINKS; i++) {
		    if ((1 << i) & links_) {
		        reg = calc_jtx_dformat_base(i);
			if ((i % ADI_APOLLO_NUM_JTX_LINKS_PER_SIDE) == 0)
				ret = adi_apollo_hal_bf_set(&phy->ad9088, BF_INVALID_EN_0_INFO(reg), enable);
			else
				ret = adi_apollo_hal_bf_set(&phy->ad9088, BF_INVALID_EN_1_INFO(reg), enable);
			if (ret)
				return ret;
		    }
		}

		ret = adi_apollo_fsrc_en(&phy->ad9088, chan->output ? ADI_APOLLO_TX : ADI_APOLLO_RX,
				         ADI_APOLLO_FSRC_ALL, enable);
		if (ret)
			return ret;

		//ret = adi_apollo_fsrc_bypass(&phy->ad9088, chan->output ? ADI_APOLLO_TX : ADI_APOLLO_RX,
		//			     ADI_APOLLO_FSRC_ALL, !enable);
		//if (ret)
		//	return ret;

		ret = ad9088_fsrc_apply(indio_dev, chan);
		if (!phy->trig_sync_en && phy->iio_axi_fsrc) {
			ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
								"tx_active", enable);
			if (ret < 0) {
				dev_err(&phy->spi->dev, "Failed to set axi_fsrc tx_active\n");
				return ret;
			}
		}
		if (ret)
			return ret;

		ret = adi_apollo_clk_mcs_dyn_sync_sequence_run(&phy->ad9088);
		if (ret)
			return ret;
		adi_apollo_hal_delay_us(&phy->ad9088, 100);

		adi_apollo_jrx_rm_fifo_reset(&phy->ad9088, ADI_APOLLO_LINK_ALL);

		if (chan->output)
			phy->fsrc.tx_en = enable;
		else
			phy->fsrc.rx_en = enable;

		// clear irqs...
		break;
	default:
		return -EINVAL;
	}

	return len;
}

int ad9088_fsrc_setup(struct ad9088_phy *phy, adi_apollo_terminal_e terminal)
{
	adi_apollo_fsrc_sel_e fsrcs = ADI_APOLLO_FSRC_ALL;
	int ret;

	adi_apollo_fsrc_pgm_t config = {
		.sample_frac_delay = 0,
		.ptr_syncrstval = 0,
		.ptr_overwrite = 0,
		.fsrc_data_mult_dither_en = 0,
		.fsrc_dither_en = 1,
		.fsrc_4t4r_split = 1,
		.fsrc_bypass = 0,
		.fsrc_en = 0,
		.fsrc_1x_mode = 0,
	};

	ret = adi_api_utils_ratio_decomposition(1, 1, 48, &config.fsrc_rate_int, &config.fsrc_rate_frac_a, &config.fsrc_rate_frac_b);
	if (ret)
		return ret;

	config.gain_reduction = adi_api_utils_div_floor_u64(4096ull * (uint64_t)1, (uint64_t)1);

	ret = adi_apollo_fsrc_pgm(&phy->ad9088, terminal, fsrcs, &config);
	if (ret)
		return ret;

	return adi_apollo_clk_mcs_man_reconfig_sync(&phy->ad9088);
}

int ad9088_fsrc_probe(struct ad9088_phy *phy)
{
	phy->fsrc.rx_m = 1;
	phy->fsrc.tx_m = 1;
	phy->fsrc.rx_n = 1;
	phy->fsrc.tx_n = 1;
	phy->fsrc.rx_en = 0;
	phy->fsrc.tx_en = 0;

	return 0;
}

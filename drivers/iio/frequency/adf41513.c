// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADF41513 SPI PLL Frequency Synthesizer driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/frequency/adf41513.h>

enum {
	ADF41513_FREQ,
	ADF41513_POWER_DOWN,
	ADF41513_FREQ_RESOLUTION,
	ADF41513_FREQ_REFIN
};

enum {
	ADF41510_ID,
	ADF41513_ID,
};

enum adf41513_pll_mode {
	ADF41513_MODE_INTEGER_N,
	ADF41513_MODE_FIXED_MODULUS,
	ADF41513_MODE_VARIABLE_MODULUS,
	ADF41513_MODE_INVALID
};

struct adf41513_pll_settings {
	enum adf41513_pll_mode		mode;

	/* Frequency information */
	u64				target_frequency;
	u64				actual_frequency;
	u64				resolution;
	u32				pfd_frequency;

	/* PLL parameters */
	u16				int_value;
	u32				frac1;
	u32				frac2;
	u32				mod2;

	/* Reference path parameters */
	u8				r_counter;
	u8				ref_doubler;
	u8				ref_div2;
	u8				prescaler;
};

struct adf41513_state {
	struct spi_device		*spi;
	struct gpio_desc		*lock_detect;
	struct regulator		*reg;
	struct clk			*ref_clk;
	struct adf41513_platform_data	*pdata;

	struct clk_hw			clk_hw;
	struct clk			*clk_out;

	u32				device_id;
	u64				ref_freq;

	/*
	 * Lock for accessing device registers. Some operations require
	 * multiple consecutive R/W operations, during which the device
	 * shouldn't be interrupted. The buffers are also shared across
	 * all operations so need to be protected on stand alone reads and
	 * writes.
	 */
	struct mutex			lock;

	/* Cached register values */
	u32				regs[14];
	u32				regs_hw[14];

	/* PLL configuration */
	struct adf41513_pll_settings	settings;

	/*
	 * DMA (thus cache coherency maintenance) may require that
	 * transfer buffers live in their own cache lines.
	 */
	__be32				buf __aligned(IIO_DMA_MINALIGN);
};

#define to_adf41513_state(_hw) container_of(_hw, struct adf41513_state, clk_hw)

static const u32 adf41513_cp_voltage_mv[] = {
	810, 1620, 2430, 3240, 4050, 4860, 5670, 6480, 7290, 8100,
	8910, 9720, 10530, 11340, 12150, 12960
};

static int adf41513_sync_config(struct adf41513_state *st, u16 sync_mask)
{
	int ret;
	int i;

	/* Write registers in reverse order (R13 to R0) as per datasheet */
	for (i = ADF41513_REG13; i >= ADF41513_REG0; i--) {
		if (st->regs_hw[i] != st->regs[i] ||
		    sync_mask & (1 << i)) {
			st->buf = cpu_to_be32(st->regs[i] | i);
			ret = spi_write(st->spi, &st->buf, 4);
			if (ret < 0)
				return ret;
			st->regs_hw[i] = st->regs[i];
			dev_dbg(&st->spi->dev, "[%d] 0x%X\n",
				i, st->regs[i] | i);
		}
	}

	return 0;
}

static unsigned long adf41513_pll_get_rate(struct adf41513_state *st)
{
	u64 tmp;

	if (st->settings.mode == ADF41513_MODE_INVALID) {
		/* get PLL output frequency from regs_hw */
		st->settings.int_value = FIELD_GET(ADF41513_REG0_INT_MSK, st->regs_hw[ADF41513_REG0]);
		st->settings.frac1 = FIELD_GET(ADF41513_REG1_FRAC1_MSK, st->regs_hw[ADF41513_REG1]);
		st->settings.frac2 = FIELD_GET(ADF41513_REG3_FRAC2_MSK, st->regs_hw[ADF41513_REG3]);
		st->settings.mod2 = FIELD_GET(ADF41513_REG4_MOD2_MSK, st->regs_hw[ADF41513_REG4]);
		st->settings.r_counter = FIELD_GET(ADF41513_REG5_R_CNT_MSK, st->regs_hw[ADF41513_REG5]);
		st->settings.ref_doubler = FIELD_GET(ADF41513_REG5_REF_DOUBLER_MSK, st->regs_hw[ADF41513_REG5]);
		st->settings.ref_div2 = FIELD_GET(ADF41513_REG5_RDIV2_MSK, st->regs_hw[ADF41513_REG5]);
		st->settings.prescaler = FIELD_GET(ADF41513_REG5_PRESCALER_MSK, st->regs_hw[ADF41513_REG5]);

		/* calculate pfd frequency */
		st->settings.pfd_frequency = st->ref_freq;
		if (st->settings.ref_doubler)
			st->settings.pfd_frequency *= 2;
		if (st->settings.ref_div2)
			st->settings.pfd_frequency /= 2;
		st->settings.pfd_frequency /= st->settings.r_counter;

		/* check if int mode is selected */
		if (FIELD_GET(ADF41513_REG6_INT_MODE_MSK, st->regs_hw[ADF41513_REG6])) {
			st->settings.mode = ADF41513_MODE_INTEGER_N;
			st->settings.target_frequency = (u64)st->settings.int_value * st->settings.pfd_frequency;
			st->settings.resolution = st->settings.pfd_frequency;
		} else if (FIELD_GET(ADF41513_REG0_VAR_MOD_MSK, st->regs_hw[ADF41513_REG0])) {
			st->settings.mode = ADF41513_MODE_VARIABLE_MODULUS;
			tmp = (u64)st->settings.frac1 * st->settings.mod2 + st->settings.frac2;
			st->settings.target_frequency = (u64)st->settings.int_value * st->settings.pfd_frequency +
							div64_u64(tmp * st->settings.pfd_frequency,
							ADF41513_FIXED_MODULUS * st->settings.mod2);
			st->settings.resolution = div64_u64(st->settings.pfd_frequency, (u64)st->settings.mod2 * ADF41513_FIXED_MODULUS);
		} else {
			st->settings.mode = ADF41513_MODE_FIXED_MODULUS;
			tmp = (u64)st->settings.frac1;
			st->settings.target_frequency = (u64)st->settings.int_value * st->settings.pfd_frequency +
							div64_u64(tmp * st->settings.pfd_frequency,
							ADF41513_FIXED_MODULUS);
			st->settings.resolution = div64_u64(st->settings.pfd_frequency, ADF41513_FIXED_MODULUS);
		}

		st->settings.actual_frequency = st->settings.target_frequency;
	}

	return st->settings.actual_frequency;
}

static int adf41513_calc_pfd_frequency(struct adf41513_state *st,
				       struct adf41513_pll_settings *result,
				       bool is_fractional_mode)
{
	const u32 fpfd_limit = is_fractional_mode ? ADF41513_MAX_PFD_FREQ_FRAC_N :
					     ADF41513_MAX_PFD_FREQ_INT_N;

	/* set initial values from platform data */
	result->ref_div2 = st->pdata->ref_div2_en ? 1 : 0;
	result->ref_doubler = st->pdata->ref_doubler_en ? 1 : 0;

	if (st->pdata->ref_doubler_en &&
	    st->ref_freq > ADF41513_MAX_REF_FREQ_DOUBLER) {
		result->ref_doubler = 0;
		dev_warn(&st->spi->dev, "Disabling reference doubler due to high reference frequency\n");
	}

	/* set R counter starting with the div factor from platform data */
	result->r_counter = st->pdata->ref_div_factor - 1;

	do {
		result->r_counter++;
		result->pfd_frequency = (st->ref_freq *
					  (result->ref_doubler ? 2 : 1)) /
					  ((result->r_counter) *
					  (result->ref_div2 ? 2 : 1));
	} while (result->pfd_frequency >= fpfd_limit);

	if (result->r_counter > ADF41513_MAX_R_CNT) {
		dev_err(&st->spi->dev, "Cannot optimize PFD frequency\n");
		return -ERANGE;
	}

	return 0;
}

static int adf41513_calc_integer_n(struct adf41513_state *st,
				   struct adf41513_pll_settings *result)
{
	const u64 n_total = div64_u64(result->target_frequency, result->pfd_frequency);
	const u16 max_int = (st->device_id == ADF41513_ID) ?
			    ADF41513_MAX_INT_8_9 : ADF41513_MAX_INT_4_5;

	u64 fractional_part, freq_error;
	u16 int_value = (u16)n_total;

	/* check if N is close to an integer (within tolerance) */
	fractional_part = result->target_frequency - n_total * result->pfd_frequency;
	if (fractional_part > result->pfd_frequency / 2 && int_value < max_int)
		int_value++;

	/* set prescaler */
	if (st->device_id == ADF41513_ID &&
	    int_value >= ADF41513_MIN_INT_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_4_5 &&
		 int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	/* Check if frequency is close enough to integer multiple */
	result->actual_frequency = (u64)int_value * result->pfd_frequency;
	freq_error = (result->actual_frequency > result->target_frequency) ?
		     (result->actual_frequency - result->target_frequency) :
		     (result->target_frequency - result->actual_frequency);

	if (freq_error > st->pdata->target_resolution_hz)
		return -ERANGE;

	result->mode = ADF41513_MODE_INTEGER_N;
	result->int_value = int_value;
	result->frac1 = 0;
	result->frac2 = 0;
	result->mod2 = 0;
	result->resolution = result->pfd_frequency;

	return 0;
}

static int adf41513_calc_fixed_modulus(struct adf41513_state *st,
				       struct adf41513_pll_settings *result)
{
	const u64 n_total = div64_u64(result->target_frequency, result->pfd_frequency);

	u64 fractional_part, freq_error;
	u16 int_value = (u16)n_total;
	u32 frac1;

	/* set prescaler */
	if (st->device_id == ADF41513_ID &&
	    int_value >= ADF41513_MIN_INT_FRAC_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_FRAC_4_5 &&
		 int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	/* calculate fractional part */
	fractional_part = result->target_frequency - (u64)int_value * result->pfd_frequency;
	frac1 = (u32)div64_u64(fractional_part * ADF41513_FIXED_MODULUS, result->pfd_frequency);

	if (frac1 > ADF41513_FIXED_MODULUS - 1)
		frac1 = ADF41513_FIXED_MODULUS - 1;

	/* calculate actual frequency and error */
	result->actual_frequency = (u64)int_value * result->pfd_frequency +
				   div64_u64((u64)frac1 * result->pfd_frequency,
					     ADF41513_FIXED_MODULUS);
	freq_error = (result->actual_frequency > result->target_frequency) ?
		     (result->actual_frequency - result->target_frequency) :
		     (result->target_frequency - result->actual_frequency);

	if (freq_error > st->pdata->target_resolution_hz)
		return -ERANGE;

	result->resolution = div64_u64(result->pfd_frequency, ADF41513_FIXED_MODULUS);

	result->mode = ADF41513_MODE_FIXED_MODULUS;
	result->int_value = int_value;
	result->frac1 = frac1;
	result->frac2 = 0;
	result->mod2 = 0;

	return 0;
}

static int adf41513_calc_variable_modulus(struct adf41513_state *st,
					  struct adf41513_pll_settings *result)
{
	const u64 n_total = div64_u64(result->target_frequency, result->pfd_frequency);

	u64 fractional_part, tmp, remaining_frac, freq_error;
	u16 int_value = (u16)n_total;
	u32 frac1, frac2, mod2;

	/* set prescaler */
	if (st->device_id == ADF41513_ID &&
	    int_value >= ADF41513_MIN_INT_FRAC_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_FRAC_4_5 &&
		 int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	/* calculate required MOD2 based on target resolution / 2 */
	mod2 = (u32)min_t(u64, div64_u64((u64)result->pfd_frequency * 2,
					  st->pdata->target_resolution_hz * ADF41513_FIXED_MODULUS),
					  ADF41513_MAX_MOD2);

	/* ensure MOD2 is at least 2 for meaningful operation */
	if (mod2 < 2)
		mod2 = 2;

	/* calculate fractional part */
	fractional_part = result->target_frequency - (u64)int_value * result->pfd_frequency;
	tmp = fractional_part * ADF41513_FIXED_MODULUS;
	frac1 = (u32)div64_u64(tmp, result->pfd_frequency);
	remaining_frac = tmp - (u64)frac1 * result->pfd_frequency;

	frac2 = (u32)div64_u64(remaining_frac * mod2, result->pfd_frequency);

	/* ensure values are within limits */
	if (frac1 > ADF41513_FIXED_MODULUS - 1) {
		frac1 = ADF41513_FIXED_MODULUS - 1;
		frac2 = 0;
	}

	if (frac2 >= mod2)
		frac2 = mod2 - 1;

	/* calculate actual frequency and error */
	tmp = (u64)frac1 * mod2 + frac2;
	result->actual_frequency = (u64)int_value * result->pfd_frequency +
				   div64_u64(tmp * result->pfd_frequency,
					     ADF41513_FIXED_MODULUS * mod2);
	freq_error = (result->actual_frequency > result->target_frequency) ?
		     (result->actual_frequency - result->target_frequency) :
		     (result->target_frequency - result->actual_frequency);

	if (freq_error > st->pdata->target_resolution_hz)
		return -ERANGE;

	result->resolution = div64_u64(result->pfd_frequency,
				       ADF41513_FIXED_MODULUS * mod2);

	result->mode = ADF41513_MODE_VARIABLE_MODULUS;
	result->int_value = int_value;
	result->frac1 = frac1;
	result->frac2 = frac2;
	result->mod2 = mod2;

	return 0;
}

static int adf41513_calculate_pll_settings(struct adf41513_state *st,
					   struct adf41513_pll_settings *result,
					   u64 rf_out)
{
	const u64 max_rf_freq = (st->device_id == ADF41513_ID) ?
				ADF41513_MAX_RF_FREQ : ADF41510_MAX_RF_FREQ;
	int ret;

	result->target_frequency = rf_out;

	/* input validation */
	if (rf_out < ADF41513_MIN_RF_FREQ ||
	    rf_out > max_rf_freq) {
		dev_err(&st->spi->dev, "RF frequency %llu Hz out of range [%llu, %llu] Hz\n",
			rf_out, ADF41513_MIN_RF_FREQ, max_rf_freq);
		return -EINVAL;
	}

	/* try integer-N first (best phase noise performance) */
	ret = adf41513_calc_pfd_frequency(st, result, false);
	if (ret < 0)
		return ret;

	if (adf41513_calc_integer_n(st, result) < 0) {
		/* try fractional-N: recompute pfd frequency */
		ret = adf41513_calc_pfd_frequency(st, result, true);
		if (ret < 0)
			return ret;

		if (adf41513_calc_fixed_modulus(st, result) < 0 &&
		    adf41513_calc_variable_modulus(st, result) < 0) {
			dev_err(&st->spi->dev, "no valid PLL configuration found for %llu Hz\n", rf_out);
			return -EINVAL;
		}
	}

	return 0;
}

static int adf41513_set_frequency(struct adf41513_state *st, u64 freq, u16 sync_mask)
{
	struct adf41513_pll_settings result;
	u32 bleed_value = 0;
	int ret;

	/* calculate pll settings candidate */
	ret = adf41513_calculate_pll_settings(st, &result, freq);
	if (ret < 0)
		return ret;

	/* apply computed results to state pll settings */
	memcpy(&st->settings, &result, sizeof(struct adf41513_pll_settings));

	/* log calculation result */
	dev_dbg(&st->spi->dev, "%s mode: int=%u, frac1=%u, mod2=%u, fpdf=%u Hz, prescaler=%s\n",
		(result.mode == ADF41513_MODE_INTEGER_N) ? "integer-n" :
		(result.mode == ADF41513_MODE_FIXED_MODULUS) ? "fixed-modulus" : "variable-modulus",
		result.int_value, result.frac1, result.mod2, result.pfd_frequency,
		result.prescaler ? "8/9" : "4/5");

	/* int */
	st->regs[ADF41513_REG0] = ADF41513_REG0_INT(st->settings.int_value);
	if (st->settings.mode == ADF41513_MODE_VARIABLE_MODULUS)
		st->regs[ADF41513_REG0] |= ADF41513_REG0_VAR_MOD_MSK;
	/* frac1 */
	st->regs[ADF41513_REG1] = ADF41513_REG1_FRAC1(st->settings.frac1);
	if (st->settings.mode != ADF41513_MODE_INTEGER_N)
		st->regs[ADF41513_REG1] |= ADF41513_REG1_DITHER2_MSK;

	/* frac2 */
	st->regs[ADF41513_REG3] = ADF41513_REG3_FRAC2(st->settings.frac2);
	/* mod2 */
	st->regs[ADF41513_REG4] &= ADF41513_REG4_MOD2_MSK;
	st->regs[ADF41513_REG4] |= ADF41513_REG4_MOD2(st->settings.mod2);

	/* r-cnt | doubler | rdiv2 | prescaler */
	st->regs[ADF41513_REG5] &= ~(ADF41513_REG5_R_CNT_MSK |
				     ADF41513_REG5_REF_DOUBLER_MSK |
				     ADF41513_REG5_RDIV2_MSK |
				     ADF41513_REG5_PRESCALER_MSK);
	st->regs[ADF41513_REG5] |= ADF41513_REG5_R_CNT(st->settings.r_counter % 32) |
				   ADF41513_REG5_REF_DOUBLER(st->settings.ref_doubler) |
				   ADF41513_REG5_RDIV2(st->settings.ref_div2) |
				   ADF41513_REG5_PRESCALER(st->settings.prescaler);

	/* Enable integer mode if no fractional part */
	if (st->settings.frac1 == 0 &&
	    st->settings.frac2 == 0) {
		/* integer mode */
		st->regs[ADF41513_REG6] |= ADF41513_REG6_INT_MODE_MSK;
		st->regs[ADF41513_REG6] &= ~ADF41513_REG6_BLEED_ENABLE_MSK;
	} else {
		/* fractional mode */
		st->regs[ADF41513_REG6] &= ~ADF41513_REG6_INT_MODE_MSK;
		st->regs[ADF41513_REG6] |= ADF41513_REG6_BLEED_ENABLE_MSK;
	}

	/* set bleed current value */
	if (st->pdata->phase_detector_polarity)
		bleed_value = 90;
	else
		bleed_value = 144;

	bleed_value *= (((st->settings.pfd_frequency / 1000) *
			 (FIELD_GET(ADF41513_REG5_CP_CURRENT_MSK, st->regs[ADF41513_REG5]) + 1))
			 / 1600000);

	st->regs[ADF41513_REG6] &= ~ADF41513_REG6_BLEED_CURRENT_MSK;
	st->regs[ADF41513_REG6] |= ADF41513_REG6_BLEED_CURRENT(bleed_value);

	return adf41513_sync_config(st, sync_mask | ADF41513_SYNC_REG0);
}

static ssize_t adf41513_read(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     char *buf)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	u64 val = 0;
	int ret = 0;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADF41513_FREQ:
		val = adf41513_pll_get_rate(st);
		if (st->lock_detect)
			if (!gpiod_get_value(st->lock_detect)) {
				dev_dbg(&st->spi->dev, "PLL un-locked\n");
				ret = -EBUSY;
			}
		break;
	case ADF41513_FREQ_REFIN:
		if (st->ref_clk)
			st->ref_freq = clk_get_rate(st->ref_clk);

		val = st->ref_freq;
		break;
	case ADF41513_FREQ_RESOLUTION:
		val = st->pdata->target_resolution_hz;
		break;
	case ADF41513_POWER_DOWN:
		val = (u64)FIELD_GET(ADF41513_REG6_POWER_DOWN_MSK, st->regs_hw[ADF41513_REG6]);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

static ssize_t adf41513_write(struct iio_dev *indio_dev,
			      uintptr_t private,
			      const struct iio_chan_spec *chan,
			      const char *buf, size_t len)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	unsigned long tmp;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADF41513_FREQ:
		ret = adf41513_set_frequency(st, readin, false);
		break;
	case ADF41513_FREQ_REFIN:
		if (readin < ADF41513_MIN_REF_FREQ || readin > ADF41513_MAX_REF_FREQ) {
			ret = -EINVAL;
			break;
		}

		if (st->ref_clk) {
			tmp = clk_round_rate(st->ref_clk, readin);
			if (tmp != readin) {
				ret = -EINVAL;
				break;
			}
			ret = clk_set_rate(st->ref_clk, tmp);
			if (ret < 0)
				break;
		}
		st->ref_freq = readin;
		/* update RF out */
		ret = adf41513_set_frequency(st, st->settings.target_frequency, false);
		break;
	case ADF41513_FREQ_RESOLUTION:
		if (readin == 0)
			ret = -EINVAL;
		else
			st->pdata->target_resolution_hz = readin;
		break;
	case ADF41513_POWER_DOWN:
		if (readin)
			st->regs[ADF41513_REG6] |= ADF41513_REG6_POWER_DOWN_MSK;
		else
			st->regs[ADF41513_REG6] &= ~ADF41513_REG6_POWER_DOWN_MSK;

		if (st->regs[ADF41513_REG6] != st->regs_hw[ADF41513_REG6])
			ret = adf41513_sync_config(st, ADF41513_SYNC_DIFF);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

#define _ADF41513_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = adf41513_read, \
	.write = adf41513_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info adf41513_ext_info[] = {
	/*
	 * Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 */
	_ADF41513_EXT_INFO("frequency", ADF41513_FREQ),
	_ADF41513_EXT_INFO("frequency_resolution", ADF41513_FREQ_RESOLUTION),
	_ADF41513_EXT_INFO("refin_frequency", ADF41513_FREQ_REFIN),
	_ADF41513_EXT_INFO("powerdown", ADF41513_POWER_DOWN),
	{ },
};

static const struct iio_chan_spec adf41513_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.ext_info = adf41513_ext_info,
};

static int adf41513_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg,
			       unsigned int writeval,
			       unsigned int *readval)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	int ret;

	if (reg > ADF41513_REG13)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (!readval) {
		/* direct register access invalidates cached pll settings */
		st->settings.mode = ADF41513_MODE_INVALID;

		st->regs[reg] = writeval & ~0xF; /* Clear control bits */
		ret = adf41513_sync_config(st, 1 << reg);
	} else {
		*readval = st->regs_hw[reg];
		ret = 0;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_info adf41513_info = {
	.debugfs_reg_access = &adf41513_reg_access,
};

static void adf41513_clk_del_provider(void *data)
{
	struct adf41513_state *st = data;

	of_clk_del_provider(st->spi->dev.of_node);
}

static unsigned long adf41513_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct adf41513_state *st = to_adf41513_state(hw);
	unsigned long tmp;

	mutex_lock(&st->lock);
	tmp = adf41513_pll_get_rate(st);
	mutex_unlock(&st->lock);

	return tmp;
}

static long adf41513_clk_round_rate(struct clk_hw *hw,
				    unsigned long rate,
				    unsigned long *parent_rate)
{
	return rate;
}

static int adf41513_clk_set_rate(struct clk_hw *hw,
				 unsigned long rate,
				 unsigned long parent_rate)
{
	struct adf41513_state *st = to_adf41513_state(hw);
	int ret;

	if (parent_rate < ADF41513_MIN_REF_FREQ ||
	    parent_rate > ADF41513_MAX_REF_FREQ)
		return -EINVAL;

	mutex_lock(&st->lock);
	st->ref_freq = parent_rate;
	ret = adf41513_set_frequency(st, rate, false);
	mutex_unlock(&st->lock);

	return ret;
}

static int adf41513_clk_prepare(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);
	int ret;

	/* power up */
	mutex_lock(&st->lock);
	st->regs[ADF41513_REG6] &= ~ADF41513_REG6_POWER_DOWN_MSK;
	ret = adf41513_sync_config(st, ADF41513_SYNC_DIFF);
	mutex_unlock(&st->lock);

	return ret;
}

static void adf41513_clk_unprepare(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	/* power down */
	mutex_lock(&st->lock);
	st->regs[ADF41513_REG6] |= ADF41513_REG6_POWER_DOWN_MSK;
	adf41513_sync_config(st, ADF41513_SYNC_DIFF);
	mutex_unlock(&st->lock);
}

static int adf41513_clk_is_enabled(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);
	int ret;

	mutex_lock(&st->lock);
	ret = (st->regs_hw[ADF41513_REG6] & ADF41513_REG6_POWER_DOWN_MSK) == 0;
	mutex_unlock(&st->lock);
	return ret;
}

static const struct clk_ops adf41513_clk_ops = {
	.recalc_rate = adf41513_clk_recalc_rate,
	.round_rate = adf41513_clk_round_rate,
	.set_rate = adf41513_clk_set_rate,
	.prepare = adf41513_clk_prepare,
	.unprepare = adf41513_clk_unprepare,
	.is_enabled = adf41513_clk_is_enabled,
};

static int adf41513_clk_register(struct adf41513_state *st)
{
	struct spi_device *spi = st->spi;
	struct clk_init_data init;
	struct clk *clk = NULL;
	const char *parent_name;
	int ret;

	if (!device_property_present(&spi->dev, "#clock-cells"))
		return 0;

	if (device_property_read_string(&spi->dev, "clock-output-names", &init.name)) {
		init.name = devm_kasprintf(&spi->dev, GFP_KERNEL, "%s-clk",
					   fwnode_get_name(dev_fwnode(&spi->dev)));
		if (!init.name)
			return -ENOMEM;
	}

	parent_name = of_clk_get_parent_name(spi->dev.of_node, 0);
	if (!parent_name)
		return -EINVAL;

	init.ops = &adf41513_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_PARENT;

	st->clk_hw.init = &init;
	clk = devm_clk_register(&spi->dev, &st->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk);
	if (ret)
		return ret;

	st->clk_out = clk;

	return devm_add_action_or_reset(&spi->dev, adf41513_clk_del_provider, st);
}

static struct adf41513_platform_data *adf41513_parse_dt(struct device *dev)
{
	struct adf41513_platform_data *pdata;
	u32 tmp;
	u32 cp_resistance;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	snprintf(pdata->name, sizeof(pdata->name), "%pfw", dev_fwnode(dev));

	/* power-up frequency - optional */
	pdata->power_up_frequency = 0;
	device_property_read_u64(dev, "adi,power-up-frequency",
				 &pdata->power_up_frequency);

	/* reference divider factor - optional minimal value */
	if (!device_property_read_u32(dev, "adi,reference-div-factor", &tmp)) {
		if (tmp >= ADF41513_MIN_R_CNT && tmp <= ADF41513_MAX_R_CNT) {
			pdata->ref_div_factor = tmp;
		} else {
			dev_warn(dev, "Invalid reference div factor %u\n", tmp);
			pdata->ref_div_factor = ADF41513_MIN_R_CNT;
		}
	} else {
		pdata->ref_div_factor = ADF41513_MIN_R_CNT; /* Default R = 1 */
	}

	/* reference controls */
	pdata->ref_doubler_en = device_property_read_bool(dev,
							  "adi,reference-doubler-enable");
	pdata->ref_div2_en = device_property_read_bool(dev,
						       "adi,reference-div2-enable");

	/* charge pump resistor */
	cp_resistance = 2700; /* Default 2.7 kOhms */
	if (!device_property_read_u32(dev, "adi,charge-pump-resistor", &tmp)) {
		if (tmp < 1800) {
			dev_warn(dev, "RSET value too low %u Ohms\n", tmp);
			tmp = 1800;
		} else if (tmp > 10000) {
			dev_warn(dev, "RSET value too high %u Ohms\n", tmp);
			tmp = 10000;
		}
		cp_resistance = tmp;
	}

	/* charge pump current */
	if (!device_property_read_u32(dev, "adi,charge-pump-current", &tmp)) {
		tmp = (tmp * cp_resistance) / 1000; /* Convert to mV */
		 /* Validate charge pump voltage */
		if (tmp < ADF41513_MIN_CP_VOLTAGE_MV) {
			dev_warn(dev, "Charge pump current too low %u mV\n", tmp);
			tmp = ADF41513_MIN_CP_VOLTAGE_MV;
		} else if (tmp > ADF41513_MAX_CP_VOLTAGE_MV) {
			dev_warn(dev, "Charge pump current too high %u mV\n", tmp);
			tmp = ADF41513_MAX_CP_VOLTAGE_MV;
		}
		pdata->charge_pump_voltage_mv = tmp;
	} else {
		pdata->charge_pump_voltage_mv = 6480; /* Default 2.7 kOhm * 2.4 mA = 6.48 V */
	}

	/* phase detector polarity */
	pdata->phase_detector_polarity = device_property_read_bool(dev,
		"adi,phase-detector-polarity-positive");

	/* muxout selection */
	pdata->muxout_select = ADF41513_MUXOUT_TRISTATE; /* Default to tristate */
	if (!device_property_read_u32(dev, "adi,muxout-select", &tmp)) {
		if (tmp <= ADF41513_MUXOUT_N_DIV2)
			pdata->muxout_select = tmp;
		else
			dev_warn(dev, "Invalid muxout select %u\n", tmp);
	}

	/* muxout logic level */
	pdata->mux_out_3V3_en = device_property_read_bool(dev, "adi,muxout-level-3v3-enable");

	/* lock detector settings */
	if (!device_property_read_u32(dev, "adi,lock-detect-precision", &tmp)) {
		if (tmp > 3)
			tmp = 3;
		pdata->lock_detect_precision = tmp;
	} else {
		pdata->lock_detect_precision = 0; /* Default */
	}

	if (!device_property_read_u32(dev, "adi,lock-detect-count", &tmp)) {
		if (tmp < 2) {
			dev_warn(dev, "Lock detect count too low %u\n", tmp);
			tmp = 2;
		} else if (tmp > 8192) {
			dev_warn(dev, "Lock detect count too high %u\n", tmp);
			tmp = 8192;
		}
		pdata->lock_detect_count = tmp;
	} else {
		pdata->lock_detect_count = 1024; /* Default */
	}

	/* Target resolution for algorithm */
	pdata->target_resolution_hz = 1; /* Default to 1 Hz */
	device_property_read_u64(dev, "adi,target-resolution-hz", &pdata->target_resolution_hz);
	if (pdata->target_resolution_hz == 0)
		pdata->target_resolution_hz = 1;

	return pdata;
}

static int adf41513_setup(struct adf41513_state *st)
{
	int ret;
	u32 cp_index;

	memset(st->regs_hw, 0xFF, sizeof(st->regs_hw));
	memset(st->regs, 0x00, sizeof(st->regs));

	/* assume DLD pin is used for digital lock detect */
	st->regs[ADF41513_REG5] |= ADF41513_REG5_DLD_MODES(ADF41513_DLD_DIG_LD);

	/* configure charge pump current settings */
	cp_index = find_closest(st->pdata->charge_pump_voltage_mv,
				adf41513_cp_voltage_mv,
				ARRAY_SIZE(adf41513_cp_voltage_mv));
	st->regs[ADF41513_REG5] |= ADF41513_REG5_CP_CURRENT(cp_index);

	/* Configure phase detector polarity */
	if (st->pdata->phase_detector_polarity)
		st->regs[ADF41513_REG6] |= ADF41513_REG6_PD_POLARITY_MSK;

	/* narrow ABP | loss of lock detect enable | SD reset | LDP from pdata */
	st->regs[ADF41513_REG6] |= ADF41513_REG6_ABP_MSK |
				   ADF41513_REG6_LOL_ENABLE_MSK |
				   ADF41513_REG6_SD_RESET_MSK |
				   ADF41513_REG6_LDP(st->pdata->lock_detect_precision);

	/* LD count from pdata | PS bias */
	st->regs[ADF41513_REG7] |= ADF41513_REG7_LD_COUNT(st->pdata->lock_detect_count) |
				   ADF41513_REG7_PS_BIAS(2);

	if (st->pdata->phase_resync_clk1_div != 0 &&
	    st->pdata->phase_resync_clk2_div != 0) {
		/* enable phase resync and configure the clk divs */
		st->regs[ADF41513_REG5] |= ADF41513_REG5_CLK1_DIV(st->pdata->phase_resync_clk1_div);
		st->regs[ADF41513_REG7] |= ADF41513_REG7_CLK_DIV_MODE(2) |
					   ADF41513_REG7_CLK2_DIV(st->pdata->phase_resync_clk2_div);
	}

	/* lock detect bias */
	st->regs[ADF41513_REG9] |= ADF41513_REG9_LD_BIAS(st->pdata->lock_detect_bias);

	/* power down select */
	st->regs[ADF41513_REG11] |= ADF41513_REG11_POWER_DOWN_SEL_MSK;

	/* muxout */
	st->regs[ADF41513_REG12] |= ADF41513_REG12_MUXOUT(st->pdata->muxout_select) |
				    ADF41513_REG12_LOGIC_LEVEL(st->pdata->mux_out_3V3_en);

	/* set power-up frequency if specified */
	if (st->pdata->power_up_frequency) {
		ret = adf41513_set_frequency(st, st->pdata->power_up_frequency, ADF41513_SYNC_ALL);
		if (ret < 0) {
			dev_err(&st->spi->dev, "Failed to set power-up frequency: %d\n", ret);
			return ret;
		}
	} else {
		/* powered down */
		st->regs[ADF41513_REG6] |= ADF41513_REG6_POWER_DOWN(1);

		/* write initial configuration */
		ret = adf41513_sync_config(st, ADF41513_SYNC_ALL);
		if (ret < 0) {
			dev_err(&st->spi->dev, "Failed to write initial config: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static void adf41513_power_down(void *data)
{
	struct adf41513_state *st = data;

	mutex_lock(&st->lock);
	st->regs[ADF41513_REG6] |= ADF41513_REG6_POWER_DOWN(1);
	adf41513_sync_config(st, ADF41513_SYNC_DIFF);
	mutex_unlock(&st->lock);
}

static int adf41513_probe(struct spi_device *spi)
{
	struct adf41513_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct adf41513_state *st;
	struct clk *ref_clk = NULL;
	int ret;

	/* Parse device tree or platform data */
	if (dev_fwnode(&spi->dev)) {
		pdata = adf41513_parse_dt(&spi->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	} else {
		pdata = spi->dev.platform_data;
		if (!pdata) {
			dev_err(&spi->dev, "No platform data provided\n");
			return -EINVAL;
		}
	}

	/* Get reference clock */
	if (!pdata->clkin) {
		ref_clk = devm_clk_get(&spi->dev, "clkin");
		if (IS_ERR(ref_clk))
			return -EPROBE_DEFER;

		ret = clk_prepare_enable(ref_clk);
		if (ret < 0)
			return ret;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		ret = -ENOMEM;
		goto error_disable_clk;
	}

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->pdata = pdata;
	st->ref_clk = ref_clk;
	st->device_id = (u32)spi_get_device_id(spi)->driver_data;

	spi_set_drvdata(spi, indio_dev);

	/* vcc regulator */
	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			goto error_disable_clk;
	}

	/* get lock detect gpio */
	st->lock_detect = devm_gpiod_get_optional(&spi->dev, NULL, GPIOD_IN);
	if (IS_ERR(st->lock_detect)) {
		dev_err(&spi->dev, "fail to request lock detect GPIO\n");
		ret = PTR_ERR(st->lock_detect);
		goto error_disable_reg;
	}

	/* set reference frequency */
	if (st->ref_clk)
		st->ref_freq = clk_get_rate(st->ref_clk);
	else
		st->ref_freq = pdata->clkin;

	/* validate reference frequency */
	if (st->ref_freq < ADF41513_MIN_REF_FREQ ||
	    st->ref_freq > ADF41513_MAX_REF_FREQ) {
		dev_err(&spi->dev, "Reference frequency %llu Hz out of range\n",
			st->ref_freq);
		ret = -EINVAL;
		goto error_disable_reg;
	}

	mutex_init(&st->lock);

	ret = adf41513_clk_register(st);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register clock: %d\n", ret);
		goto error_disable_reg;
	}

	/* configure IIO device */
	indio_dev->name = pdata->name[0] ? pdata->name : "adf41513";
	indio_dev->info = &adf41513_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &adf41513_chan;
	indio_dev->num_channels = 1;

	/* initialize device */
	ret = adf41513_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "Device setup failed: %d\n", ret);
		goto error_disable_reg;
	}

	/* add power down action */
	ret = devm_add_action_or_reset(&spi->dev, adf41513_power_down, st);
	if (ret) {
		dev_err(&spi->dev, "Failed to add power down action: %d\n", ret);
		goto error_disable_reg;
	}

	/* register IIO device */
	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register IIO device: %d\n", ret);
		goto error_disable_reg;
	}

	dev_info(&spi->dev, "ADF41513 PLL synthesizer registered\n");

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
error_disable_clk:
	if (ref_clk)
		clk_disable_unprepare(ref_clk);

	return ret;
}

static const struct of_device_id adf41513_of_match[] = {
	{ .compatible = "adi,adf41510" },
	{ .compatible = "adi,adf41513" },
	{ }
};
MODULE_DEVICE_TABLE(of, adf41513_of_match);

static const struct spi_device_id adf41513_id[] = {
	{"adf41510", ADF41510_ID},
	{"adf41513", ADF41513_ID},
	{}
};
MODULE_DEVICE_TABLE(spi, adf41513_id);

static struct spi_driver adf41513_driver = {
	.driver = {
		.name = "adf41513",
		.of_match_table = adf41513_of_match,
	},
	.probe = adf41513_probe,
	.id_table = adf41513_id,
};
module_spi_driver(adf41513_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF41513 PLL Frequency Synthesizer");
MODULE_LICENSE("GPL");

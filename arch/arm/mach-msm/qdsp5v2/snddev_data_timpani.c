/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/mfd/msm-adie-codec.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/board.h>
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/snddev_mi2s.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_acdb_def.h>
#include <mach/qdsp5v2/snddev_virtual.h>
#include "timpani_profile_7x30.h"
#include <mach/qdsp5v2/audio_dev_ctl.h>

/* define the value for BT_SCO */
#define BT_SCO_PCM_CTL_VAL (PCM_CTL__RPCM_WIDTH__LINEAR_V |\
		PCM_CTL__TPCM_WIDTH__LINEAR_V)
#define BT_SCO_DATA_FORMAT_PADDING (DATA_FORMAT_PADDING_INFO__RPCM_FORMAT_V |\
		DATA_FORMAT_PADDING_INFO__TPCM_FORMAT_V)
#define BT_SCO_AUX_CODEC_INTF   AUX_CODEC_INTF_CTL__PCMINTF_DATA_EN_V

/* FUJITSU:2011-12-01 timpani start */
/* FUJITSU:2011-12-01 speaker amp start */
void request_enable_spaker_amp(void);
void request_disable_spaker_amp(void);
/* FUJITSU:2011-12-01 camera sound start */
void request_enable_spaker_amp2(void);
void request_disable_spaker_amp2(void);
/* FUJITSU:2011-12-01 camera sound end */
void request_enable_earspaker_amp(void);
void request_disable_earspaker_amp(void);
/* FUJITSU:2011-12-01 speaker amp end */
/* FUJITSU:2011-12-01 notify earpiece start */
#include "../smd_private.h"
void start_earpiece(void);
void stop_earpiece(void);
/* FUJITSU:2011-12-01 notify earpiece end */

/* FUJITSU:2012-03-12 notify earamp start */
void request_enable_earamp(void);
void request_disable_earamp(void);
void start_ear_amp(void);
void stop_ear_amp(void);
/* FUJITSU:2012-03-12 notify earamp end */

/* FUJITSU:2012-05-01 bigVoice start */
#if defined(CONFIG_MACH_F12NAD)
static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions[] =
	EAR2_PRI_MONO_8000_OSR_256;
/*FUJITSU:2012-06-01 nad separate by prototype start */
static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions2[] =
	EAR3_PRI_MONO_8000_OSR_256;
/*FUJITSU:2012-06-01 nad separate by prototype end */
#else
/* FUJITSU:2012-05-01 bigVoice end */
static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions[] =
	EAR_PRI_MONO_8000_OSR_256;
#endif /* FUJITSU:2012-05-01 bigVoice add */

static struct adie_codec_hwsetting_entry iearpiece_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_settings),
};

static struct snddev_icodec_data snddev_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#if 0 /* FUJITSU:2011-12-01 notify earpiece */
//	.pamp_on = NULL,
//	.pamp_off = NULL,
#else
	.pamp_on = start_earpiece,
	.pamp_off = stop_earpiece,
#endif
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearpiece_device = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data },
};

/* FUJITSU:2012-03-12 notify earamp start */
static struct snddev_icodec_data snddev_iearpiece_data2 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = start_ear_amp,
	.pamp_off = stop_ear_amp,
	.property = SIDE_TONE_MASK,
/* FUJITSU:2012-04-17 bigVoice start */
/*	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
*/
	.max_voice_rx_vol[VOC_NB_INDEX] = 0,
	.min_voice_rx_vol[VOC_NB_INDEX] = 0,
	.max_voice_rx_vol[VOC_WB_INDEX] = 0,
	.min_voice_rx_vol[VOC_WB_INDEX] = 0
/* FUJITSU:2012-04-17 bigVoice end */
};

static struct platform_device msm_iearpiece_device2 = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data2 },
};
/* FUJITSU:2012-03-12 notify earamp end */

static struct adie_codec_action_unit imic_48KHz_osr256_actions[] =
	AMIC_PRI_MONO_8000_OSR_256;

static struct adie_codec_hwsetting_entry imic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings,
	.setting_sz = ARRAY_SIZE(imic_settings),
};

static enum hsed_controller imic_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_device = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data },
};

static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_AB_CPLS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};

static struct platform_device msm_ihs_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data },
};

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions[] =
	AMIC1_HEADSET_TX_MONO_PRIMARY_OSR256;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings),
};

static enum hsed_controller ispk_pmctl_id1[] = {PM_HSED_CONTROLLER_1}; /* FUJITSU:2011-12-01 HSED_BIAS1 setting */

static struct snddev_icodec_data snddev_ihs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile,
	.channel_mode = 1,
/*	.pmctl_id = NULL,
	.pmctl_id_sz = 0, FUJITSU:2011-12-01 DEL */
/* FUJITSU:2011-12-01 HSED_BIAS1 setting start */
	.pmctl_id = ispk_pmctl_id1,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id1),
/* FUJITSU:2011-12-01 HSED_BIAS1 setting end */
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data },
};

static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions[] =
/*   SPEAKER_STEREO_RX_48000_OSR_256; FUJITSU:2011-12-01 DEL */
   SPEAKER_PRI_STEREO_48000_OSR_256; /* FUJITSU:2011-12-01 use mono */

static struct adie_codec_hwsetting_entry ispeaker_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
/*	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO, FUJITSU:2011-12-01 DEL */
	.acdb_id = ACDB_ID_SPKR_PHONE_MONO, /* FUJITSU:2011-12-01 use mono */
	.profile = &ispeaker_rx_profile,
/*	.channel_mode = 2, FUJITSU:2011-12-01 use mono */
	.channel_mode = 1, /* FUJITSU:2011-12-01 use mono */
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
/*	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off, FUJITSU:2011-12-01 speaker amp */
	.pamp_on = &request_enable_spaker_amp,   /* FUJITSU:2011-12-01 speaker amp */
	.pamp_off = &request_disable_spaker_amp, /* FUJITSU:2011-12-01 speaker amp */
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};

static struct platform_device msm_ispeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data },

};

/* FUJITSU:2011-12-01 camera sound start */
static struct snddev_icodec_data snddev_ispeaker2_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_camera_sound_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_CAMERA_RX,
	.profile = &ispeaker_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &request_enable_spaker_amp2,
	.pamp_off = &request_disable_spaker_amp2,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};

static struct platform_device msm_ispeaker2_rx_device = {
	.name = "snddev_icodec",
	.id = 38,
	.dev = { .platform_data = &snddev_ispeaker2_rx_data },

};
/* FUJITSU:2011-12-01 camera sound end */

/* FUJITSU:2011-12-01 separate ADIE reg start */
static struct adie_codec_action_unit iear_rx_48KHz_osr256_actions[] =
   EAR_PRI_STEREO_48000_OSR_256;

static struct adie_codec_hwsetting_entry iear_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iear_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iear_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iear_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iear_rx_settings,
	.setting_sz = ARRAY_SIZE(iear_rx_settings),
};

/* FUJITSU:2011-12-01 separate ADIE reg end */

/* FUJITSU:2011-12-01 speaker for earpiece start */
static struct snddev_icodec_data snddev_iearspeaker_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
/* FUJITSU:2011-12-01 separate ADIE reg start */
/*	.profile = &ispeaker_rx_profile,*/
	.profile = &iear_rx_profile,
/* FUJITSU:2011-12-01 separate ADIE reg end */
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &request_enable_earspaker_amp,
	.pamp_off = &request_disable_earspaker_amp,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearspeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearspeaker_rx_data },

};
/* FUJITSU:2011-12-01 speaker for earpiece end */

static struct snddev_ecodec_data snddev_bt_sco_earpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};

static struct snddev_ecodec_data snddev_bt_sco_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_MIC,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};

struct platform_device msm_bt_sco_earpiece_device = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data },
};

struct platform_device msm_bt_sco_mic_device = {
	.name = "msm_snddev_ecodec",
	.id = 1,
	.dev = { .platform_data = &snddev_bt_sco_mic_data },
};

/* FUJITSU:2011-12-01 bt(sco) ec off start */
static struct snddev_ecodec_data snddev_bt_sco_ec_off_earpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_ec_off_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR_EC_OFF,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};

static struct snddev_ecodec_data snddev_bt_sco_ec_off_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_ec_off_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_MIC_EC_OFF,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};

struct platform_device msm_bt_sco_ec_off_earpiece_device = {
	.name = "msm_snddev_ecodec",
	.id = 2,
	.dev = { .platform_data = &snddev_bt_sco_ec_off_earpiece_data },
};

struct platform_device msm_bt_sco_ec_off_mic_device = {
	.name = "msm_snddev_ecodec",
	.id = 3,
	.dev = { .platform_data = &snddev_bt_sco_ec_off_mic_data },
};
/* FUJITSU:2011-12-01 bt(sco) ec off end */

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions[] =
	AMIC_PRI_MONO_8000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings),
};

static enum hsed_controller ispk_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_device = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data },
};

/* FUJITSU:2011-12-01 vr_mode start */
static struct snddev_icodec_data snddev_ispeaker_tx_for_vrec_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_for_vrec_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_FOR_VREC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_for_vrec_device = {
	.name = "snddev_icodec",
	.id = 36,
	.dev = { .platform_data = &snddev_ispeaker_tx_for_vrec_data },
};
/* FUJITSU:2011-12-01 vr_mode end */

/* FUJITSU:2011-12-01 sleep support start */
static struct snddev_icodec_data snddev_ispeaker_tx_for_sleep_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_for_sleep_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SLEEP_SUPPORT_TX,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_for_sleep_device = {
	.name = "snddev_icodec",
	.id = 37,
	.dev = { .platform_data = &snddev_ispeaker_tx_for_sleep_data },
};
/* FUJITSU:2011-12-01 sleep support end */

static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions[] =
	/*HEADSET_STEREO_SPEAKER_STEREO_RX_CAPLESS_48000_OSR_256; FUJITSU:2011-12-01 stereo->mono */
	HEADSET_STEREO_SPEAKER_STEREO_RX_CAPLESS_48000_OSR_256; /* FUJITSU:2011-12-01 stereo->mono */


static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,
	.profile = &ihs_stereo_speaker_stereo_rx_profile,
/*	.channel_mode = 2, FUJITSU:2011-12-01 use mono */
	.channel_mode = 1, /* FUJITSU:2011-12-01 use mono */
	.default_sample_rate = 48000,
/*	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off, FUJITSU:2011-12-01 speaker amp */
	.pamp_on = &request_enable_spaker_amp,   /* FUJITSU:2011-12-01 speaker amp */
	.pamp_off = &request_disable_spaker_amp, /* FUJITSU:2011-12-01 speaker amp */
/*	.voltage_on = msm_snddev_hsed_voltage_on,  FUJITSU:2012-1-10 del */
/*	.voltage_off = msm_snddev_hsed_voltage_off,FUJITSU:2012-1-10 del */
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
};

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data },
};

/* FUJITSU:2011-12-01 voice message start */
static struct adie_codec_action_unit dummy_rx_actions[] =
	DUMMY_RX_SETTING;

static struct adie_codec_hwsetting_entry dummy_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = dummy_rx_actions,
		.action_sz = ARRAY_SIZE(dummy_rx_actions),
	}
};

static struct adie_codec_dev_profile dummy_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = dummy_rx_settings,
	.setting_sz = ARRAY_SIZE(dummy_rx_settings),
};

static struct snddev_icodec_data snddev_dummy_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "dummy_rx",
	.copp_id = 0,
/*	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,*//*FUJITSU:2011-12-01 change ID for Voice Message*/
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,/*FUJITSU:2012-06-26 change ID for Voice Message*/
	.profile = &dummy_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_dummy_rx_device = {
	.name = "snddev_icodec",
	.id = 32,
	.dev = { .platform_data = &snddev_dummy_rx_data },
};

static struct adie_codec_action_unit dummy_actions[] =
	DUMMY_TX_SETTING;

static struct adie_codec_hwsetting_entry dummy_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = dummy_actions,
		.action_sz = ARRAY_SIZE(dummy_actions),
	}
};

static struct adie_codec_dev_profile dummy_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = dummy_tx_settings,
	.setting_sz = ARRAY_SIZE(dummy_tx_settings),
};

static struct snddev_icodec_data snddev_dummy_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "dummy_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &dummy_tx_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_dummy_tx_device = {
	.name = "snddev_icodec",
	.id = 33,
	.dev = { .platform_data = &snddev_dummy_tx_data },
};
/* FUJITSU:2011-12-01 voice message end */
/* FUJITSU:2011-12-01 for guidance start */

// FJFEAT_cut_start
/* [COM]
*  Delete the AFE loop(Using the QTR loop)
*/
// FJFEAT_cut_end
/* FUJITSU:2012-03-21 Delete the AFE loop start */
#if 1 /* FUJITSU:2012-2-22 uplink_rx */
static struct adie_codec_action_unit guidance_actions[] =
	GUIDANCE_TX_SETTING;

static struct adie_codec_hwsetting_entry guidance_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = guidance_actions,
		.action_sz = ARRAY_SIZE(guidance_actions),
	}
};

static struct adie_codec_dev_profile guidance_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = guidance_tx_settings,
	.setting_sz = ARRAY_SIZE(guidance_tx_settings),
};
#endif /* FUJITSU:2012-2-22 uplink_rx */
static struct snddev_icodec_data snddev_guidance_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "guidance_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_GUIDANCE_TX,
	.profile = &guidance_tx_profile,
//	.profile = &dummy_tx_profile, /* FUJITSU:2012-2-22 uplink_rx */
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};
/* FUJITSU:2012-03-21 Delete the AFE loop end */

static struct platform_device msm_guidance_tx_device = {
	.name = "snddev_icodec",
	.id = 31,
	.dev = { .platform_data = &snddev_guidance_tx_data },
};
/* FUJITSU:2011-12-01 for guidance end */
/* FUJITSU:2011-12-01 for guidance start */

// FJFEAT_cut_start
/* [COM]
*  Delete the AFE loop(Using the QTR loop)
*/
// FJFEAT_cut_end
/* FUJITSU:2012-03-21 Delete the AFE loop start */
#if 1 /* FUJITSU:2012-2-1 uplink_rx */
static struct adie_codec_action_unit guidance_rx_actions[] =
	GUIDANCE_RX_SETTING;

static struct adie_codec_hwsetting_entry guidance_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = guidance_rx_actions,
		.action_sz = ARRAY_SIZE(guidance_rx_actions),
	}
};

static struct adie_codec_dev_profile guidance_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = guidance_rx_settings,
	.setting_sz = ARRAY_SIZE(guidance_rx_settings),
};
#endif
static struct snddev_icodec_data snddev_guidance_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "guidance_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_GUIDANCE_RX,
	.profile = &guidance_rx_profile,
//	.profile = &dummy_rx_profile, /* FUJITSU:2012-2-1 uplink_rx */
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};
/* FUJITSU:2012-03-21 Delete the AFE loop end */

static struct platform_device msm_guidance_rx_device = {
	.name = "snddev_icodec",
	.id = 30,
	.dev = { .platform_data = &snddev_guidance_rx_data },
};
/* FUJITSU:2011-12-01 for guidance end */
/* FUJITSU:2012-2-1 uplink_rx st */
static struct snddev_virtual_data snddev_uplink_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "uplink_rx",
	.copp_id = 5,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct platform_device msm_uplink_rx_device = {
	.name = "snddev_virtual",
	.id = 2,
	.dev = { .platform_data = &snddev_uplink_rx_data },
};
/* FUJITSU:2012-2-1 uplink_rx ed */
/* FUJITSU:2012-2-23 nad start */
/* FUJITSU:2012-05-01 bigVoice start */
#if defined(CONFIG_MACH_F12NAD)
static struct adie_codec_action_unit idual_mic_endfire_8KHz_osr256_actions[] =
	AMIC2_DUAL_8000_OSR_256;
#else
/* FUJITSU:2012-05-01 bigVoice end */
static struct adie_codec_action_unit idual_mic_endfire_8KHz_osr256_actions[] =
	AMIC_DUAL_8000_OSR_256;
#endif /* FUJITSU:2012-05-01 bigVoice add */

static struct adie_codec_hwsetting_entry idual_mic_endfire_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_endfire_settings),
};

static enum hsed_controller idual_mic_endfire_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = /*ACDB_ID_HANDSET_MIC_ENDFIRE*/ACDB_ID_HANDSET_MIC, /* FUJITSU:2012-4-18 ACDB of ACDB_ID_HANDSET_MIC is used. */
	.profile = &idual_mic_endfire_profile,
	.channel_mode = REAL_STEREO_CHANNEL_MODE,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 60,
	.dev = { .platform_data = &snddev_idual_mic_endfire_data },
};

static struct snddev_icodec_data snddev_idual_spkmic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = /*ACDB_ID_SPKR_PHONE_MIC_ENDFIRE*/ACDB_ID_SPKR_PHONE_MIC, /* FUJITSU:2012-4-18 ACDB of ACDB_ID_SPKR_PHONE_MIC is used. */
	.profile = &idual_mic_endfire_profile,
	.channel_mode = REAL_STEREO_CHANNEL_MODE,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_spkmic_endfire_device = {
	.name = "snddev_icodec",
	.id = 61,
	.dev = { .platform_data = &snddev_idual_spkmic_endfire_data },
};

static struct adie_codec_action_unit ibackmic_48KHz_osr256_actions[] =
	BACK_AMIC_PRI_MONO_8000_OSR_256;

static struct adie_codec_hwsetting_entry ibackmic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ibackmic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ibackmic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ibackmic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ibackmic_settings,
	.setting_sz = ARRAY_SIZE(ibackmic_settings),
};

static enum hsed_controller ibackmic_pmctl_id[] = {PM_HSED_CONTROLLER_2};

static struct snddev_icodec_data snddev_ibackmic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_back_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_BACKMIC, /* FUJITSU:2012-4-18 ACDB of ACDB_ID_BACKMIC is used. */
	.profile = &ibackmic_profile,
	.channel_mode = 1,
	.pmctl_id = ibackmic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ibackmic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_ibackmic_device = {
	.name = "snddev_icodec",
	.id = 62,
	.dev = { .platform_data = &snddev_ibackmic_data },
};
/* FUJITSU:2012-2-23 nad start */
/* FUJITSU:2012-2-24 ace start */
static struct snddev_icodec_data snddev_iearpiece_voice_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_voice_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_VOICE_SPKR,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = start_earpiece,
	.pamp_off = stop_earpiece,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearpiece_voice_device = {
	.name = "snddev_icodec",
	.id = 63,
	.dev = { .platform_data = &snddev_iearpiece_voice_data },
};

static struct snddev_icodec_data snddev_imic_voice_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_voice_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_VOICE_MIC,
	.profile = &imic_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_voice_device = {
	.name = "snddev_icodec",
	.id = 64,
	.dev = { .platform_data = &snddev_imic_voice_data },
};

static struct snddev_icodec_data snddev_ispeaker_rx_voice_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_voice_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_VOICE_MONO,
	.profile = &ispeaker_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &request_enable_spaker_amp,
	.pamp_off = &request_disable_spaker_amp,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};

static struct platform_device msm_ispeaker_rx_voice_device = {
	.name = "snddev_icodec",
	.id = 65,
	.dev = { .platform_data = &snddev_ispeaker_rx_voice_data },
};

static struct snddev_icodec_data snddev_ispeaker_tx_voice_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_voice_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_VOICE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_voice_device = {
	.name = "snddev_icodec",
	.id = 66,
	.dev = { .platform_data = &snddev_ispeaker_tx_voice_data },
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_voice_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_voice_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_VOICE_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};

static struct platform_device msm_ihs_stereo_rx_voice_device = {
	.name = "snddev_icodec",
	.id = 67,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_voice_data },
};

/* FUJITSU:2012-06-26 New HEADSET TX setting start */
static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions2[] =
	AMIC2_HEADSET_TX_MONO_PRIMARY_OSR256;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings2[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions2,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions2),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile2 = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings2,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings2),
};
/* FUJITSU:2012-06-26 New HEADSET TX setting end */

static struct snddev_icodec_data snddev_ihs_mono_tx_voice_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_voice_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_VOICE_MIC,
	.profile = &ihs_mono_tx_profile2, /* FUJITSU:2012-06-26 New HEADSET TX setting mod */
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id1,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id1),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_voice_device = {
	.name = "snddev_icodec",
	.id = 68,
	.dev = { .platform_data = &snddev_ihs_mono_tx_voice_data },
};
/* FUJITSU:2012-2-24 ace end */

static struct platform_device *snd_devices_sky[] __initdata = {
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	/*&msm_ihs_mono_rx_device, FUJITSU:2011-12-01 not use */
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	/*&msm_ifmradio_handset_device, FUJITSU:2011-12-01 not use */
	&msm_ispeaker_rx_device,
	&msm_ispeaker2_rx_device, /* FUJITSU:2011-12-01 camera sound */
	/*&msm_ifmradio_speaker_device, FUJITSU:2011-12-01 not use */
	/*&msm_ifmradio_headset_device, FUJITSU:2011-12-01 not use */
	/*&msm_itty_hs_mono_tx_device, FUJITSU:2011-12-01 not use */
	/*&msm_itty_hs_mono_rx_device, FUJITSU:2011-12-01 not use */
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	/*&msm_a2dp_rx_device, FUJITSU:2011-12-01 not use */
	/*&msm_a2dp_tx_device, FUJITSU:2011-12-01 not use */
	/*&msm_snddev_mi2s_stereo_rx_device, FUJITSU:2011-12-01 not use */
	/*&msm_snddev_mi2s_fm_tx_device, FUJITSU:2011-12-01 not use */
	/*&msm_uplink_rx_device, FUJITSU:2011-12-01 not use */
/* FUJITSU:2011-12-01 bt(sco) ec off start */
	&msm_bt_sco_ec_off_earpiece_device,
	&msm_bt_sco_ec_off_mic_device,
/* FUJITSU:2011-12-01 bt(sco) ec off end */
/* FUJITSU:2011-12-01 wm0010 start */
/*	&msm_iearpiece_voice_device,     FUJITSU:2011-12-01 not use */
/*	&msm_imic_voice_device,          FUJITSU:2011-12-01 not use */
/*	&msm_ispeaker_rx_voice_device,   FUJITSU:2011-12-01 not use */
/*	&msm_ispeaker_tx_voice_device,   FUJITSU:2011-12-01 not use */
/*	&msm_ihs_stereo_rx_voice_device, FUJITSU:2011-12-01 not use */
/*	&msm_ihs_mono_tx_voice_device,   FUJITSU:2011-12-01 not use */
/* FUJITSU:2011-12-01 wm0010 end */
/* FUJITSU:2011-12-01 vr_mode start */
	&msm_ispeaker_tx_for_vrec_device,
/* FUJITSU:2011-12-01 vr_mode end */
/* FUJITSU:2011-12-01 for guidance start */
	&msm_guidance_rx_device,
	&msm_guidance_tx_device,
/* FUJITSU:2011-12-01 for guidance end */
/* FUJITSU:2011-12-01 sleep support start */
	&msm_ispeaker_tx_for_sleep_device,
/* FUJITSU:2011-12-01 sleep support end */
/* FUJITSU:2011-12-01 voice message start */
	&msm_dummy_rx_device,
	&msm_dummy_tx_device,
/* FUJITSU:2011-12-01 voice message end */
	&msm_uplink_rx_device, /* FUJITSU:2012-2-1 uplink_rx */
};

/* FUJITSU:2011-12-01 speaker for earpiece start */
/* FUJITSU:2012-5-24 Change the structure name start */
static struct platform_device *snd_devices_apo[] __initdata = {
/* FUJITSU:2012-5-24 Change the structure name end */
	&msm_iearspeaker_rx_device,
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ispeaker_rx_device,
	&msm_ispeaker2_rx_device, /* FUJITSU:2011-12-01 camera sound */
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_bt_sco_ec_off_earpiece_device,
	&msm_bt_sco_ec_off_mic_device,
/*	&msm_iearpiece_voice_device,     FUJITSU:2011-12-01 not use */
/*	&msm_imic_voice_device,          FUJITSU:2011-12-01 not use */
/*	&msm_ispeaker_rx_voice_device,   FUJITSU:2011-12-01 not use */
/*	&msm_ispeaker_tx_voice_device,   FUJITSU:2011-12-01 not use */
/*	&msm_ihs_stereo_rx_voice_device, FUJITSU:2011-12-01 not use */
/*	&msm_ihs_mono_tx_voice_device,   FUJITSU:2011-12-01 not use */
	&msm_ispeaker_tx_for_vrec_device,
	&msm_guidance_rx_device,
	&msm_guidance_tx_device,
/* FUJITSU:2011-12-01 sleep support start */
	&msm_ispeaker_tx_for_sleep_device,
/* FUJITSU:2011-12-01 sleep support end */
/* FUJITSU:2011-12-01 voice message start */
	&msm_dummy_rx_device,
	&msm_dummy_tx_device,
/* FUJITSU:2011-12-01 voice message end */
	&msm_uplink_rx_device, /* FUJITSU:2012-2-1 uplink_rx */
};
/* FUJITSU:2011-12-01 speaker for earpiece end */

/* FUJITSU:2012-2-24 for ace start */
/* FUJITSU:2012-5-24 Change the structure name start */
static struct platform_device *snd_devices_ace[] __initdata = {
/* FUJITSU:2012-5-24 Change the structure name end */
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ispeaker_rx_device,
	&msm_ispeaker2_rx_device,
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_bt_sco_ec_off_earpiece_device,
	&msm_bt_sco_ec_off_mic_device,
	&msm_iearpiece_voice_device,     /* for ace */
	&msm_imic_voice_device,          /* for ace */
	&msm_ispeaker_rx_voice_device,   /* for ace */
	&msm_ispeaker_tx_voice_device,   /* for ace */
	&msm_ihs_stereo_rx_voice_device, /* for ace */
	&msm_ihs_mono_tx_voice_device,   /* for ace */
	&msm_ispeaker_tx_for_vrec_device,
	&msm_guidance_rx_device,
	&msm_guidance_tx_device,
	&msm_ispeaker_tx_for_sleep_device,
	&msm_dummy_rx_device,
	&msm_dummy_tx_device,
	&msm_uplink_rx_device,
};
/* FUJITSU:2012-2-24 for ace end */

/* FUJITSU:2012-2-24 for nad start */
/* FUJITSU:2012-5-24 Change the structure name start */
static struct platform_device *snd_devices_nad[] __initdata = {
/* FUJITSU:2012-5-24 Change the structure name end */
/* FUJITSU:2012-03-12 notify earamp start */
//	&msm_iearpiece_device,
	&msm_iearpiece_device2,
/* FUJITSU:2012-03-12 notify earamp end */
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ispeaker_rx_device,
	&msm_ispeaker2_rx_device,
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_bt_sco_ec_off_earpiece_device,
	&msm_bt_sco_ec_off_mic_device,
	&msm_ispeaker_tx_for_vrec_device,
	&msm_guidance_rx_device,
	&msm_guidance_tx_device,
	&msm_ispeaker_tx_for_sleep_device,
	&msm_dummy_rx_device,
	&msm_dummy_tx_device,
	&msm_uplink_rx_device,
	&msm_idual_mic_endfire_device,    /* for nad */
	&msm_idual_spkmic_endfire_device, /* for nad */
	&msm_ibackmic_device,             /* for nad */
	&msm_ihs_mono_tx_voice_device,    /* for nad  FUJITSU:2012-06-26 New HEADSET TX setting mod */
};
/* FUJITSU:2012-2-24 for nad end */

/* FUJITSU:2011-12-01 notify earpiece start */
void start_earpiece(void){
    unsigned char *handset = (unsigned char *) smem_alloc_vendor1(SMEM_OEM_012);
    if(handset!=NULL) *handset = 1;
}

void stop_earpiece(void){
    unsigned char *handset = (unsigned char *) smem_alloc_vendor1(SMEM_OEM_012);
    if(handset!=NULL) *handset = 0;
}
/* FUJITSU:2011-12-01 notify earpiece end */
/* FUJITSU:2011-12-01 timpani end */

/* FUJITSU:2012-03-12 notify earamp start */
void start_ear_amp(void){
    unsigned char *handset = (unsigned char *) smem_alloc_vendor1(SMEM_OEM_012);
    if(handset!=NULL) *handset = 1;
	request_enable_earamp();
}

void stop_ear_amp(void){
    unsigned char *handset = (unsigned char *) smem_alloc_vendor1(SMEM_OEM_012);
    if(handset!=NULL) *handset = 0;
	request_disable_earamp();
}
/* FUJITSU:2012-03-12 notify earamp end */

void __ref msm_snddev_init_timpani(void)
{
	/*platform_add_devices(snd_devices_ffa,
			ARRAY_SIZE(snd_devices_ffa)); FUJITSU:2011-12-01 timpani */
/* FUJITSU:2012-2-24 add ace,nad start */
	if( machine_is_f12ace() ){
		platform_add_devices(snd_devices_ace,ARRAY_SIZE(snd_devices_ace)); // FUJITSU:2012-5-24 Change the structure name
	} else if(machine_is_f12apon()){
		platform_add_devices(snd_devices_apo,ARRAY_SIZE(snd_devices_apo)); // FUJITSU:2012-5-24 Change the structure name
//	} else if(machine_is_f11sky()){
	} else if(0){
		platform_add_devices(snd_devices_sky,ARRAY_SIZE(snd_devices_sky));
	} else { /* nad */
/*FUJITSU:2012-06-01 nad separate by prototype start */
        printk(KERN_WARNING "msm_snddev_init_timpani system_rev= %x\n",system_rev);
        if(system_rev == 0x01){
            iearpiece_settings[0].actions = iearpiece_48KHz_osr256_actions2;
        }
/*FUJITSU:2012-06-01 nad separate by prototype end */
		platform_add_devices(snd_devices_nad,ARRAY_SIZE(snd_devices_nad)); // FUJITSU:2012-5-24 Change the structure name
	}
/* FUJITSU:2012-2-24 add ace,nad end */
}

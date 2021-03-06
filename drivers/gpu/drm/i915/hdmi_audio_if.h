/*
 * Copyright (c) 2010, Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	jim liu <jim.liu@intel.com>
 *	Uma Shankar <uma.shankar@intel.com>
 */


#ifndef __HDMI_AUDIO_IF_H
#define __HDMI_AUDIO_IF_H

#include <linux/types.h>
#include <drm/drmP.h>

/* HDMI AUDIO INTERRUPT TYPE */
#define HDMI_AUDIO_UNDERRUN     (1UL<<0)
#define HDMI_AUDIO_BUFFER_DONE  (1UL<<1)

/* the monitor type HDMI or DVI */
#define MONITOR_TYPE_HDMI 1
#define MONITOR_TYPE_DVI  2

extern int i915_hdmi_state;

enum had_caps_list {
	HAD_GET_ELD = 1,
	HAD_GET_SAMPLING_FREQ,
	HAD_GET_DISPLAY_RATE,
	HAD_GET_HDCP_STATUS,
	HAD_GET_AUDIO_STATUS,
	HAD_SET_ENABLE_AUDIO,
	HAD_SET_DISABLE_AUDIO,
	HAD_SET_ENABLE_AUDIO_INT,
	HAD_SET_DISABLE_AUDIO_INT,
	OTHERS_TBD,
};

enum had_event_type {
	HAD_EVENT_HOT_PLUG = 1,
	HAD_EVENT_HOT_UNPLUG,
	HAD_EVENT_MODE_CHANGING,
	HAD_EVENT_PM_CHANGING,
	HAD_EVENT_AUDIO_BUFFER_DONE,
	HAD_EVENT_AUDIO_BUFFER_UNDERRUN,
	HAD_EVENT_QUERY_IS_AUDIO_BUSY,
	HAD_EVENT_QUERY_IS_AUDIO_SUSPENDED,
};

/*
 * HDMI Display Controller Audio Interface
 *
 */
typedef int (*had_event_call_back) (enum had_event_type event_type,
		void *ctxt_info);

struct hdmi_audio_registers_ops {
	int (*hdmi_audio_get_register_base)(uint32_t *reg_base);
	int (*hdmi_audio_read_register)(uint32_t reg_addr, uint32_t *data);
	int (*hdmi_audio_write_register)(uint32_t reg_addr, uint32_t data);
	int (*hdmi_audio_read_modify)(uint32_t reg_addr, uint32_t data,
			uint32_t mask);
};

struct hdmi_audio_query_set_ops {
	int (*hdmi_audio_get_caps)(enum had_caps_list query_element,
			void *capabilties);
	int (*hdmi_audio_set_caps)(enum had_caps_list set_element,
			void *capabilties);
};

struct hdmi_audio_event {
	int type;
};

struct snd_intel_had_interface {
	const char *name;
	int (*query)(void *had_data, struct hdmi_audio_event event);
	int (*suspend)(void *had_data, struct hdmi_audio_event event);
	int (*resume)(void *had_data);
};

struct hdmi_audio_priv {
	struct drm_device *dev;
	u32 hdmi_reg;
	u32 hdmi_lpe_audio_reg;
	bool is_hdcp_supported;
	bool hdmi_hpd_connected;
	int monitor_type;
	void *context;
};

extern void i915_hdmi_audio_init(struct hdmi_audio_priv *p_hdmi_priv);

extern bool mid_hdmi_audio_is_busy(struct drm_device *dev);
extern bool mid_hdmi_audio_suspend(struct drm_device *dev);
extern void mid_hdmi_audio_resume(struct drm_device *dev);
extern void mid_hdmi_audio_signal_event(struct drm_device *dev,
		enum had_event_type event);

extern void hdmi_get_eld(uint8_t *eld);
extern int i915_enable_hdmi_audio_int(struct drm_device *dev);
extern int i915_disable_hdmi_audio_int(struct drm_device *dev);
extern int mid_hdmi_audio_setup(
	had_event_call_back audio_callbacks,
	struct hdmi_audio_registers_ops *reg_ops,
	struct hdmi_audio_query_set_ops *query_ops);
extern int mid_hdmi_audio_register(
	struct snd_intel_had_interface *driver,
	void *had_data);

#endif /* __HDMI_AUDIO_IF_H */

/*
 * Copyright (C) 2021-2024 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#define LOG_NDEBUG 0
#include "AlsCorrection.h"

#include <android-base/properties.h>
#include <android/binder_manager.h>
#include <binder/IBinder.h>
#include <binder/IServiceManager.h>
#include <cmath>
#include <fstream>
#include <log/log.h>
#include <utils/Timers.h>

using aidl::vendor::lineage::oplus_als::AreaRgbCaptureResult;
using aidl::vendor::lineage::oplus_als::IAreaCapture;
using android::base::GetBoolProperty;
using android::base::GetIntProperty;
using android::base::GetProperty;

#define ALS_CALI_DIR "/proc/sensor/als_cali/"
#define BRIGHTNESS_DIR "/sys/class/backlight/panel0-backlight/"

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace implementation {

static const std::string rgbw_max_lux_paths[4] = {
    ALS_CALI_DIR "red_max_lux",
    ALS_CALI_DIR "green_max_lux",
    ALS_CALI_DIR "blue_max_lux",
    ALS_CALI_DIR "white_max_lux",
};

struct als_config {
    bool hbr;
    float rgbw_max_lux[4];
    float rgbw_max_lux_div[4];
    float rgbw_lux_postmul[4];
    float rgbw_poly[4][4];
    float grayscale_weights[3];
    float sensor_gaincal_points[4];
    float sensor_inverse_gain[4];
    float agc_threshold;
    float calib_gain;
    float bias;
    float post_bias;
    float max_brightness;
};

static struct {
    float middle;
    float min, max;
} hysteresis_ranges[] = {
    { 0, 0, 4 },
    { 7, 1, 12 },
    { 15, 5, 30 },
    { 30, 10, 50 },
    { 360, 25, 700 },
    { 1200, 300, 1600 },
    { 2250, 1000, 2940 },
    { 4600, 2000, 5900 },
    { 10000, 4000, 80000 },
    { HUGE_VALF, 8000, HUGE_VALF },
};

static struct {
    nsecs_t last_update, last_forced_update;
    bool force_update;
    float hyst_min, hyst_max;
    float last_corrected_value;
    float last_agc_gain;
} state = {
    .last_update = 0,
    .force_update = true,
    .hyst_min = -1.0, .hyst_max = -1.0,
    .last_agc_gain = 0.0,
};

static als_config conf;
static std::shared_ptr<IAreaCapture> service;

template <typename T>
static T get(const std::string& path, const T& def) {
    std::ifstream file(path);
    T result;

    file >> result;
    return file.fail() ? def : result;
}

void AlsCorrection::init() {
    ALOGE("----------- ALS INIT ------------");
    std::istringstream is;
    conf.post_bias = GetIntProperty("persist.vendor.sensors.als_correction.post_bias", -14);
    conf.hbr = GetBoolProperty("persist.vendor.sensors.als_correction.hbr", false);
    conf.bias = GetIntProperty("persist.vendor.sensors.als_correction.bias", 0);
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.rgbw_max_lux", ""));
    is >> conf.rgbw_max_lux[0] >> conf.rgbw_max_lux[1]
        >> conf.rgbw_max_lux[2] >> conf.rgbw_max_lux[3];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.rgbw_max_lux_div", ""));
    is >> conf.rgbw_max_lux_div[0] >> conf.rgbw_max_lux_div[1]
        >> conf.rgbw_max_lux_div[2] >> conf.rgbw_max_lux_div[3];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.rgbw_poly1", ""));
    is >> conf.rgbw_poly[0][0] >> conf.rgbw_poly[0][1]
        >> conf.rgbw_poly[0][2] >> conf.rgbw_poly[0][3];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.rgbw_poly2", ""));
    is >> conf.rgbw_poly[1][0] >> conf.rgbw_poly[1][1]
        >> conf.rgbw_poly[1][2] >> conf.rgbw_poly[1][3];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.rgbw_poly3", ""));
    is >> conf.rgbw_poly[2][0] >> conf.rgbw_poly[2][1]
        >> conf.rgbw_poly[2][2] >> conf.rgbw_poly[2][3];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.rgbw_poly4", ""));
    is >> conf.rgbw_poly[3][0] >> conf.rgbw_poly[3][1]
        >> conf.rgbw_poly[3][2] >> conf.rgbw_poly[3][3];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.grayscale_weights", ""));
    is >> conf.grayscale_weights[0] >> conf.grayscale_weights[1] >> conf.grayscale_weights[2];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.sensor_gaincal_points", ""));
    is >> conf.sensor_gaincal_points[0] >> conf.sensor_gaincal_points[1]
        >> conf.sensor_gaincal_points[2] >> conf.sensor_gaincal_points[3];
    is = std::istringstream(GetProperty("persist.vendor.sensors.als_correction.sensor_inverse_gain", ""));
    is >> conf.sensor_inverse_gain[0] >> conf.sensor_inverse_gain[1]
        >> conf.sensor_inverse_gain[2] >> conf.sensor_inverse_gain[3];

    float rgbw_acc = 0.0;
    for (int i = 0; i < 4; i++) {
        if (i < 3) {
            rgbw_acc += conf.rgbw_max_lux[i];
            conf.rgbw_lux_postmul[i] = conf.rgbw_max_lux[i] / conf.rgbw_max_lux_div[i];
        } else {
            rgbw_acc -= conf.rgbw_max_lux[i];
            conf.rgbw_lux_postmul[i] = rgbw_acc / conf.rgbw_max_lux_div[i];
        }
    }
    ALOGI("Display maximums: R=%.0f G=%.0f B=%.0f W=%.0f",
        conf.rgbw_max_lux[0], conf.rgbw_max_lux[1],
        conf.rgbw_max_lux[2], conf.rgbw_max_lux[3]);

    float row_coe = GetIntProperty("persist.vendor.sensors.als_correction.row_coe", 0);
    if (row_coe != 0.0) {
        conf.sensor_inverse_gain[0] = row_coe / 1000.0;
    }
    conf.agc_threshold = 800.0 / conf.sensor_inverse_gain[0];
    ALOGI("ALS: agc_threshold: %.2fx", conf.agc_threshold);

    float cali_coe = GetIntProperty("persist.vendor.sensors.als_correction.cali_coe", 0);
    conf.calib_gain = cali_coe > 0.0 ? cali_coe / 1000.0 : 1.0;
    ALOGI("ALS: Calibrated sensor gain: %.2fx", conf.calib_gain);

    conf.max_brightness = get(BRIGHTNESS_DIR "max_brightness", 1023.0);
    ALOGI("ALS: max_brightness: %.2fx", conf.max_brightness);

    for (auto& range : hysteresis_ranges) {
        range.min /= conf.calib_gain * conf.sensor_inverse_gain[0];
        range.max /= conf.calib_gain * conf.sensor_inverse_gain[0];
    }
    hysteresis_ranges[0].min = -1.0;
    for (auto& range : hysteresis_ranges) {
        ALOGI("ALS: hysteresis_range: min=%f, middle=%f, max=%f", range.min, range.middle, range.max);
    }

    const auto instancename = std::string(IAreaCapture::descriptor) + "/default";

    if (AServiceManager_isDeclared(instancename.c_str())) {
        service = IAreaCapture::fromBinder(::ndk::SpAIBinder(
            AServiceManager_waitForService(instancename.c_str())));
    } else {
        ALOGE("Service is not registered");
    }
}

void AlsCorrection::process(Event& event) {
    static AreaRgbCaptureResult screenshot = { 0.0, 0.0, 0.0 };

    ALOGV("ALS: %f %f %f %f",
            event.u.data[0],
            event.u.data[1],
            event.u.data[2],
            event.u.data[4]
         );

    ALOGV("ALS: Raw sensor reading: %.0f", event.u.scalar);

    if (event.u.scalar > conf.bias) {
        event.u.scalar -= conf.bias;
    }

    nsecs_t now = systemTime(SYSTEM_TIME_BOOTTIME);
    float brightness = get(BRIGHTNESS_DIR "brightness", 0.0);
    ALOGV("ALS: Got brightness %f", brightness);

    if (state.last_update == 0) {
        state.last_update = now;
        state.last_forced_update = now;
    } else {
        if (brightness > 0.0 && (now - state.last_forced_update) > s2ns(3)) {
            ALOGV("ALS: Forcing screenshot");
            state.last_forced_update = now;
            state.force_update = true;
        }
        if ((now - state.last_update) < ms2ns(100)) {
            ALOGE("ALS: Events coming too fast, dropping");
            // TODO figure out a better way to drop events
            event.sensorHandle = 0;
            return;
        }
        state.last_update = now;
    }

    float sensor_raw_calibrated = event.u.scalar * conf.calib_gain * state.last_agc_gain;
    ALOGV("ALS: Sensor raw calibrated: %.f", sensor_raw_calibrated);
    if (state.force_update
            || ((event.u.scalar < state.hyst_min || event.u.scalar > state.hyst_max)
                && (sensor_raw_calibrated < 10.0 || sensor_raw_calibrated > (5.0 / .07)))) {

        if (service == nullptr || !service->getAreaBrightness(&screenshot).isOk()) {
            ALOGE("ALS: Could not get area above sensor");
            // TODO figure out a better way to drop events
            event.sensorHandle = 0;
            return;
        }
        ALOGV("ALS: Screen color above sensor: %f %f %f", screenshot.r, screenshot.g, screenshot.b);

        float rgbw[4] = {
            screenshot.r, screenshot.g, screenshot.b,
            screenshot.r * conf.grayscale_weights[0]
                + screenshot.g * conf.grayscale_weights[1]
                + screenshot.b * conf.grayscale_weights[2]
        };
        float cumulative_correction = 0.0;
        for (int i = 0; i < 4; i++) {
            float corr = 0.0;
            for (float coef : conf.rgbw_poly[i]) {
                corr *= rgbw[i];
                corr += coef;
            }
            corr *= conf.rgbw_lux_postmul[i];
            if (i < 3) {
                cumulative_correction += std::max(corr, 0.0f);
            } else {
                cumulative_correction -= corr;
            }
        }
        ALOGV("ALS: Cumulative_correction color: %.0f", cumulative_correction);

        cumulative_correction *= brightness / conf.max_brightness;
        float brightness_fullwhite = conf.rgbw_max_lux[3] * brightness / conf.max_brightness;
        float brightness_grayscale_gamma = std::pow(rgbw[3] / 255.0, 2.2) * brightness_fullwhite;
        ALOGV("ALS: Min of cumulative_correction color and brightness_fullwhite: %.0f and %.0f", cumulative_correction, brightness_fullwhite);
        cumulative_correction = std::min(cumulative_correction, brightness_fullwhite);
        ALOGV("ALS: Max of cumulative_correction color and brightness_grayscale_gamma: %.0f and %.0f", cumulative_correction, brightness_grayscale_gamma);
        cumulative_correction = std::max(cumulative_correction, brightness_grayscale_gamma);
        ALOGV("ALS: Estimated screen brightness: %.0f", cumulative_correction);

        float sensor_raw_corrected = std::max(event.u.scalar - cumulative_correction, 0.0f);

        float agc_gain = conf.sensor_inverse_gain[0];
        ALOGV("ALS: sensor_raw_corrected1: %f lux, agc_gain=%f agc_threshold=%.0f", sensor_raw_corrected, agc_gain, conf.agc_threshold);
        if (sensor_raw_corrected > conf.agc_threshold) {
            float gain_estimate = 0;
            if (conf.hbr) {
                gain_estimate = event.u.data[2] * 1000.0 / sensor_raw_corrected;
            } else {
                gain_estimate = sensor_raw_corrected / event.u.data[2];
            }
            ALOGV("ALS: Using gain_estimate: %.0f", gain_estimate);
            for (int i = 0; i < 4; i++) {
                if (gain_estimate > conf.sensor_gaincal_points[i]) {
                    ALOGV("ALS: Using gain_estimate: sensor_gaincal_points[%d]: %.0f %.0f", i, conf.sensor_gaincal_points[i], conf.sensor_inverse_gain[i]);
                    agc_gain = conf.sensor_inverse_gain[i];
                }
            }
        }
        ALOGV("ALS: AGC gain: %f", agc_gain);

        if (cumulative_correction <= event.u.scalar * 1.35
                || event.u.scalar * conf.calib_gain * agc_gain < 10000.0
                || state.force_update) {
            float sensor_corrected = sensor_raw_corrected * conf.calib_gain * agc_gain;
            ALOGV("ALS: Corrected sensor value after applying conf.calib_gain (%f) and agc_gain (%f): %.0f lux", conf.calib_gain, agc_gain, sensor_corrected);
            state.last_agc_gain = agc_gain;
            for (auto& range : hysteresis_ranges) {
                if (sensor_corrected <= range.middle) {
                    state.hyst_min = range.min;
                    state.hyst_max = range.max + brightness_fullwhite;
                    break;
                }
            }
            sensor_corrected = std::max(sensor_corrected + conf.post_bias, 0.0f);
            event.u.scalar = sensor_corrected;
            state.last_corrected_value = sensor_corrected;
            ALOGV("ALS: Final corrected sensor value: %.0f lux", sensor_corrected);
        } else {
            event.u.scalar = state.last_corrected_value;
            ALOGV("ALS: Reusing cached value 1: %.0f lux", event.u.scalar);
        }

        state.force_update = false;
    } else {
        event.u.scalar = state.last_corrected_value;
        ALOGV("ALS: Reusing cached value 2: %.0f lux", event.u.scalar);
    }
}

}  // namespace implementation
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android

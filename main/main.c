#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "bsp/esp32_p4_function_ev_board.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#define TARGET_CAMERA_FPS 30.0f
#define MIN_TARGET_CAMERA_FPS 24.0f
#define CAMERA_FRAME_BUFFERS 4
#define MAX_DECODE_WIDTH 1280
#define MAX_DECODE_HEIGHT 720
#define CAMERA_FRAME_SIZE (4 * 1024 * 1024)
#define FRAME_QUEUE_LENGTH 2
#define JPEG_DECODE_TIMEOUT_MS 100

typedef struct {
    uint8_t dev_addr;
    uint8_t stream_index;
    unsigned width;
    unsigned height;
    unsigned decoded_width;
    unsigned decoded_height;
    uint32_t frame_interval;
    float fps;
} camera_mode_t;

static const char *TAG = "zve10_stream";


static esp_lcd_panel_handle_t display_panel;
static jpeg_decoder_handle_t jpeg_decoder;
static ppa_client_handle_t ppa_client;

static uint8_t *decode_frame;
static size_t decode_frame_size;
static uint8_t *display_frame;
static size_t display_frame_size;

static const jpeg_decode_cfg_t jpeg_decode_config = {
    .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
    .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
    .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
};
static SemaphoreHandle_t camera_disconnected;
static SemaphoreHandle_t camera_available;
static SemaphoreHandle_t frame_queue_drained;
static QueueHandle_t frame_queue;
static uvc_host_stream_hdl_t active_stream;
static camera_mode_t selected_mode;

static const char *uvc_format_name(enum uvc_host_stream_format format)
{
    switch (format) {
    case UVC_VS_FORMAT_DEFAULT:
        return "default";
    case UVC_VS_FORMAT_MJPEG:
        return "MJPEG";
    case UVC_VS_FORMAT_YUY2:
        return "YUY2";
    case UVC_VS_FORMAT_H264:
        return "H.264";
    case UVC_VS_FORMAT_H265:
        return "H.265";
    default:
        return "unknown";
    }
}

static bool is_better_fps(float candidate, float current)
{
    const bool candidate_at_or_below_target = candidate <= TARGET_CAMERA_FPS + 0.05f;
    const bool current_at_or_below_target = current <= TARGET_CAMERA_FPS + 0.05f;
    if (candidate_at_or_below_target != current_at_or_below_target) {
        return candidate_at_or_below_target;
    }
    if (candidate_at_or_below_target) {
        return candidate > current;
    }
    return candidate < current;
}

static bool is_better_mode(const camera_mode_t *candidate, const camera_mode_t *current)
{
    const bool candidate_meets_target = candidate->fps >= MIN_TARGET_CAMERA_FPS;
    const bool current_meets_target = current->fps >= MIN_TARGET_CAMERA_FPS;
    if (candidate_meets_target != current_meets_target) {
        return candidate_meets_target;
    }

    const uint64_t candidate_pixels = (uint64_t)candidate->width * candidate->height;
    const uint64_t current_pixels = (uint64_t)current->width * current->height;
    if (candidate_meets_target && candidate_pixels != current_pixels) {
        return candidate_pixels < current_pixels;
    }
    return is_better_fps(candidate->fps, current->fps);
}

static void consider_interval(uint32_t interval, uint32_t *best_interval, float *best_fps)
{
    if (interval == 0) {
        return;
    }

    const float fps = 10000000.0f / interval;
    if (*best_interval == 0 || is_better_fps(fps, *best_fps)) {
        *best_interval = interval;
        *best_fps = fps;
    }
}

static bool select_frame_interval(const uvc_host_frame_info_t *frame, uint32_t *interval, float *fps)
{
    *interval = 0;
    *fps = 0.0f;
    consider_interval(frame->default_interval, interval, fps);

    if (frame->interval_type == 0) {
        if (frame->interval_min == 0 || frame->interval_max == 0) {
            return *interval != 0;
        }

        const uint32_t target_interval = (uint32_t)(10000000.0f / TARGET_CAMERA_FPS);
        uint32_t candidate = target_interval;
        if (candidate < frame->interval_min) {
            candidate = frame->interval_min;
        } else if (candidate > frame->interval_max) {
            candidate = frame->interval_max;
        } else if (frame->interval_step != 0) {
            const uint32_t steps =
                (candidate - frame->interval_min + frame->interval_step - 1) / frame->interval_step;
            candidate = frame->interval_min + steps * frame->interval_step;
            if (candidate > frame->interval_max) {
                candidate = frame->interval_max;
            }
        }
        consider_interval(candidate, interval, fps);
    } else {
        const size_t count = frame->interval_type < CONFIG_UVC_INTERVAL_ARRAY_SIZE
                                 ? frame->interval_type
                                 : CONFIG_UVC_INTERVAL_ARRAY_SIZE;
        for (size_t i = 0; i < count; ++i) {
            consider_interval(frame->interval[i], interval, fps);
        }
    }
    return *interval != 0;
}

static bool select_camera_mode(const uvc_host_frame_info_t *frame, camera_mode_t *mode)
{
    if (frame->format != UVC_VS_FORMAT_MJPEG || frame->h_res == 0 || frame->v_res == 0) {
        return false;
    }

    uint32_t interval;
    float fps;
    if (!select_frame_interval(frame, &interval, &fps)) {
        return false;
    }

    if ((frame->h_res % 16) != 0 || (frame->v_res % 16) != 0 ||
        frame->h_res > MAX_DECODE_WIDTH || frame->v_res > MAX_DECODE_HEIGHT) {
        return false;
    }

    mode->width = frame->h_res;
    mode->height = frame->v_res;
    const float fit_scale = fminf((float)BSP_LCD_H_RES / frame->h_res,
                                  (float)BSP_LCD_V_RES / frame->v_res);
    mode->decoded_width = (unsigned)(frame->h_res * fit_scale) & ~1U;
    mode->decoded_height = (unsigned)(frame->v_res * fit_scale) & ~1U;
    mode->frame_interval = interval;
    mode->fps = fps;
    return true;
}

static void camera_driver_event_callback(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        return;
    }

    const size_t count = event->device_connected.frame_info_num;
    uvc_host_frame_info_t *frames = calloc(count, sizeof(*frames));
    if (frames == NULL) {
        ESP_LOGE(TAG, "Could not allocate camera mode list");
        return;
    }

    size_t frame_count = count;
    esp_err_t err = uvc_host_get_frame_list(
        event->device_connected.dev_addr,
        event->device_connected.uvc_stream_index,
        (uvc_host_frame_info_t (*)[])frames,
        &frame_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read camera modes: %s", esp_err_to_name(err));
        free(frames);
        return;
    }

    camera_mode_t best_mode = {0};
    uint64_t best_source_pixels = 0;
    for (size_t i = 0; i < frame_count; ++i) {
        camera_mode_t candidate = {0};
        ESP_LOGI(TAG, "Camera mode: %s (%d), %ux%u @ %.1f fps",
                 uvc_format_name(frames[i].format), frames[i].format,
                 frames[i].h_res, frames[i].v_res,
                 frames[i].default_interval
                     ? 10000000.0f / frames[i].default_interval
                     : 0.0f);
        if (!select_camera_mode(&frames[i], &candidate)) {
            continue;
        }

        const uint64_t source_pixels = (uint64_t)candidate.width * candidate.height;
        if (best_source_pixels == 0 || is_better_mode(&candidate, &best_mode) ||
            (candidate.fps == best_mode.fps && source_pixels < best_source_pixels)) {
            best_mode = candidate;
            best_source_pixels = source_pixels;
        }
    }
    free(frames);

    if (best_source_pixels == 0) {
        ESP_LOGE(TAG, "Camera has no suitable MJPEG mode for the LCD");
        return;
    }

    best_mode.dev_addr = event->device_connected.dev_addr;
    best_mode.stream_index = event->device_connected.uvc_stream_index;
    selected_mode = best_mode;
    ESP_LOGI(TAG, "Selected MJPEG mode: %ux%u @ %.1f fps (decoded to %ux%u)",
             selected_mode.width, selected_mode.height, selected_mode.fps,
             selected_mode.decoded_width, selected_mode.decoded_height);
    xSemaphoreGive(camera_available);
}

static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) {
        ESP_LOGW(TAG, "Ignoring unsupported UVC format %d", frame->vs_format.format);
        return true;
    }

    uvc_host_frame_t *queued_frame = (uvc_host_frame_t *)frame;
    return xQueueSend(frame_queue, &queued_frame, 0) != pdPASS;
}

static void process_frame(const uvc_host_frame_t *frame)
{
    if (frame->data_len > UINT32_MAX) {
        ESP_LOGW(TAG, "JPEG frame exceeds hardware decoder limits");
        return;
    }

    jpeg_decode_picture_info_t picture_info;
    esp_err_t err = jpeg_decoder_get_info(frame->data, (uint32_t)frame->data_len, &picture_info);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not read JPEG header: %s", esp_err_to_name(err));
        return;
    }

    if (picture_info.width != frame->vs_format.h_res ||
        picture_info.height != frame->vs_format.v_res ||
        picture_info.width > MAX_DECODE_WIDTH ||
        picture_info.height > MAX_DECODE_HEIGHT ||
        picture_info.width % 16 != 0 ||
        picture_info.height % 16 != 0) {
        ESP_LOGW(TAG, "Unsupported JPEG dimensions %ux%u",
                 picture_info.width, picture_info.height);
        return;
    }

    uint32_t output_size = 0;
    err = jpeg_decoder_process(jpeg_decoder, &jpeg_decode_config,
                               frame->data, (uint32_t)frame->data_len,
                               decode_frame, (uint32_t)decode_frame_size, &output_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Hardware JPEG decode failed: %s", esp_err_to_name(err));
        return;
    }
    if (output_size < picture_info.width * picture_info.height * sizeof(uint16_t)) {
        ESP_LOGW(TAG, "JPEG decoder returned a short frame (%" PRIu32 " bytes)", output_size);
        return;
    }

    const ppa_srm_oper_config_t scale_config = {
        .in = {
            .buffer = decode_frame,
            .pic_w = picture_info.width,
            .pic_h = picture_info.height,
            .block_w = picture_info.width,
            .block_h = picture_info.height,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = display_frame,
            .buffer_size = (uint32_t)display_frame_size,
            .pic_w = BSP_LCD_H_RES,
            .pic_h = BSP_LCD_V_RES,
            .block_offset_x = (BSP_LCD_H_RES - selected_mode.decoded_width) / 2,
            .block_offset_y = (BSP_LCD_V_RES - selected_mode.decoded_height) / 2,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = (float)selected_mode.decoded_width / picture_info.width,
        .scale_y = (float)selected_mode.decoded_height / picture_info.height,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    err = ppa_do_scale_rotate_mirror(ppa_client, &scale_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PPA scale failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_lcd_panel_draw_bitmap(display_panel, 0, 0,
                                    BSP_LCD_H_RES, BSP_LCD_V_RES, display_frame);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Display update failed: %s", esp_err_to_name(err));
        return;
    }

    static int64_t report_time;
    static unsigned displayed_frames;
    displayed_frames++;
    const int64_t now = esp_timer_get_time();
    if (report_time == 0) {
        report_time = now;
    } else if (now - report_time >= 1000000) {
        ESP_LOGI(TAG, "Displayed %.1f fps", displayed_frames * 1000000.0f / (now - report_time));
        displayed_frames = 0;
        report_time = now;
    }
}

static void frame_processing_task(void *arg)
{
    while (true) {
        uvc_host_frame_t *frame = NULL;
        xQueueReceive(frame_queue, &frame, portMAX_DELAY);
        if (frame == NULL) {
            xSemaphoreGive(frame_queue_drained);
            continue;
        }

        process_frame(frame);
        esp_err_t err = uvc_host_frame_return(active_stream, frame);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not return UVC frame: %s", esp_err_to_name(err));
        }
        vTaskDelay(1);
    }
}

static void stream_event_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB transfer failed: %s", esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED: {
        ESP_LOGW(TAG, "Camera disconnected");
        uvc_host_frame_t *drain_marker = NULL;
        xQueueSend(frame_queue, &drain_marker, portMAX_DELAY);
        xSemaphoreGive(camera_disconnected);
        break;
    }
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "UVC frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "UVC frame buffer underflow");
        break;
// #ifdef UVC_HOST_SUSPEND_RESUME_API_SUPPORTED
//     case UVC_HOST_DEVICE_SUSPENDED:
//         ESP_LOGW(TAG, "Camera suspended");
//         break;
//     case UVC_HOST_DEVICE_RESUMED:
//         ESP_LOGI(TAG, "Camera resumed");
//         break;
// #endif
    default:
        ESP_LOGW(TAG, "Unhandled UVC event %d", event->type);
        break;
    }
}

static void usb_host_task(void *arg)
{
    while (true) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
    }
}

void app_main(void)
{
    const bsp_display_config_t display_config = {
        .hdmi_resolution = BSP_HDMI_RES_NONE,
        .dsi_bus = {
            .phy_clk_src = 0,
            .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
        },
    };
    esp_lcd_panel_io_handle_t display_io;
    ESP_ERROR_CHECK(bsp_display_new(&display_config, &display_panel, &display_io));
    ESP_ERROR_CHECK(bsp_display_backlight_on());

    jpeg_decode_memory_alloc_cfg_t output_mem_config = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    decode_frame = jpeg_alloc_decoder_mem(
        MAX_DECODE_WIDTH * MAX_DECODE_HEIGHT * sizeof(uint16_t),
        &output_mem_config, &decode_frame_size);
    display_frame = jpeg_alloc_decoder_mem(
        BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t),
        &output_mem_config, &display_frame_size);
    ESP_LOGI(TAG, "PSRAM: total=%u free=%u largest_free=%u",
             (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    if (decode_frame == NULL || display_frame == NULL) {
        ESP_LOGE(TAG, "Could not allocate JPEG or display output buffers");
        abort();
    }
    const jpeg_decode_engine_cfg_t jpeg_engine_config = {
        .timeout_ms = JPEG_DECODE_TIMEOUT_MS,
    };
    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&jpeg_engine_config, &jpeg_decoder));
    const ppa_client_config_t ppa_config = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
        .data_burst_length = PPA_DATA_BURST_LENGTH_128,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_config, &ppa_client));

    camera_disconnected = xSemaphoreCreateBinary();
    camera_available = xSemaphoreCreateBinary();
    frame_queue_drained = xSemaphoreCreateBinary();
    frame_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(uvc_host_frame_t *));
    if (camera_disconnected == NULL || camera_available == NULL ||
        frame_queue_drained == NULL || frame_queue == NULL) {
        ESP_LOGE(TAG, "Could not create camera event semaphores");
        abort();
    }
    if (xTaskCreate(frame_processing_task, "frame_process", 8192, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not create frame processing task");
        abort();
    }

    const usb_host_config_t usb_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_LOGI(TAG, "Installing USB Host");
    ESP_ERROR_CHECK(usb_host_install(&usb_config));
    if (xTaskCreate(usb_host_task, "usb_host", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not create USB Host task");
        abort();
    }

    const uvc_host_driver_config_t driver_config = {
        .driver_task_stack_size = 6 * 1024,
        .driver_task_priority = 6,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = camera_driver_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&driver_config));

    while (true) {
        ESP_LOGI(TAG, "Waiting for camera and supported MJPEG mode");
        xSemaphoreTake(camera_available, portMAX_DELAY);

        const uvc_host_stream_config_t stream_config = {
            .event_cb = stream_event_callback,
            .frame_cb = frame_callback,
            .usb = {
                .dev_addr = selected_mode.dev_addr,
                .uvc_stream_index = selected_mode.stream_index,
            },
            .vs_format = {
                .h_res = selected_mode.width,
                .v_res = selected_mode.height,
                .fps = selected_mode.fps,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = CAMERA_FRAME_BUFFERS,
                .frame_size = CAMERA_FRAME_SIZE,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
                .number_of_urbs = 3,
                .urb_size = 4 * 1024,
            },
        };

        uvc_host_stream_hdl_t stream = NULL;
        ESP_LOGI(TAG, "Opening %ux%u MJPEG @ %.1f fps",
                 selected_mode.width, selected_mode.height, selected_mode.fps);
        esp_err_t err = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(5000), &stream);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Could not open camera mode: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            xSemaphoreGive(camera_available);
            continue;
        }

        active_stream = stream;
        err = uvc_host_stream_start(stream);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not start camera stream: %s", esp_err_to_name(err));
            ESP_ERROR_CHECK(uvc_host_stream_close(stream));
            active_stream = NULL;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Camera stream started");
        xSemaphoreTake(camera_disconnected, portMAX_DELAY);
        xSemaphoreTake(frame_queue_drained, portMAX_DELAY);
        ESP_ERROR_CHECK(uvc_host_stream_close(stream));
        active_stream = NULL;
        ESP_LOGI(TAG, "Waiting for camera reconnection");
    }
}

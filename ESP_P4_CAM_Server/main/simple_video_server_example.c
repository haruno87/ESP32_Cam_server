/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"
#include "mdns.h"
#include "lwip/inet.h"
#include "lwip/apps/netbiosns.h"
#include "example_video_common.h"

#define EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER  CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER

#define EXAMPLE_JPEG_ENC_QUALITY            CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY

#define EXAMPLE_MDNS_INSTANCE               CONFIG_EXAMPLE_MDNS_INSTANCE
#define EXAMPLE_MDNS_HOST_NAME              CONFIG_EXAMPLE_MDNS_HOST_NAME
#define EXAMPLE_STREAM_MAX_FPS              30
#define EXAMPLE_RECORD_DURATION_SEC         CONFIG_EXAMPLE_RECORD_DURATION_SEC
#define EXAMPLE_BUTTON_DEBOUNCE_MS          CONFIG_EXAMPLE_BUTTON_DEBOUNCE_MS
#define EXAMPLE_CAPTURE_BUTTON_GPIO         CONFIG_EXAMPLE_BUTTON_GPIO_CAPTURE
#define EXAMPLE_PREVIEW_BUTTON_GPIO         CONFIG_EXAMPLE_BUTTON_GPIO_PREVIEW
#define EXAMPLE_RECORD_BUTTON_GPIO          CONFIG_EXAMPLE_BUTTON_GPIO_RECORD
#define EXAMPLE_MAX_EVENT_CLIENTS           4

#define EXAMPLE_PART_BOUNDARY               CONFIG_EXAMPLE_HTTP_PART_BOUNDARY

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" EXAMPLE_PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" EXAMPLE_PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";
static const char *PHOTO_UPLOAD_PATH = "/api/photo";
static const char *RECORD_TRIGGER_PATH = "/api/record";

extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");
extern const uint8_t loading_jpg_gz_start[] asm("_binary_loading_jpg_gz_start");
extern const uint8_t loading_jpg_gz_end[] asm("_binary_loading_jpg_gz_end");
extern const uint8_t favicon_ico_gz_start[] asm("_binary_favicon_ico_gz_start");
extern const uint8_t favicon_ico_gz_end[] asm("_binary_favicon_ico_gz_end");
extern const uint8_t assets_index_js_gz_start[] asm("_binary_index_js_gz_start");
extern const uint8_t assets_index_js_gz_end[] asm("_binary_index_js_gz_end");
extern const uint8_t assets_index_css_gz_start[] asm("_binary_index_css_gz_start");
extern const uint8_t assets_index_css_gz_end[] asm("_binary_index_css_gz_end");

/**
 * @brief Web cam control structure
 */
typedef struct web_cam_video {
    int fd;
    uint8_t index;
    char dev_name[32];

    example_encoder_handle_t encoder_handle;
    uint8_t *jpeg_out_buf;
    uint32_t jpeg_out_size;

    uint8_t *buffer[EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER];
    uint32_t buffer_size;

    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t jpeg_quality;

    uint32_t frame_rate;

    SemaphoreHandle_t sem;
    SemaphoreHandle_t io_lock;

    struct web_cam_image_format_option *format_options;
    uint32_t format_option_count;
    int current_format_index;

    uint32_t support_control_jpeg_quality   : 1;
    uint32_t stream_on                      : 1;
} web_cam_video_t;

typedef struct web_cam {
    uint8_t video_count;
    web_cam_video_t video[0];
} web_cam_t;

typedef struct web_cam_video_config {
    const char *dev_name;
    uint32_t buffer_count;
} web_cam_video_config_t;

typedef struct request_desc {
    int index;
} request_desc_t;

typedef struct web_cam_image_format_option {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t frame_interval_num;
    uint32_t frame_interval_den;
} web_cam_image_format_option_t;

typedef struct camera_runtime_state {
    web_cam_t *web_cam;
    SemaphoreHandle_t lock;
    TimerHandle_t record_timer;
    bool preview_requested;
    bool recording_active;
    bool capture_in_progress;
    bool stream_enabled;
} camera_runtime_state_t;

typedef struct event_client_registry {
    httpd_handle_t server;
    SemaphoreHandle_t lock;
    int fds[EXAMPLE_MAX_EVENT_CLIENTS];
} event_client_registry_t;

static const char *TAG = "example";
static camera_runtime_state_t s_runtime = {0};
static event_client_registry_t s_event_clients = {0};

static esp_err_t init_web_cam_video(web_cam_video_t *video, const web_cam_video_config_t *config, int index,
                                    const web_cam_image_format_option_t *requested_format);
static esp_err_t deinit_web_cam_video(web_cam_video_t *video);
static bool is_valid_web_cam(web_cam_video_t *video);
static esp_err_t set_runtime_stream_enabled(bool enable);
static esp_err_t apply_runtime_stream_policy(void);
static bool runtime_should_stream(void);
static esp_err_t post_capture_to_pc_helper(const uint8_t *jpeg_data, size_t jpeg_size);
static esp_err_t trigger_record_on_pc_helper(uint32_t duration_sec);
static esp_err_t broadcast_button_event(const char *event_name);

static void log_access_urls(void)
{
    esp_netif_t *netif = get_example_netif();
    esp_netif_ip_info_t ip_info = {0};

    if (netif == NULL) {
        ESP_LOGW(TAG, "No active network interface found, skip access URL logging");
        return;
    }

    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
        ESP_LOGW(TAG, "Network interface has no IPv4 address yet");
        return;
    }

    ESP_LOGI(TAG, "Web UI: http://" IPSTR "/", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Legacy capture: http://" IPSTR "/capture", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Legacy status: http://" IPSTR "/status", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "MJPEG stream: http://" IPSTR ":81/stream", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "mDNS URL: http://%s.local/", EXAMPLE_MDNS_HOST_NAME);
}

static esp_err_t get_board_ipv4_string(char *buffer, size_t buffer_size)
{
    esp_netif_t *netif = get_example_netif();
    esp_netif_ip_info_t ip_info = {0};

    ESP_RETURN_ON_FALSE(netif != NULL, ESP_ERR_INVALID_STATE, TAG, "no active network interface found");
    ESP_RETURN_ON_ERROR(esp_netif_get_ip_info(netif, &ip_info), TAG, "failed to get ip info");
    ESP_RETURN_ON_FALSE(ip_info.ip.addr != 0, ESP_ERR_INVALID_STATE, TAG, "board has no ipv4 address");
    ESP_RETURN_ON_FALSE(snprintf(buffer, buffer_size, IPSTR, IP2STR(&ip_info.ip)) > 0, ESP_FAIL, TAG, "failed to format ip string");
    return ESP_OK;
}

static bool pc_helper_is_configured(void)
{
    return strlen(CONFIG_EXAMPLE_PC_RECEIVER_BASE_URL) > 0;
}

static esp_err_t build_pc_helper_url(const char *path, char *buffer, size_t buffer_size)
{
    ESP_RETURN_ON_FALSE(pc_helper_is_configured(), ESP_ERR_NOT_FOUND, TAG, "pc receiver base url is empty");
    ESP_RETURN_ON_FALSE(snprintf(buffer, buffer_size, "%s%s", CONFIG_EXAMPLE_PC_RECEIVER_BASE_URL, path) > 0,
                        ESP_FAIL, TAG, "failed to build pc helper url");
    return ESP_OK;
}

static esp_err_t init_event_client_registry(void)
{
    if (s_event_clients.lock == NULL) {
        s_event_clients.lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_event_clients.lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create event client lock");
    }

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        s_event_clients.fds[i] = -1;
    }
    return ESP_OK;
}

static void register_event_client(httpd_handle_t server, int fd)
{
    if (fd < 0 || s_event_clients.lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_event_clients.lock, portMAX_DELAY) != pdPASS) {
        return;
    }

    s_event_clients.server = server;
    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (s_event_clients.fds[i] == fd) {
            xSemaphoreGive(s_event_clients.lock);
            return;
        }
    }

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (s_event_clients.fds[i] == -1) {
            s_event_clients.fds[i] = fd;
            ESP_LOGI(TAG, "event client connected on socket %d", fd);
            xSemaphoreGive(s_event_clients.lock);
            return;
        }
    }

    xSemaphoreGive(s_event_clients.lock);
    ESP_LOGW(TAG, "event client list is full, ignoring socket %d", fd);
}

static void unregister_event_client(int fd)
{
    if (fd < 0 || s_event_clients.lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_event_clients.lock, portMAX_DELAY) != pdPASS) {
        return;
    }

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (s_event_clients.fds[i] == fd) {
            s_event_clients.fds[i] = -1;
            ESP_LOGI(TAG, "event client disconnected on socket %d", fd);
            break;
        }
    }

    xSemaphoreGive(s_event_clients.lock);
}

static esp_err_t broadcast_button_event(const char *event_name)
{
    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)event_name,
        .len = strlen(event_name),
    };
    int fds[EXAMPLE_MAX_EVENT_CLIENTS];
    httpd_handle_t server;

    ESP_RETURN_ON_FALSE(event_name != NULL, ESP_ERR_INVALID_ARG, TAG, "event name is null");
    ESP_RETURN_ON_FALSE(s_event_clients.lock != NULL, ESP_ERR_INVALID_STATE, TAG, "event registry is not initialized");

    if (xSemaphoreTake(s_event_clients.lock, portMAX_DELAY) != pdPASS) {
        return ESP_FAIL;
    }

    server = s_event_clients.server;
    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        fds[i] = s_event_clients.fds[i];
    }
    xSemaphoreGive(s_event_clients.lock);

    ESP_RETURN_ON_FALSE(server != NULL, ESP_ERR_INVALID_STATE, TAG, "event server is not ready");

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (fds[i] < 0) {
            continue;
        }

        if (httpd_ws_get_fd_info(server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) {
            unregister_event_client(fds[i]);
            continue;
        }

        if (httpd_ws_send_frame_async(server, fds[i], &frame) != ESP_OK) {
            ESP_LOGW(TAG, "failed to push event '%s' to socket %d", event_name, fds[i]);
            unregister_event_client(fds[i]);
        }
    }

    return ESP_OK;
}

static esp_err_t set_cors_headers(httpd_req_t *req)
{
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"), TAG, "failed to set allow-origin");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*"), TAG, "failed to set allow-headers");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS"), TAG, "failed to set allow-methods");
    return ESP_OK;
}

static bool runtime_should_stream(void)
{
    return s_runtime.preview_requested || s_runtime.recording_active;
}

static esp_err_t set_runtime_stream_enabled(bool enable)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ESP_RETURN_ON_FALSE(s_runtime.web_cam != NULL, ESP_ERR_INVALID_STATE, TAG, "runtime camera is not initialized");

    if (s_runtime.stream_enabled == enable) {
        return ESP_OK;
    }

    for (int i = 0; i < s_runtime.web_cam->video_count; i++) {
        web_cam_video_t *video = &s_runtime.web_cam->video[i];

        if (!is_valid_web_cam(video)) {
            continue;
        }

        if (enable) {
            ESP_RETURN_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take io lock");
            int ret = ioctl(video->fd, VIDIOC_STREAMON, &type);
            xSemaphoreGive(video->io_lock);
            ESP_RETURN_ON_FALSE(ret == 0, ESP_FAIL, TAG, "failed to stream on video%d", video->index);
            video->stream_on = 1;
        } else if (video->stream_on) {
            ESP_RETURN_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take io lock");
            ioctl(video->fd, VIDIOC_STREAMOFF, &type);
            xSemaphoreGive(video->io_lock);
            video->stream_on = 0;
        }
    }

    s_runtime.stream_enabled = enable;
    ESP_LOGI(TAG, "camera stream %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

static esp_err_t apply_runtime_stream_policy(void)
{
    bool should_stream;

    ESP_RETURN_ON_FALSE(s_runtime.lock != NULL, ESP_ERR_INVALID_STATE, TAG, "runtime lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take runtime lock");
    should_stream = runtime_should_stream();
    xSemaphoreGive(s_runtime.lock);

    return set_runtime_stream_enabled(should_stream);
}

static esp_err_t capture_video_image_to_memory(web_cam_video_t *video, bool is_jpeg, uint8_t **out_data, size_t *out_size)
{
    esp_err_t ret = ESP_OK;
    struct v4l2_buffer buf = {0};
    uint8_t *payload = NULL;
    uint32_t payload_size = 0;
    bool io_locked = false;
    bool sem_locked = false;

    ESP_RETURN_ON_FALSE(out_data && out_size, ESP_ERR_INVALID_ARG, TAG, "invalid capture output buffer");

    *out_data = NULL;
    *out_size = 0;

    ESP_GOTO_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, fail, TAG, "failed to take io lock");
    io_locked = true;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl(video->fd, VIDIOC_DQBUF, &buf), fail, TAG, "failed to receive video frame");
    ESP_GOTO_ON_FALSE(buf.flags & V4L2_BUF_FLAG_DONE, ESP_ERR_INVALID_RESPONSE, fail, TAG, "received video frame is not done");

    if (!is_jpeg || video->pixel_format == V4L2_PIX_FMT_JPEG) {
        payload_size = buf.bytesused;
        payload = malloc(payload_size);
        ESP_GOTO_ON_FALSE(payload != NULL, ESP_ERR_NO_MEM, fail, TAG, "failed to allocate jpeg copy buffer");
        memcpy(payload, video->buffer[buf.index], payload_size);
    } else {
        ESP_GOTO_ON_FALSE(xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS, ESP_FAIL, fail, TAG, "failed to take semaphore");
        sem_locked = true;
        ESP_GOTO_ON_ERROR(example_encoder_process(video->encoder_handle, video->buffer[buf.index], video->buffer_size,
                                                  video->jpeg_out_buf, video->jpeg_out_size, &payload_size),
                          fail, TAG, "failed to encode capture frame");
        payload = malloc(payload_size);
        ESP_GOTO_ON_FALSE(payload != NULL, ESP_ERR_NO_MEM, fail, TAG, "failed to allocate encoded jpeg copy buffer");
        memcpy(payload, video->jpeg_out_buf, payload_size);
    }

    ESP_GOTO_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), fail, TAG, "failed to re-queue video frame");

    if (sem_locked) {
        xSemaphoreGive(video->sem);
        sem_locked = false;
    }
    if (io_locked) {
        xSemaphoreGive(video->io_lock);
        io_locked = false;
    }

    *out_data = payload;
    *out_size = payload_size;
    return ESP_OK;

fail:
    if (buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
        ioctl(video->fd, VIDIOC_QBUF, &buf);
    }
    if (sem_locked) {
        xSemaphoreGive(video->sem);
    }
    if (io_locked) {
        xSemaphoreGive(video->io_lock);
    }
    free(payload);
    return ret;
}

static esp_err_t http_post_binary(const char *url, const char *content_type, const char *filename, const uint8_t *data, size_t size)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
    };
    char board_ip[32] = {0};
    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_RETURN_ON_FALSE(client != NULL, ESP_ERR_NO_MEM, TAG, "failed to init http client");

    if (get_board_ipv4_string(board_ip, sizeof(board_ip)) == ESP_OK) {
        esp_http_client_set_header(client, "X-Board-IP", board_ip);
    }
    esp_http_client_set_header(client, "Content-Type", content_type);
    if (filename && filename[0] != '\0') {
        esp_http_client_set_header(client, "X-Filename", filename);
    }
    esp_http_client_set_post_field(client, (const char *)data, size);

    esp_err_t ret = esp_http_client_perform(client);
    if (ret == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code < 200 || status_code >= 300) {
            ret = ESP_FAIL;
            ESP_LOGE(TAG, "pc helper returned http status %d", status_code);
        }
    }

    esp_http_client_cleanup(client);
    return ret;
}

static esp_err_t http_post_json(const char *url, const char *json_body)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_RETURN_ON_FALSE(client != NULL, ESP_ERR_NO_MEM, TAG, "failed to init http client");
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    esp_err_t ret = esp_http_client_perform(client);
    if (ret == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code < 200 || status_code >= 300) {
            ret = ESP_FAIL;
            ESP_LOGE(TAG, "pc helper returned http status %d", status_code);
        }
    }

    esp_http_client_cleanup(client);
    return ret;
}

static esp_err_t post_capture_to_pc_helper(const uint8_t *jpeg_data, size_t jpeg_size)
{
    char url[192];
    char filename[64];
    int64_t timestamp_ms = esp_timer_get_time() / 1000;

    ESP_RETURN_ON_ERROR(build_pc_helper_url(PHOTO_UPLOAD_PATH, url, sizeof(url)), TAG, "failed to build photo upload url");
    ESP_RETURN_ON_FALSE(snprintf(filename, sizeof(filename), "esp32p4_capture_%lld.jpg", (long long)timestamp_ms) > 0,
                        ESP_FAIL, TAG, "failed to build photo file name");
    return http_post_binary(url, "image/jpeg", filename, jpeg_data, jpeg_size);
}

static esp_err_t trigger_record_on_pc_helper(uint32_t duration_sec)
{
    char url[192];
    char board_ip[32];
    char stream_url[96];
    char json_body[256];

    ESP_RETURN_ON_ERROR(build_pc_helper_url(RECORD_TRIGGER_PATH, url, sizeof(url)), TAG, "failed to build record trigger url");
    ESP_RETURN_ON_ERROR(get_board_ipv4_string(board_ip, sizeof(board_ip)), TAG, "failed to get board ip");
    ESP_RETURN_ON_FALSE(snprintf(stream_url, sizeof(stream_url), "http://%s:81/stream", board_ip) > 0,
                        ESP_FAIL, TAG, "failed to build stream url");
    ESP_RETURN_ON_FALSE(snprintf(json_body, sizeof(json_body),
                                 "{\"board_ip\":\"%s\",\"stream_url\":\"%s\",\"duration_s\":%" PRIu32 "}",
                                 board_ip, stream_url, duration_sec) > 0,
                        ESP_FAIL, TAG, "failed to build record json");
    return http_post_json(url, json_body);
}

static bool is_valid_web_cam(web_cam_video_t *video)
{
    return video->fd != -1;
}

static uint32_t get_effective_stream_fps(const web_cam_video_t *video)
{
    if (video->frame_rate == 0) {
        return EXAMPLE_STREAM_MAX_FPS;
    }
    return video->frame_rate > EXAMPLE_STREAM_MAX_FPS ? EXAMPLE_STREAM_MAX_FPS : video->frame_rate;
}

static void free_video_format_options(web_cam_video_t *video)
{
    free(video->format_options);
    video->format_options = NULL;
    video->format_option_count = 0;
    video->current_format_index = -1;
}

static bool is_same_format_option(const web_cam_image_format_option_t *lhs, const web_cam_image_format_option_t *rhs)
{
    return lhs->pixel_format == rhs->pixel_format &&
           lhs->width == rhs->width &&
           lhs->height == rhs->height &&
           lhs->frame_interval_num == rhs->frame_interval_num &&
           lhs->frame_interval_den == rhs->frame_interval_den;
}

static bool is_encoder_pixel_format_supported(uint32_t pixel_format)
{
#if CONFIG_EXAMPLE_SELECT_JPEG_HW_DRIVER
    switch (pixel_format) {
    case V4L2_PIX_FMT_JPEG:
    case V4L2_PIX_FMT_SBGGR8:
    case V4L2_PIX_FMT_GREY:
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_RGB24:
    case V4L2_PIX_FMT_UYVY:
#if CONFIG_ESP32P4_REV_MIN_FULL >= 300
    case V4L2_PIX_FMT_YUV420:
    case V4L2_PIX_FMT_YUV444:
#endif
        return true;
    default:
        return false;
    }
#else
    switch (pixel_format) {
    case V4L2_PIX_FMT_JPEG:
    case V4L2_PIX_FMT_SBGGR8:
    case V4L2_PIX_FMT_GREY:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_RGB565X:
        return true;
    default:
        return false;
    }
#endif
}

static esp_err_t append_video_format_option(web_cam_video_t *video, const web_cam_image_format_option_t *option)
{
    for (uint32_t i = 0; i < video->format_option_count; i++) {
        if (is_same_format_option(&video->format_options[i], option)) {
            return ESP_OK;
        }
    }

    web_cam_image_format_option_t *new_options = realloc(video->format_options,
                                                         sizeof(web_cam_image_format_option_t) * (video->format_option_count + 1));
    ESP_RETURN_ON_FALSE(new_options, ESP_ERR_NO_MEM, TAG, "failed to grow format option list");

    video->format_options = new_options;
    video->format_options[video->format_option_count] = *option;
    video->format_option_count++;
    return ESP_OK;
}

static int find_current_format_index(const web_cam_video_t *video)
{
    for (uint32_t i = 0; i < video->format_option_count; i++) {
        const web_cam_image_format_option_t *option = &video->format_options[i];
        uint32_t fps = 0;

        if (option->frame_interval_num != 0) {
            fps = option->frame_interval_den / option->frame_interval_num;
        }

        if (option->pixel_format == video->pixel_format &&
            option->width == video->width &&
            option->height == video->height &&
            (fps == 0 || fps == video->frame_rate)) {
            return (int)i;
        }
    }

    return -1;
}

static esp_err_t enumerate_video_format_options(web_cam_video_t *video, int fd)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    free_video_format_options(video);

    for (int fmt_index = 0; ; fmt_index++) {
        struct v4l2_fmtdesc fmtdesc = {
            .index = fmt_index,
            .type = type,
        };

        if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
            break;
        }

        if (!is_encoder_pixel_format_supported(fmtdesc.pixelformat)) {
            ESP_LOGI(TAG, "skip unsupported stream format %c%c%c%c",
                     V4L2_FMT_STR_ARG(fmtdesc.pixelformat));
            continue;
        }

        for (int size_index = 0; ; size_index++) {
            struct v4l2_frmsizeenum frmsize = {
                .index = size_index,
                .pixel_format = fmtdesc.pixelformat,
                .type = type,
            };

            if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) != 0) {
                if (size_index == 0) {
                    web_cam_image_format_option_t option = {
                        .pixel_format = fmtdesc.pixelformat,
                        .width = video->width,
                        .height = video->height,
                        .frame_interval_num = 1,
                        .frame_interval_den = video->frame_rate,
                    };
                    ESP_RETURN_ON_ERROR(append_video_format_option(video, &option), TAG, "failed to append fallback format option");
                }
                break;
            }

            bool has_interval = false;
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                for (int interval_index = 0; ; interval_index++) {
                    struct v4l2_frmivalenum frmival = {
                        .index = interval_index,
                        .pixel_format = fmtdesc.pixelformat,
                        .type = type,
                        .width = frmsize.discrete.width,
                        .height = frmsize.discrete.height,
                    };

                    if (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) != 0) {
                        break;
                    }

                    if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                        web_cam_image_format_option_t option = {
                            .pixel_format = fmtdesc.pixelformat,
                            .width = frmsize.discrete.width,
                            .height = frmsize.discrete.height,
                            .frame_interval_num = frmival.discrete.numerator,
                            .frame_interval_den = frmival.discrete.denominator,
                        };
                        ESP_RETURN_ON_ERROR(append_video_format_option(video, &option), TAG, "failed to append format option");
                        has_interval = true;
                    }
                }
            }

            if (!has_interval && frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                web_cam_image_format_option_t option = {
                    .pixel_format = fmtdesc.pixelformat,
                    .width = frmsize.discrete.width,
                    .height = frmsize.discrete.height,
                    .frame_interval_num = 1,
                    .frame_interval_den = video->frame_rate,
                };
                ESP_RETURN_ON_ERROR(append_video_format_option(video, &option), TAG, "failed to append default fps format option");
            }
        }
    }

    video->current_format_index = find_current_format_index(video);
    return ESP_OK;
}

static esp_err_t release_video_buffers(web_cam_video_t *video)
{
    struct v4l2_requestbuffers req = {
        .count = 0,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        if (video->buffer[i]) {
            munmap(video->buffer[i], video->buffer_size);
            video->buffer[i] = NULL;
        }
    }
    video->buffer_size = 0;

    if (video->fd != -1) {
        ioctl(video->fd, VIDIOC_REQBUFS, &req);
    }

    return ESP_OK;
}

static void format_option_description(const web_cam_image_format_option_t *option, char *buffer, size_t buffer_size)
{
    uint32_t fps = 0;
    uint32_t display_fps = 0;

    if (option->frame_interval_num != 0) {
        fps = option->frame_interval_den / option->frame_interval_num;
    }

    if (fps > 0) {
        display_fps = fps > EXAMPLE_STREAM_MAX_FPS ? EXAMPLE_STREAM_MAX_FPS : fps;
        snprintf(buffer, buffer_size, "%c%c%c%c %" PRIu32 "x%" PRIu32 " @ %" PRIu32 "fps",
                 V4L2_FMT_STR_ARG(option->pixel_format), option->width, option->height, display_fps);
    } else {
        snprintf(buffer, buffer_size, "%c%c%c%c %" PRIu32 "x%" PRIu32,
                 V4L2_FMT_STR_ARG(option->pixel_format), option->width, option->height);
    }
}

static web_cam_video_t *get_default_video(web_cam_t *web_cam)
{
    for (int i = 0; i < web_cam->video_count; i++) {
        if (is_valid_web_cam(&web_cam->video[i])) {
            return &web_cam->video[i];
        }
    }
    return NULL;
}

static esp_err_t decode_request(web_cam_t *web_cam, httpd_req_t *req, request_desc_t *desc)
{
    esp_err_t ret;
    int index = -1;
    char buffer[32];

    if ((ret = httpd_req_get_url_query_str(req, buffer, sizeof(buffer))) != ESP_OK) {
        return ret;
    }
    ESP_LOGD(TAG, "source: %s", buffer);

    for (int i = 0; i < web_cam->video_count; i++) {
        char source_str[16];

        if (snprintf(source_str, sizeof(source_str), "source=%d", i) <= 0) {
            return ESP_FAIL;
        }

        if (strcmp(buffer, source_str) == 0) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    desc->index = index;
    return ESP_OK;
}

static esp_err_t capture_video_image(httpd_req_t *req, web_cam_video_t *video, bool is_jpeg)
{
    esp_err_t ret;
    struct v4l2_buffer buf = {0};
    const char *type_str = is_jpeg ? "JPEG" : "binary";
    uint32_t jpeg_encoded_size;
    bool io_locked = false;

    ESP_GOTO_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, fail0, TAG, "failed to take io lock");
    io_locked = true;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_DQBUF, &buf), TAG, "failed to receive video frame");
    if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (!is_jpeg || video->pixel_format == V4L2_PIX_FMT_JPEG) {
        /* Directly send the buffer of raw data */
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->buffer[buf.index], buf.bytesused), fail0, TAG, "failed to send %s", type_str);
        jpeg_encoded_size = buf.bytesused;
    } else {
        ESP_GOTO_ON_FALSE(xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS, ESP_FAIL, fail0, TAG, "failed to take semaphore");
        ret = example_encoder_process(video->encoder_handle, video->buffer[buf.index], video->buffer_size,
                                      video->jpeg_out_buf, video->jpeg_out_size, &jpeg_encoded_size);
        xSemaphoreGive(video->sem);
        ESP_GOTO_ON_ERROR(ret, fail0, TAG, "failed to encode video frame");
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->jpeg_out_buf, jpeg_encoded_size), fail0, TAG, "failed to send %s", type_str);
    }

    ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue video frame");

    ESP_GOTO_ON_ERROR(httpd_resp_sendstr_chunk(req, NULL), fail0, TAG, "failed to send null");

    ESP_LOGD(TAG, "send %s image%d size: %" PRIu32, type_str, video->index, jpeg_encoded_size);

    xSemaphoreGive(video->io_lock);
    return ESP_OK;

fail0:
    if (io_locked) {
        xSemaphoreGive(video->io_lock);
    }
    ioctl(video->fd, VIDIOC_QBUF, &buf);
    return ret;
}

static char *get_cameras_json(web_cam_t *web_cam)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *cameras = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "cameras", cameras);

    for (int i = 0; i < web_cam->video_count; i++) {
        char src_str[32];

        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        cJSON *camera = cJSON_CreateObject();
        web_cam_image_format_option_t current_option = {
            .pixel_format = web_cam->video[i].pixel_format,
            .width = web_cam->video[i].width,
            .height = web_cam->video[i].height,
            .frame_interval_num = 1,
            .frame_interval_den = web_cam->video[i].frame_rate,
        };
        cJSON_AddNumberToObject(camera, "index", i);
        assert(snprintf(src_str, sizeof(src_str), ":%d/stream", i + 81) > 0);
        cJSON_AddStringToObject(camera, "src", src_str);
        cJSON_AddNumberToObject(camera, "currentFrameRate", get_effective_stream_fps(&web_cam->video[i]));
        cJSON_AddNumberToObject(camera, "currentImageFormat", web_cam->video[i].current_format_index >= 0 ? web_cam->video[i].current_format_index : 0);
        format_option_description(&current_option, src_str, sizeof(src_str));
        cJSON_AddStringToObject(camera, "currentImageFormatDescription", src_str);

        if (web_cam->video[i].support_control_jpeg_quality) {
            cJSON_AddNumberToObject(camera, "currentQuality", web_cam->video[i].jpeg_quality);
        }

        cJSON *current_resolution = cJSON_CreateObject();
        cJSON_AddNumberToObject(current_resolution, "width", web_cam->video[i].width);
        cJSON_AddNumberToObject(current_resolution, "height", web_cam->video[i].height);
        cJSON_AddItemToObject(camera, "currentResolution", current_resolution);

        cJSON *image_formats = cJSON_CreateArray();

        for (uint32_t format_index = 0; format_index < web_cam->video[i].format_option_count; format_index++) {
            cJSON *image_format = cJSON_CreateObject();
            cJSON_AddNumberToObject(image_format, "id", format_index);
            format_option_description(&web_cam->video[i].format_options[format_index], src_str, sizeof(src_str));
            cJSON_AddStringToObject(image_format, "description", src_str);

            if (web_cam->video[i].support_control_jpeg_quality) {
                cJSON *image_format_quality = cJSON_CreateObject();

                int min_quality = 1;
                int max_quality = 100;
                int step_quality = 1;
                int default_quality = EXAMPLE_JPEG_ENC_QUALITY;
                if (web_cam->video[i].pixel_format == V4L2_PIX_FMT_JPEG) {
                    struct v4l2_query_ext_ctrl qctrl = {0};

                    qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
                    if (ioctl(web_cam->video[i].fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
                        min_quality = qctrl.minimum;
                        max_quality = qctrl.maximum;
                        step_quality = qctrl.step;
                        default_quality = qctrl.default_value;
                    }
                }

                cJSON_AddNumberToObject(image_format_quality, "min", min_quality);
                cJSON_AddNumberToObject(image_format_quality, "max", max_quality);
                cJSON_AddNumberToObject(image_format_quality, "step", step_quality);
                cJSON_AddNumberToObject(image_format_quality, "default", default_quality);
                cJSON_AddItemToObject(image_format, "quality", image_format_quality);
            }
            cJSON_AddItemToArray(image_formats, image_format);
        }

        cJSON_AddItemToObject(camera, "imageFormats", image_formats);
        cJSON_AddItemToArray(cameras, camera);
    }

    char *output = cJSON_Print(root);
    cJSON_Delete(root);
    return output;
}

static esp_err_t set_camera_jpeg_quality(web_cam_video_t *video, int quality)
{
    esp_err_t ret = ESP_OK;
    int quality_reset = quality;

    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        struct v4l2_query_ext_ctrl qctrl = {0};

        qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
        if (ioctl(video->fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
            if ((quality > qctrl.maximum) || (quality < qctrl.minimum) ||
                    (((quality - qctrl.minimum) % qctrl.step) != 0)) {

                if (quality > qctrl.maximum) {
                    quality_reset = qctrl.maximum;
                } else if (quality < qctrl.minimum) {
                    quality_reset = qctrl.minimum;
                } else {
                    quality_reset = qctrl.minimum + ((quality - qctrl.minimum) / qctrl.step) * qctrl.step;
                }

                ESP_LOGW(TAG, "video%d: JPEG compression quality=%d is out of sensor's range, reset to %d", video->index, quality, quality_reset);
            }

            controls.ctrl_class = V4L2_CID_JPEG_CLASS;
            controls.count = 1;
            controls.controls = control;
            control[0].id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
            control[0].value = quality_reset;
            ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_S_EXT_CTRLS, &controls), TAG, "failed to set jpeg compression quality");

            video->jpeg_quality = quality_reset;
            video->support_control_jpeg_quality = 1;
        } else {
            video->support_control_jpeg_quality = 0;
            ESP_LOGW(TAG, "video%d: JPEG compression quality control is not supported", video->index);
        }
    } else {
        ESP_RETURN_ON_ERROR(example_encoder_set_jpeg_quality(video->encoder_handle, quality_reset), TAG, "failed to set jpeg quality");
        video->jpeg_quality = quality_reset;
    }

    if (video->support_control_jpeg_quality) {
        ESP_LOGI(TAG, "video%d: set jpeg quality %d success", video->index, quality_reset);
    }

    return ret;
}

static esp_err_t camera_info_handler(httpd_req_t *req)
{
    esp_err_t ret;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    char *output = get_cameras_json(web_cam);

    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_sendstr(req, output);
    free(output);

    return ret;
}

static esp_err_t set_camera_image_format(web_cam_video_t *video, int image_format_index)
{
    esp_err_t ret;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int stream_ret;
    web_cam_image_format_option_t selected_format;
    web_cam_image_format_option_t previous_format = {
        .pixel_format = video->pixel_format,
        .width = video->width,
        .height = video->height,
        .frame_interval_num = 1,
        .frame_interval_den = video->frame_rate,
    };
    web_cam_video_config_t config = {
        .dev_name = video->dev_name,
    };

    ESP_RETURN_ON_FALSE(image_format_index >= 0 && image_format_index < (int)video->format_option_count,
                        ESP_ERR_INVALID_ARG, TAG, "invalid image format index");

    if (image_format_index == video->current_format_index) {
        return ESP_OK;
    }

    selected_format = video->format_options[image_format_index];
    ESP_RETURN_ON_FALSE(is_encoder_pixel_format_supported(selected_format.pixel_format), ESP_ERR_NOT_SUPPORTED,
                        TAG, "selected pixel format is not supported by encoder");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take io lock");

    deinit_web_cam_video(video);
    ret = init_web_cam_video(video, &config, video->index, &selected_format);
    if (ret == ESP_OK) {
        stream_ret = ioctl(video->fd, VIDIOC_STREAMON, &type);
        if (stream_ret == 0) {
            video->stream_on = 1;
            ret = ESP_OK;
        } else {
            ret = ESP_FAIL;
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "video%d: failed to switch format, rolling back to previous format", video->index);
        deinit_web_cam_video(video);
        if (init_web_cam_video(video, &config, video->index, &previous_format) == ESP_OK) {
            if (ioctl(video->fd, VIDIOC_STREAMON, &type) != 0) {
                ESP_LOGE(TAG, "video%d: rollback stream on failed", video->index);
            } else {
                video->stream_on = 1;
            }
        } else {
            ESP_LOGE(TAG, "video%d: rollback init failed", video->index);
        }
    }

    xSemaphoreGive(video->io_lock);
    if (ret == ESP_OK && !runtime_should_stream()) {
        ESP_RETURN_ON_ERROR(set_runtime_stream_enabled(false), TAG, "failed to restore stream policy after format switch");
    }
    return ret;
}

static esp_err_t legacy_status_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    web_cam_video_t *video = get_default_video(web_cam);
    cJSON *root = cJSON_CreateObject();

    ESP_RETURN_ON_FALSE(root, ESP_ERR_NO_MEM, TAG, "failed to allocate status json");
    ESP_GOTO_ON_FALSE(video, ESP_ERR_NOT_FOUND, fail0, TAG, "no active camera stream found");

    cJSON_AddNumberToObject(root, "index", video->index);
    cJSON_AddNumberToObject(root, "width", video->width);
    cJSON_AddNumberToObject(root, "height", video->height);
    cJSON_AddNumberToObject(root, "frame_rate", get_effective_stream_fps(video));
    cJSON_AddNumberToObject(root, "jpeg_quality", video->jpeg_quality);
    cJSON_AddStringToObject(root, "stream_path", "/stream");
    cJSON_AddStringToObject(root, "capture_path", "/capture");

    char *output = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    ESP_RETURN_ON_FALSE(output, ESP_ERR_NO_MEM, TAG, "failed to render status json");

    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_sendstr(req, output);
    free(output);
    return ret;

fail0:
    cJSON_Delete(root);
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

static esp_err_t camera_settings_handler(httpd_req_t *req)
{
    esp_err_t ret;
    char *content;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    set_cors_headers(req);
    content = (char *)calloc(1, req->content_len + 1);
    ESP_RETURN_ON_FALSE(content, ESP_ERR_NO_MEM, TAG, "failed to allocate memory");

    ESP_GOTO_ON_FALSE(httpd_req_recv(req, content, req->content_len) > 0, ESP_FAIL, fail0, TAG, "failed to recv content");
    ESP_LOGD(TAG, "content: %s", content);

    cJSON *json_root = cJSON_Parse(content);
    free(content);
    content = NULL;
    ESP_GOTO_ON_FALSE(json_root, ESP_FAIL, fail0, TAG, "failed to parse JSON");

    cJSON *json_index = cJSON_GetObjectItem(json_root, "index");
    ESP_GOTO_ON_FALSE(json_index && cJSON_IsNumber(json_index), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid index field");
    int index = json_index->valueint;
    ESP_GOTO_ON_FALSE(index >= 0 && index < web_cam->video_count && is_valid_web_cam(&web_cam->video[index]), ESP_ERR_INVALID_ARG, fail1, TAG, "invalid index");

    cJSON *json_image_format = cJSON_GetObjectItem(json_root, "image_format");
    ESP_GOTO_ON_FALSE(json_image_format && cJSON_IsNumber(json_image_format), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid image_format field");
    int image_format = json_image_format->valueint;

    cJSON *json_jpeg_quality = cJSON_GetObjectItem(json_root, "jpeg_quality");
    ESP_GOTO_ON_FALSE(json_jpeg_quality && cJSON_IsNumber(json_jpeg_quality), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid jpeg_quality field");
    int jpeg_quality = json_jpeg_quality->valueint;

    ESP_LOGI(TAG, "JSON parse success - index:%d, image_format:%d, jpeg_quality:%d", index, image_format, jpeg_quality);
    cJSON_Delete(json_root);
    json_root = NULL;

    ESP_GOTO_ON_ERROR(set_camera_image_format(&web_cam->video[index], image_format), fail1, TAG, "failed to set camera image format");
    ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(&web_cam->video[index], jpeg_quality), fail1, TAG, "failed to set camera jpeg quality");

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;

fail1:
    if (json_root) {
        cJSON_Delete(json_root);
    }
fail0:
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        httpd_resp_send_408(req);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
    }
    if (content) {
        free(content);
    }
    return ret;
}

static esp_err_t button_event_ws_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        register_event_client(req->handle, fd);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        unregister_event_client(fd);
        return ret;
    }

    if (ws_pkt.len > 0) {
        ws_pkt.payload = calloc(1, ws_pkt.len + 1);
        ESP_RETURN_ON_FALSE(ws_pkt.payload != NULL, ESP_ERR_NO_MEM, TAG, "failed to alloc ws payload");
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(ws_pkt.payload);
            unregister_event_client(fd);
            return ret;
        }
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        unregister_event_client(fd);
    } else if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.payload != NULL) {
        ESP_LOGI(TAG, "button event client says: %s", (char *)ws_pkt.payload);
    }

    free(ws_pkt.payload);
    return ESP_OK;
}

static esp_err_t static_file_handler(httpd_req_t *req)
{
    const char *uri = req->uri;

    /* Route to appropriate static file based on URI */
    if (strcmp(uri, "/") == 0) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
    } else if (strcmp(uri, "/loading.jpg") == 0) {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)loading_jpg_gz_start, loading_jpg_gz_end - loading_jpg_gz_start);
    } else if (strcmp(uri, "/favicon.ico") == 0) {
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)favicon_ico_gz_start, favicon_ico_gz_end - favicon_ico_gz_start);
    } else if (strcmp(uri, "/assets/index.js") == 0) {
        httpd_resp_set_type(req, "application/javascript");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_js_gz_start, assets_index_js_gz_end - assets_index_js_gz_start);
    } else if (strcmp(uri, "/assets/index.css") == 0) {
        httpd_resp_set_type(req, "text/css");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_css_gz_start, assets_index_css_gz_end - assets_index_css_gz_start);
    }

    /* If no static file matches, return 404 */
    ESP_LOGW(TAG, "File not found: %s", uri);
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t image_stream_handler(httpd_req_t *req)
{
    esp_err_t ret;
    struct v4l2_buffer buf;
    char http_string[128];
    bool locked = false;
    bool io_locked = false;
    int64_t next_frame_deadline_us = 0;
    web_cam_video_t *video = (web_cam_video_t *)req->user_ctx;
    uint32_t stream_fps = get_effective_stream_fps(video);
    int64_t stream_interval_us = 1000000LL / stream_fps;

    if (!video->stream_on) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "preview disabled");
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(snprintf(http_string, sizeof(http_string), "%" PRIu32, stream_fps) > 0,
                        ESP_FAIL, TAG, "failed to format framerate buffer");

    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, STREAM_CONTENT_TYPE), TAG, "failed to set content type");
    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "X-Framerate", http_string), TAG, "failed to set x framerate");

    while (1) {
        int hlen;
        struct timespec ts;
        uint32_t jpeg_encoded_size;

        locked = false;
        io_locked = false;

        ESP_RETURN_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take io lock");
        io_locked = true;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_DQBUF, &buf), TAG, "failed to receive video frame");
        if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
            ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue video frame");
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        if (next_frame_deadline_us != 0 && now_us < next_frame_deadline_us) {
            ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue skipped video frame");
            xSemaphoreGive(video->io_lock);
            io_locked = false;
            continue;
        }

        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)), fail0, TAG, "failed to send boundary");

        if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
            video->jpeg_out_buf = video->buffer[buf.index];
            jpeg_encoded_size = buf.bytesused;
        } else {
            ESP_GOTO_ON_FALSE(xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS, ESP_FAIL, fail0, TAG, "failed to take semaphore");
            locked = true;

            ESP_GOTO_ON_ERROR(example_encoder_process(video->encoder_handle, video->buffer[buf.index], video->buffer_size,
                              video->jpeg_out_buf, video->jpeg_out_size, &jpeg_encoded_size),
                              fail0, TAG, "failed to encode video frame");
        }

        ESP_GOTO_ON_ERROR(clock_gettime(CLOCK_MONOTONIC, &ts), fail0, TAG, "failed to get time");
        ESP_GOTO_ON_FALSE((hlen = snprintf(http_string, sizeof(http_string), STREAM_PART, jpeg_encoded_size, ts.tv_sec, ts.tv_nsec)) > 0,
                          ESP_FAIL, fail0, TAG, "failed to format part buffer");
        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, http_string, hlen), fail0, TAG, "failed to send boundary");

        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, (char *)video->jpeg_out_buf, jpeg_encoded_size), fail0, TAG, "failed to send jpeg");
        if (locked) {
            xSemaphoreGive(video->sem);
            locked = false;
        }

        ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue video frame");
        next_frame_deadline_us = now_us + stream_interval_us;
        xSemaphoreGive(video->io_lock);
        io_locked = false;
    }

    return ESP_OK;

fail0:
    if (locked) {
        xSemaphoreGive(video->sem);
    }
    if (io_locked) {
        xSemaphoreGive(video->io_lock);
    }
    ioctl(video->fd, VIDIOC_QBUF, &buf);
    if (ret == ESP_FAIL || ret == ESP_ERR_HTTPD_RESP_SEND) {
        ESP_LOGI(TAG, "stream client disconnected");
        return ESP_OK;
    }
    return ret;
}

static esp_err_t capture_image_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    char type_ptr[32];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "image/jpeg;name=image%d.jpg", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], true);
}

static esp_err_t capture_binary_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    char type_ptr[56];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "application/octet-stream;name=image_binary%d.bin", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], false);
}

static esp_err_t legacy_capture_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    web_cam_video_t *video = get_default_video(web_cam);

    ESP_RETURN_ON_FALSE(video, ESP_ERR_NOT_FOUND, TAG, "no active camera stream found");
    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    return capture_video_image(req, video, true);
}

static esp_err_t legacy_control_handler(httpd_req_t *req)
{
    esp_err_t ret;
    char query[64];
    char variable[32];
    char value[16];
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    web_cam_video_t *video = get_default_video(web_cam);

    ESP_RETURN_ON_FALSE(video, ESP_ERR_NOT_FOUND, TAG, "no active camera stream found");

    ret = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (ret != ESP_OK) {
        httpd_resp_send_404(req);
        return ret;
    }

    if (httpd_query_key_value(query, "var", variable, sizeof(variable)) != ESP_OK ||
        httpd_query_key_value(query, "val", value, sizeof(value)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(variable, "quality") == 0 || strcmp(variable, "jpeg_quality") == 0) {
        ESP_RETURN_ON_ERROR(set_camera_jpeg_quality(video, atoi(value)), TAG, "failed to set jpeg quality");
    } else {
        ESP_LOGW(TAG, "legacy control '%s' is not implemented on esp_video backend", variable);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unsupported var on esp32-p4 backend");
        return ESP_ERR_NOT_SUPPORTED;
    }

    set_cors_headers(req);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t init_web_cam_video(web_cam_video_t *video, const web_cam_video_config_t *config, int index,
                                    const web_cam_image_format_option_t *requested_format)
{
    int fd;
    int ret;
    struct v4l2_format format;
    struct v4l2_streamparm sparm;
    struct v4l2_requestbuffers req;
    struct v4l2_captureparm *cparam = &sparm.parm.capture;
    struct v4l2_fract *timeperframe = &cparam->timeperframe;

    fd = open(config->dev_name, O_RDWR);
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_ERR_NOT_FOUND, TAG, "Open video device %s failed", config->dev_name);

    snprintf(video->dev_name, sizeof(video->dev_name), "%s", config->dev_name);

    if (requested_format) {
        memset(&format, 0, sizeof(struct v4l2_format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = requested_format->width;
        format.fmt.pix.height = requested_format->height;
        format.fmt.pix.pixelformat = requested_format->pixel_format;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_S_FMT, &format), fail0, TAG, "Failed set fmt on %s", config->dev_name);

        memset(&sparm, 0, sizeof(sparm));
        sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        sparm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        sparm.parm.capture.timeperframe.numerator = requested_format->frame_interval_num;
        sparm.parm.capture.timeperframe.denominator = requested_format->frame_interval_den;
        ioctl(fd, VIDIOC_S_PARM, &sparm);
    }

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_FMT, &format), fail0, TAG, "Failed get fmt from %s", config->dev_name);

#if CONFIG_EXAMPLE_SELECT_JPEG_HW_DRIVER
    if (format.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565X) {
#if CONFIG_ESP_VIDEO_ENABLE_SWAP_BYTE
        ESP_LOGW(TAG, "The hardware JPEG encoder does not support RGB565 big endian. Instead, use RGB565 little endian by enabling the byte swap function.");

        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_S_FMT, &format), fail0, TAG, "failed to set fmt to %s", config->dev_name);
#else
        ESP_GOTO_ON_ERROR(ESP_FAIL, fail0, TAG, "The hardware JPEG encoder does not support RGB565 big endian. Please enable the byte swap function ESP_VIDEO_ENABLE_SWAP_BYTE in menuconfig.");
#endif
    }
#endif

    memset(&sparm, 0, sizeof(sparm));
    sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_PARM, &sparm), fail0, TAG, "failed to get frame rate from %s", config->dev_name);
    video->frame_rate = timeperframe->denominator / timeperframe->numerator;

#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CROP
    /**
     * Command VIDIOC_S_SELECTION should be called before VIDIOC_REQBUFS.
     */

    struct v4l2_selection selection;

    memset(&selection, 0, sizeof(selection));
    selection.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    selection.target = V4L2_SEL_TGT_CROP;
    selection.r.left = CONFIG_EXAMPLE_MIPI_CSI_CROP_LEFT;
    selection.r.width = CONFIG_EXAMPLE_MIPI_CSI_CROP_WIDTH;
    selection.r.top = CONFIG_EXAMPLE_MIPI_CSI_CROP_TOP;
    selection.r.height = CONFIG_EXAMPLE_MIPI_CSI_CROP_HEIGHT;
    if (ioctl(fd, VIDIOC_S_SELECTION, &selection) != 0) {
        ESP_LOGE(TAG, "failed to set selection");
    }
#endif

    memset(&req, 0, sizeof(req));
    req.count  = EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_REQBUFS, &req), fail0, TAG, "failed to req buffers from %s", config->dev_name);

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QUERYBUF, &buf), fail0, TAG, "failed to query vbuf from %s", config->dev_name);

        video->buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(video->buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, fail0, TAG, "failed to mmap buffer");
        video->buffer_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QBUF, &buf), fail0, TAG, "failed to queue frame vbuf from %s", config->dev_name);
    }

    video->fd = fd;
    video->width = format.fmt.pix.width;
    video->height = format.fmt.pix.height;
    video->pixel_format = format.fmt.pix.pixelformat;
    video->jpeg_quality = EXAMPLE_JPEG_ENC_QUALITY;

    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(video, EXAMPLE_JPEG_ENC_QUALITY), fail0, TAG, "failed to set jpeg quality");
    } else {
        example_encoder_config_t encoder_config = {0};

        encoder_config.width = video->width;
        encoder_config.height = video->height;
        encoder_config.pixel_format = video->pixel_format;
        encoder_config.quality = EXAMPLE_JPEG_ENC_QUALITY;
        ESP_GOTO_ON_ERROR(example_encoder_init(&encoder_config, &video->encoder_handle), fail0, TAG, "failed to init encoder");

        ESP_GOTO_ON_ERROR(example_encoder_alloc_output_buffer(video->encoder_handle, &video->jpeg_out_buf, &video->jpeg_out_size),
                          fail1, TAG, "failed to alloc jpeg output buf");

        video->support_control_jpeg_quality = 1;
    }

    video->sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(video->sem, ESP_ERR_NO_MEM, fail2, TAG, "failed to create semaphore");
    xSemaphoreGive(video->sem);

    if (video->io_lock == NULL) {
        video->io_lock = xSemaphoreCreateMutex();
        ESP_GOTO_ON_FALSE(video->io_lock, ESP_ERR_NO_MEM, fail3, TAG, "failed to create io lock");
    }

    ESP_GOTO_ON_ERROR(enumerate_video_format_options(video, fd), fail3, TAG, "failed to enumerate video formats");

    return ESP_OK;

fail3:
    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }
fail2:
    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_free_output_buffer(video->encoder_handle, video->jpeg_out_buf);
        video->jpeg_out_buf = NULL;
    }
fail1:
    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_deinit(video->encoder_handle);
        video->encoder_handle = NULL;
    }
fail0:
    close(fd);
    video->fd = -1;
    return ret;
}

static esp_err_t deinit_web_cam_video(web_cam_video_t *video)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }

    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_free_output_buffer(video->encoder_handle, video->jpeg_out_buf);
        example_encoder_deinit(video->encoder_handle);
    }

    video->encoder_handle = NULL;
    video->jpeg_out_buf = NULL;
    video->jpeg_out_size = 0;

    if (video->fd != -1) {
        ioctl(video->fd, VIDIOC_STREAMOFF, &type);
        video->stream_on = 0;
        release_video_buffers(video);
        close(video->fd);
        video->fd = -1;
    }

    free_video_format_options(video);
    return ESP_OK;
}

static esp_err_t new_web_cam(const web_cam_video_config_t *config, int config_count, web_cam_t **ret_wc)
{
    int i;
    web_cam_t *wc;
    esp_err_t ret = ESP_FAIL;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    wc = calloc(1, sizeof(web_cam_t) + config_count * sizeof(web_cam_video_t));
    ESP_RETURN_ON_FALSE(wc, ESP_ERR_NO_MEM, TAG, "failed to alloc web cam");
    wc->video_count = config_count;

    for (i = 0; i < config_count; i++) {
        wc->video[i].index = i;
        wc->video[i].fd = -1;
        wc->video[i].stream_on = 0;

        ret = init_web_cam_video(&wc->video[i], &config[i], i, NULL);
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "failed to find web_cam %d", i);
            continue;
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to initialize web_cam %d", i);
            goto fail0;
        }

        ESP_LOGI(TAG, "video%d: width=%" PRIu32 " height=%" PRIu32 " format=" V4L2_FMT_STR, i, wc->video[i].width,
                 wc->video[i].height, V4L2_FMT_STR_ARG(wc->video[i].pixel_format));
    }

    for (i = 0; i < config_count; i++) {
        if (is_valid_web_cam(&wc->video[i])) {
            ESP_GOTO_ON_ERROR(ioctl(wc->video[i].fd, VIDIOC_STREAMON, &type), fail1, TAG, "failed to start stream");
            wc->video[i].stream_on = 1;
        }
    }

    *ret_wc = wc;

    return ESP_OK;

fail1:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            ioctl(wc->video[j].fd, VIDIOC_STREAMOFF, &type);
        }
    }
    i = config_count; // deinit all web_cam
fail0:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            deinit_web_cam_video(&wc->video[j]);
        }
    }
    free(wc);
    return ret;
}

static void free_web_cam(web_cam_t *web_cam)
{
    for (int i = 0; i < web_cam->video_count; i++) {
        if (is_valid_web_cam(&web_cam->video[i])) {
            deinit_web_cam_video(&web_cam->video[i]);
        }
    }
    free(web_cam);
}

static esp_err_t http_server_init(web_cam_t *web_cam)
{
    httpd_handle_t stream_httpd;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 12;
    ESP_RETURN_ON_ERROR(init_event_client_registry(), TAG, "failed to init event registry");

    /* Unified static file handler for all static resources */
    httpd_uri_t static_file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = (void *)web_cam
    };

    /* API handlers */
    httpd_uri_t capture_image_uri = {
        .uri = "/api/capture_image",
        .method = HTTP_GET,
        .handler = capture_image_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t capture_binary_uri = {
        .uri = "/api/capture_binary",
        .method = HTTP_GET,
        .handler = capture_binary_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_info_uri = {
        .uri = "/api/get_camera_info",
        .method = HTTP_GET,
        .handler = camera_info_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_settings_uri = {
        .uri = "/api/set_camera_config",
        .method = HTTP_POST,
        .handler = camera_settings_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t legacy_capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = legacy_capture_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t legacy_status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = legacy_status_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t legacy_control_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = legacy_control_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t button_event_ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = button_event_ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };

    config.stack_size = 1024 * 6;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        /* Register API handlers (more specific URIs) */
        httpd_register_uri_handler(stream_httpd, &capture_image_uri);
        httpd_register_uri_handler(stream_httpd, &capture_binary_uri);
        httpd_register_uri_handler(stream_httpd, &camera_info_uri);
        httpd_register_uri_handler(stream_httpd, &camera_settings_uri);
        httpd_register_uri_handler(stream_httpd, &legacy_capture_uri);
        httpd_register_uri_handler(stream_httpd, &legacy_status_uri);
        httpd_register_uri_handler(stream_httpd, &legacy_control_uri);
        httpd_register_uri_handler(stream_httpd, &button_event_ws_uri);

        /* Register wildcard static file handler to catch all other requests */
        httpd_register_uri_handler(stream_httpd, &static_file_uri);
    }

    for (int i = 0; i < web_cam->video_count; i++) {
        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        httpd_uri_t stream_0_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = image_stream_handler,
            .user_ctx = (void *) &web_cam->video[i]
        };

        config.stack_size = 1024 * 6;
        config.server_port += 1;
        config.ctrl_port += 1;
        if (httpd_start(&stream_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(stream_httpd, &stream_0_uri);
        }
    }

    return ESP_OK;
}

static esp_err_t start_cam_web_server(const web_cam_video_config_t *config, int config_count)
{
    esp_err_t ret;
    web_cam_t *web_cam;

    ESP_RETURN_ON_ERROR(new_web_cam(config, config_count, &web_cam), TAG, "Failed to new web cam");
    ESP_GOTO_ON_ERROR(http_server_init(web_cam), fail0, TAG, "Failed to init http server");
    s_runtime.web_cam = web_cam;

    return ESP_OK;

fail0:
    free_web_cam(web_cam);
    return ret;
}

static void record_timer_callback(TimerHandle_t timer)
{
    if (s_runtime.lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
        s_runtime.recording_active = false;
        xSemaphoreGive(s_runtime.lock);
        apply_runtime_stream_policy();
        ESP_LOGI(TAG, "record window finished");
    }
}

static esp_err_t init_runtime_state(void)
{
    if (s_runtime.lock == NULL) {
        s_runtime.lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_runtime.lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create runtime lock");
    }

    if (s_runtime.record_timer == NULL) {
        s_runtime.record_timer = xTimerCreate("record_timer",
                                              pdMS_TO_TICKS(EXAMPLE_RECORD_DURATION_SEC * 1000),
                                              pdFALSE,
                                              NULL,
                                              record_timer_callback);
        ESP_RETURN_ON_FALSE(s_runtime.record_timer != NULL, ESP_ERR_NO_MEM, TAG, "failed to create record timer");
    }

    s_runtime.preview_requested = true;
    s_runtime.recording_active = false;
    s_runtime.capture_in_progress = false;
    s_runtime.stream_enabled = true;
    return ESP_OK;
}

static esp_err_t with_temporary_stream_enabled(bool *enabled_here)
{
    bool should_enable = false;

    ESP_RETURN_ON_FALSE(enabled_here != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid temporary stream marker");
    *enabled_here = false;

    ESP_RETURN_ON_FALSE(s_runtime.lock != NULL, ESP_ERR_INVALID_STATE, TAG, "runtime lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take runtime lock");
    should_enable = !s_runtime.stream_enabled;
    xSemaphoreGive(s_runtime.lock);

    if (should_enable) {
        ESP_RETURN_ON_ERROR(set_runtime_stream_enabled(true), TAG, "failed to enable temporary stream");
        *enabled_here = true;
    }

    return ESP_OK;
}

static void restore_temporary_stream(bool enabled_here)
{
    if (enabled_here) {
        apply_runtime_stream_policy();
    }
}

static void capture_button_task(void *arg)
{
    uint8_t *jpeg_data = NULL;
    size_t jpeg_size = 0;
    bool temporary_stream = false;
    web_cam_video_t *video = get_default_video(s_runtime.web_cam);

    if (video == NULL) {
        ESP_LOGE(TAG, "capture button ignored because no active camera exists");
        goto exit_task;
    }

    ESP_LOGI(TAG, "capture button pressed");
    broadcast_button_event("TAKE_PHOTO");

    if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
        if (s_runtime.capture_in_progress) {
            ESP_LOGW(TAG, "capture already in progress");
            xSemaphoreGive(s_runtime.lock);
            goto exit_task;
        }
        s_runtime.capture_in_progress = true;
        xSemaphoreGive(s_runtime.lock);
    }

    if (!pc_helper_is_configured()) {
        ESP_LOGI(TAG, "browser event dispatched, local upload skipped");
    } else if (with_temporary_stream_enabled(&temporary_stream) == ESP_OK &&
               capture_video_image_to_memory(video, true, &jpeg_data, &jpeg_size) == ESP_OK) {
        esp_err_t upload_ret = post_capture_to_pc_helper(jpeg_data, jpeg_size);
        ESP_LOGI(TAG, "capture upload %s", upload_ret == ESP_OK ? "succeeded" : "failed");
    } else {
        ESP_LOGE(TAG, "capture button action failed");
    }

    restore_temporary_stream(temporary_stream);
    free(jpeg_data);

    if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
        s_runtime.capture_in_progress = false;
        xSemaphoreGive(s_runtime.lock);
    }

exit_task:
    vTaskDelete(NULL);
}

static void record_button_task(void *arg)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "record button pressed");
    broadcast_button_event("RECORD_START");
    if (s_runtime.lock == NULL) {
        ESP_LOGE(TAG, "runtime lock is not initialized");
        vTaskDelete(NULL);
        return;
    }
    if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "failed to take runtime lock");
        vTaskDelete(NULL);
        return;
    }
    if (s_runtime.recording_active) {
        ESP_LOGW(TAG, "recording is already active");
        xSemaphoreGive(s_runtime.lock);
        vTaskDelete(NULL);
        return;
    }
    s_runtime.recording_active = true;
    xSemaphoreGive(s_runtime.lock);

    ret = apply_runtime_stream_policy();
    if (ret == ESP_OK && pc_helper_is_configured()) {
        ret = trigger_record_on_pc_helper(EXAMPLE_RECORD_DURATION_SEC);
    } else if (!pc_helper_is_configured()) {
        ESP_LOGI(TAG, "browser event dispatched, local recording trigger skipped");
    }

    if (ret == ESP_OK) {
        xTimerStop(s_runtime.record_timer, 0);
        xTimerChangePeriod(s_runtime.record_timer, pdMS_TO_TICKS(EXAMPLE_RECORD_DURATION_SEC * 1000), 0);
        xTimerStart(s_runtime.record_timer, 0);
        ESP_LOGI(TAG, "recording started for %d seconds", EXAMPLE_RECORD_DURATION_SEC);
    } else {
        ESP_LOGE(TAG, "failed to trigger recording on pc helper");
        if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
            s_runtime.recording_active = false;
            xSemaphoreGive(s_runtime.lock);
        }
        apply_runtime_stream_policy();
    }

    vTaskDelete(NULL);
}

static void ensure_preview_from_button(void)
{
    bool preview_enabled = false;

    if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
        s_runtime.preview_requested = true;
        preview_enabled = s_runtime.preview_requested;
        xSemaphoreGive(s_runtime.lock);
    }

    if (apply_runtime_stream_policy() == ESP_OK) {
        broadcast_button_event("STREAM_START");
        ESP_LOGI(TAG, "preview %s by button", preview_enabled ? "enabled" : "disabled");
    }
}

static void handle_button_press(gpio_num_t gpio)
{
    if (gpio == EXAMPLE_CAPTURE_BUTTON_GPIO) {
        xTaskCreate(capture_button_task, "capture_button", 8192, NULL, 5, NULL);
    } else if (gpio == EXAMPLE_PREVIEW_BUTTON_GPIO) {
        ensure_preview_from_button();
    } else if (gpio == EXAMPLE_RECORD_BUTTON_GPIO) {
        xTaskCreate(record_button_task, "record_button", 8192, NULL, 5, NULL);
    }
}

static void button_control_task(void *arg)
{
    const gpio_num_t buttons[] = {
        EXAMPLE_CAPTURE_BUTTON_GPIO,
        EXAMPLE_PREVIEW_BUTTON_GPIO,
        EXAMPLE_RECORD_BUTTON_GPIO,
    };
    bool stable_level[3] = {true, true, true};
    bool last_sample[3] = {true, true, true};
    int64_t last_change_ms[3] = {0, 0, 0};

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        for (size_t i = 0; i < sizeof(buttons) / sizeof(buttons[0]); i++) {
            bool sample_high = gpio_get_level(buttons[i]) != 0;

            if (sample_high != last_sample[i]) {
                last_sample[i] = sample_high;
                last_change_ms[i] = now_ms;
            }

            if (sample_high != stable_level[i] && (now_ms - last_change_ms[i]) >= EXAMPLE_BUTTON_DEBOUNCE_MS) {
                stable_level[i] = sample_high;
                if (!stable_level[i]) {
                    handle_button_press(buttons[i]);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t init_button_control(void)
{
    const gpio_num_t buttons[] = {
        EXAMPLE_CAPTURE_BUTTON_GPIO,
        EXAMPLE_PREVIEW_BUTTON_GPIO,
        EXAMPLE_RECORD_BUTTON_GPIO,
    };

    for (size_t i = 0; i < sizeof(buttons) / sizeof(buttons[0]); i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << buttons[i],
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "failed to config button gpio %d", buttons[i]);
    }

    ESP_RETURN_ON_FALSE(xTaskCreate(button_control_task, "button_ctrl", 4096, NULL, 4, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "failed to create button control task");

    ESP_LOGI(TAG, "button control ready: capture=%d preview=%d record=%d",
             EXAMPLE_CAPTURE_BUTTON_GPIO, EXAMPLE_PREVIEW_BUTTON_GPIO, EXAMPLE_RECORD_BUTTON_GPIO);
    return ESP_OK;
}

static void initialise_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(EXAMPLE_MDNS_HOST_NAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(EXAMPLE_MDNS_INSTANCE));

    mdns_txt_item_t serviceTxtData[] = {
        {"board", CONFIG_IDF_TARGET},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /*For camera devices that require the host to provide XCLK, the video_init() must be called immediately after the device is restarted,
    otherwise the camera device may not be able to start due to the lack of the main clock.*/
    ESP_ERROR_CHECK(example_video_init());

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(EXAMPLE_MDNS_HOST_NAME);

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ret = example_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Network connection failed. Please check Wi-Fi settings in sdkconfig/menuconfig.");
        ESP_LOGE(TAG, "Current Wi-Fi SSID: %s", CONFIG_EXAMPLE_WIFI_SSID);
        if (strcmp(CONFIG_EXAMPLE_WIFI_SSID, "myssid") == 0 || strcmp(CONFIG_EXAMPLE_WIFI_PASSWORD, "mypassword") == 0) {
            ESP_LOGE(TAG, "The firmware is still using the example Wi-Fi credentials.");
        }
        return;
    }

    log_access_urls();

    web_cam_video_config_t config[] = {
#if EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_MIPI_CSI_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR */
#if EXAMPLE_ENABLE_DVP_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_DVP_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_DVP_CAM_SENSOR */
#if EXAMPLE_ENABLE_SPI_CAM_0_SENSOR
        {
            .dev_name = ESP_VIDEO_SPI_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_SPI_CAM_0_SENSOR */
#if EXAMPLE_ENABLE_SPI_CAM_1_SENSOR
        {
            .dev_name = ESP_VIDEO_SPI_DEVICE_1_NAME,
        },
#endif /* EXAMPLE_ENABLE_SPI_CAM_1_SENSOR */
#if EXAMPLE_ENABLE_USB_UVC_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_USB_UVC_DEVICE_NAME(0),
        },
#endif /* EXAMPLE_ENABLE_USB_UVC_CAM_SENSOR */
    };

    int config_count = sizeof(config) / sizeof(config[0]);

    assert(config_count > 0);
    ESP_ERROR_CHECK(start_cam_web_server(config, config_count));
    ESP_ERROR_CHECK(init_runtime_state());
#if CONFIG_EXAMPLE_ENABLE_GPIO_BUTTON_CONTROL
    ESP_ERROR_CHECK(apply_runtime_stream_policy());
    ESP_ERROR_CHECK(init_button_control());
#endif

    ESP_LOGI(TAG, "Camera web server starts");
}

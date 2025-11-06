// ===== TAG / INITIATOR: FTM (bursty) + CSI RX (1Hz print, anchor-filtered) =====
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"

#define TAG   "TAG"
#define SSID  "FTM"
#define PASS  "ftmftmftm"

#define FTM_FRM_COUNT      32
#define FTM_BURST_PERIOD   3
#define FTM_RECYCLE_GAP_MS 200

#ifndef MAX_FRAMES_USED
#define MAX_FRAMES_USED 32
#endif

#ifndef EXPECT_FRAMES
#define EXPECT_FRAMES   32
#endif

// ===================== Anchor config =====================
typedef struct {
    const char *name;
    uint8_t     bssid[6];
    uint8_t     channel;
    float       gt_dist_m;
} anchor_t;

static anchor_t g_anchors[] = {
        { "A1", {0x24,0xEC,0x4A,0x03,0x58,0x5D}, 1, -1.0f },
        { "A2", {0x24,0xEC,0x4A,0x04,0x39,0x15}, 6, -1.0f },
        { "A3", {0x24,0xEC,0x4A,0x03,0x56,0x41}, 11, -1.0f }
};
static int g_anchor_count = sizeof(g_anchors) / sizeof(g_anchors[0]);
static const anchor_t *g_curr = &g_anchors[0]; // Current Anchor No.
// ========================================================

// ---- Events ----
static EventGroupHandle_t s_wifi_eg;
static EventGroupHandle_t s_ftm_eg;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int FTM_REPORT_BIT     = BIT1;
static const int FTM_FAIL_BIT       = BIT2;
static const int WIFI_DISCONNECTED_BIT = BIT3;

// ---- FTM snapshot from event ----
static volatile uint32_t g_rtt_raw_ns_min = 0;
static volatile uint32_t g_rtt_est_ns     = 0;
static volatile uint32_t g_dist_cm        = 0;
static volatile uint8_t g_ftm_entries = 0;

static wifi_ftm_report_entry_t *s_ftm_copy = NULL;
static volatile uint8_t         s_ftm_copy_num = 0;

// ---- save AP BSSID and Channel----
static uint8_t g_ap_bssid[6] = {0};
static uint8_t g_ap_channel  = 0;

// ====== Anchor snapshot for ISR-safe filtering ======
static volatile uint8_t s_curr_bssid[6]   = {0};
static volatile uint8_t s_curr_channel    = 0;
static volatile int     s_curr_anchor_idx = 0;

// ---- CSI Queue ----
typedef struct {
    uint16_t len;
    int8_t   data[128];
    uint8_t  anchor_idx;
    int8_t   rssi;
} csi_item_t;

static inline bool keep_sc(int k){
    // 20 MHz, HT-LTF: keep ±1..±28
    if (k >= -28 && k <= -1) return true;
    if (k >=  +1 && k <= +28) return true;
    return false;
}

static inline bool is_pilot_k(int k) {
    // 802.11n HT-LTF pilots (20 MHz): k = ±7, ±21
    return (k == -21 || k == -7 || k == 7 || k == 21);
}

static void detrend_phase(float *y, const float *x, int n){
    double Sx=0,Sy=0,Sxx=0,Sxy=0;
    for(int i=0;i<n;++i){ double xi=x[i], yi=y[i]; Sx+=xi; Sy+=yi; Sxx+=xi*xi; Sxy+=xi*yi; }
    double den = n*Sxx - Sx*Sx;
    double a = (den!=0) ? (n*Sxy - Sx*Sy)/den : 0.0;
    double b = (Sy - a*Sx)/n;
    double m=0;
    for(int i=0;i<n;++i){ y[i] = (float)(y[i] - (a*x[i] + b)); m += y[i]; }
    m /= n;
    for(int i=0;i<n;++i) y[i] -= (float)m;
}

static QueueHandle_t s_csi_q;

static void print_csi_features_from_iq(const int8_t *iq, int nbytes,
                                       int rssi_dbm, const char *aname)
{
    if (!iq || nbytes < 4) return;
    const int nbin = nbytes / 2; // except 64 pairs of (I,Q)
    float mag[64], ph[64], xk[64];
    int   keep = 0;

    double csum = 0.0, ssum = 0.0;
    int pilot_cnt = 0;

    for (int i = 0; i < nbin; ++i) {
        int k = i - 32;
        if (k == 0) continue;
        if (!keep_sc(k)) continue;

        float I = (float)iq[2*i + 0];
        float Q = (float)iq[2*i + 1];

        mag[keep] = sqrtf(I*I + Q*Q);
        ph[keep]  = atan2f(Q, I);
        xk[keep]  = (float)k;

        if (is_pilot_k(k)) {
            csum += cos((double)ph[keep]);
            ssum += sin((double)ph[keep]);
            pilot_cnt++;
        }
        keep++;
    }
    if (keep <= 0) return;

    if (pilot_cnt >= 2) {
        float phi0 = (float)atan2(ssum, csum);
        for (int i = 0; i < keep; ++i) {
            ph[i] -= phi0;
            if (ph[i] >  (float)M_PI)  ph[i] -= 2.0f*(float)M_PI;
            if (ph[i] < -(float)M_PI)  ph[i] += 2.0f*(float)M_PI;
        }
    }

    // unwrap
    for (int i = 1; i < keep; ++i) {
        float d = ph[i] - ph[i-1];
        if (d >  (float)M_PI)  ph[i] -= 2.0f*(float)M_PI;
        if (d < -(float)M_PI)  ph[i] += 2.0f*(float)M_PI;
    }

    detrend_phase(ph, xk, keep);

    printf("csi_feat,%s,rssi=%d,mag=[", aname, rssi_dbm);
    for (int i = 0; i < keep; ++i) { if (i) putchar(','); printf("%.3f", mag[i]); }
    fputs("],phi=[", stdout);
    for (int i = 0; i < keep; ++i) { if (i) putchar(','); printf("%.4f", ph[i]); }
    fputs("]\n", stdout);
}


// —— CSI one time print ——
static volatile bool     g_csi_print_armed   = false;
static volatile uint64_t g_csi_earliest_us   = 0;
static volatile int      g_csi_expected_idx  = -1;

static int s_ftm_fail_streak = 0;

static esp_netif_t *s_sta_netif = NULL;

static bool is_connected_to(const uint8_t bssid[6]);

// ===== RTT/RSSI static =====
static int cmp_u32(const void *a, const void *b){
    uint32_t x=*(const uint32_t*)a, y=*(const uint32_t*)b;
    return (x>y)-(x<y);
}

static int cmp_double(const void *a, const void *b) {
    double x = *(const double*)a;
    double y = *(const double*)b;
    return (x > y) - (x < y);
}

static double quantile_u32_lin(const uint32_t *xs, int n, double p){
    if(n<=0) return NAN;
    if(n==1) return (double)xs[0];
    if(p<=0) return (double)xs[0];
    if(p>=1) return (double)xs[n-1];
    double idx = p*(n-1);
    int lo = (int)floor(idx), hi = (int)ceil(idx);
    double w = idx - lo;
    return (1.0-w)*xs[lo] + w*xs[hi];
}

// population std (ddof=0)
static double mean_double(const double *x, int n){
    if(n<=0) return NAN;
    double s=0; for(int i=0;i<n;++i) s+=x[i]; return s/n;
}
static double std_double(const double *x, int n){
    if(n<=1) return 0.0;
    double mu = mean_double(x,n), acc=0;
    for(int i=0;i<n;++i){ double d=x[i]-mu; acc+=d*d; }
    return sqrt(acc/n);
}

typedef struct {
    // RTT (ps)
    double rtt_min, rtt_k2, rtt_median, rtt_p10, rtt_p25, rtt_p75;
    double rtt_iqr, rtt_mad, rtt_mean_trim, rtt_std, rtt_valid_ratio;
    // RSSI (dBm)
    double rssi_mean, rssi_min, rssi_std;
} rtt_feat_t;

static rtt_feat_t make_rtt_feats(const uint32_t *rtt_ps, const int32_t *rssi_dbm, int len){
    rtt_feat_t f = {0};
    if(len<=0){ f.rtt_valid_ratio = 0.0; f.rssi_min = NAN; f.rssi_mean = NAN; return f; }

    uint32_t tmp[MAX_FRAMES_USED];
    for(int i=0;i<len;++i) tmp[i]=rtt_ps[i];
    qsort(tmp, len, sizeof(uint32_t), cmp_u32);

    f.rtt_min    = (double)tmp[0];
    f.rtt_k2     = (double)(len>=2 ? tmp[1] : tmp[0]);
    f.rtt_median = quantile_u32_lin(tmp, len, 0.5);
    f.rtt_p10    = quantile_u32_lin(tmp, len, 0.10);
    f.rtt_p25    = quantile_u32_lin(tmp, len, 0.25);
    f.rtt_p75    = quantile_u32_lin(tmp, len, 0.75);
    f.rtt_iqr    = f.rtt_p75 - f.rtt_p25;

    // MAD = median(|x - median|)
    double med = f.rtt_median;
    double absdev[MAX_FRAMES_USED];
    for(int i=0;i<len;++i) absdev[i] = fabs(((double)tmp[i]) - med);
    qsort(absdev, len, sizeof(double), cmp_double);
    if(len==1) f.rtt_mad = 0;
    else{
        int lo = (len-1)/2, hi = len/2;
        f.rtt_mad = (absdev[lo]+absdev[hi])*0.5;
    }

    // trimmed mean (10%~90%);
    if(len>=10){
        int a = (int)(0.10*len), b = (int)(0.90*len);
        if(b<=a) b=a+1;
        double s=0; int c=0;
        for(int i=a;i<b;++i){ s += (double)tmp[i]; ++c; }
        f.rtt_mean_trim = (c>0) ? s/c : (double)tmp[0];
    }else{
        double s=0; for(int i=0;i<len;++i) s+=(double)tmp[i];
        f.rtt_mean_trim = s/len;
    }

    // std
    double xd[MAX_FRAMES_USED];
    for(int i=0;i<len;++i) xd[i] = (double)rtt_ps[i];
    f.rtt_std = std_double(xd, len);

    f.rtt_valid_ratio = (double)len / ((double)EXPECT_FRAMES); // = n / 15.0

    // RSSI
    int32_t rmin = rssi_dbm[0];
    double  rs = 0.0;
    double  rssi_d[MAX_FRAMES_USED];
    for (int i = 0; i < len; ++i) {
        if (rssi_dbm[i] < rmin) rmin = rssi_dbm[i];
        rs += (double)rssi_dbm[i];
        rssi_d[i] = (double)rssi_dbm[i];
    }
    f.rssi_min  = (double)rmin;
    f.rssi_mean = rs / len;
    f.rssi_std  = std_double(rssi_d, len);

    return f;
}

static inline void kick_udp_to_gw(void) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return;
    struct sockaddr_in to = {0};
    to.sin_family = AF_INET;
    to.sin_port   = htons(33333);
    to.sin_addr.s_addr = inet_addr("192.168.4.1"); // TODO: change to your value
    uint8_t b = 0;
    (void)sendto(s, &b, 1, 0, (struct sockaddr*)&to, sizeof(to));
    close(s);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *ev = (wifi_event_sta_connected_t*)data;

        memcpy((void*)g_ap_bssid, ev->bssid, 6);
        g_ap_channel = ev->channel;

        uint8_t pri = 0; wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        if (esp_wifi_get_channel(&pri, &sec) == ESP_OK && pri != 0) g_ap_channel = pri;

        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "STA connected to " MACSTR ", ch=%u (sec=%d)",
                MAC2STR(g_ap_bssid), g_ap_channel, (int)sec);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_ap_channel = 0;
        memset((void*)g_ap_bssid, 0, 6);
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *ev = (wifi_event_ftm_report_t*)data;
        g_rtt_raw_ns_min = ev->rtt_raw;
        g_rtt_est_ns     = ev->rtt_est;
        g_dist_cm        = ev->dist_est;
        g_ftm_entries    = ev->ftm_report_num_entries;

        if (ev->ftm_report_num_entries > 0 && ev->ftm_report_data) {
            size_t sz = sizeof(wifi_ftm_report_entry_t) * ev->ftm_report_num_entries;
            if (s_ftm_copy) { free(s_ftm_copy); s_ftm_copy = NULL; s_ftm_copy_num = 0; }
            s_ftm_copy = (wifi_ftm_report_entry_t*)heap_caps_malloc(sz, MALLOC_CAP_8BIT);
            if (s_ftm_copy) {
                memcpy(s_ftm_copy, ev->ftm_report_data, sz);
                s_ftm_copy_num = ev->ftm_report_num_entries;
            }
        }

        if (ev->status == FTM_STATUS_SUCCESS) xEventGroupSetBits(s_ftm_eg, FTM_REPORT_BIT);
        else                                  xEventGroupSetBits(s_ftm_eg, FTM_FAIL_BIT);
    }else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_ap_channel = 0;
        memset((void*)g_ap_bssid, 0, 6);
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);
        esp_wifi_connect();
    }
}

static bool wait_sta_connected(uint32_t timeout_ms)
{
    EventBits_t b = xEventGroupWaitBits(
            s_wifi_eg, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms)
    );
    return (b & WIFI_CONNECTED_BIT) != 0;
}

static bool wait_sta_connected(uint32_t timeout_ms);

static esp_err_t assoc_to_anchor(const anchor_t* a, uint32_t timeout_ms)
{
    if (is_connected_to(a->bssid)) return ESP_OK;

    wifi_config_t sta = {0};
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &sta));
    sta.sta.bssid_set = true;
    memcpy(sta.sta.bssid, a->bssid, 6);
    sta.sta.channel = a->channel;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));

    xEventGroupClearBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);
    (void)esp_wifi_disconnect();
    (void)xEventGroupWaitBits(s_wifi_eg, WIFI_DISCONNECTED_BIT,
                              pdTRUE, pdTRUE, pdMS_TO_TICKS(1500));

    vTaskDelay(pdMS_TO_TICKS(50));

    esp_err_t rc = esp_wifi_connect();
    if (rc != ESP_OK && rc != ESP_ERR_WIFI_CONN) {
        return rc;
    }

    return wait_sta_connected(timeout_ms) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void switch_snapshot_to(int idx)
{
    g_curr = &g_anchors[idx];
    for (int i = 0; i < 6; ++i) s_curr_bssid[i] = g_curr->bssid[i];
    s_curr_channel    = (g_curr->channel != 0) ? g_curr->channel : g_ap_channel;
    s_curr_anchor_idx = idx;

    xQueueReset(s_csi_q);
    g_csi_print_armed = false;
}

// ===== Init =====
static void set_uart_baudrate(void) {
    uart_set_baudrate(UART_NUM_0, 921600);
}

static bool is_connected_to(const uint8_t bssid[6]) {
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return memcmp(ap.bssid, bssid, 6) == 0;
    }
    return false;
}

static void wifi_init_and_connect(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    esp_netif_ip_info_t ip;
    IP4_ADDR(&ip.ip,      192,168,4,2);
    IP4_ADDR(&ip.netmask, 255,255,255,0);
    IP4_ADDR(&ip.gw,      192,168,4,1);
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_sta_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_sta_netif, &ip));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta = (wifi_config_t){0};
    strncpy((char*)sta.sta.ssid, SSID, sizeof(sta.sta.ssid)-1);
    strncpy((char*)sta.sta.password, PASS, sizeof(sta.sta.password)-1);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_protocol(
            WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));

    esp_err_t rc = esp_wifi_connect();
    if (rc != ESP_OK && rc != ESP_ERR_WIFI_CONN) {
        ESP_ERROR_CHECK(rc);
    }
}

static void csi_enable(void) {
    wifi_promiscuous_filter_t f = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
    };
    esp_wifi_set_promiscuous_filter(&f);
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t cfg = {
            .lltf_en = true, .htltf_en = true, .stbc_htltf2_en = true,
            .ltf_merge_en = true, .channel_filter_en = true,
            .manu_scale = false, .shift = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

// ===== CSI RX callback: filter by anchor BSSID (snapshot), enqueue =====
static void csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    if (!info || !info->buf) return;
    if (memcmp(info->mac, (const void*)s_curr_bssid, 6) != 0) return;

    csi_item_t it = {0};
    it.len = (info->len > 128) ? 128 : info->len;
    for (int i = 0; i < it.len; ++i) it.data[i] = ((const int8_t*)info->buf)[i];
    it.anchor_idx = (uint8_t)s_curr_anchor_idx;
    it.rssi = (int8_t)info->rx_ctrl.rssi;

    (void)xQueueOverwriteFromISR(s_csi_q, &it, NULL);
}

static inline void csi_pause(bool pause) {
    if (pause) {
        esp_wifi_set_csi(false);
        esp_wifi_set_promiscuous(false);
    } else {
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_csi(true);
    }
}

static esp_err_t do_ftm_once_and_print_frames(void) {
    EventBits_t b = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                        pdFALSE, pdTRUE, pdMS_TO_TICKS(2000));
    if (!(b & WIFI_CONNECTED_BIT)) return ESP_FAIL;

    size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    if (free_int < 24 * 1024) {
        ESP_LOGW(TAG, "Low INT heap (%uB), skip FTM.", (unsigned)free_int);
        esp_restart();
        return ESP_ERR_NO_MEM;
    }

    uint8_t ch = s_curr_channel ? s_curr_channel : g_ap_channel;
    if (ch == 0) {
        ESP_LOGW(TAG, "Anchor channel unknown; skip.");
        return ESP_FAIL;
    }

    wifi_ftm_initiator_cfg_t cfg = {
            .frm_count    = FTM_FRM_COUNT,
            .burst_period = FTM_BURST_PERIOD,
    };
    memcpy(cfg.resp_mac, g_curr->bssid, 6);
    cfg.channel = ch;

    xEventGroupClearBits(s_ftm_eg, FTM_REPORT_BIT | FTM_FAIL_BIT);

    csi_pause(true);

    esp_err_t r = esp_wifi_ftm_initiate_session(&cfg);
    if (r != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        csi_pause(false);
        return r;
    }

    EventBits_t e = xEventGroupWaitBits(s_ftm_eg, FTM_REPORT_BIT | FTM_FAIL_BIT,
                                        pdTRUE, pdFALSE, pdMS_TO_TICKS(4000));

    wifi_ftm_report_entry_t *frames = NULL;
    uint8_t nframes = 0;
    esp_err_t grc = ESP_FAIL;

    if ((e & FTM_REPORT_BIT) && g_ftm_entries > 0) {
        nframes = g_ftm_entries;
        size_t sz = sizeof(wifi_ftm_report_entry_t) * nframes;
        frames = (wifi_ftm_report_entry_t*)heap_caps_malloc(sz, MALLOC_CAP_8BIT);
        if (frames) {
            memset(frames, 0, sz);
            grc = esp_wifi_ftm_get_report(frames, nframes);
            if (grc != ESP_OK) {
                free(frames); frames = NULL; nframes = 0;
            }
        }
    }

    (void)esp_wifi_ftm_end_session();
    vTaskDelay(pdMS_TO_TICKS(FTM_RECYCLE_GAP_MS));
    csi_pause(false);

    g_csi_earliest_us  = esp_timer_get_time() + 30 * 1000;
    g_csi_expected_idx = s_curr_anchor_idx;
    g_csi_print_armed  = true;

    if (e & FTM_FAIL_BIT) return ESP_FAIL;
    if (!(e & FTM_REPORT_BIT)) return ESP_ERR_TIMEOUT;

    float est_m = (g_dist_cm > 0) ? (g_dist_cm / 100.0f) : -1.0f;
    float err_m = (g_curr->gt_dist_m > 0 && est_m > 0) ? (est_m - g_curr->gt_dist_m) : NAN;
    printf("anchor,%s," MACSTR ",ch=%u\n", g_curr->name, MAC2STR(g_curr->bssid), (unsigned)cfg.channel);
    printf("ftm_report,%s,%u,%u,%.3f", g_curr->name,
           (unsigned)g_rtt_raw_ns_min, (unsigned)g_rtt_est_ns, est_m);
    if (!isnan(err_m)) printf(",err=%.3f", err_m);
    putchar('\n');

    if (!frames || nframes == 0) {
        if (s_ftm_copy && s_ftm_copy_num > 0) {
            frames  = s_ftm_copy;
            nframes = s_ftm_copy_num;
            s_ftm_copy = NULL;
            s_ftm_copy_num = 0;
        }
    }

    if (frames && nframes) {
        int use = (nframes > MAX_FRAMES_USED) ? MAX_FRAMES_USED : nframes;

        uint32_t rtt_ps[MAX_FRAMES_USED];
        int32_t  rssi_dbm[MAX_FRAMES_USED];
        for (int i=0;i<use;++i){
            rtt_ps[i]   = frames[i].rtt;
            rssi_dbm[i] = frames[i].rssi;
        }

        rtt_feat_t F = make_rtt_feats(rtt_ps, rssi_dbm, use);

        printf("ftm_feats,%s,", g_curr->name);
        printf("rtt_ps_min=%.0f,k2=%.0f,median=%.0f,p10=%.0f,p25=%.0f,p75=%.0f,",
               F.rtt_min,F.rtt_k2,F.rtt_median,F.rtt_p10,F.rtt_p25,F.rtt_p75);
        printf("iqr=%.0f,mad=%.0f,mean_trim=%.1f,std=%.1f,valid_ratio=%.3f,",
               F.rtt_iqr,F.rtt_mad,F.rtt_mean_trim,F.rtt_std,F.rtt_valid_ratio);
        printf("rssi_mean=%.2f,rssi_min=%.0f,rssi_std=%.2f,",
               F.rssi_mean, F.rssi_min, F.rssi_std);

        putchar_unlocked('[');
        for(int i=0;i<use;++i){
            if(i) putchar_unlocked(',');
            printf("%u", (unsigned)rtt_ps[i]);
        }
        putchar_unlocked(']');
        putchar_unlocked('\n');

        free(frames);
    }
    return ESP_OK;
}

// ===== CSI logger task: print ~1Hz (take latest only) =====
static void csi_logger_task(void *arg) {
    const TickType_t poll = pdMS_TO_TICKS(20);
    csi_item_t it;

    for (;;) {
        if (g_csi_print_armed) {
            bool have = false;
            csi_item_t last;
            while (xQueueReceive(s_csi_q, &it, 0) == pdTRUE) { last = it; have = true; }

            if (have) {
                uint64_t now = esp_timer_get_time();
                if (now >= g_csi_earliest_us && last.anchor_idx == g_csi_expected_idx) {
                    print_csi_features_from_iq(
                            last.data,
                            last.len,
                            last.rssi,
                            g_anchors[last.anchor_idx].name
                    );
                    g_csi_print_armed = false;
                }
            }
        } else {
            while (xQueueReceive(s_csi_q, &it, 0) == pdTRUE) { /* drop */ }
        }

        vTaskDelay(poll);
    }
}


static void wifi_soft_reset_and_reconnect(void)
{
    ESP_LOGW(TAG, "WiFi soft reset to recover buffers...");

    xEventGroupClearBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);
    (void)esp_wifi_disconnect();
    (void)xEventGroupWaitBits(s_wifi_eg, WIFI_DISCONNECTED_BIT,
                              pdTRUE, pdTRUE, pdMS_TO_TICKS(1500));

    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_err_t rc = esp_wifi_connect();
    if (rc != ESP_OK && rc != ESP_ERR_WIFI_CONN) {
        ESP_LOGW(TAG, "connect rc=%s", esp_err_to_name(rc));
        return;
    }

    if (!wait_sta_connected(8000)) {
        ESP_LOGW(TAG, "Reconnect timeout.");
    } else {
        ESP_LOGI(TAG, "STA reconnected.");
    }
}


// ===== Measure loop: round-robin anchors =====
static void measure_task(void *arg) {
    int idx = 0;
    for (;;) {
        // (a) Associate to the current anchor
        if (assoc_to_anchor(&g_anchors[idx], 6000) != ESP_OK) {
            ESP_LOGW(TAG, "Assoc to %s timeout, skip.", g_anchors[idx].name);
            idx = (idx + 1) % g_anchor_count;
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        // (b) Switch snapshot and clear queues
        switch_snapshot_to(idx);
        vTaskDelay(pdMS_TO_TICKS(20));

        // (c) Let CSI capture briefly first
        vTaskDelay(pdMS_TO_TICKS(120));

        // (d) Launch FTM for this anchor (inside it: pause CSI → run FTM → resume CSI)
        esp_err_t r = do_ftm_once_and_print_frames();
        vTaskDelay(pdMS_TO_TICKS(200));

        // Kick a UDP heartbeat to the gateway/logger
        kick_udp_to_gw();
        vTaskDelay(pdMS_TO_TICKS(30));

        if (r == ESP_OK) {
            s_ftm_fail_streak = 0;
        } else {
            s_ftm_fail_streak++;
            ESP_LOGW(TAG, "FTM to %s failed (%s), streak=%d",
                     g_curr->name, esp_err_to_name(r), s_ftm_fail_streak);
            if (s_ftm_fail_streak >= 3) {
                wifi_soft_reset_and_reconnect();
                s_ftm_fail_streak = 0;
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }

        // (e) Next anchor & inter-anchor interval
        idx = (idx + 1) % g_anchor_count;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}


void app_main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    esp_log_level_set("wifi", ESP_LOG_ERROR);
    esp_log_level_set("phy",  ESP_LOG_ERROR);

    ESP_ERROR_CHECK(nvs_flash_init());
    set_uart_baudrate();

    s_wifi_eg = xEventGroupCreate();
    s_ftm_eg  = xEventGroupCreate();
    s_csi_q   = xQueueCreate(1, sizeof(csi_item_t));

    wifi_init_and_connect();
    wait_sta_connected(8000);

    // CSI
    csi_enable();
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));

    // Tasks
    xTaskCreate(csi_logger_task, "csi_logger", 4096, NULL, 4, NULL);
    xTaskCreate(measure_task,   "measure",     8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "Ready (multi-anchor): FTM burst + CSI 1Hz, UART=921600.");
}

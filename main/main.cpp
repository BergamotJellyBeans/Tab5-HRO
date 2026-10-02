#include <atomic>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <ctime>
#include <sys/time.h>
#include <sys/stat.h>
#include <cmath>
#include <errno.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp_heap_caps.h"
#include <dirent.h>

#include <M5Unified.h>
#include "esp_rtl_sdr.h"
#include "esp_timer.h"
#include "dsps_fft2r.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "nvs.h"
#include "esp_sntp.h"

#include "hro_config.h"

static const char *TAG = "Tab5-HRO";

// HRO受信周波数
static uint32_t g_hro_lo_frequency_hz = 77399000;	// FM test
//static constexpr uint32_t HRO_FREQUENCY_HZ = 53749220;

// まずはOrcSDRで実績のある960 kS/s
//static constexpr uint32_t RTL_SAMPLE_RATE = ESP_RTL_SDR_RATE_960K;
static constexpr uint32_t RTL_SAMPLE_RATE = ESP_RTL_SDR_RATE_256K;

static esp_rtl_sdr_handle_t g_rtl = nullptr;

static std::atomic<bool> g_rtl_ready{false};
static std::atomic<bool> g_stream_started{false};

// 1秒ごとにexchange(0)するので32bitで十分
static std::atomic<uint32_t> g_interval_bytes{0};
static std::atomic<uint32_t> g_interval_blocks{0};

static constexpr size_t IQ_SLOT_BYTES = 32768;
static constexpr uint8_t IQ_SLOT_COUNT = 32;

struct RtlIqBlock {
    uint8_t *data;
    size_t bytes;
    uint8_t slot;
};

static uint8_t *g_iq_slots[IQ_SLOT_COUNT] = {};

static QueueHandle_t g_free_q = nullptr;
static QueueHandle_t g_filled_q = nullptr;

static std::atomic<uint32_t> g_pipeline_drops{0};
static std::atomic<uint32_t> g_dsp_interval_bytes{0};
static std::atomic<uint32_t> g_dsp_interval_blocks{0};

static constexpr float HRO_SAMPLE_RATE = 576000.0f;
//static constexpr float HRO_SAMPLE_RATE = 960000.0f;
static float g_hro_freq_shift = -220.0f;

// 1000Hz TEST TONE
static constexpr bool HRO_TEST_TONE = false;
static constexpr float HRO_TEST_TONE_HZ = 1000.0f;


static constexpr float TWO_PI = 6.2831853071795864769f;
//static constexpr float HRO_PHASE_STEP = TWO_PI * g_hro_freq_shift / HRO_SAMPLE_RATE;
static std::atomic<uint32_t> g_decim_interval_samples{0};

static const char *SD_MOUNT_POINT = "/sdcard";
static sdmmc_card_t *g_sd_card = nullptr;
static sd_pwr_ctrl_handle_t g_sd_pwr = nullptr;

static constexpr int FIR_TAPS = 127;

// 256 kS/s input
// 10 kHz low-pass FIR
// 127 taps, Hamming window
// DC gain normalized to 1.0
static constexpr float FIR_COEFF[FIR_TAPS] = {
 9.7969972426e-05f,
 1.9451394183e-04f,
 2.8764188956e-04f,
 3.7461947908e-04f,
 4.5158521043e-04f,
 5.1334816156e-04f,
 5.5343070207e-04f,
 5.6439428827e-04f,
 5.3845366071e-04f,
 4.6834854878e-04f,
 3.4840593308e-04f,
 1.7569380591e-04f,
-4.8857178075e-05f,
-3.1950164486e-04f,
-6.2503544346e-04f,
-9.4858476656e-04f,
-1.2679263132e-03f,
-1.5563963412e-03f,
-1.7843964007e-03f,
-1.9214472235e-03f,
-1.9386842802e-03f,
-1.8116340274e-03f,
-1.5230640144e-03f,
-1.0656676283e-03f,
-4.4432930142e-04f,
 3.2227870331e-04f,
 1.2009899999e-03f,
 2.1445024350e-03f,
 3.0929084339e-03f,
 3.9764228701e-03f,
 4.7192425715e-03f,
 5.2443770763e-03f,
 5.4791998573e-03f,
 5.3613903044e-03f,
 4.8448761924e-03f,
 3.9053500985e-03f,
 2.5449256840e-03f,
 7.9552341906e-04f,
-1.2793695062e-03f,
-3.5848357301e-03f,
-5.9967216622e-03f,
-8.3659213728e-03f,
-1.0524644038e-02f,
-1.2294418328e-02f,
-1.3495455440e-02f,
-1.3956878931e-02f,
-1.3527242950e-02f,
-1.2084707978e-02f,
-9.5462300624e-03f,
-5.8751481138e-03f,
-1.0866240511e-03f,
 4.7495008686e-03f,
 1.1508729126e-02f,
 1.9014939760e-02f,
 2.7046382015e-02f,
 3.5344399078e-02f,
 4.3624478609e-02f,
 5.1589074996e-02f,
 5.8941527668e-02f,
 6.5400318703e-02f,
 7.0712877492e-02f,
 7.4668153172e-02f,
 7.7107236888e-02f,
 7.7931422223e-02f,
 7.7107236888e-02f,
 7.4668153172e-02f,
 7.0712877492e-02f,
 6.5400318703e-02f,
 5.8941527668e-02f,
 5.1589074996e-02f,
 4.3624478609e-02f,
 3.5344399078e-02f,
 2.7046382015e-02f,
 1.9014939760e-02f,
 1.1508729126e-02f,
 4.7495008686e-03f,
-1.0866240511e-03f,
-5.8751481138e-03f,
-9.5462300624e-03f,
-1.2084707978e-02f,
-1.3527242950e-02f,
-1.3956878931e-02f,
-1.3495455440e-02f,
-1.2294418328e-02f,
-1.0524644038e-02f,
-8.3659213728e-03f,
-5.9967216622e-03f,
-3.5848357301e-03f,
-1.2793695062e-03f,
 7.9552341906e-04f,
 2.5449256840e-03f,
 3.9053500985e-03f,
 4.8448761924e-03f,
 5.3613903044e-03f,
 5.4791998573e-03f,
 5.2443770763e-03f,
 4.7192425715e-03f,
 3.9764228701e-03f,
 3.0929084339e-03f,
 2.1445024350e-03f,
 1.2009899999e-03f,
 3.2227870331e-04f,
-4.4432930142e-04f,
-1.0656676283e-03f,
-1.5230640144e-03f,
-1.8116340274e-03f,
-1.9386842802e-03f,
-1.9214472235e-03f,
-1.7843964007e-03f,
-1.5563963412e-03f,
-1.2679263132e-03f,
-9.4858476656e-04f,
-6.2503544346e-04f,
-3.1950164486e-04f,
-4.8857178075e-05f,
 1.7569380591e-04f,
 3.4840593308e-04f,
 4.6834854878e-04f,
 5.3845366071e-04f,
 5.6439428827e-04f,
 5.5343070207e-04f,
 5.1334816156e-04f,
 4.5158521043e-04f,
 3.7461947908e-04f,
 2.8764188956e-04f,
 1.9451394183e-04f,
 9.7969972426e-05f
};
static std::atomic<uint32_t> g_decim_power{0};
static constexpr size_t DECIM_BUFFER_SAMPLES = 1024;

struct ComplexSample {
    float i;
    float q;
};

static ComplexSample g_decim_buffer[DECIM_BUFFER_SAMPLES];
static size_t g_decim_buffer_pos = 0;
static constexpr int RESAMP_L = 16;
static constexpr int RESAMP_M = 125;

static constexpr int RESAMP_TAPS_PER_PHASE = 64;
static constexpr int RESAMP_TOTAL_TAPS = RESAMP_L * RESAMP_TAPS_PER_PHASE;

static constexpr float RESAMP_INPUT_RATE = 64000.0f;
static constexpr float RESAMP_CUTOFF_HZ = 1800.0f;

static float g_resamp_coeff[RESAMP_L][RESAMP_TAPS_PER_PHASE];
static ComplexSample g_resamp_delay[RESAMP_TAPS_PER_PHASE] = {};

static int g_resamp_delay_pos = 0;
static uint64_t g_resamp_input_count = 0;
static uint64_t g_resamp_next_time = 0;
static std::atomic<uint32_t>g_resamp_interval_samples{0};
static std::atomic<uint32_t>g_resamp_power{0};
static std::atomic<uint32_t> g_resamp_10sec_samples{0};

static constexpr size_t FFT_SIZE = 8192;
static constexpr int FFT_BUFFER_COUNT = 2;

struct FftFrame {
    float *i;
    float *q;
    uint8_t slot;
};

static float *g_fft_i[FFT_BUFFER_COUNT] = {};
static float *g_fft_q[FFT_BUFFER_COUNT] = {};

static QueueHandle_t g_fft_free_q = nullptr;
static QueueHandle_t g_fft_ready_q = nullptr;

static uint8_t g_fft_write_slot = 0;
static size_t g_fft_pos = 0;

static std::atomic<uint32_t> g_fft_frames{0};
static std::atomic<uint32_t> g_fft_processed{0};
static std::atomic<uint32_t> g_fft_drops{0};

static float *g_fft_work = nullptr;
static constexpr int INTERVAL_SEC = 120;
static constexpr int TOTAL_SEC    = 600;

// HRO spectrum range
static constexpr int HRO_BIN_MIN = 530;
static constexpr int HRO_BIN_MAX = 1030;
static constexpr int HRO_BIN_COUNT = HRO_BIN_MAX - HRO_BIN_MIN + 1;   // 501 bins

// Latest 1-second spectrum
static float g_hro_spectrum[HRO_BIN_COUNT];
static std::atomic<uint32_t> g_hro_spectrum_sequence{0};

static constexpr int HRO_HISTORY_SECONDS = 600;
static constexpr int WF_SECONDS = 600;

// 600秒周期の位置
static uint16_t g_wf_scroll_phase = 0;

// 600 sec × 501 bins
// PSRAMへ動的確保する
static float *g_hro_history = nullptr;

// 次に書き込む位置 0～599
static uint16_t g_hro_history_write_pos = 0;

// 現在保存されている秒数 0～600
static uint16_t g_hro_history_count = 0;

// 1秒分追加されるたびに増加
static std::atomic<uint32_t> g_hro_history_sequence{0};

// Tab5 UI定義
static constexpr int WF_X = 50;
static constexpr int WF_Y = 98;
static constexpr int WF_W = 1138;   // 右端1208を維持
static constexpr int WF_H = 263;

static constexpr int SPEC_X = 35;
static constexpr int SPEC_Y = 521;
static constexpr int SPEC_W = 966;
static constexpr int SPEC_H = 127;

static constexpr int LEVEL_X = WF_X;
static constexpr int LEVEL_Y = WF_Y + WF_H + 10;
static constexpr int LEVEL_W = WF_W;
static constexpr int LEVEL_H = 80;

static constexpr float LEVEL_DB_MIN = -10.0f;
static constexpr float LEVEL_DB_MAX =  30.0f;

static constexpr int SHUTDOWN_X = 780;
static constexpr int SHUTDOWN_Y = 530;
static constexpr int SHUTDOWN_W = 230;
static constexpr int SHUTDOWN_H = 40;

static constexpr int CAP_X = 0;
static constexpr int CAP_Y = 0;
static constexpr int CAP_W = 1280;
static constexpr int CAP_H = 490;

// Combined Sprite 内部のY座標
static constexpr int WF_SPRITE_Y    = 0;
static constexpr int LEVEL_SPRITE_Y = WF_H;

// Waterfall + Level Graph
static constexpr int WF_SPRITE_H = WF_H + LEVEL_H;
    
static bool g_ntp_started = false;
static std::atomic<bool> g_ntp_synced{false};

static constexpr const char *HRO_BACKGROUND_BASE = "/sdcard/radio_meteor_observation_base(3).png";

static bool g_wifi_setup_mode = false;

// -------------------------
// HRO Audio
// -------------------------

static constexpr uint32_t AUDIO_SAMPLE_RATE   = 8192;
static constexpr size_t   AUDIO_BLOCK_SAMPLES = 256;
static constexpr int      AUDIO_BUFFER_COUNT  = 3;

struct AudioBlock
{
    int16_t samples[AUDIO_BLOCK_SAMPLES];
};

static QueueHandle_t g_audio_queue = nullptr;

static AudioBlock g_audio_build_block;
static size_t g_audio_build_pos = 0;

static AudioBlock g_audio_play_buffer[AUDIO_BUFFER_COUNT];

static std::atomic<bool> g_audio_mute{false};
static std::atomic<int>  g_audio_volume{100};
static std::atomic<uint32_t> g_audio_drops{0};

struct DmsValue
{
    char direction;
    int degrees;
    int minutes;
    double seconds;
};

static constexpr int AUDIO_VOL_MINUS_X = 1025;
static constexpr int AUDIO_VOL_MINUS_Y = 557;
static constexpr int AUDIO_VOL_MINUS_W = 62;
static constexpr int AUDIO_VOL_MINUS_H = 38;

static constexpr int AUDIO_VOL_PLUS_X = 1193;
static constexpr int AUDIO_VOL_PLUS_Y = 557;
static constexpr int AUDIO_VOL_PLUS_W = 62;
static constexpr int AUDIO_VOL_PLUS_H = 38;

static constexpr int AUDIO_MUTE_X = 1080;
static constexpr int AUDIO_MUTE_Y = 605;
static constexpr int AUDIO_MUTE_W = 120;
static constexpr int AUDIO_MUTE_H = 34;

static uint32_t g_level_phase = 0;
static bool g_level_have_prev = false;
static int g_level_prev_x = 0;
static int g_level_prev_y = 0;

static std::atomic<bool> g_audio_config_dirty{false};
static TickType_t g_audio_config_change_tick = 0;
static std::atomic<int> g_touch_beep_samples{0};
static float g_touch_beep_phase = 0.0f;

static constexpr int64_t SHUTDOWN_HOLD_US = 2 * 1000 * 1000LL;
static bool g_shutdown_holding = false;
static int64_t g_shutdown_press_us = 0;
static std::atomic<bool> g_hro_shutdown_requested{ false };
static std::atomic<bool> g_hro_storage_stopped{ false };

static time_t g_screenshot_block_start = 0;

// -------------------------
// SD Card Recoding
// -------------------------
static constexpr int HRO_SAVE_BUFFER_COUNT = 16;
struct HroSaveRecord
{
    time_t timestamp;
    float spectrum[HRO_BIN_COUNT];
};

static constexpr int HRO_SAVE_BATCH_SECONDS = 10;
struct HroSaveBatch
{
    time_t first_timestamp;
    int count;
    float spectrum[HRO_SAVE_BATCH_SECONDS][HRO_BIN_COUNT];
};

static HroSaveRecord *g_hro_save_buffers = nullptr;
static HroSaveBatch *g_hro_save_batch = nullptr;
static QueueHandle_t g_hro_save_free_queue = nullptr;
static QueueHandle_t g_hro_save_write_queue = nullptr;
static std::atomic<uint32_t> g_hro_save_drops{0};

static constexpr uint32_t HRO_FILE_VERSION       = 1;
static constexpr size_t   HRO_HEADER_SIZE        = 8192;
static constexpr size_t   HRO_VALIDITY_MAP_SIZE  = 512;
static constexpr int      HRO_RECORD_SLOTS       = 3600;

static constexpr const char *HRO_DATA_ROOT = "/sdcard/tab5-hro/data";

// -------------------------
// Sprite
// -------------------------
static M5Canvas g_base_screen( &M5.Display );
static M5Canvas g_waterfall( &M5.Display );
static M5Canvas g_level_graph( &M5.Display );

static time_t get_screenshot_block_start( time_t timestamp )
{
    static constexpr time_t BLOCK_SECONDS = 10 * 60;
    return timestamp - ( timestamp % BLOCK_SECONDS );
}

static bool make_screenshot_filename( time_t block_start, char *filename,  size_t filename_size )
{
    if ( filename == nullptr || filename_size == 0 ) {
        return false;
    }

    if ( !is_valid_file_prefix( g_hro_config.screenshot_prefix ) ) {
        return false;
    }

    // block_start は Unix time (UTC)。
    // ファイル名はJSTで生成する。
    const time_t jst_timestamp = block_start + 9 * 60 * 60;

    struct tm jst_tm;
    gmtime_r( &jst_timestamp, &jst_tm );

    const int ret = snprintf(
        filename,
        filename_size,
        "%s%04d%02d%02d%02d%02d.png",
        g_hro_config.screenshot_prefix,
        jst_tm.tm_year + 1900,
        jst_tm.tm_mon + 1,
        jst_tm.tm_mday,
        jst_tm.tm_hour,
        jst_tm.tm_min
    );

    return ret > 0 && static_cast<size_t>( ret ) < filename_size;
}

static void clear_screenshot_info( void )
{
    M5.Display.fillRect( 30, 450, 530, 35, TFT_BLACK );
}

static void draw_screenshot_info( time_t block_start, const char *filename )
{
    if ( filename == nullptr ) {
        return;
    }

    // UTC Unix time → JST
    const time_t jst_timestamp = block_start + 9 * 60 * 60;

    struct tm jst_tm;
    gmtime_r( &jst_timestamp, &jst_tm );

    char text[128];
    snprintf(
        text,
        sizeof( text ),
        "%04d/%02d/%02d %02d:%02d(JST)    %s",
        jst_tm.tm_year + 1900,
        jst_tm.tm_mon + 1,
        jst_tm.tm_mday,
        jst_tm.tm_hour,
        jst_tm.tm_min,
        filename
    );

    M5.Display.setFont( &fonts::Font2 );
    M5.Display.setTextDatum( middle_left );
    M5.Display.setTextColor( TFT_WHITE, TFT_BLACK );

    // 仮座標。赤枠位置に合わせて微調整
    M5.Display.drawString( text, 40, 466 );
}

static void log_battery_status( void )
{
    const int32_t level   = M5.Power.getBatteryLevel();
    const int16_t voltage = M5.Power.getBatteryVoltage();
    const int32_t current = M5.Power.getBatteryCurrent();
    const bool charging   = M5.Power.isCharging();

    ESP_LOGI( TAG, "BAT: level=%ld%% voltage=%d mV current=%ld mA charging=%d", static_cast<long>( level ), static_cast<int>( voltage ), static_cast<long>( current ), charging ? 1 : 0 );
}

static bool save_screenshot_png( time_t block_start, const char *filename )
{
    if ( filename == nullptr || filename[0] == '\0' ) {
        return false;
    }

    // ログ表示用にJSTへ変換
    const time_t jst_timestamp =
        block_start + 9 * 60 * 60;

    struct tm local_tm;
    gmtime_r( &jst_timestamp, &local_tm );

    char path[128];

    snprintf( path, sizeof( path ), "/sdcard/tab5-hro/%s", filename );

    ESP_LOGI(
        TAG,
        "Screenshot: block=%04d/%02d/%02d %02d:%02d:00 JST file=%s",
        local_tm.tm_year + 1900,
        local_tm.tm_mon + 1,
        local_tm.tm_mday,
        local_tm.tm_hour,
        local_tm.tm_min,
        filename
    );

    ESP_LOGI( TAG, "Screenshot: start" );

    const int64_t total_start = esp_timer_get_time();

    // ---- LCD -> PNG ----
    size_t png_size = 0;
    const int64_t encode_start = esp_timer_get_time();

    void* png_data = M5.Display.createPng( &png_size, CAP_X, CAP_Y, CAP_W, CAP_H );

    const int64_t encode_end = esp_timer_get_time();
    if ( png_data == nullptr || png_size == 0 ) {
        ESP_LOGE( TAG, "Screenshot: PNG encode failed" );
        if ( png_data != nullptr ) {
            free( png_data );
        }
        return false;
    }

    ESP_LOGI( TAG, "Screenshot: PNG encoded size=%u bytes time=%lld ms", (unsigned)png_size, (long long)(( encode_end - encode_start ) / 1000 ) );

    // ---- PNG -> SD ----
    const int64_t write_start = esp_timer_get_time();

    FILE* fp = fopen( path, "wb" );
    if ( fp == nullptr ) {
        ESP_LOGE( TAG, "Screenshot: fopen failed: %s", path );
        free( png_data );
        return false;
    }

    const size_t written = fwrite( png_data, 1, png_size, fp );
    fclose( fp );

    const int64_t write_end = esp_timer_get_time();

    free( png_data );

    if ( written != png_size ) {
        ESP_LOGE( TAG, "Screenshot: short write %u / %u bytes", (unsigned)written, (unsigned)png_size );
        return false;
    }

    const int64_t total_end = esp_timer_get_time();
    ESP_LOGI( TAG, "Screenshot: saved %s write=%lld ms total=%lld ms", path, (long long)(( write_end - write_start ) / 1000 ), (long long)(( total_end - total_start ) / 1000 ) );
    return true;
}

static void draw_battery_status( void )
{
    const int32_t level   = M5.Power.getBatteryLevel();
    const int16_t voltage = M5.Power.getBatteryVoltage();
    const int32_t current = M5.Power.getBatteryCurrent();
    const bool charging   = M5.Power.isCharging();

    char buf[64];
    snprintf( buf, sizeof( buf ), "BAT %ld%%  %dmV  %ldmA%s", static_cast<long>( level ), static_cast<int>( voltage ), static_cast<long>( current ), charging ? " CHG" : "" );

    // 動作確認用表示領域
    constexpr int X = 764;
    constexpr int Y = 682;
    constexpr int W = 220;
    constexpr int H = 18;

    // 前回表示を消す
    M5.Display.fillRect( X, Y, W, H, DARKGREEN );

    M5.Display.setTextSize( 1 );
    M5.Display.setTextColor( WHITE );
    M5.Display.setTextDatum( middle_right );

    M5.Display.drawString( buf, X + W - 5, Y + H / 2 );
}

static bool ensure_directory( const char *path )
{
    struct stat st;

    if ( stat( path, &st ) == 0 ) {
        return S_ISDIR( st.st_mode );
    }

    if ( mkdir( path, 0775 ) == 0 ) {
        return true;
    }

    if ( errno == EEXIST ) {
        return true;
    }
    ESP_LOGE( TAG, "mkdir failed: %s errno=%d", path, errno );

    return false;
}

static bool make_hro_file_path( time_t timestamp, char *path, size_t path_size )
{
    struct tm local_tm;

    // HROファイル名はJST (UTC+9)
    const time_t jst_timestamp = timestamp + 9 * 60 * 60;
    if ( gmtime_r( &jst_timestamp, &local_tm ) == nullptr ) {
        return false;
    }

    char date_dir[128];
    snprintf( date_dir, sizeof( date_dir ), "%s/%04d%02d%02d", HRO_DATA_ROOT, local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday );

    // /sdcard/tab5-hro/data
    if ( !ensure_directory( HRO_DATA_ROOT ) ) {
        return false;
    }

    // /sdcard/tab5-hro/data/YYYYMMDD
    if ( !ensure_directory( date_dir ) ) {
        return false;
    }
    snprintf( path, path_size, "%s/HRO_%04d%02d%02d_%02d0000.hro", date_dir, local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday, local_tm.tm_hour );

    return true;
}

static bool write_hro_header( FILE *fp, time_t timestamp )
{
    if ( fp == nullptr ) {
        return false;
    }

    // ---------------------------------------------------------
    // Headerバッファは大きいのでスタックには置かずPSRAMに確保
    // ---------------------------------------------------------
    char *header = static_cast<char *>( heap_caps_malloc( HRO_HEADER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );
    if ( header == nullptr ) {
        ESP_LOGE( TAG, "Failed to allocate HRO header buffer" );
        return false;
    }
    memset( header, ' ', HRO_HEADER_SIZE );

    // ---------------------------------------------------------
    // このレコードが属するJSTの「正時」を求める
    //
    // timestamp は Unix time (UTC基準)
    // ファイルの区切りはJSTの正時なので、一度JSTへ変換して
    // 分・秒を切り捨てる。
    // ---------------------------------------------------------

    const time_t jst_timestamp = timestamp + 9 * 60 * 60;

    struct tm jst_tm;

    if ( gmtime_r( &jst_timestamp, &jst_tm ) == nullptr ) {
        ESP_LOGE( TAG, "Failed to convert HRO file time" );
        return false;
    }

    const int minute = jst_tm.tm_min;
    const int second = jst_tm.tm_sec;

    // このHROファイルの開始時刻（Unix time）
    const time_t file_start_utc = timestamp - static_cast<time_t>( minute ) * 60 - static_cast<time_t>( second );

    // ---------------------------------------------------------
    // FILE_HOUR_LOCAL
    // ---------------------------------------------------------

    char file_hour_local[64];

    snprintf( file_hour_local, sizeof( file_hour_local ), "%04d-%02d-%02dT%02d:00:00+09:00",
        jst_tm.tm_year + 1900,
        jst_tm.tm_mon + 1,
        jst_tm.tm_mday,
        jst_tm.tm_hour
    );

    // ---------------------------------------------------------
    // FILE_START_UTC
    // ---------------------------------------------------------

    struct tm utc_tm;

    if ( gmtime_r( &file_start_utc, &utc_tm ) == nullptr ) {
        ESP_LOGE( TAG, "Failed to convert HRO UTC time" );
        return false;
    }

    char file_start_utc_text[64];

    snprintf( file_start_utc_text, sizeof( file_start_utc_text ), "%04d-%02d-%02dT%02d:%02d:%02dZ",
        utc_tm.tm_year + 1900,
        utc_tm.tm_mon + 1,
        utc_tm.tm_mday,
        utc_tm.tm_hour,
        utc_tm.tm_min,
        utc_tm.tm_sec
    );

    // ---------------------------------------------------------
    // FFT表示範囲
    // 現在は 1 Hz/bin
    // ---------------------------------------------------------

    const int32_t fft_min_hz = g_hro_config.fft_center_hz - g_hro_config.fft_range_hz;
    const int32_t fft_max_hz = g_hro_config.fft_center_hz + g_hro_config.fft_range_hz;
    const int32_t fft_bin_count = fft_max_hz - fft_min_hz + 1;

    const int length = snprintf(
        header,
        HRO_HEADER_SIZE,

        "HRO_FILE_VERSION=%u\n"
        "HEADER_SIZE=%u\n"
        "VALIDITY_MAP_SIZE=%u\n"
        "DATA_TYPE=FLOAT32\n"
        "BYTE_ORDER=LITTLE_ENDIAN\n"
        "\n"

        "FILE_START_UTC=%s\n"
        "FILE_HOUR_LOCAL=%s\n"
        "TIME_ZONE=Asia/Tokyo\n"
        "\n"

        "OBSERVER=%s\n"
        "LOCATION=%s\n"
        "LONGITUDE=%.8f\n"
        "LATITUDE=%.8f\n"
        "\n"

        "RECEIVER=%s\n"
        "ANTENNA=%s\n"
        "RF_FREQUENCY_HZ=%lu\n"
        "\n"

        "FFT_SIZE=8192\n"
        "FFT_OUTPUT_RATE_HZ=8192\n"
        "FFT_RESOLUTION_HZ=1.0\n"
        "FFT_CENTER_HZ=%ld\n"
        "FFT_RANGE_HZ=%ld\n"
        "FFT_MIN_HZ=%ld\n"
        "FFT_MAX_HZ=%ld\n"
        "FFT_BIN_COUNT=%ld\n"
        "\n"

        "RECORD_INTERVAL_SEC=1\n"
        "RECORD_SLOTS=%d\n"
        "\n"

        "DEVICE=M5Stack Tab5\n"
        "SOFTWARE=Tab5-HRO\n"
        "\n"

        "END_HEADER\n",

        static_cast<unsigned>( HRO_FILE_VERSION ),
        static_cast<unsigned>( HRO_HEADER_SIZE ),
        static_cast<unsigned>( HRO_VALIDITY_MAP_SIZE ),

        file_start_utc_text,
        file_hour_local,

        g_hro_config.observer,
        g_hro_config.location,
        g_hro_config.longitude,
        g_hro_config.latitude,

        g_hro_config.receiver,
        g_hro_config.antenna,

        static_cast<unsigned long>( g_hro_config.frequency_hz ),
        static_cast<long>( g_hro_config.fft_center_hz ),
        static_cast<long>( g_hro_config.fft_range_hz ),
        static_cast<long>( fft_min_hz ),
        static_cast<long>( fft_max_hz ),
        static_cast<long>( fft_bin_count ),

        HRO_RECORD_SLOTS
    );

    if ( length < 0 || static_cast<size_t>( length ) >= sizeof( header ) ) {
        ESP_LOGE( TAG, "HRO header too large: %d bytes", length );
        return false;
    }

    // ---------------------------------------------------------
    // 必ずファイル先頭から8192 bytes書く
    // ---------------------------------------------------------

    if ( fseek( fp, 0, SEEK_SET ) != 0 ) {
        ESP_LOGE( TAG, "Failed to seek HRO header" );
        return false;
    }

    if ( fwrite( header, 1, sizeof( header ), fp ) != sizeof( header ) ) {
        ESP_LOGE( TAG, "Failed to write HRO header" );
        return false;
    }
    return true;
}

static bool write_hro_fft_batch( FILE *fp, const HroSaveBatch *batch )
{
    if ( fp == nullptr || batch == nullptr || batch->count <= 0 ) {
        return false;
    }

    const time_t jst_timestamp =
        batch->first_timestamp + 9 * 60 * 60;

    struct tm jst_tm;

    if ( gmtime_r( &jst_timestamp, &jst_tm ) == nullptr ) {
        ESP_LOGE( TAG, "Failed to convert HRO batch time" );
        return false;
    }

    const int first_slot =
        jst_tm.tm_min * 60 + jst_tm.tm_sec;

    if ( first_slot < 0 ||
         first_slot + batch->count > HRO_RECORD_SLOTS ) {

        ESP_LOGE(
            TAG,
            "Invalid HRO batch: slot=%d count=%d",
            first_slot,
            batch->count
        );

        return false;
    }

    // --------------------------------------------------
    // FFT data write
    // --------------------------------------------------

    const long slot_size =
        static_cast<long>(
            HRO_BIN_COUNT * sizeof( float )
        );

    const long data_offset =
        static_cast<long>(
            HRO_HEADER_SIZE +
            HRO_VALIDITY_MAP_SIZE
        )
        +
        static_cast<long>( first_slot ) * slot_size;

    if ( fseek( fp, data_offset, SEEK_SET ) != 0 ) {
        ESP_LOGE(
            TAG,
            "Failed to seek HRO batch: slot=%d offset=%ld",
            first_slot,
            data_offset
        );

        return false;
    }

    const size_t value_count =
        static_cast<size_t>( batch->count ) *
        HRO_BIN_COUNT;

    const size_t written =
        fwrite(
            batch->spectrum,
            sizeof( float ),
            value_count,
            fp
        );

    if ( written != value_count ) {
        ESP_LOGE(
            TAG,
            "Failed to write HRO batch: %u/%u floats",
            static_cast<unsigned>( written ),
            static_cast<unsigned>( value_count )
        );

        return false;
    }

    // --------------------------------------------------
    // TEST:
    // Validity Map update intentionally disabled
    // --------------------------------------------------

    return true;
}

#if 0
static bool write_hro_fft_batch( FILE *fp, const HroSaveBatch *batch )
{
    if ( fp == nullptr || batch == nullptr || batch->count <= 0 ) {
        return false;
    }

    const time_t jst_timestamp = batch->first_timestamp + 9 * 60 * 60;

    struct tm jst_tm;
    if ( gmtime_r( &jst_timestamp, &jst_tm ) == nullptr ) {
        ESP_LOGE( TAG, "Failed to convert HRO batch time" );
        return false;
    }

    const int first_slot = jst_tm.tm_min * 60 + jst_tm.tm_sec;

    if ( first_slot < 0 || first_slot + batch->count > HRO_RECORD_SLOTS ) {
        ESP_LOGE( TAG, "Invalid HRO batch: slot=%d count=%d", first_slot, batch->count );
        return false;
    }

    // --------------------------------------------------
    // FFT data write
    // --------------------------------------------------

    const long slot_size = static_cast<long>( HRO_BIN_COUNT * sizeof( float ) );
    const long data_offset = static_cast<long>( HRO_HEADER_SIZE + HRO_VALIDITY_MAP_SIZE ) + static_cast<long>( first_slot ) * slot_size;

    if ( fseek( fp, data_offset, SEEK_SET ) != 0 ) {
        ESP_LOGE( TAG, "Failed to seek HRO batch: slot=%d offset=%ld", first_slot, data_offset );
        return false;
    }

    const size_t value_count = static_cast<size_t>( batch->count ) * HRO_BIN_COUNT;
    const size_t written = fwrite( batch->spectrum, sizeof( float ), value_count, fp );

    if ( written != value_count ) {
        ESP_LOGE( TAG, "Failed to write HRO batch: %u/%u floats", static_cast<unsigned>( written ), static_cast<unsigned>( value_count ) );
        return false;
    }

    // --------------------------------------------------
    // Validity Map update
    //
    // FFT data write が成功した後でのみ valid にする
    // --------------------------------------------------

    const int last_slot = first_slot + batch->count - 1;
    const int first_byte = first_slot / 8;
    const int last_byte  = last_slot  / 8;
    const int validity_bytes = last_byte - first_byte + 1;

    // 最大でも10秒batchなので通常2～3 bytes程度
    uint8_t validity[4] = { 0 };

    const long validity_offset = static_cast<long>( HRO_HEADER_SIZE + first_byte );

    // 現在のValidity Mapを読む
    if ( fseek( fp, validity_offset, SEEK_SET ) != 0 ) {
        ESP_LOGE( TAG, "Failed to seek HRO validity map" );
        return false;
    }

    const size_t validity_read = fread( validity, 1, validity_bytes, fp );
    if ( validity_read != static_cast<size_t>( validity_bytes ) ) {
        ESP_LOGE( TAG, "Failed to read HRO validity map" );
        return false;
    }

    // 今回書き込んだslotをvalidにする
    for ( int slot = first_slot; slot <= last_slot; ++slot ) {
        const int byte_index = slot / 8 - first_byte;
        const int bit_index = slot % 8;
        validity[byte_index] |= static_cast<uint8_t>( 1U << bit_index );
    }

    // 同じ部分を書き戻す
    if ( fseek( fp, validity_offset, SEEK_SET ) != 0 ) {
        ESP_LOGE( TAG, "Failed to seek HRO validity map for write" );
        return false;
    }

    const size_t validity_written = fwrite( validity, 1, validity_bytes, fp );
    if ( validity_written != static_cast<size_t>( validity_bytes ) ) {
        ESP_LOGE( TAG, "Failed to write HRO validity map" );
        return false;
    }

    return true;
}
#endif

static bool write_hro_fft_slot( FILE *fp, const HroSaveRecord *record )
{
    if ( fp == nullptr || record == nullptr ) {
        return false;
    }

    // ---------------------------------------------------------
    // recordの時刻をJSTへ変換
    // グローバルTZ設定には依存しない
    // ---------------------------------------------------------
    const time_t jst_timestamp = record->timestamp + 9 * 60 * 60;
    struct tm jst_tm;

    if ( gmtime_r( &jst_timestamp, &jst_tm ) == nullptr ) {
        ESP_LOGE( TAG, "Failed to convert HRO record time" );
        return false;
    }

    // 00:00 ～ 59:59 → 0 ～ 3599
    const int slot =
        jst_tm.tm_min * 60 +
        jst_tm.tm_sec;

    if ( slot < 0 || slot >= HRO_RECORD_SLOTS ) {
        ESP_LOGE( TAG, "Invalid HRO slot: %d", slot );
        return false;
    }

    // ---------------------------------------------------------
    // FFTデータ位置
    //
    // Header       8192 bytes
    // Validity Map  512 bytes
    // FFT data      slot × (HRO_BIN_COUNT × 4)
    // ---------------------------------------------------------
    const long data_offset = static_cast<long>( HRO_HEADER_SIZE + HRO_VALIDITY_MAP_SIZE );
    const long slot_size = static_cast<long>( HRO_BIN_COUNT * sizeof( float ) );
    const long offset = data_offset + static_cast<long>( slot ) * slot_size;

    if ( fseek( fp, offset, SEEK_SET ) != 0 ) {
        ESP_LOGE( TAG, "Failed to seek HRO slot: %d offset=%ld", slot, offset );
        return false;
    }

    const size_t written = fwrite( record->spectrum, sizeof( float ), HRO_BIN_COUNT, fp );
    if ( written != HRO_BIN_COUNT ) {
        ESP_LOGE( TAG, "Failed to write HRO slot: %d written=%u/%u", slot, static_cast<unsigned>( written ), static_cast<unsigned>( HRO_BIN_COUNT ) );
        return false;
    }
    return true;
}

static bool create_hro_test_file( time_t timestamp )
{
    char path[192];

    // HROファイルのパスを作る
    if ( !make_hro_file_path( timestamp, path, sizeof( path ) ) ) {
        ESP_LOGE( TAG, "Failed to make HRO file path" );
        return false;
    }

    // すでにファイルが存在する場合は作り直さない
    struct stat st;

    if ( stat( path, &st ) == 0 ) {
        ESP_LOGI( TAG, "HRO file already exists: %s size=%lld", path, static_cast<long long>( st.st_size ) );
        return true;
    }

    // 新規ファイル作成
    FILE *fp = fopen( path, "wb" );

    if ( fp == nullptr ) {
        ESP_LOGE( TAG, "Failed to create HRO file: %s errno=%d", path, errno );
        return false;
    }

    // ---------------------------------------------------------
    // 8192 byte Header
    // ---------------------------------------------------------
#if 1
    if ( !write_hro_header( fp, timestamp ) ) {
        ESP_LOGE( TAG, "Failed to write HRO header" );
        fclose( fp );
        return false;
    }
#endif
    // ---------------------------------------------------------
    // 512 byte Validity Map
    //
    // 新規ファイルなので全slotを無効(0)で初期化
    // ---------------------------------------------------------

    uint8_t validity[HRO_VALIDITY_MAP_SIZE];

    memset( validity, 0, sizeof( validity ) );

    if ( fwrite( validity, 1, sizeof( validity ), fp ) != sizeof( validity ) ) {
        ESP_LOGE( TAG, "Failed to write HRO validity map" );
        fclose( fp );
        return false;
    }

    // SDカードへ反映
    fflush( fp );

    // ファイルを閉じる
    fclose( fp );

    ESP_LOGI( TAG, "Created HRO test file: %s size=%u", path, static_cast<unsigned>( HRO_HEADER_SIZE + HRO_VALIDITY_MAP_SIZE ) );
    return true;
}


static void draw_waterfall_time_axis( time_t axis_end = 0 )
{
    const int axis_y = WF_Y - 6;
    const int text_y = WF_Y - 22;

    M5.Display.fillRect( WF_X, WF_Y - 24, WF_W, 24, BLACK );

    time_t now;
    if ( axis_end != 0 ) {
        now = axis_end;
    } else {
        time( &now );
    }

    M5.Display.setTextSize( 1 );
    M5.Display.setTextColor( CYAN, BLACK );
    M5.Display.setTextDatum( top_center );

    for ( int sec = 0; sec <= TOTAL_SEC; sec += INTERVAL_SEC ) {
        const int x = WF_X + WF_W - static_cast<int>( static_cast<int64_t>( sec ) * WF_W / TOTAL_SEC );
        const time_t t = now - sec;

        struct tm tm_local;
        localtime_r( &t, &tm_local );

        char buf[16];

        snprintf( buf, sizeof( buf ), "%02d:%02d", tm_local.tm_hour, tm_local.tm_min );

        M5.Display.drawFastVLine( x, axis_y, 5, CYAN );
        M5.Display.drawString( buf, x, text_y );
    }
}

#if 0
static void draw_waterfall_time_axis( void )
{
	const int axis_y = WF_Y - 6;
	const int text_y = WF_Y - 22;

    // -----------------------------------------
    // 時刻軸表示領域を消去
    // -----------------------------------------
	M5.Display.fillRect( WF_X, WF_Y - 24, WF_W, 24, BLACK );

    time_t now;
    time( &now );

    M5.Display.setTextSize( 1 );
    M5.Display.setTextColor( CYAN, BLACK );
    M5.Display.setTextDatum( top_center );

    for ( int sec = 0; sec <= TOTAL_SEC; sec += INTERVAL_SEC ) {
        const int x = WF_X + WF_W - static_cast<int>( static_cast<int64_t>( sec ) * WF_W / TOTAL_SEC );
        const time_t t = now - sec;

        struct tm tm_local;
        localtime_r( &t, &tm_local );

        char buf[16];
        snprintf( buf, sizeof(buf), "%02d:%02d", tm_local.tm_hour, tm_local.tm_min );

		M5.Display.drawFastVLine( x, axis_y, 5, CYAN );
        M5.Display.drawString( buf, x, text_y );
    }
}
#endif

static int level_db_to_y( float db )
{
    if ( db < LEVEL_DB_MIN ) {
        db = LEVEL_DB_MIN;
    } else if ( db > LEVEL_DB_MAX ) {
        db = LEVEL_DB_MAX;
    }
    const float normalized = (db - LEVEL_DB_MIN) / ( LEVEL_DB_MAX - LEVEL_DB_MIN );
    return LEVEL_H - 1 - static_cast<int>( normalized * (LEVEL_H - 1 ) );
}

static int hro_center_bin( void )
{
    const int32_t min_hz = hro_fft_min_hz();
    int bin = static_cast<int>( g_hro_config.fft_center_hz - min_hz );

    if ( bin < 0 ) {
        bin = 0;
    } else if ( bin >= HRO_BIN_COUNT ) {
        bin = HRO_BIN_COUNT - 1;
    }
    return bin;
}

static void play_touch_beep( void )
{
    // 約40ms
    g_touch_beep_samples.store( AUDIO_SAMPLE_RATE * 40 / 1000, std::memory_order_relaxed );
}

static bool init_hro_save_buffers( void )
{
    g_hro_save_buffers = static_cast<HroSaveRecord *>( heap_caps_malloc( sizeof( HroSaveRecord ) * HRO_SAVE_BUFFER_COUNT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );

    if ( g_hro_save_buffers == nullptr ) {
        ESP_LOGE( TAG, "Failed to allocate HRO save buffers" );
        return false;
    }

    g_hro_save_free_queue = xQueueCreate( HRO_SAVE_BUFFER_COUNT, sizeof( HroSaveRecord * ) );
    g_hro_save_write_queue = xQueueCreate( HRO_SAVE_BUFFER_COUNT, sizeof( HroSaveRecord * ) );

    if ( g_hro_save_free_queue == nullptr || g_hro_save_write_queue == nullptr ) {
        ESP_LOGE( TAG, "Failed to create HRO save queues" );
        if ( g_hro_save_free_queue != nullptr ) {
            vQueueDelete( g_hro_save_free_queue );
            g_hro_save_free_queue = nullptr;
        }

        if ( g_hro_save_write_queue != nullptr ) {
            vQueueDelete( g_hro_save_write_queue );
            g_hro_save_write_queue = nullptr;
        }
        heap_caps_free( g_hro_save_buffers );
        g_hro_save_buffers = nullptr;

        return false;
    }

    // 全バッファを空きQueueへ登録
    for ( int i = 0; i < HRO_SAVE_BUFFER_COUNT; ++i ) {
        HroSaveRecord *record = &g_hro_save_buffers[i];
        xQueueSend( g_hro_save_free_queue, &record, portMAX_DELAY );
    }
    ESP_LOGI( TAG, "HRO save buffers initialized: %d buffers, %u bytes", HRO_SAVE_BUFFER_COUNT, static_cast<unsigned>( sizeof( HroSaveRecord ) * HRO_SAVE_BUFFER_COUNT ) );

	g_hro_save_batch = static_cast<HroSaveBatch *>( heap_caps_malloc( sizeof( HroSaveBatch ), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );
	if ( g_hro_save_batch == nullptr ) {
	    ESP_LOGE( TAG, "Failed to allocate HRO batch buffer" );
	    return false;
	}
	memset( g_hro_save_batch, 0, sizeof( HroSaveBatch ) );
	ESP_LOGI( TAG, "HRO batch buffer initialized: %u bytes", static_cast<unsigned>( sizeof( HroSaveBatch ) ) );

    return true;
}

static DmsValue longitude_to_dms( double value )
{
    DmsValue dms = {};

    dms.direction = ( value < 0.0 ) ? 'W' : 'E';
    double v = fabs( value );
    dms.degrees = static_cast<int>( v );
    double m = ( v - dms.degrees ) * 60.0;
    dms.minutes = static_cast<int>( m );
    dms.seconds = ( m - dms.minutes ) * 60.0;

    return dms;
}

static DmsValue latitude_to_dms( double value )
{
    DmsValue dms = {};

    dms.direction = ( value < 0.0 ) ? 'S' : 'N';
    double v = fabs( value );
    dms.degrees = static_cast<int>( v );
    double m = ( v - dms.degrees ) * 60.0;
    dms.minutes = static_cast<int>( m );
    dms.seconds = ( m - dms.minutes ) * 60.0;

    return dms;
}

static int hex_value( char c )
{
    if ( c >= '0' && c <= '9' ) return c - '0';
    if ( c >= 'a' && c <= 'f' ) return c - 'a' + 10;
    if ( c >= 'A' && c <= 'F' ) return c - 'A' + 10;
    return -1;
}

static void url_decode( char *dst, size_t dst_size, const char *src )
{
    if ( dst_size == 0 ) {
        return;
    }

    size_t di = 0;
    while ( *src != '\0' && di < dst_size - 1 ) {
        if ( *src == '+' ) {
            dst[di++] = ' ';
            src++;
        } else if ( *src == '%' && src[1] != '\0' && src[2] != '\0' ) {
            int hi = hex_value( src[1] );
            int lo = hex_value( src[2] );
            if ( hi >= 0 && lo >= 0 ) {
                dst[di++] = static_cast<char>( ( hi << 4 ) | lo );
                src += 3;
            } else {
                dst[di++] = *src++;
            }
        } else {
            dst[di++] = *src++;
        }
    }
    dst[di] = '\0';
}

static bool get_form_value( const char *body, const char *name, char *out, size_t out_size )
{
    if ( !body || !name || !out || out_size == 0 ) {
        return false;
    }

    char key[64];
    snprintf( key, sizeof( key ), "%s=", name );

    const char *p = strstr( body, key );
    while ( p ) {
        // 先頭、または & の直後だけをキーとして認識
        if ( p == body || *( p - 1 ) == '&' ) {
            break;
        }
        p = strstr( p + 1, key );
    }

    if ( !p ) {
        return false;
    }

    p += strlen( key );

    const char *end = strchr( p, '&' );
    size_t len = end ? static_cast<size_t>( end - p ): strlen( p );

    // URLエンコード状態の一時バッファ
    char encoded[256];

    if ( len >= sizeof( encoded ) ) {
        len = sizeof( encoded ) - 1;
    }

    memcpy( encoded, p, len );
    encoded[len] = '\0';

    // 既存のURLデコード関数を使用
    url_decode( out, out_size, encoded );

    return true;
}

static HroTuning make_hro_tuning( void )
{
    HroTuning t = {};

    t.target_rf_hz  = g_hro_config.frequency_hz;
    t.fft_center_hz = g_hro_config.fft_center_hz;

#if 0
    // ------------------------------------------------------------
    // TEST 1: 目的RFをRTL-SDR LOから +1 kHz
    // ------------------------------------------------------------
    t.ideal_lo_hz = t.target_rf_hz - static_cast<uint32_t>( t.fft_center_hz );
    t.actual_lo_hz = ( t.ideal_lo_hz / 1000U ) * 1000U;

    t.actual_if_hz = static_cast<int32_t>( t.target_rf_hz - t.actual_lo_hz );
    t.nco_shift_hz = static_cast<float>( t.fft_center_hz - t.actual_if_hz );

#elif 0
    // ------------------------------------------------------------
    // TEST 2: 目的RFをRTL-SDR LOから +5 kHz
    // ------------------------------------------------------------
    t.ideal_lo_hz = t.target_rf_hz - 5000U;
    t.actual_lo_hz = ( t.ideal_lo_hz / 1000U ) * 1000U;

    t.actual_if_hz = static_cast<int32_t>( t.target_rf_hz - t.actual_lo_hz );

    t.nco_shift_hz = static_cast<float>( t.fft_center_hz - t.actual_if_hz );

#elif 1
    // ------------------------------------------------------------
    // TEST 3: rtl_fm方式 Fs/4 DC回避
    //
    // Fs = 256 kHz
    // Fs/4 = 64 kHz
    //
    // 目的RFをRTL-SDR LOの -240 kHz に置く。
    // raw IQ側で +Fs/4 rotate を行いDCへ戻す。
    // その後、既存NCOでFFT中心 780 Hzへ移動する。
    // ------------------------------------------------------------

    static constexpr uint32_t FS4_HZ = RTL_SAMPLE_RATE / 4U;   // RTL_SAMPLE_RATE/ 4 Hz

    t.ideal_lo_hz = t.target_rf_hz + FS4_HZ;
    t.actual_lo_hz = ( t.ideal_lo_hz / 1000U ) * 1000U;

    // rotate前の実IF
    // 53.372000 - 53.612000 = -240000 Hz
    t.actual_if_hz = static_cast<int32_t>( t.target_rf_hz - t.actual_lo_hz );

    // Fs/4 rotate後は目的信号がほぼ0 Hzになるので、
    // 既存NCOではFFT中心まで +780 Hz移動。
    //
    // ※ 960k側のrotateを追加した場合のみ、この値を使用する。
    const int32_t if_after_fs4 = t.actual_if_hz + static_cast<int32_t>( FS4_HZ );

    t.nco_shift_hz = static_cast<float>( t.fft_center_hz - if_after_fs4 );

#endif

    return t;
}

static void audio_config_changed( void )
{
    g_audio_config_dirty.store( true, std::memory_order_relaxed );
    g_audio_config_change_tick = xTaskGetTickCount();
}

static void save_audio_config_if_needed( void )
{
    if ( !g_audio_config_dirty.load( std::memory_order_relaxed ) ) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    if ( ( now - g_audio_config_change_tick ) < pdMS_TO_TICKS( 2000 ) ) {
        return;
    }

    // 現在のAudio設定をHRO設定へ反映
    g_hro_config.audio_volume = g_audio_volume.load( std::memory_order_relaxed );
    g_hro_config.audio_mute = g_audio_mute.load( std::memory_order_relaxed );

    if ( save_hro_config() ) {
        g_audio_config_dirty.store( false, std::memory_order_relaxed );
        ESP_LOGI( TAG, "Audio config saved: volume=%d mute=%d", g_hro_config.audio_volume, g_hro_config.audio_mute ? 1 : 0 );
    }
}

static void time_sync_notification_cb( struct timeval *tv )
{
    ESP_LOGI( TAG, "NTP time synchronized" );

    time_t now;
    time( &now );

    struct tm utc_time;
    gmtime_r( &now, &utc_time );

    char buf[64];
    strftime( buf, sizeof( buf ), "%Y-%m-%d %H:%M:%S UTC", &utc_time );
    ESP_LOGI( TAG, "UTC time: %s", buf );
    
    // NTPのUTC時刻をTab5 RTCへ保存
    if ( M5.Rtc.isEnabled() ) {
        m5::rtc_datetime_t rtc_dt = {
        	{
	        	static_cast<int16_t>( utc_time.tm_year + 1900 ),
	        	static_cast<int8_t>( utc_time.tm_mon + 1 ),
	        	static_cast<int8_t>( utc_time.tm_mday )
		    },
    		{
        		static_cast<int8_t>( utc_time.tm_hour ),
	        	static_cast<int8_t>( utc_time.tm_min ),
	        	static_cast<int8_t>( utc_time.tm_sec )
	   		}
        };
		g_ntp_synced.store( true, std::memory_order_relaxed );
        M5.Rtc.setDateTime( rtc_dt );
        ESP_LOGI( TAG, "RTC synchronized from NTP (UTC)" );
		auto verify = M5.Rtc.getDateTime();
		ESP_LOGI(
    		TAG,
    		"RTC readback: %04d-%02d-%02d %02d:%02d:%02d UTC",
    		verify.date.year,
    		verify.date.month,
    		verify.date.date,
    		verify.time.hours,
    		verify.time.minutes,
    		verify.time.seconds );
    } else {
        ESP_LOGE( TAG, "RTC is not available" );
    }
}

static void start_ntp( void )
{
    if ( g_ntp_started ) {
        return;
    }

    ESP_LOGI( TAG, "Starting NTP" );

    esp_sntp_setoperatingmode( SNTP_OPMODE_POLL );
    esp_sntp_setservername( 0, "pool.ntp.org" );
    esp_sntp_set_time_sync_notification_cb( time_sync_notification_cb );

    // 1時間ごとに再同期
    esp_sntp_set_sync_interval( 60 * 60 * 1000UL );
    esp_sntp_init();
    g_ntp_started = true;
}

static bool restore_system_time_from_rtc( void )
{
    if ( !M5.Rtc.isEnabled() ) {
        ESP_LOGW( TAG, "RTC is not available" );
        return false;
    }

    auto rtc_dt = M5.Rtc.getDateTime();
    ESP_LOGI(
        TAG,
        "RTC at boot: %04d-%02d-%02d "
        "%02d:%02d:%02d UTC",
        rtc_dt.date.year,
        rtc_dt.date.month,
        rtc_dt.date.date,
        rtc_dt.time.hours,
        rtc_dt.time.minutes,
        rtc_dt.time.seconds );

    // RTC値の最低限の妥当性チェック
    if ( rtc_dt.date.year < 2024 ||
        rtc_dt.date.year > 2099 ||
        rtc_dt.date.month < 1 ||
        rtc_dt.date.month > 12 ||
        rtc_dt.date.date < 1 ||
        rtc_dt.date.date > 31 ||
        rtc_dt.time.hours < 0 ||
        rtc_dt.time.hours > 23 ||
        rtc_dt.time.minutes < 0 ||
        rtc_dt.time.minutes > 59 ||
        rtc_dt.time.seconds < 0 ||
        rtc_dt.time.seconds > 59 ) {

        ESP_LOGW( TAG, "RTC time is invalid" );
        return false;
    }

    struct tm t = {};

    t.tm_year = rtc_dt.date.year - 1900;
    t.tm_mon  = rtc_dt.date.month - 1;
    t.tm_mday = rtc_dt.date.date;

    t.tm_hour = rtc_dt.time.hours;
    t.tm_min  = rtc_dt.time.minutes;
    t.tm_sec  = rtc_dt.time.seconds;

    // RTCはUTCで保存しているのでUTCとして変換
	// mktime() にUTCとして解釈させる。
	setenv( "TZ", "UTC0", 1 );
	tzset();
	time_t epoch = mktime( &t );

    if ( epoch < 0 ) {
        ESP_LOGW( TAG, "Failed to convert RTC time" );
        return false;
    }

    struct timeval tv = {};
    tv.tv_sec  = epoch;
    tv.tv_usec = 0;

    if ( settimeofday( &tv, nullptr ) != 0 ) {
        ESP_LOGE( TAG, "settimeofday failed" );
        return false;
    }

    ESP_LOGI( TAG, "System clock restored from RTC" );
    return true;
}

static esp_err_t save_wifi_settings( const char *ssid, const char *password )
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open( "wifi", NVS_READWRITE, &handle );
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "nvs_open failed: %s", esp_err_to_name( err ) );
        return err;
    }

    err = nvs_set_str( handle, "ssid", ssid );
    if ( err == ESP_OK ) {
        err = nvs_set_str( handle, "password", password );
    }
    if ( err == ESP_OK ) {
        err = nvs_commit( handle );
    }
    nvs_close( handle );

    if ( err == ESP_OK ) {
        ESP_LOGI( TAG, "Wi-Fi settings saved to NVS" );
    } else {
        ESP_LOGE( TAG, "Failed to save Wi-Fi settings: %s", esp_err_to_name( err ) );
    }
    return err;
}

static void wifi_event_handler( void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data )
{
    if ( event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START ) {
        ESP_LOGI( TAG, "Wi-Fi STA started" );
    } else if ( event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED ) {
        ESP_LOGI( TAG, "Wi-Fi STA connected" );
    } else if ( event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED ) {
	    auto *event = static_cast<wifi_event_sta_disconnected_t *>( event_data );
	    ESP_LOGW( TAG, "Wi-Fi STA disconnected, reason=%d", event->reason );
		if ( !g_wifi_setup_mode ) {
		    ESP_LOGI( TAG, "Reconnecting Wi-Fi STA..." );
		    esp_err_t err = esp_wifi_connect();
		    if ( err != ESP_OK ) {
        		ESP_LOGW( TAG, "esp_wifi_connect failed: %s", esp_err_to_name( err ) );
		    }
		} else {
		    ESP_LOGI( TAG, "Wi-Fi setup mode: STA reconnect disabled" );
		}
    } else if ( event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP ) {
	    auto *event = static_cast<ip_event_got_ip_t *>( event_data );
	    ESP_LOGI( TAG, "Wi-Fi STA got IP: " IPSTR, IP2STR( &event->ip_info.ip ) );
	    start_ntp();	// NTPサーバから日付時刻取得
    }
}

static esp_err_t wifi_connect_sta( const char *ssid, const char *password )
{
    wifi_config_t sta_config = {};

    strncpy( reinterpret_cast<char *>( sta_config.sta.ssid ), ssid, sizeof( sta_config.sta.ssid ) - 1 );
    strncpy( reinterpret_cast<char *>( sta_config.sta.password ), password, sizeof( sta_config.sta.password ) - 1 );

    ESP_LOGI( TAG, "Connecting STA to SSID=%s", ssid );

    esp_err_t err = esp_wifi_set_config( WIFI_IF_STA, &sta_config );
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "esp_wifi_set_config(STA) failed: %s", esp_err_to_name( err ) );
        return err;
    }

    err = esp_wifi_connect();
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "esp_wifi_connect failed: %s", esp_err_to_name( err ) );
    }
    return err;
}

static bool load_wifi_settings( char *ssid, size_t ssid_size, char *password, size_t password_size )
{
    nvs_handle_t handle;

    esp_err_t err = nvs_open( "wifi", NVS_READONLY, &handle );
    if ( err != ESP_OK ) {
        ESP_LOGI( TAG, "No saved Wi-Fi settings" );
        return false;
    }

    size_t ssid_len = ssid_size;
    err = nvs_get_str( handle, "ssid", ssid, &ssid_len );
    if ( err != ESP_OK ) {
        nvs_close( handle );
        ESP_LOGI( TAG, "No saved Wi-Fi SSID" );
        return false;
    }

    size_t password_len = password_size;
    err = nvs_get_str( handle, "password", password, &password_len );
    if ( err == ESP_ERR_NVS_NOT_FOUND ) {
        // Open network
        password[0] = '\0';
        err = ESP_OK;
    }
    nvs_close( handle );

    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "Failed to load Wi-Fi settings: %s", esp_err_to_name( err ) );
        return false;
    }

    ESP_LOGI( TAG, "Saved Wi-Fi settings found: SSID=%s", ssid );
    ESP_LOGI( TAG, "Saved password length: %u", static_cast<unsigned>( strlen( password ) ) );

    return true;
}

static esp_err_t wifi_setup_root_handler( httpd_req_t *req )
{
    ESP_LOGI(TAG, "Wi-Fi scan requested");
	if ( !g_wifi_setup_mode ) {
	    ESP_LOGI( TAG, "Entering Wi-Fi setup mode" );
	    g_wifi_setup_mode = true;
	    // 旧APへの接続動作を停止
    	esp_err_t err = esp_wifi_disconnect();
	    if ( err != ESP_OK && err != ESP_ERR_WIFI_NOT_CONNECT ) {
	        ESP_LOGW( TAG, "esp_wifi_disconnect: %s", esp_err_to_name( err ) );
	    }
	    // Hosted/C6側の状態遷移が完了するのを少し待つ
    	vTaskDelay( pdMS_TO_TICKS( 500 ) );
	}

    wifi_scan_config_t scan_config = {};
    esp_err_t err = esp_wifi_scan_start( &scan_config, true );      // blocking scan
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "Wi-Fi scan failed: %s", esp_err_to_name( err ) );
        httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Wi-Fi scan failed" );
        return err;
    }

    uint16_t ap_count = 0;

    ESP_ERROR_CHECK( esp_wifi_scan_get_ap_num( &ap_count ) );

    ESP_LOGI( TAG, "Wi-Fi scan found %u APs", ap_count );

    // 表示数を制限
    if ( ap_count > 30 ) {
        ap_count = 30;
    }

    wifi_ap_record_t *records = nullptr;
    if ( ap_count > 0 ) {
        records = static_cast<wifi_ap_record_t *>( calloc( ap_count, sizeof( wifi_ap_record_t ) ) );
        if ( records == nullptr ) {
            httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory" );
            return ESP_ERR_NO_MEM;
        }

        uint16_t number = ap_count;
        err = esp_wifi_scan_get_ap_records( &number, records );
        if ( err != ESP_OK ) {
            free( records );
            httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot read scan results" );
            return err;
        }
        ap_count = number;
    }

    httpd_resp_set_type( req, "text/html; charset=utf-8" );

    // chunked response
	httpd_resp_sendstr_chunk(
    	req,
    	"<!DOCTYPE html>"
    	"<html>"
    	"<head>"
    	"<meta charset=\"UTF-8\">"
    	"<meta name=\"viewport\" "
    	"content=\"width=device-width,initial-scale=1\">"
    	"<title>Tab5-HRO Wi-Fi Setup</title>"
    	"</head>"
    	"<body>"
    	"<h1>Tab5-HRO Wi-Fi Setup</h1>"
    	"<p>Select a Wi-Fi network:</p>"
    	"<form method=\"POST\" action=\"/connect\">"
    	"<select name=\"ssid\">" );
    

    for ( uint16_t i = 0; i < ap_count; ++i ) {
        // 空SSID（hidden network）は今回は表示しない
        if ( records[i].ssid[0] == '\0' ) {
            continue;
        }

        char option[192];
        snprintf( option, sizeof( option ), 
        	"<option value=\"%s\">"
            "%s (%d dBm)"
            "</option>",
            reinterpret_cast<char *>( records[i].ssid ),
            reinterpret_cast<char *>( records[i].ssid ),
            records[i].rssi );

        httpd_resp_sendstr_chunk( req, option );
    }

	httpd_resp_sendstr_chunk(
    	req,
    	"</select>"
	    "<p>"
    	"Password:<br>"
    	"<input type=\"password\" "
    	"name=\"password\" "
    	"maxlength=\"64\">"
    	"</p>"
	    "<p>"
    	"<button type=\"submit\">"
    	"Connect"
    	"</button>"
    	"</p>"
	    "</form>"
	    "<hr>"
	    "<p>"
    	"<button onclick=\"location.reload()\">"
    	"Scan Again"
    	"</button>"
    	"</p>"
	    "</body>"
    	"</html>" );

    // chunked response終了
    httpd_resp_sendstr_chunk( req, nullptr );
    free( records );

    return ESP_OK;
}

static esp_err_t wifi_connect_handler( httpd_req_t *req )
{
    ESP_LOGI( TAG, "Wi-Fi connect form received" );

    const int total_len = req->content_len;
    if ( total_len <= 0 || total_len >= 512 ) {
        httpd_resp_send_err( req, HTTPD_400_BAD_REQUEST, "Invalid request" );
        return ESP_FAIL;
    }

    char content[512];
    int received = 0;

    while ( received < total_len ) {
        int ret = httpd_req_recv( req, content + received, total_len - received );
        if ( ret <= 0 ) {
            if ( ret == HTTPD_SOCK_ERR_TIMEOUT ) {
                continue;
            }
            return ESP_FAIL;
        }
        received += ret;
    }
    content[received] = '\0';

	char ssid_encoded[128] = {};
	char password_encoded[256] = {};

    char ssid[33] = {};
    char password[65] = {};

	esp_err_t ssid_result = httpd_query_key_value( content, "ssid", ssid_encoded, sizeof( ssid_encoded ) );
	esp_err_t password_result = httpd_query_key_value( content, "password", password_encoded, sizeof( password_encoded ) );

	if ( ssid_result != ESP_OK ) {
	    httpd_resp_send_err( req, HTTPD_400_BAD_REQUEST, "SSID missing" );
	    return ESP_FAIL;
	}

	if ( password_result != ESP_OK ) {
	    password_encoded[0] = '\0';
	}

	url_decode( ssid, sizeof( ssid ), ssid_encoded );
	url_decode( password, sizeof( password ), password_encoded );

    ESP_LOGI( TAG, "Selected SSID: %s", ssid );

    // Passwordそのものはログに出さない
    ESP_LOGI( TAG, "Password length: %u", static_cast<unsigned>( strlen( password ) ) );

	esp_err_t save_err = save_wifi_settings( ssid, password );
	if ( save_err != ESP_OK ) {
	    httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save Wi-Fi settings" );
	    return save_err;
	}

    static const char response[] =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset=\"UTF-8\">"
        "<meta name=\"viewport\" "
        "content=\"width=device-width,initial-scale=1\">"
        "<title>Tab5-HRO</title>"
        "</head>"
        "<body>"
        "<h1>Wi-Fi Settings Received</h1>"
        "<p>The Wi-Fi settings were received "
        "by Tab5-HRO.</p>"
        "<p>No connection has been attempted yet.</p>"
        "<p><a href=\"/\">Back</a></p>"
        "</body>"
        "</html>";

    httpd_resp_set_type( req, "text/html; charset=utf-8" );
    return httpd_resp_send( req, response, HTTPD_RESP_USE_STRLEN );
}

static esp_err_t station_get_handler( httpd_req_t *req )
{
    ESP_LOGI( TAG, "Station settings requested" );

    const double frequency_mhz = static_cast<double>( g_hro_config.frequency_hz ) / 1000000.0;
	DmsValue lon = longitude_to_dms( g_hro_config.longitude );
	DmsValue lat = latitude_to_dms( g_hro_config.latitude );
	
    // HTML全体をPSRAM側に確保
    constexpr size_t HTML_SIZE = 16384;

    char *html = static_cast<char *>(heap_caps_malloc( HTML_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );

    if ( !html ) {
        ESP_LOGE( TAG, "Failed to allocate station HTML" );
        httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory" );
        return ESP_FAIL;
    }

    snprintf(
        html,
        HTML_SIZE,

        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset=\"UTF-8\">"
        "<meta name=\"viewport\" "
        "content=\"width=device-width,initial-scale=1\">"

        "<title>Tab5-HRO Station Settings</title>"

        "<style>"

        "body{"
            "font-family:Arial,sans-serif;"
            "max-width:700px;"
            "margin:30px auto;"
            "padding:0 20px;"
            "background:#f5f5f5;"
        "}"

		".dms{"
    		"display:flex;"
    		"align-items:center;"
    		"gap:6px;"
    		"margin-top:5px;"
		"}"

		".dms input{"
    		"width:100px;"
    		"margin-top:0;"
		"}"

		".dms select{"
    		"padding:9px;"
    		"font-size:16px;"
		"}"

		".dms span{"
    		"font-size:16px;"
		"}"

		"@media(max-width:600px){"
    		".dms input{width:80px;}"
		"}"

        ".card{"
            "background:white;"
            "padding:24px;"
            "border-radius:10px;"
            "box-shadow:0 2px 8px rgba(0,0,0,.15);"
        "}"

        "h1{font-size:24px;}"
        "h2{font-size:18px;margin-top:28px;}"

        "label{"
            "display:block;"
            "margin-top:14px;"
            "font-weight:bold;"
        "}"

        "input{"
            "width:100%%;"
            "box-sizing:border-box;"
            "padding:9px;"
            "margin-top:5px;"
            "font-size:16px;"
        "}"

        ".info{"
            "margin-top:16px;"
            "padding:12px;"
            "background:#eef5ff;"
            "border-radius:6px;"
        "}"

        ".error{"
            "margin-top:10px;"
            "color:#c00000;"
            "font-weight:bold;"
            "min-height:20px;"
        "}"

        ".note{"
            "font-size:13px;"
            "color:#666;"
            "margin-top:10px;"
        "}"

        "button{"
            "margin-top:20px;"
            "padding:11px 28px;"
            "font-size:16px;"
            "cursor:pointer;"
        "}"

        "button:disabled{"
            "opacity:0.4;"
            "cursor:not-allowed;"
        "}"

        "</style>"
        "</head>"

        "<body>"
        "<div class=\"card\">"

        "<h1>Tab5-HRO Station Settings</h1>"

        "<form method=\"POST\" action=\"/station/save\">"

        "<h2>Station</h2>"

        "<label>Observer</label>"
        "<input "
            "name=\"observer\" "
            "type=\"text\" "
            "maxlength=\"63\" "
            "value=\"%s\">"

        "<label>Receiving Location</label>"
        "<input "
            "name=\"location\" "
            "type=\"text\" "
            "maxlength=\"127\" "
            "value=\"%s\">"

		"<label>Longitude</label>"
		"<div class=\"dms\">"

		"<select id=\"lon_dir\" name=\"lon_dir\" "
        	"onchange=\"validateSettings()\">"
			"<option value=\"E\" %s>E</option>"
			"<option value=\"W\" %s>W</option>"
		"</select>"

		"<input id=\"lon_deg\" name=\"lon_deg\" "
       		"type=\"number\" min=\"0\" max=\"180\" step=\"1\" "
       		"value=\"%d\" oninput=\"validateSettings()\">"
		"<span>°</span>"

		"<input id=\"lon_min\" name=\"lon_min\" "
       		"type=\"number\" min=\"0\" max=\"59\" step=\"1\" "
       		"value=\"%d\" oninput=\"validateSettings()\">"
		"<span>'</span>"

		"<input id=\"lon_sec\" name=\"lon_sec\" "
       		"type=\"number\" min=\"0\" max=\"59.999999\" step=\"0.001\" "
       		"value=\"%.3f\" oninput=\"validateSettings()\">"
		"<span>\"</span>"

		"</div>"

		"<label>Latitude</label>"
		"<div class=\"dms\">"

		"<select id=\"lat_dir\" name=\"lat_dir\" "
        	"onchange=\"validateSettings()\">"
			"<option value=\"N\" %s>N</option>"
			"<option value=\"S\" %s>S</option>"
		"</select>"

		"<input id=\"lat_deg\" name=\"lat_deg\" "
       		"type=\"number\" min=\"0\" max=\"90\" step=\"1\" "
       		"value=\"%d\" oninput=\"validateSettings()\">"
		"<span>°</span>"

		"<input id=\"lat_min\" name=\"lat_min\" "
       		"type=\"number\" min=\"0\" max=\"59\" step=\"1\" "
       		"value=\"%d\" oninput=\"validateSettings()\">"
		"<span>'</span>"

		"<input id=\"lat_sec\" name=\"lat_sec\" "
       		"type=\"number\" min=\"0\" max=\"59.999999\" step=\"0.001\" "
       		"value=\"%.3f\" oninput=\"validateSettings()\">"
		"<span>\"</span>"

		"</div>"

        "<h2>Receiver</h2>"

        "<label>Receiver</label>"
        "<input "
            "name=\"receiver\" "
            "type=\"text\" "
            "maxlength=\"63\" "
            "value=\"%s\">"

        "<label>Receiving Frequency (MHz)</label>"
        "<input "
            "id=\"frequency\" "
            "name=\"frequency\" "
            "type=\"number\" "
            "step=\"0.000001\" "
            "value=\"%.6f\" "
            "oninput=\"validateSettings()\">"

        "<label>FFT Center Frequency (Hz)</label>"
        "<input "
            "id=\"fft_center\" "
            "name=\"fft_center\" "
            "type=\"number\" "
            "step=\"1\" "
            "value=\"%ld\" "
            "oninput=\"validateSettings()\">"

        "<label>FFT Display Range +/- (Hz)</label>"
        "<input "
            "id=\"fft_range\" "
            "name=\"fft_range\" "
            "type=\"number\" "
            "step=\"1\" "
            "value=\"%ld\" "
            "oninput=\"validateSettings()\">"

		"<label>Level Average Range &plusmn;Hz</label>"
		"<input type=\"number\" "
			"name=\"level_average_range_hz\" "
			"min=\"0\" "
			"step=\"1\" "
			"value=\"%ld\">"

        "<label>Receiving Antenna</label>"
        "<input "
            "name=\"antenna\" "
            "type=\"text\" "
            "maxlength=\"127\" "
            "value=\"%s\">"

        "<h2>Screenshot</h2>"

        "<label>File Prefix</label>"
        "<input "
            "id=\"screenshot_prefix\" "
            "name=\"screenshot_prefix\" "
            "type=\"text\" "
            "maxlength=\"8\" "
            "pattern=\"[A-Za-z0-9]{1,8}\" "
            "value=\"%s\" "
            "oninput=\"validateSettings()\">"

        "<p class=\"note\">"
        "1 - 8 alphanumeric characters (A-Z, a-z, 0-9)"
        "</p>"

        "<div id=\"fft_info\" class=\"info\"></div>"
        "<div id=\"error_msg\" class=\"error\"></div>"

        "<button "
            "id=\"save_button\" "
            "type=\"submit\">"
            "Save"
        "</button>"

        "<p class=\"note\">"
        "FFT processing bandwidth: 0 - 1800 Hz"
        "</p>"

        "</form>"
        "</div>"

        "<script>"

        "function validateSettings(){"

			"const lonDeg=Number(document.getElementById('lon_deg').value);"
			"const lonMin=Number(document.getElementById('lon_min').value);"
			"const lonSec=Number(document.getElementById('lon_sec').value);"

			"const latDeg=Number(document.getElementById('lat_deg').value);"
			"const latMin=Number(document.getElementById('lat_min').value);"
			"const latSec=Number(document.getElementById('lat_sec').value);"

            "const frequency="
                "Number(document.getElementById('frequency').value);"

            "const center="
                "Number(document.getElementById('fft_center').value);"

            "const range="
                "Number(document.getElementById('fft_range').value);"

	        "const prefix="
	            "document.getElementById('screenshot_prefix').value;"

            "const info="
                "document.getElementById('fft_info');"

            "const error="
                "document.getElementById('error_msg');"

            "const save="
                "document.getElementById('save_button');"

            "let message='';"

            "const minHz=center-range;"
            "const maxHz=center+range;"
            "const bins=maxHz-minHz+1;"

			"if(!Number.isInteger(lonDeg)||lonDeg<0||lonDeg>180){"
			    "message='Longitude degrees must be between 0 and 180';"

			"}else if(!Number.isInteger(lonMin)||lonMin<0||lonMin>59){"
    			"message='Longitude minutes must be between 0 and 59';"

			"}else if(!Number.isFinite(lonSec)||lonSec<0||lonSec>=60){"
    			"message='Longitude seconds must be between 0 and 59.999';"

			"}else if(lonDeg===180&&(lonMin!==0||lonSec!==0)){"
    			"message='At 180 degrees, longitude minutes and seconds must be 0';"

			"}else if(!Number.isInteger(latDeg)||latDeg<0||latDeg>90){"
    			"message='Latitude degrees must be between 0 and 90';"

			"}else if(!Number.isInteger(latMin)||latMin<0||latMin>59){"
    			"message='Latitude minutes must be between 0 and 59';"

			"}else if(!Number.isFinite(latSec)||latSec<0||latSec>=60){"
    			"message='Latitude seconds must be between 0 and 59.999';"

			"}else if(latDeg===90&&(latMin!==0||latSec!==0)){"
    			"message='At 90 degrees, latitude minutes and seconds must be 0';"
    
            "}else if(!Number.isFinite(frequency)||"
                "frequency < 1 || frequency > 2000){"

                "message='Receiving Frequency must be between 1 and 2000 MHz';"

            "}else if(!Number.isInteger(center)||center < 0){"

                "message='FFT Center must be 0 Hz or greater';"

            "}else if(!Number.isInteger(range)||range < 1){"

                "message='FFT Display Range must be 1 Hz or greater';"

            "}else if(minHz < 0){"

                "message='FFT display lower frequency must be 0 Hz or greater';"

            "}else if(maxHz > 1800){"

                "message='FFT display upper frequency must not exceed 1800 Hz';"

            "}else if(maxHz > 1800){"

                "message='FFT display upper frequency must not exceed 1800 Hz';"

            "}else if(!/^[A-Za-z0-9]{1,8}$/.test(prefix)){"

                "message='Screenshot File Prefix must be 1 to 8 alphanumeric characters';"

            "}"

            "if(Number.isFinite(minHz)&&"
                "Number.isFinite(maxHz)&&"
                "Number.isFinite(bins)){"

                "info.innerHTML="
                    "'FFT Display: '+minHz+' - '+maxHz+' Hz<br>'"
                    "+'Bins: '+bins;"

            "}else{"

                "info.innerHTML='FFT Display: ---';"

            "}"

            "error.textContent=message;"
            "save.disabled=(message!=='');"

        "}"

        "validateSettings();"

        "</script>"

        "</body>"
        "</html>",

        g_hro_config.observer,
        g_hro_config.location,

		( lon.direction == 'E' ) ? "selected" : "",
		( lon.direction == 'W' ) ? "selected" : "",
		lon.degrees,
		lon.minutes,
		lon.seconds,

		( lat.direction == 'N' ) ? "selected" : "",
		( lat.direction == 'S' ) ? "selected" : "",
		lat.degrees,
		lat.minutes,
		lat.seconds,

        g_hro_config.receiver,
        frequency_mhz,

        static_cast<long>( g_hro_config.fft_center_hz ),
        static_cast<long>( g_hro_config.fft_range_hz ),
        static_cast<long>( g_hro_config.level_average_range_hz ),

        g_hro_config.antenna,
        g_hro_config.screenshot_prefix
    );

    httpd_resp_set_type( req, "text/html; charset=utf-8" );

    esp_err_t result = httpd_resp_send( req, html, HTTPD_RESP_USE_STRLEN );

    free(html);

    return result;
}

static esp_err_t station_save_handler( httpd_req_t *req )
{
    ESP_LOGI( TAG, "Station settings save requested" );

    if ( req->content_len <= 0 || req->content_len > 4096 ) {
        httpd_resp_send_err( req, HTTPD_400_BAD_REQUEST, "Invalid form data" );
        return ESP_FAIL;
    }

    char *body = static_cast<char *>( malloc( req->content_len + 1 ) );

    if ( !body ) {
        httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory" );
        return ESP_FAIL;
    }

    size_t received = 0;
    while ( received < req->content_len ) {
        int ret = httpd_req_recv( req, body + received, req->content_len - received );
        if ( ret <= 0 ) {
            free( body );

            httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed receiving form data" );
            return ESP_FAIL;
        }
        received += ret;
    }

    body[received] = '\0';

    // 現在値をベースに一時設定を作る
    HroConfig new_config = g_hro_config;

    char value[256];

    // -------------------------
    // Station
    // -------------------------


    if ( get_form_value( body, "observer", value, sizeof( value ) ) ) {
		snprintf( new_config.observer, sizeof( new_config.observer ), "%.*s", static_cast<int>( sizeof( new_config.observer ) - 1 ), value );
    }
    if ( get_form_value( body, "location", value, sizeof( value ) ) ) {
		snprintf( new_config.location, sizeof( new_config.location ), "%.*s", static_cast<int>( sizeof( new_config.location ) - 1 ), value );
    }
    
	char lon_dir[8] = "E";
	char lat_dir[8] = "N";

	int lon_deg = 0;
	int lon_min = 0;
	double lon_sec = 0.0;

	int lat_deg = 0;
	int lat_min = 0;
	double lat_sec = 0.0;

	if ( get_form_value( body, "lon_dir", value, sizeof( value ) ) ) {
	    snprintf( lon_dir, sizeof( lon_dir ), "%.1s", value );
	}
	if ( get_form_value( body, "lon_deg", value, sizeof( value ) ) ) {
	    lon_deg = static_cast<int>( strtol( value, nullptr, 10 ) );
	}
	if ( get_form_value( body, "lon_min", value, sizeof( value ) ) ) {
	    lon_min = static_cast<int>( strtol( value, nullptr, 10 ) );
	}
	if ( get_form_value( body, "lon_sec", value, sizeof( value ) ) ) {
	    lon_sec = strtod(value, nullptr);
	}

	if ( get_form_value( body, "lat_dir", value, sizeof( value ) ) ) {
	    snprintf( lat_dir, sizeof( lat_dir ), "%.1s", value );
	}
	if ( get_form_value( body, "lat_deg", value, sizeof( value ) ) ) {
	    lat_deg = static_cast<int>( strtol( value, nullptr, 10 ) );
	}
	if ( get_form_value( body, "lat_min", value, sizeof( value ) ) ) {
	    lat_min = static_cast<int>( strtol( value, nullptr, 10 ) );
	}
	if ( get_form_value( body, "lat_sec", value, sizeof( value ) ) ) {
	    lat_sec = strtod( value, nullptr );
	}

	if ( lon_deg < 0 || lon_deg > 180 || lon_min < 0 || lon_min > 59 || lon_sec < 0.0 || lon_sec >= 60.0 || 
		( lon_deg == 180 && ( lon_min != 0 || lon_sec != 0.0 ) ) ) {
		free(body);

    	httpd_resp_send_err( req, HTTPD_400_BAD_REQUEST, "Invalid longitude" );
	    return ESP_FAIL;
	}

	if ( lat_deg < 0 || lat_deg > 90 || lat_min < 0 || lat_min > 59 || lat_sec < 0.0 || lat_sec >= 60.0 ||
	    (lat_deg == 90 && ( lat_min != 0 || lat_sec != 0.0 ) ) ) {
	    free(body);

    	httpd_resp_send_err( req, HTTPD_400_BAD_REQUEST, "Invalid latitude" );
	    return ESP_FAIL;
	}

	if ( ( lon_dir[0] != 'E' && lon_dir[0] != 'W' ) || ( lat_dir[0] != 'N' && lat_dir[0] != 'S' ) ) {
	    free(body);

    	httpd_resp_send_err( req, HTTPD_400_BAD_REQUEST, "Invalid coordinate direction" );
	    return ESP_FAIL;
	}

	double longitude = static_cast<double>( lon_deg ) + static_cast<double>( lon_min ) / 60.0 + lon_sec / 3600.0;
	if ( lon_dir[0] == 'W' ) {
	    longitude = -longitude;
	}

	double latitude = static_cast<double>( lat_deg ) + static_cast<double>( lat_min ) / 60.0 + lat_sec / 3600.0;
	if ( lat_dir[0] == 'S' ) {
	    latitude = -latitude;
	}

	new_config.longitude = longitude;
	new_config.latitude  = latitude;

    // -------------------------
    // Receiver
    // -------------------------
    if ( get_form_value( body, "receiver", value, sizeof( value ) ) ) {
		snprintf( new_config.receiver, sizeof( new_config.receiver ), "%.*s", static_cast<int>( sizeof( new_config.receiver ) - 1), value );
    }
    if ( get_form_value( body, "antenna", value, sizeof( value ) ) ) {
		snprintf( new_config.antenna, sizeof( new_config.antenna ), "%.*s", static_cast<int>( sizeof( new_config.antenna ) - 1 ), value );
    }

    // Web側はMHzなのでHzへ変換
    if ( get_form_value( body, "frequency", value, sizeof( value ) ) ) {
        double mhz = strtod( value, nullptr );
        new_config.frequency_hz = static_cast<uint32_t>( mhz * 1000000.0 + 0.5 );
    }
    if ( get_form_value( body, "fft_center", value, sizeof( value ) ) ) {
        new_config.fft_center_hz = static_cast<int32_t>( strtol( value, nullptr, 10 ) );
    }
    if ( get_form_value( body, "fft_range", value, sizeof( value ) ) ) {
        new_config.fft_range_hz = static_cast<int32_t>( strtol( value, nullptr, 10 ) );
    }
	if ( get_form_value( body, "level_average_range_hz", value, sizeof( value ) ) ) {
	    new_config.level_average_range_hz = static_cast<int32_t>( strtol( value, nullptr, 10 ) );
	}

    // -------------------------
    // Screenshot
    // -------------------------
    if ( get_form_value( body, "screenshot_prefix", value, sizeof( value ) ) ) {
        snprintf( new_config.screenshot_prefix, sizeof( new_config.screenshot_prefix ), "%.*s", static_cast<int>( sizeof( new_config.screenshot_prefix ) - 1 ), value );
    }

    free( body );

    // -------------------------
    // C++側でも再検証
    // -------------------------
    char error_msg[128];

    if ( !validate_hro_config( new_config, error_msg, sizeof( error_msg ) ) ) {
        ESP_LOGW( TAG, "Station settings rejected: %s", error_msg );
        httpd_resp_send_err( req, HTTPD_400_BAD_REQUEST, error_msg );
        return ESP_FAIL;
    }

    // -------------------------
    // 一時ファイルへ保存
    // -------------------------
    if ( !write_hro_config_file( HRO_CONFIG_TEMP_FILE, new_config ) ) {
        httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed writing configuration" );
        return ESP_FAIL;
    }

    // -------------------------
    // config.ini と入れ替え
    // -------------------------
    if ( remove( HRO_CONFIG_FILE ) != 0 ) {
        // ファイルが存在しない場合以外は警告
        if ( errno != ENOENT ) {
            ESP_LOGE( TAG, "Failed removing old config: errno=%d", errno );
            remove( HRO_CONFIG_TEMP_FILE );
            httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed replacing configuration" );
            return ESP_FAIL;
        }
    }

    if ( rename( HRO_CONFIG_TEMP_FILE, HRO_CONFIG_FILE ) != 0 ) {
        ESP_LOGE( TAG, "Failed renaming config: errno=%d", errno );
        httpd_resp_send_err( req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed replacing configuration" );
        return ESP_FAIL;
    }

    // 全処理成功後に現在設定を更新
    g_hro_config = new_config;

    ESP_LOGI( TAG, "Station settings saved" );

    ESP_LOGI( TAG, "HRO: RF=%lu Hz FFT center=%ld Hz range=+/- %ld Hz", static_cast<unsigned long>( g_hro_config.frequency_hz ), static_cast<long>( g_hro_config.fft_center_hz ), static_cast<long>( g_hro_config.fft_range_hz ) );

    // 保存後 /station へ戻す
    httpd_resp_set_status( req, "303 See Other" );
    httpd_resp_set_hdr( req, "Location", "/station" );
    httpd_resp_send( req, nullptr, 0 );

    return ESP_OK;
}

static httpd_handle_t start_wifi_setup_webserver( void )
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = nullptr;

    ESP_LOGI( TAG, "Starting Wi-Fi setup web server" );

    esp_err_t err = httpd_start( &server, &config );
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "Failed to start HTTP server: %s", esp_err_to_name( err ) );
        return nullptr;
    }

    static const httpd_uri_t root_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = wifi_setup_root_handler,
        .user_ctx = nullptr
    };
    static const httpd_uri_t connect_uri = {
    	.uri      = "/connect",
    	.method   = HTTP_POST,
    	.handler  = wifi_connect_handler,
    	.user_ctx = nullptr
	};
	static const httpd_uri_t station_uri = {
    	.uri      = "/station",
    	.method   = HTTP_GET,
    	.handler  = station_get_handler,
    	.user_ctx = nullptr
	};
	static const httpd_uri_t station_save_uri = {
    	.uri      = "/station/save",
    	.method   = HTTP_POST,
    	.handler  = station_save_handler,
    	.user_ctx = nullptr
	};

    err = httpd_register_uri_handler( server, &root_uri );
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "Failed to register root URI: %s", esp_err_to_name( err ) );
        httpd_stop( server );
        return nullptr;
    }
	err = httpd_register_uri_handler( server, &connect_uri );
	if ( err != ESP_OK ) {
	    ESP_LOGE( TAG, "Failed to register connect URI: %s", esp_err_to_name( err ) );
	    httpd_stop( server );
	    return nullptr;
	}
	err = httpd_register_uri_handler( server, &station_uri );
	if ( err != ESP_OK ) {
	    ESP_LOGE( TAG, "Failed to register station URI: %s", esp_err_to_name( err ) );
	    httpd_stop( server );
	    return nullptr;
	}
	err = httpd_register_uri_handler( server, &station_save_uri );
	if ( err != ESP_OK ) {
    	ESP_LOGE( TAG, "Failed to register station save URI: %s", esp_err_to_name( err ) );
	    httpd_stop( server );
	    return nullptr;
	}

    ESP_LOGI( TAG, "Wi-Fi setup web server started" );
    return server;
}

static void wifi_start_ap( const char *saved_ssid, const char *saved_password )
{
    ESP_LOGI( TAG, "Starting Wi-Fi SoftAP" );

    // TCP/IP stack
    ESP_ERROR_CHECK( esp_netif_init() );

    // Default event loop
    esp_err_t err = esp_event_loop_create_default();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK( err );
    }

    // SoftAP network interface
    esp_netif_create_default_wifi_ap();
	esp_netif_create_default_wifi_sta();

    // Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
    wifi_config_t wifi_config = {};
    
	ESP_ERROR_CHECK( esp_event_handler_register( WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr ) );
	ESP_ERROR_CHECK( esp_event_handler_register( IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr ) );
        
	uint8_t mac[6];
	ESP_ERROR_CHECK( esp_efuse_mac_get_default( mac ) );

	char ssid[32];
	snprintf( ssid, sizeof( ssid ), "Tab5-HRO-%02X%02X%02X", mac[3], mac[4], mac[5] );
	strncpy( reinterpret_cast<char *>(wifi_config.ap.ssid), ssid, sizeof( wifi_config.ap.ssid ) - 1 );
	wifi_config.ap.ssid[sizeof( wifi_config.ap.ssid ) - 1] = '\0';
	wifi_config.ap.ssid_len = strlen( reinterpret_cast<char *>(wifi_config.ap.ssid) );
            
    // 初期テストなのでパスワードなし
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config.ap.max_connection = 4;
	ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_APSTA ) );
//    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_AP ) );
    ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_AP, &wifi_config ) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_LOGI( TAG, "Wi-Fi SoftAP started: SSID=%s", ssid );

	if (saved_ssid[0] != '\0') {
    	wifi_connect_sta( saved_ssid, saved_password );
	}
    start_wifi_setup_webserver();
}

// -------------------------
// history_index:
//   0 = oldest
//   g_hro_history_count - 1 = newest
//
// 戻り値:
//   その1秒分（501 bins）の先頭ポインタ
//   データが存在しない場合は nullptr
// -------------------------
static const float *get_hro_history( uint16_t history_index )
{
    if ( g_hro_history == nullptr ) {
        return nullptr;
    }

    if ( history_index >= g_hro_history_count ) {
        return nullptr;
    }

    uint16_t physical_index;

    if ( g_hro_history_count < HRO_HISTORY_SECONDS ) {
        // まだ600秒に達していない
        // 0番から順番に格納されている
        physical_index = history_index;
    } else {
        // リングが満杯
        // write_pos が現在の最古データ
        physical_index = ( g_hro_history_write_pos + history_index ) % HRO_HISTORY_SECONDS;
    }
    return &g_hro_history[static_cast<size_t>(physical_index) * HRO_BIN_COUNT];
}

// -------------------------
// Waterfall用の色変換
// -------------------------
static uint16_t waterfall_color( float db )
{
    constexpr float DB_MIN = -20.0f;
    constexpr float DB_MAX =  30.0f;

    float t = ( db - DB_MIN ) / ( DB_MAX - DB_MIN );

    if ( t < 0.0f ) t = 0.0f;
    if ( t > 1.0f ) t = 1.0f;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    // dark blue -> blue -> cyan -> yellow -> white
    if ( t < 0.25f ) {
        const float u = t / 0.25f;
        r = 0;
        g = 0;
        b = static_cast<uint8_t>( 30 + 180 * u );
    } else if ( t < 0.50f ) {
        const float u = ( t - 0.25f ) / 0.25f;
        r = 0;
        g = static_cast<uint8_t>( 220 * u );
        b = 220;
    } else if ( t < 0.75f ) {
        const float u = ( t - 0.50f ) / 0.25f;
        r = static_cast<uint8_t>( 255 * u );
        g = 220;
        b = static_cast<uint8_t>( 220 * ( 1.0f - u ) );
    } else {
        const float u = ( t - 0.75f ) / 0.25f;
        r = 255;
        g = static_cast<uint8_t>( 220 + 35 * u );
        b = static_cast<uint8_t>( 255 * u);
    }

    return M5.Display.color565( r, g, b );
}

static void draw_waterfall_column( const float *spectrum )
{
    if ( spectrum == nullptr ) {
        return;
    }

    // 600秒 = WF_W pixel
    const uint32_t old_pos = static_cast<uint32_t>( g_wf_scroll_phase ) * WF_W / WF_SECONDS;
    g_wf_scroll_phase++;
    const uint32_t new_pos = static_cast<uint32_t>( g_wf_scroll_phase ) * WF_W / WF_SECONDS;
    const int column_w = static_cast<int>( new_pos - old_pos );
    if ( g_wf_scroll_phase >= WF_SECONDS ) {
        g_wf_scroll_phase = 0;
    }

    // WF_W / 600 なので通常1～2 pixel
    if ( column_w <= 0 ) {
        return;
    }

    // Waterfall Spriteを左へスクロール
    g_waterfall.scroll( -column_w, 0 );

    // 右端に最新Spectrumを追加
    for ( int y = 0; y < WF_H; ++y ) {
        const int bin = ( WF_H - 1 - y ) * ( HRO_BIN_COUNT - 1 ) / ( WF_H - 1 );
        const float db = spectrum[bin];
        const uint16_t color = waterfall_color( db );
        g_waterfall.fillRect( WF_W - column_w, y, column_w, 1, color );
    }

    // LCDへ転送
    g_waterfall.pushSprite( WF_X, WF_Y );
}


// -------------------------
// Waterfall表示
// -------------------------
static void draw_hro_waterfall( void )
{
    if ( g_hro_history == nullptr ) {
        return;
    }

    const uint16_t count = g_hro_history_count;
    if ( count == 0 ) {
        return;
    }

    // まだ600秒溜まっていない部分を背景色で消す
    M5.Display.fillRect( WF_X, WF_Y, WF_W, WF_H, M5.Display.color565( 0, 10, 18 ) );

    // 左端 = 10分前
    // 右端 = NOW
    //
    // 600秒未満の場合は右詰め表示する
    const int first_x = WF_X + ( HRO_HISTORY_SECONDS - count ) * WF_W / HRO_HISTORY_SECONDS;
    for ( uint16_t h = 0; h < count; ++h ) {
        const float *spectrum = get_hro_history( h );
        if ( spectrum == nullptr ) {
            continue;
        }
        const int x0 = first_x + static_cast<int>( static_cast<uint32_t>( h ) * WF_W / HRO_HISTORY_SECONDS );
        const int x1 = first_x + static_cast<int>( static_cast<uint32_t>( h + 1 ) * WF_W / HRO_HISTORY_SECONDS );
        int width = x1 - x0;
        if ( width < 1 ) {
            width = 1;
        }

        // frequency:
        // top    = 1030 Hz
        // bottom = 530 Hz
        for ( int y = 0; y < WF_H; ++y ) {
            const int bin = ( WF_H - 1 - y ) * ( HRO_BIN_COUNT - 1 ) / ( WF_H - 1 );
            const float db = spectrum[bin];
            const uint16_t color = waterfall_color( db );
            M5.Display.fillRect( x0, WF_Y + y, width, 1, color );
        }
    }
}

static float hro_average_level_db( const float *spectrum )
{
    if ( spectrum == nullptr ) {
        return LEVEL_DB_MIN;
    }

    const int center_bin = hro_center_bin();
    const int range_hz = g_hro_config.level_average_range_hz;

    // 現在は 1 Hz/bin
    int first_bin = center_bin - range_hz;
    int last_bin  = center_bin + range_hz;

    if ( first_bin < 0 ) {
        first_bin = 0;
    }

    if ( last_bin >= HRO_BIN_COUNT ) {
        last_bin = HRO_BIN_COUNT - 1;
    }

    double power_sum = 0.0;
    int bin_count = 0;

    for ( int bin = first_bin; bin <= last_bin; ++bin ) {
        // dB -> linear power
        power_sum += pow( 10.0, spectrum[bin] / 10.0 );
        ++bin_count;
    }

    if ( bin_count == 0 ) {
        return LEVEL_DB_MIN;
    }

    const double average_power = power_sum / static_cast<double>( bin_count );

    // linear power -> dB
    return static_cast<float>( 10.0 * log10( average_power ) );
}

static float hro_max_level_db( const float *spectrum )
{
    if ( spectrum == nullptr ) {
        return LEVEL_DB_MIN;
    }

    const int center_bin = hro_center_bin();
    const int range_hz = g_hro_config.level_average_range_hz;

    // 現在は 1 Hz/bin
    int first_bin = center_bin - range_hz;
    int last_bin  = center_bin + range_hz;

    if ( first_bin < 0 ) {
        first_bin = 0;
    }

    if ( last_bin >= HRO_BIN_COUNT ) {
        last_bin = HRO_BIN_COUNT - 1;
    }

    float max_db = LEVEL_DB_MIN;

    for ( int bin = first_bin; bin <= last_bin; ++bin ) {
        if ( spectrum[bin] > max_db ) {
            max_db = spectrum[bin];
        }
    }

    return max_db;
}

static void draw_hro_level_point( const float *spectrum )
{
    if ( spectrum == nullptr ) {
        return;
    }

    // 今回の1秒が担当するX範囲
    const int x0 = LEVEL_X + static_cast<int>( static_cast<uint32_t>( g_level_phase ) * LEVEL_W / HRO_HISTORY_SECONDS );
    const int x1 = LEVEL_X + static_cast<int>( static_cast<uint32_t>( g_level_phase + 1 ) * LEVEL_W / HRO_HISTORY_SECONDS );
    const int column_w = x1 - x0;

    // 新しいデータを書き込む場所だけ消す
    if ( column_w > 0 ) {
        M5.Display.fillRect( x0, LEVEL_Y, column_w, LEVEL_H, BLACK );
    }

    const float db = hro_average_level_db( spectrum );
    const int x = x0 + column_w / 2;
    const int y = LEVEL_Y + level_db_to_y( db );

    // 直前の点と今回の点だけ結ぶ
    if ( g_level_have_prev ) {
        M5.Display.drawLine( g_level_prev_x, g_level_prev_y, x, y, CYAN );
    }

    g_level_prev_x = x;
    g_level_prev_y = y;
    g_level_have_prev = true;

    ++g_level_phase;

    // 600秒で一周
    if ( g_level_phase >= HRO_HISTORY_SECONDS ) {
        g_level_phase = 0;

        // 右端から左端へ線を結ばない
        g_level_have_prev = false;
    }
}

static void draw_hro_level_history( void )
{
    if ( g_hro_history == nullptr ) {
        return;
    }

    // Sprite上をクリア
    g_level_graph.fillSprite( BLACK );

    const uint16_t count = g_hro_history_count;

    if ( count != 0 ) {

        const int first_x = ( HRO_HISTORY_SECONDS - count ) * LEVEL_W / HRO_HISTORY_SECONDS;

        for ( uint16_t h = 0; h < count; ++h ) {
            const float *spectrum = get_hro_history( h );
            if ( spectrum == nullptr ) {
                continue;
            }

            // この1秒の表示範囲内の最大dB
            const float db = hro_max_level_db( spectrum );

            // 1秒分が占めるX範囲
            const int x0 = first_x + static_cast<int>( static_cast<uint32_t>( h ) * LEVEL_W / HRO_HISTORY_SECONDS );
            const int x1 = first_x + static_cast<int>( static_cast<uint32_t>( h + 1 ) * LEVEL_W / HRO_HISTORY_SECONDS );
            const int bar_w = x1 - x0;
            if ( bar_w <= 0 ) {
                continue;
            }

            // dB値をバーの高さへ変換
            const int y = level_db_to_y( db );

            // Waterfallと同じdB→色変換
            const uint16_t color = waterfall_color( db );

            // 下端から上方向へバー表示
            g_level_graph.fillRect( x0, y, bar_w, LEVEL_H - y, color );
        }
    }

    // LCDへ一括転送
    g_level_graph.pushSprite( LEVEL_X, LEVEL_Y );
}

#if 0
static void draw_hro_level_history( void )
{
    if ( g_hro_history == nullptr ) {
        return;
    }

    // LCDではなくSprite上をクリア
    g_level_graph.fillSprite( BLACK );

    const uint16_t count = g_hro_history_count;
    if ( count != 0 ) {
        const int first_x = ( HRO_HISTORY_SECONDS - count ) * LEVEL_W / HRO_HISTORY_SECONDS;
        bool have_prev = false;
        int prev_x = 0;
        int prev_y = 0;

        for ( uint16_t h = 0; h < count; ++h ) {
            const float *spectrum = get_hro_history( h );
            if ( spectrum == nullptr ) {
                continue;
            }

            // Sprite内座標なのでLEVEL_Xは加えない
            const int x0 = first_x + static_cast<int>( static_cast<uint32_t>( h ) * LEVEL_W / HRO_HISTORY_SECONDS );
            const int x1 = first_x + static_cast<int>( static_cast<uint32_t>( h + 1 ) * LEVEL_W / HRO_HISTORY_SECONDS );
            const int x = x0 + ( x1 - x0 ) / 2;
            const float db = hro_average_level_db( spectrum );

            // level_db_to_y() は0～LEVEL_H-1を返すので
            // Sprite内ではLEVEL_Yを加えない
            const int y = level_db_to_y( db );

            if ( have_prev ) {
                g_level_graph.drawLine( prev_x, prev_y, x, y, CYAN );
            }
            prev_x = x;
            prev_y = y;
            have_prev = true;
        }
    }

    // 完成したLevelグラフをLCDへ一括転送
    g_level_graph.pushSprite( LEVEL_X, LEVEL_Y );
}
#endif

// -------------------------
// スペクトルグラフ表示
// -------------------------
static void draw_hro_spectrum( void )
{
    if ( g_hro_spectrum_sequence.load( std::memory_order_acquire ) == 0 ) {
        return;
    }

    constexpr int X = SPEC_X;
    constexpr int Y = SPEC_Y;
    constexpr int W = SPEC_W;
    constexpr int H = SPEC_H;

    // 現段階では表示レンジを固定
    constexpr float DB_MIN = -20.0f;
    constexpr float DB_MAX = 30.0f;

    // -------------------------
    // グラフ内部をクリア
    // -------------------------
    M5.Display.fillRect( X, Y, W, H, M5.Display.color565( 0, 15, 24 ) );

    // -------------------------
    // Grid
    // -------------------------
    const uint16_t grid_color = M5.Display.color565( 0, 55, 75 );

    // vertical
    for ( int n = 0; n <= 10; ++n ) {
        int x = X + ( W - 1 ) * n / 10;
        M5.Display.drawFastVLine( x, Y, H, grid_color );
    }

    // horizontal
    for ( int n = 0; n <= 4; ++n ) {
        int y = Y + ( H - 1 ) * n / 4;
        M5.Display.drawFastHLine( X, y, W, grid_color );
    }

    // -------------------------
    // 780 Hz center marker
    // -------------------------
    const int center_x = X + ( 780 - HRO_BIN_MIN ) * ( W - 1 ) / ( HRO_BIN_COUNT - 1 );
    const uint16_t marker_color = M5.Display.color565( 0, 220, 220 );
    for ( int y = Y; y < Y + H; y += 6 ) {
        M5.Display.drawFastVLine( center_x, y, 3, marker_color );
    }

    // -------------------------
    // Spectrum
    // -------------------------
    const uint16_t spectrum_color = M5.Display.color565( 0, 255, 255 );
    int prev_x = X;
    int prev_y = Y + H - 1;

    for ( int n = 0; n < HRO_BIN_COUNT; ++n ) {
        const float db = g_hro_spectrum[n];
        float normalized = ( db - DB_MIN ) / ( DB_MAX - DB_MIN );
        if ( normalized < 0.0f ) normalized = 0.0f;
        if ( normalized > 1.0f ) normalized = 1.0f;

        const int x = X + n * ( W - 1 ) / ( HRO_BIN_COUNT - 1 );
        const int y = Y + H - 1 - static_cast<int>( normalized * ( H - 1 ) );
        if ( n > 0 ) {
            M5.Display.drawLine( prev_x, prev_y, x, y, spectrum_color );
        }
        prev_x = x;
        prev_y = y;
    }
}

// -------------------------
// 現在時刻表示
// -------------------------
static void draw_current_time( void )
{
    time_t now;
    struct tm timeinfo;

    // Tab5-HROは日本標準時で動作
    setenv( "TZ", "JST-9", 1 );
    tzset();

    time( &now );
    localtime_r( &now, &timeinfo );

    char buf[32];

    constexpr int X = 540;
    constexpr int Y = 678;
    constexpr int W = 210;
    constexpr int H = 25;

    const bool ntp_synced = g_ntp_synced.load( std::memory_order_relaxed );
    // NTP同期済み = 青
    // RTC動作中   = オレンジ
    uint16_t bg_color;

	if ( ntp_synced ) {
	    strftime( buf, sizeof( buf ), "%Y/%m/%d  %H:%M:%S [NTP]", &timeinfo );
	    bg_color = M5.Display.color565( 0, 60, 120 );
	} else {
	    strftime( buf, sizeof( buf ), "%Y/%m/%d  %H:%M:%S [RTC]", &timeinfo );
	    bg_color = M5.Display.color565( 160, 80, 0 );
	}

    // 前の表示を背景色で消去
    M5.Display.fillRect( X, Y, W, H, bg_color );

    M5.Display.setFont( &fonts::Font2 );
    M5.Display.setTextDatum( middle_center );
    M5.Display.setTextColor( 0xFFFFFF );
    M5.Display.drawString( buf, X + W / 2, Y + H / 2 );
}

static bool point_in_rect( int x, int y, int rx, int ry, int rw, int rh )
{
    return x >= rx &&
           x <  rx + rw &&
           y >= ry &&
           y <  ry + rh;
}

// --------------------------------------------------
// Shutdownボタン操作UI
// --------------------------------------------------
static void draw_shutdown_button( bool safe = false )
{
    M5.Display.setTextColor( WHITE );
    M5.Display.setTextDatum( middle_center );

    if ( safe ) {
	    M5.Display.fillRoundRect( SHUTDOWN_X, SHUTDOWN_Y, SHUTDOWN_W, SHUTDOWN_H, 6, RED );
    	M5.Display.drawRoundRect( SHUTDOWN_X, SHUTDOWN_Y, SHUTDOWN_W, SHUTDOWN_H, 6, WHITE );
        M5.Display.setFont( &fonts::Font2 );
        M5.Display.drawString( "SAFE TO POWER OFF", SHUTDOWN_X + SHUTDOWN_W / 2, SHUTDOWN_Y + SHUTDOWN_H / 2 );
    } else {
	    M5.Display.fillRoundRect( SHUTDOWN_X, SHUTDOWN_Y, SHUTDOWN_W, SHUTDOWN_H, 6, DARKGREY );
    	M5.Display.drawRoundRect( SHUTDOWN_X, SHUTDOWN_Y, SHUTDOWN_W, SHUTDOWN_H, 6, WHITE );
	    M5.Display.setFont( &fonts::Font2 );
    	M5.Display.drawString( "HOLD 2 SEC TO SHUTDOWN", SHUTDOWN_X + SHUTDOWN_W / 2, SHUTDOWN_Y + SHUTDOWN_H / 2 );
    }
}

// --------------------------------------------------
// Audio操作UI
// --------------------------------------------------
static void draw_audio_controls( void )
{
    constexpr int PANEL_X = 1015;
    constexpr int PANEL_Y = 520;
    constexpr int PANEL_W = 250;
    constexpr int PANEL_H = 129;

    constexpr int BTN_Y = 557;
    constexpr int BTN_W = 62;
    constexpr int BTN_H = 38;

    constexpr int MINUS_X = 1025;
    constexpr int PLUS_X  = 1193;

    constexpr int MUTE_X = 1080;
    constexpr int MUTE_Y = 605;
    constexpr int MUTE_W = 120;
    constexpr int MUTE_H = 34;

    const int volume = g_audio_volume.load( std::memory_order_relaxed );
    const bool mute = g_audio_mute.load( std::memory_order_relaxed );

    // タイトル
    M5.Display.setFont( &fonts::Font2 );
    M5.Display.setTextDatum( middle_center );
    M5.Display.setTextColor( WHITE );

    M5.Display.drawString( "AUDIO", PANEL_X + PANEL_W / 2, PANEL_Y + 15 );

    // VOL -
    M5.Display.drawRect( MINUS_X, BTN_Y, BTN_W, BTN_H, WHITE );
    M5.Display.drawString( "VOL-", MINUS_X + BTN_W / 2, BTN_Y + BTN_H / 2 );

    // VOL +
    M5.Display.drawRect( PLUS_X, BTN_Y, BTN_W, BTN_H, WHITE );
    M5.Display.drawString( "VOL+", PLUS_X + BTN_W / 2, BTN_Y + BTN_H / 2 );

	// 音量表示領域を消去
	M5.Display.fillRect( 1090, BTN_Y, 95, BTN_H, BLACK );

    // 現在音量
    char buf[16];
    snprintf( buf, sizeof( buf ), "%d%%", volume );

	M5.Display.setTextColor( CYAN );
	M5.Display.drawString( buf, PANEL_X + PANEL_W / 2, BTN_Y + BTN_H / 2 );

	// MUTEボタン領域を消去してから
	M5.Display.fillRect( MUTE_X, MUTE_Y, MUTE_W, MUTE_H, BLACK );
    M5.Display.drawRect( MUTE_X, MUTE_Y, MUTE_W, MUTE_H, mute ? RED : WHITE );
    M5.Display.setTextColor( mute ? RED : WHITE );
    M5.Display.drawString( mute ? "MUTED" : "MUTE", MUTE_X + MUTE_W / 2, MUTE_Y + MUTE_H / 2 );
}

// --------------------------------------------------
// Audio操作タッチパネル
// --------------------------------------------------
static void handle_audio_touch()
{
    auto detail = M5.Touch.getDetail();

    const int x = detail.x;
    const int y = detail.y;

    // ---------------------------------------------------------
    // SHUTDOWN : 2秒長押し
    // ---------------------------------------------------------

    if ( detail.wasPressed() &&
         point_in_rect(
             x, y,
             SHUTDOWN_X,
             SHUTDOWN_Y,
             SHUTDOWN_W,
             SHUTDOWN_H ) ) {

        g_shutdown_holding = true;
        g_shutdown_press_us = esp_timer_get_time();

        ESP_LOGI( TAG, "SHUTDOWN hold started" );
        return;
    }

    if ( g_shutdown_holding ) {
        // 指を離した、またはボタン領域から外れた
        if ( !detail.isPressed() ||
             !point_in_rect(
                 x, y,
                 SHUTDOWN_X,
                 SHUTDOWN_Y,
                 SHUTDOWN_W,
                 SHUTDOWN_H ) ) {

            g_shutdown_holding = false;

            ESP_LOGI( TAG, "SHUTDOWN hold cancelled" );
            return;
        }

        // 2秒長押し成立
        if ( esp_timer_get_time() - g_shutdown_press_us >= SHUTDOWN_HOLD_US ) {
            g_shutdown_holding = false;
		    // 2秒長押し成立を音で知らせる
		    play_touch_beep();

            ESP_LOGI( TAG, "SHUTDOWN requested" );
			g_hro_shutdown_requested.store( true, std::memory_order_release );
            return;
        }

        return;
    }
    
    // ---------------------------------------------------------
    // AUDIO : 従来通り、押した瞬間だけ処理
    // ---------------------------------------------------------
    if ( !detail.wasPressed() ) {
        return;
    }

    if ( point_in_rect(
            x, y,
            AUDIO_VOL_MINUS_X,
            AUDIO_VOL_MINUS_Y,
            AUDIO_VOL_MINUS_W,
            AUDIO_VOL_MINUS_H ) ) {

        int volume = g_audio_volume.load( std::memory_order_relaxed );
        volume -= 10;
        if ( volume < 0 ) {
            volume = 0;
        }

        g_audio_volume.store( volume, std::memory_order_relaxed );
		play_touch_beep();
		audio_config_changed();
        draw_audio_controls();
        return;
    }

    if ( point_in_rect(
            x, y,
            AUDIO_VOL_PLUS_X,
            AUDIO_VOL_PLUS_Y,
            AUDIO_VOL_PLUS_W,
            AUDIO_VOL_PLUS_H ) ) {

        int volume = g_audio_volume.load(std::memory_order_relaxed );

        volume += 10;
        if ( volume > 100 ) {
            volume = 100;
        }

        g_audio_volume.store( volume, std::memory_order_relaxed );
		play_touch_beep();
		audio_config_changed();
        draw_audio_controls();
        return;
    }

    if ( point_in_rect(
            x, y,
            AUDIO_MUTE_X,
            AUDIO_MUTE_Y,
            AUDIO_MUTE_W,
            AUDIO_MUTE_H ) ) {

        const bool mute = g_audio_mute.load(std::memory_order_relaxed );
        g_audio_mute.store( !mute, std::memory_order_relaxed );
		play_touch_beep();
		audio_config_changed();
        draw_audio_controls();
    }
}

// -------------------------
// スペクトルグラフ表示用のタスク
// -------------------------
static void hro_display_task( void *arg )
{
    uint32_t last_sequence = 0;
    static int64_t last_battery_update_us = 0;
    static bool safe_displayed = false;
    bool screenshot_full_block_ready = false;

    while ( true ) {
		M5.update();
		handle_audio_touch();	// タッチパネル操作
		if ( !safe_displayed && g_hro_storage_stopped.load( std::memory_order_acquire ) ) {
    		draw_shutdown_button( true );
		    safe_displayed = true;
		}
		if ( !g_hro_shutdown_requested.load( std::memory_order_acquire ) ) {
	        save_audio_config_if_needed();	// Audio設定変更から2秒後にSDへ保存
		}
        
		const uint32_t sequence = g_hro_spectrum_sequence.load( std::memory_order_acquire );

        if ( sequence != last_sequence ) {
            last_sequence = sequence;

            const time_t now = time( nullptr );
            const time_t current_block_start = get_screenshot_block_start( now );

            if ( g_screenshot_block_start == 0 ) {
                // NTP同期後、現在の10分ブロックに参加する。
                // このブロックは途中からなので保存対象にはしない。
                g_screenshot_block_start = current_block_start;
                screenshot_full_block_ready = false;
                ESP_LOGI( TAG, "Screenshot block started: %lld (partial block)", static_cast<long long>( g_screenshot_block_start ) );

            } else if ( current_block_start != g_screenshot_block_start ) {
                const time_t completed_block_start = g_screenshot_block_start;
                if ( screenshot_full_block_ready ) {
                    char filename[64];
                    if ( make_screenshot_filename( completed_block_start, filename, sizeof( filename ) ) ) {
                        ESP_LOGI( TAG, "Screenshot block completed: filename=%s", filename );
                    }
                    // 時刻ラベル正規化
                    draw_waterfall_time_axis( completed_block_start + 10 * 60 );

                    // PNGに記録する情報を一時的にLCDへ表示
                    draw_screenshot_info( completed_block_start, filename );

                    // 表示した情報も含めてPNG保存
                    const bool saved = save_screenshot_png( completed_block_start, filename );

                    // 一時表示を消す
                    clear_screenshot_info();

                    if ( !saved ) {
                        ESP_LOGE( TAG, "Screenshot block save failed: %s", filename );
                    }
                } else {
                    ESP_LOGI( TAG, "Screenshot partial block skipped" );
                }

                // 新しい10分ブロックへ切り替える
                g_screenshot_block_start = current_block_start;
                screenshot_full_block_ready = true;

                ESP_LOGI( TAG, "Screenshot block started: %lld", static_cast<long long>( g_screenshot_block_start ) );
            }

            // 境界処理が終わってから今回のSpectrumを描く
            draw_waterfall_column( g_hro_spectrum );
            draw_hro_level_history();

            const int64_t now_us = esp_timer_get_time();

            if ( now_us - last_battery_update_us >= 10 * 1000 * 1000LL ) {
                last_battery_update_us = now_us;
                log_battery_status();
                draw_battery_status();
            }

            draw_waterfall_time_axis();

        #if 0
            draw_hro_spectrum();
        #endif

            draw_current_time();
        }
        vTaskDelay(pdMS_TO_TICKS( 20 ));
    }
}

// -------------------------
// SDCardデータ保存用のタスク
// -------------------------
static void hro_save_writer_task( void *arg )
{
    HroSaveRecord *record = nullptr;
    FILE *fp = nullptr;
    char current_path[192] = { 0 };
    uint32_t batch_write_count = 0;
    g_hro_save_batch->count = 0;

    ESP_LOGI( TAG, "HRO batch writer task started" );

    while ( true ) {
		if ( xQueueReceive( g_hro_save_write_queue, &record, pdMS_TO_TICKS( 100 ) ) != pdTRUE ) {
		    // shutdown要求後、queueが完全に空になった
    		if ( g_hro_shutdown_requested.load( std::memory_order_acquire ) ) {
		        ESP_LOGI( TAG, "HRO writer queue drained, partial batch=%d", g_hro_save_batch->count );
		        break;
    		}
		    continue;
		}

        // -----------------------------------------------------
        // まずPSRAM batchへコピー
        // -----------------------------------------------------
        if ( g_hro_save_batch->count == 0 ) {
            g_hro_save_batch->first_timestamp = record->timestamp;
        }
        memcpy( g_hro_save_batch->spectrum[g_hro_save_batch->count], record->spectrum, HRO_BIN_COUNT * sizeof( float ) );
        g_hro_save_batch->count++;

		// --------------------------------------------------
		// 59:59なら、このrecordで現在のhourを終了させる
		// --------------------------------------------------
		const time_t record_jst_timestamp = record->timestamp + 9 * 60 * 60;
		struct tm record_jst_tm;
		gmtime_r( &record_jst_timestamp, &record_jst_tm );

		const bool hour_end = ( record_jst_tm.tm_min == 59 && record_jst_tm.tm_sec == 59 );

		// recordはここでfree queueへ返してよい
		if ( xQueueSend( g_hro_save_free_queue, &record, portMAX_DELAY ) != pdTRUE ) {
		    ESP_LOGE( TAG, "Failed to return HRO save buffer" );
		}
		record = nullptr;

		// 通常は10秒で書く。
		// ただし59:59なら10秒未満でもここで書く。
		if ( g_hro_save_batch->count < HRO_SAVE_BATCH_SECONDS && !hour_end ) {
	    	continue;
		}

        // -----------------------------------------------------
        // batch先頭時刻からファイルを決定
        // -----------------------------------------------------
        char path[192];

        if ( !make_hro_file_path( g_hro_save_batch->first_timestamp, path, sizeof( path ) ) ) {
            ESP_LOGE( TAG, "Failed to make HRO batch path" );
            g_hro_save_batch->count = 0;
            continue;
        }

        // -----------------------------------------------------
        // ファイル切替
        // -----------------------------------------------------
        if ( fp == nullptr || strcmp( path, current_path ) != 0 ) {
            if ( fp != nullptr ) {
                fflush( fp );
                fclose( fp );
                fp = nullptr;
            }

            struct stat st;
            if ( stat( path, &st ) != 0 ) {
                if ( !create_hro_test_file( g_hro_save_batch->first_timestamp ) ) {
                    ESP_LOGE( TAG, "Failed to create HRO file" );
                    g_hro_save_batch->count = 0;
                    continue;
                }
            }

            fp = fopen( path, "r+b" );
            if ( fp == nullptr ) {
                ESP_LOGE( TAG, "Failed to open HRO file: %s errno=%d", path, errno );
                g_hro_save_batch->count = 0;
                continue;
            }
            strncpy( current_path, path, sizeof( current_path ) - 1 );
            current_path[sizeof( current_path ) - 1] = '\0';
            ESP_LOGI( TAG, "HRO file opened: %s", current_path );
        }

        // -----------------------------------------------------
        // 10秒 = 約20KBを1回でSDへ
        // -----------------------------------------------------
        if ( write_hro_fft_batch( fp, g_hro_save_batch ) ) {
            batch_write_count++;
            ESP_LOGI(
                TAG,
                "HRO BATCH WRITE: "
                "batch=%lu seconds=%d "
                "savedrops=%lu free=%u write=%u",
                static_cast<unsigned long>( batch_write_count ),
                g_hro_save_batch->count,
                static_cast<unsigned long>( g_hro_save_drops.load( std::memory_order_relaxed ) ),
                static_cast<unsigned>( uxQueueMessagesWaiting( g_hro_save_free_queue ) ),
                static_cast<unsigned>( uxQueueMessagesWaiting( g_hro_save_write_queue ) )
            );
        }
        g_hro_save_batch->count = 0;
    }

	// ---------------------------------------------------------
	// shutdown : 残っている1～9秒分を書き込む
	// ---------------------------------------------------------
	if ( g_hro_save_batch->count > 0 ) {
	    char path[192];
	    if ( make_hro_file_path( g_hro_save_batch->first_timestamp, path, sizeof( path ) ) ) {
	        // 必要なら対象ファイルへ切り替える
        	if ( fp == nullptr || strcmp( path, current_path ) != 0 ) {
	            if ( fp != nullptr ) {
	                fflush( fp );
    	            fclose( fp );
        	        fp = nullptr;
            	}
            	
				struct stat st;
				bool file_ready = true;
				if ( stat( path, &st ) != 0 ) {
					if ( !create_hro_test_file( g_hro_save_batch->first_timestamp ) ) {
			        	ESP_LOGE( TAG, "Failed to create final HRO file" );
			        	file_ready = false;
    				}
				}

				if ( file_ready ) {
			    	fp = fopen( path, "r+b" );
			    	if ( fp != nullptr ) {
			        	strncpy( current_path, path, sizeof( current_path ) - 1 );
			        	current_path[sizeof( current_path ) - 1] = '\0';
			    	} else {
			        	ESP_LOGE( TAG, "Failed to open final HRO file: %s errno=%d", path, errno );
			    	}
				}            	
        	}

        	if ( fp != nullptr ) {
	            const int final_count = g_hro_save_batch->count;
	            if ( write_hro_fft_batch( fp, g_hro_save_batch ) ) {
	                ESP_LOGI( TAG, "HRO FINAL BATCH WRITE: seconds=%d", final_count );
	            } else {
    	            ESP_LOGE( TAG, "HRO FINAL BATCH WRITE failed" );
	            }
	        }
    	}
	    g_hro_save_batch->count = 0;
	}

	// ---------------------------------------------------------
	// SDファイルを確実にflush / close
	// ---------------------------------------------------------
	if ( fp != nullptr ) {
	    ESP_LOGI( TAG, "Flushing HRO file..." );
	    fflush( fp );
	    ESP_LOGI( TAG, "Closing HRO file..." );
		fclose( fp );
    	fp = nullptr;
	}
	
	// ---------------------------------------------------------
	// SDカードをアンマウント
	// ---------------------------------------------------------
	ESP_LOGI( TAG, "Unmounting microSD..." );
	esp_err_t unmount_err = esp_vfs_fat_sdcard_unmount( SD_MOUNT_POINT, g_sd_card );
	if ( unmount_err == ESP_OK ) {
	    ESP_LOGI( TAG, "microSD unmounted safely" );
	    g_sd_card = nullptr;
	    g_hro_storage_stopped.store( true, std::memory_order_release );
	} else {
	    ESP_LOGE( TAG, "microSD unmount failed: %s", esp_err_to_name( unmount_err ) );
	}

	// タスク終了
	vTaskDelete( nullptr );    
}

static void init_resampler( void )
{
    const float fs_up = RESAMP_INPUT_RATE * RESAMP_L;
    const float center = ( RESAMP_TOTAL_TAPS - 1 ) * 0.5f;

    for ( int phase = 0; phase < RESAMP_L; ++phase ) {
        float sum = 0.0f;
        for ( int k = 0; k < RESAMP_TAPS_PER_PHASE; ++k ) {
            const int n = phase + k * RESAMP_L;
            const float x = static_cast<float>( n ) - center;
            const float norm = 2.0f * RESAMP_CUTOFF_HZ / fs_up;

            float sinc;

            if ( fabsf( x ) < 1.0e-6f ) {
                sinc = norm;
            } else {
                sinc = sinf( static_cast<float>( M_PI ) * norm * x ) / ( static_cast<float>( M_PI ) * x );
            }

            const float window = 0.54f - 0.46f * cosf( 2.0f * static_cast<float>(M_PI) * n / ( RESAMP_TOTAL_TAPS - 1 ) );
	        float c = sinc * window * RESAMP_L;
            g_resamp_coeff[phase][k] = c;
            sum += c;
        }

        // 各phaseのDCゲインを1.0へ正規化
        if ( fabsf( sum ) > 1.0e-12f ) {
            for ( int k = 0; k < RESAMP_TAPS_PER_PHASE; ++k ) {
                g_resamp_coeff[phase][k] /= sum;
            }
        }
    }

    ESP_LOGI( TAG, "Resampler initialized: 64000 -> 8192 sps, 16/125" );
}

static bool process_resampler_input( float in_i, float in_q, float &out_i, float &out_q )
{
    // 新しい64 kS/sサンプルをdelay lineへ
    g_resamp_delay[g_resamp_delay_pos].i = in_i;
    g_resamp_delay[g_resamp_delay_pos].q = in_q;

    g_resamp_delay_pos++;

    if ( g_resamp_delay_pos >= RESAMP_TAPS_PER_PHASE ) {
        g_resamp_delay_pos = 0;
    }

    const uint64_t current_input = g_resamp_input_count++;

    // 次の出力サンプルが必要とする
    // 入力サンプル位置
    const uint64_t required_input = g_resamp_next_time / RESAMP_L;

    if ( required_input > current_input ) {
        return false;
    }

    const int phase = static_cast<int>( g_resamp_next_time % RESAMP_L );
    const uint64_t base_delay = current_input - required_input;

    float acc_i = 0.0f;
    float acc_q = 0.0f;

    for ( int k = 0; k < RESAMP_TAPS_PER_PHASE; ++k ) {
        const uint64_t delay = base_delay + static_cast<uint64_t>( k );

        // 起動直後の履歴不足部分は0扱い
        if ( delay >= g_resamp_input_count || delay >= RESAMP_TAPS_PER_PHASE ) {
            continue;
        }
        int idx = g_resamp_delay_pos - 1 - static_cast<int>( delay );
        while ( idx < 0 ) {
            idx += RESAMP_TAPS_PER_PHASE;
        }

        const float c = g_resamp_coeff[phase][k];
        acc_i += g_resamp_delay[idx].i * c;
        acc_q += g_resamp_delay[idx].q * c;
    }

    out_i = acc_i;
    out_q = acc_q;

    // 次の出力時刻。
    // 入力サンプル周期の1/16単位で125進める
    g_resamp_next_time += RESAMP_M;

    return true;
}

static void list_sdcard_root( void )
{
    DIR *dir = opendir( "/sdcard" );

    if ( dir == nullptr ) {
        ESP_LOGE( TAG, "Cannot open /sdcard directory" );
        return;
    }

    ESP_LOGI( TAG, "Files in /sdcard:" );

    struct dirent *entry;

    while ( ( entry = readdir( dir ) ) != nullptr ) {
        ESP_LOGI( TAG, "  %s", entry->d_name );
    }

    closedir( dir );
}

static esp_err_t init_sdcard( void )
{
    ESP_LOGI( TAG, "Initializing microSD" );

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };

    esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo( &ldo_config, &g_sd_pwr );

    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "SD LDO init failed: %s", esp_err_to_name( err ) );
        return err;
    }

    host.pwr_ctrl_handle = g_sd_pwr;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    slot_config.width = 4;

    slot_config.clk = GPIO_NUM_43;
    slot_config.cmd = GPIO_NUM_44;
    slot_config.d0  = GPIO_NUM_39;
    slot_config.d1  = GPIO_NUM_40;
    slot_config.d2  = GPIO_NUM_41;
    slot_config.d3  = GPIO_NUM_42;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    err = esp_vfs_fat_sdmmc_mount( SD_MOUNT_POINT, &host, &slot_config, &mount_config, &g_sd_card );
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "microSD mount failed: %s", esp_err_to_name( err ) );
        return err;
    }
    sdmmc_card_print_info( stdout, g_sd_card );
    ESP_LOGI( TAG, "microSD mounted" );

    return ESP_OK;
}

static uint8_t *load_file_to_psram( const char *path, size_t *out_size )
{
    FILE *fp = fopen( path, "rb" );
    if ( fp == nullptr ) {
        ESP_LOGE( TAG, "Cannot open %s", path );
        return nullptr;
    }

    fseek( fp, 0, SEEK_END );
    long file_size = ftell( fp );
    rewind( fp );

    if ( file_size <= 0 ) {
        fclose( fp );
        return nullptr;
    }

    uint8_t *buffer = static_cast<uint8_t *>( heap_caps_malloc( file_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );
    if ( buffer == nullptr ) {
        ESP_LOGE( TAG, "PSRAM allocation failed: %ld bytes", file_size );
        fclose( fp );
        return nullptr;
    }

    size_t read_size = fread( buffer, 1, file_size, fp );
    fclose( fp );

    if ( read_size != static_cast<size_t>( file_size ) ) {
        ESP_LOGE( TAG, "PNG read failed" );
        free( buffer );
        return nullptr;
    }
    *out_size = read_size;
    ESP_LOGI( TAG, "Loaded PNG: %u bytes", static_cast<unsigned>( read_size ) );
    return buffer;
}

static void show_system_ready( void )
{
    constexpr int W = 440;
    constexpr int H = 90;
    constexpr int X = (1280 - W) / 2;
    constexpr int Y = (720 - H) / 2;

    M5.Display.fillRoundRect( X, Y, W, H, 10, BLACK );
    M5.Display.drawRoundRect( X, Y, W, H, 10, CYAN );
    M5.Display.setTextDatum( middle_center );

    M5.Display.setFont( &fonts::Font4 );
    M5.Display.setTextColor( CYAN, BLACK );
    M5.Display.drawString( "HRO SYSTEM READY", 640, Y + 30 );

    M5.Display.setFont( &fonts::Font2 );
    M5.Display.setTextColor( WHITE, BLACK );
    M5.Display.drawString( "Observation started", 640, Y + 63 );
}

static bool load_base_screen( void )
{
    size_t png_size = 0;

    uint8_t *png_data = load_file_to_psram( HRO_BACKGROUND_BASE, &png_size );
    if ( png_data == nullptr ) {
        return false;
    }

    M5Canvas base_screen( &M5.Display );
    base_screen.setPsram( true );

    if ( base_screen.createSprite( 1280, 720 ) == nullptr ) {
        ESP_LOGE( TAG, "Failed to create base screen sprite" );
        free( png_data );
        return false;
    }

    base_screen.fillSprite( BLACK );

    // PSRAM上のPNGをSpriteへ展開
    base_screen.drawPng( png_data, png_size, 0, 0 );

    // 完成した1280x720画面をLCDへ一括転送
    base_screen.pushSprite( 0, 0 );

    // 不要になったメモリを解放
    base_screen.deleteSprite();
    free( png_data );

    return true;
}

static void show_hro_mockup( void )
{
    const char *path = HRO_BACKGROUND_BASE;
    size_t png_size = 0;
    uint8_t *png = load_file_to_psram( path, &png_size );
    if ( png == nullptr ) {
        M5.Display.fillScreen( TFT_BLACK );
        M5.Display.setTextColor( TFT_RED, TFT_BLACK );
        M5.Display.setTextSize( 2 );
        M5.Display.setCursor( 30, 30 );
        M5.Display.println( "PNG file not found" );
        return;
    }

    ESP_LOGI( TAG, "Drawing HRO mockup" );
    M5.Display.fillScreen( TFT_BLACK );
    M5.Display.drawPng( png, png_size, 0, 0 );

    free( png );

    ESP_LOGI( TAG, "HRO mockup displayed" );
}

// ============================================================
// RTL-SDR driver event callback
//
// この中では start()/stop()/install() を呼ばない。
// IQもまだ処理しない。
// ============================================================
static void rtl_event_callback( esp_rtl_sdr_event_t event, const void *payload, void *user_ctx )
{
    (void)user_ctx;

    switch ( event ) {
    case ESP_RTL_SDR_EVT_ENUMERATED: {
		const auto *info = static_cast<const esp_rtl_sdr_device_info_t *>( payload );
        if ( info != nullptr ) {
            ESP_LOGI( TAG, "RTL enumerated VID=%04x PID=%04x serial=%s USB=%s",
                     info->vid,
                     info->pid,
                     info->serial,
                     info->high_speed ? "HIGH" : "FULL" );
        } else {
            ESP_LOGI( TAG, "RTL enumerated" );
        }
        break;
    }

    case ESP_RTL_SDR_EVT_READY:
        ESP_LOGI( TAG, "RTL READY" );
        g_rtl_ready.store( true, std::memory_order_release );
        break;

    case ESP_RTL_SDR_EVT_STREAM_STARTED:
        ESP_LOGI( TAG, "RTL STREAM STARTED" );
        g_stream_started.store( true, std::memory_order_release );
        break;

case ESP_RTL_SDR_EVT_IQ_BLOCK: {
    const auto *iq = static_cast<const esp_rtl_sdr_iq_block_t *>( payload );
    if ( iq == nullptr || iq->data == nullptr || iq->bytes == 0 ) {
        break;
    }

    // RTL-SDRから受け取った量
    g_interval_bytes.fetch_add( static_cast<uint32_t>( iq->bytes ), std::memory_order_relaxed );
    g_interval_blocks.fetch_add( 1, std::memory_order_relaxed );

    uint8_t slot = 0;

    // 空きslotを取得
    if ( g_free_q == nullptr || g_filled_q == nullptr || xQueueReceive( g_free_q, &slot, 0 ) != pdTRUE ) {
        g_pipeline_drops.fetch_add( 1, std::memory_order_relaxed );
        break;
    }

    // 念のためサイズ確認
    if ( iq->bytes > IQ_SLOT_BYTES ) {
        g_pipeline_drops.fetch_add( 1, std::memory_order_relaxed );
        (void)xQueueSend( g_free_q, &slot, 0 );
        break;
    }

    // esp_rtl_sdrのborrowed bufferから
    // 自前PSRAMへ即コピー
    std::memcpy( g_iq_slots[slot], iq->data, iq->bytes );

    RtlIqBlock block {
        g_iq_slots[slot],
        iq->bytes,
        slot
    };

    // DSP taskへ渡す
    if ( xQueueSend( g_filled_q, &block, 0 ) != pdTRUE ) {
        g_pipeline_drops.fetch_add( 1, std::memory_order_relaxed );

        // 失敗したらslotを返却
        (void)xQueueSend( g_free_q, &slot, 0 );
    	}
	    break;
	}
    case ESP_RTL_SDR_EVT_STOPPED:
        ESP_LOGW( TAG, "RTL STOPPED" );
        g_stream_started.store( false, std::memory_order_release );
        break;

    case ESP_RTL_SDR_EVT_DISCONNECTED:
        ESP_LOGW( TAG, "RTL DISCONNECTED" );
        g_rtl_ready.store( false, std::memory_order_release );
        g_stream_started.store( false, std::memory_order_release );
        break;

    case ESP_RTL_SDR_EVT_RETUNED: {
        const auto *hz = static_cast<const uint32_t *>( payload );
        if ( hz != nullptr ) {
            ESP_LOGI( TAG, "RTL RETUNED: %u Hz", static_cast<unsigned>( *hz ) );
        }
        break;
    }

    case ESP_RTL_SDR_EVT_ERROR: {
        const auto *err = static_cast<const esp_rtl_sdr_error_info_t *>( payload );
        ESP_LOGE( TAG, "RTL ERROR: %s", err != nullptr ? esp_rtl_sdr_err_to_name( err->code ) : "unknown" );
        break;
    }

    default:
        break;
    }
}

static void hro_fft_task( void * )
{
    FftFrame frame{};

    ESP_LOGI( TAG, "HRO FFT task started" );

    while ( true ) {
        if ( xQueueReceive( g_fft_ready_q, &frame, portMAX_DELAY) != pdTRUE ) {
            continue;
        }

        // ----------------------------------------
        // IQ -> interleaved complex FFT buffer
        // + Hann window
        // ----------------------------------------

        for ( int n = 0; n < FFT_SIZE; ++n ) {
            const float w = 0.5f - 0.5f * cosf( TWO_PI * static_cast<float>( n ) / static_cast<float>( FFT_SIZE - 1 ) );
            g_fft_work[n * 2 + 0] = frame.i[n] * w;
            g_fft_work[n * 2 + 1] = frame.q[n] * w;
        }

        // ----------------------------------------
        // 8192-point complex FFT
        // ----------------------------------------
        esp_err_t ret = dsps_fft2r_fc32( g_fft_work, FFT_SIZE );
        if ( ret == ESP_OK ) {
            ret = dsps_bit_rev_fc32( g_fft_work, FFT_SIZE );
        }

        // ----------------------------------------
        // Search 530 ... 1030 Hz
        // ----------------------------------------
        if ( ret == ESP_OK ) {
			float max_power = 0.0f;
			int max_bin = 0;
			for ( int bin = HRO_BIN_MIN; bin <= HRO_BIN_MAX; ++bin ) {
			    const float re = g_fft_work[bin * 2 + 0];
    			const float im = g_fft_work[bin * 2 + 1];
			    const float power = re * re + im * im;

			    // 530 Hz -> index 0
			    // 531 Hz -> index 1
			    // ...
			    // 1030 Hz -> index 500
			    g_hro_spectrum[bin - HRO_BIN_MIN] = 10.0f * log10f( power + 1.0e-20f );
			    if (power > max_power) {
			        max_power = power;
			        max_bin = bin;
			    }
			}
#if 0
			if ( !g_hro_shutdown_requested.load( std::memory_order_acquire ) && g_hro_save_free_queue != nullptr && g_hro_save_write_queue != nullptr ) {
			    HroSaveRecord *record = nullptr;
			    // 絶対にFFT Taskを待たせない
    			if ( xQueueReceive( g_hro_save_free_queue, &record, 0 ) == pdTRUE ) {
			        time( &record->timestamp );
			        memcpy( record->spectrum, g_hro_spectrum, HRO_BIN_COUNT * sizeof( float ) );
			        if ( xQueueSend( g_hro_save_write_queue,  &record, 0 ) != pdTRUE ) {
			            // write queueへ渡せなかったので
            			// バッファをfree queueへ戻す
			            xQueueSend( g_hro_save_free_queue, &record, 0 );
		 	    		g_hro_save_drops.fetch_add( 1, std::memory_order_relaxed );
			        }
			    } else {
			        // SD Writerが追いつかず空きバッファなし
        			g_hro_save_drops.fetch_add( 1, std::memory_order_relaxed );
			    }
			}
#endif
			// 501 bins全部を書き終わってから更新
			g_hro_spectrum_sequence.fetch_add( 1, std::memory_order_release );

			if ( g_hro_history != nullptr ) {
			    float *dst = &g_hro_history[static_cast<size_t>( g_hro_history_write_pos ) * HRO_BIN_COUNT];
			    memcpy( dst, g_hro_spectrum, HRO_BIN_COUNT * sizeof( float ) );

			    // 次の書き込み位置
    			g_hro_history_write_pos++;

			    if ( g_hro_history_write_pos >= HRO_HISTORY_SECONDS ) {
			        g_hro_history_write_pos = 0;
			    }

			    // 起動後600秒までは蓄積数を増やす
			    if ( g_hro_history_count < HRO_HISTORY_SECONDS ) {
			        g_hro_history_count++;
			    }
			    g_hro_history_sequence.fetch_add( 1, std::memory_order_release );
			}
            const float power_db = 10.0f * log10f( max_power + 1.0e-20f );
            ESP_LOGI( TAG, "FFT PEAK: %d Hz  %.1f dB", max_bin, power_db );
        } else {
            ESP_LOGE( TAG, "FFT failed: %s", esp_err_to_name( ret ) );
        }

        // ----------------------------------------
        // Frame processed
        // ----------------------------------------
        g_fft_processed.fetch_add( 1, std::memory_order_relaxed );

        // ----------------------------------------
        // Return buffer to DSP
        // ----------------------------------------
        (void)xQueueSend( g_fft_free_q, &frame.slot, portMAX_DELAY );
        
        // 10秒ログ
        if ( ( g_hro_history_sequence.load() % 10 ) == 0 ) {
		    ESP_LOGI(
        	TAG,
        	"HISTORY: count=%u write_pos=%u seq=%lu",
        	static_cast<unsigned>( g_hro_history_count ),
        	static_cast<unsigned>( g_hro_history_write_pos ),
        	static_cast<unsigned long>(
            g_hro_history_sequence.load() ) );
		}
    }
}

#if 0
static void hro_fft_task( void * )
{
    FftFrame frame{};

    ESP_LOGI( TAG, "HRO FFT task started" );

    while ( true ) {
        if ( xQueueReceive( g_fft_ready_q, &frame, portMAX_DELAY ) != pdTRUE ) {
            continue;
        }

        // ★ 今はFFTしない
        //
        // frame.i[0 ... 8191]
        // frame.q[0 ... 8191]
        //
        // に1秒分のcomplex IQが入っている。

        // コンパイラに完全に無視されないよう確認
        volatile float test = frame.i[0] + frame.q[0];
        (void)test;

        g_fft_processed.fetch_add( 1, std::memory_order_relaxed );

        // 処理終了したバッファをDSP側へ返す
        (void)xQueueSend( g_fft_free_q, &frame.slot, portMAX_DELAY );
    }
}
#endif

static void hro_dsp_task( void * )
{
    RtlIqBlock block{};

	// rtl_fm方式のFs/4 frequency shift用
	// 960 kS/s / 4 = 240 kHz
	// ブロック境界をまたいで位相を保持する
	uint8_t fs4_phase = 0;

    // NCOの位相。ブロックをまたいで保持する。
	// NCO: g_hro_freq_shift Hz
	float nco_i = 1.0f;
	float nco_q = 0.0f;
	const float nco_c = cosf( TWO_PI * g_hro_freq_shift / 64000.0f );
	const float nco_s = sinf( TWO_PI * g_hro_freq_shift / 64000.0f );
	uint32_t nco_normalize_count = 0;

    float fir_i[FIR_TAPS] = {};
	float fir_q[FIR_TAPS] = {};

	int fir_pos = 0;
	uint32_t decim_phase = 0;
	double power_sum = 0.0;
	uint32_t power_count = 0;

    ESP_LOGI( TAG, "HRO DSP task started: shift=%.1f Hz", g_hro_freq_shift );

	float test_i = 1.0f;
	float test_q = 0.0f;
	const float test_c = cosf( TWO_PI * HRO_TEST_TONE_HZ / HRO_SAMPLE_RATE );
	const float test_s = sinf( TWO_PI * HRO_TEST_TONE_HZ / HRO_SAMPLE_RATE );

   	init_resampler();

	TickType_t last_idle_yield = xTaskGetTickCount();

    while ( true ) {
        if ( xQueueReceive( g_filled_q, &block,portMAX_DELAY ) != pdTRUE ) {
            continue;
        }

        // CU8なので I,Q が1 byteずつ交互
        const size_t samples = block.bytes / 2;

		float i, q;
        for ( size_t n = 0; n < samples; ++n ) {
			if ( HRO_TEST_TONE ) {
			    // +1000 Hz complex test tone
			    i = test_i;
				q = test_q;
			    const float new_i = test_i * test_c - test_q * test_s;
			    const float new_q = test_i * test_s + test_q * test_c;
			    test_i = new_i;
			    test_q = new_q;
			} else {
			    // Real RTL-SDR IQ
    			i = ( static_cast<float>( block.data[n * 2] ) - 127.5f ) / 128.0f;
    			q = ( static_cast<float>( block.data[n * 2 + 1] ) - 127.5f ) / 128.0f;

    			// --------------------------------------------------------
    			// rtl_fm方式: +Fs/4 complex frequency shift
    			//
    			// Fs = 960 kS/s
    			// Fs/4 = 240 kHz
    			//
    			// LOより -240 kHzにある目的RFをDCへ移動する。
    			//
    			// exp(+j*pi*n/2):
    			//   0: ( I,  Q)
    			//   1: (-Q,  I)
    			//   2: (-I, -Q)
    			//   3: ( Q, -I)
    			// --------------------------------------------------------
    			const float raw_i = i;
    			const float raw_q = q;

    			switch ( fs4_phase ) {
        		case 0:
            		i =  raw_i;
            		q =  raw_q;
            		break;
		        case 1:
            		i = -raw_q;
            		q =  raw_i;
            		break;
		        case 2:
            		i = -raw_i;
            		q = -raw_q;
            		break;
		        default:    // phase 3
            		i =  raw_q;
            		q = -raw_i;
            		break;
    			}
			    fs4_phase = ( fs4_phase + 1 ) & 3;
			}
			
			// FIR delay lineへ入力
			fir_i[fir_pos] = i;
			fir_q[fir_pos] = q;

			fir_pos++;

			if ( fir_pos >= FIR_TAPS ) {
			    fir_pos = 0;
			}

			// 256 kS/s → 64 kS/s
			// 4入力サンプルに1回だけFIR出力を計算
			if ( ++decim_phase >= 4 ) {
			    decim_phase = 0;
			    float out_i = 0.0f;
    			float out_q = 0.0f;
			    int idx = fir_pos - 1;

			    if (idx < 0) {
			        idx = FIR_TAPS - 1;
			    }

    			for (int k = 0; k < FIR_TAPS; ++k) {
			        out_i += FIR_COEFF[k] * fir_i[idx];
        			out_q += FIR_COEFF[k] * fir_q[idx];
			        idx--;
			        if (idx < 0) {
            			idx = FIR_TAPS - 1;
        			}
    			}

				// 64 kS/sで目的信号をFFT中心周波数へシフト
				// 現在: +780 Hz
				const float shifted_i = out_i * nco_i - out_q * nco_q;
				const float shifted_q = out_i * nco_q + out_q * nco_i;

				// NCOを1 sample進める
				const float next_nco_i = nco_i * nco_c - nco_q * nco_s;
				const float next_nco_q = nco_i * nco_s + nco_q * nco_c;

				nco_i = next_nco_i;
				nco_q = next_nco_q;

				// 浮動小数点誤差による振幅ドリフトを補正
				if ( ++nco_normalize_count >= 65536 ) {
				    nco_normalize_count = 0;
				    const float mag = sqrtf( nco_i * nco_i + nco_q * nco_q );
				    if ( mag > 0.0f ) {
				        nco_i /= mag;
				        nco_q /= mag;
				    }
				}

    			float resamp_i = 0.0f;
				float resamp_q = 0.0f;
				if ( process_resampler_input( shifted_i, shifted_q, resamp_i, resamp_q ) ) {
				    // -------------------------
    				// Audio sample
    				// CPU1ではPCM化だけ。Gain/VolumeはAudio Task(CPU0)で処理する
    				// -------------------------
    				float s = resamp_i * 32767.0f;
				    if ( s > 32767.0f ) {
				        s = 32767.0f;
				    } else if ( s < -32768.0f ) {
				        s = -32768.0f;
				    }

				    g_audio_build_block.samples[g_audio_build_pos++] = static_cast<int16_t>( s );
				    if ( g_audio_build_pos >= AUDIO_BLOCK_SAMPLES ) {
				        if ( g_audio_queue != nullptr ) {
				            if ( xQueueSend( g_audio_queue, &g_audio_build_block, 0 ) != pdTRUE ) {
				                g_audio_drops.fetch_add( 1, std::memory_order_relaxed );
				            }
				        }
				        g_audio_build_pos = 0;
				    }
				    g_resamp_10sec_samples.fetch_add( 1, std::memory_order_relaxed );
				    const float p = resamp_i * resamp_i + resamp_q * resamp_q;
				    g_resamp_interval_samples.fetch_add( 1, std::memory_order_relaxed );
				    g_resamp_power.store( static_cast<uint32_t>( p * 1000000.0f ), std::memory_order_relaxed );

				    // 8192 samples = 1 FFT frame
					g_fft_i[g_fft_write_slot][g_fft_pos] = resamp_i;
					g_fft_q[g_fft_write_slot][g_fft_pos] = resamp_q;
					g_fft_pos++;
					if ( g_fft_pos >= FFT_SIZE ) {
					    FftFrame frame{
				    	    g_fft_i[g_fft_write_slot],
				        	g_fft_q[g_fft_write_slot],
				        	g_fft_write_slot
				    	};

					    if ( xQueueSend( g_fft_ready_q, &frame, 0 ) == pdTRUE ) {
					        g_fft_frames.fetch_add( 1, std::memory_order_relaxed );
					    } else {
					        // 通常ここには来ない
        					g_fft_drops.fetch_add( 1, std::memory_order_relaxed );
					        g_fft_pos = 0;
        					return;
					    }

					    // 次の空きFFTバッファを取得
					    uint8_t next_slot;

					    if ( xQueueReceive( g_fft_free_q, &next_slot, 0 ) == pdTRUE ) {
					        g_fft_write_slot = next_slot;
					        g_fft_pos = 0;
					    } else {
					        // FFT側がまだ前のバッファを処理中
					        g_fft_drops.fetch_add( 1, std::memory_order_relaxed );
					        g_fft_pos = 0;
					    }
					}
				}

				g_decim_buffer[g_decim_buffer_pos].i = out_i;
				g_decim_buffer[g_decim_buffer_pos].q = out_q;
				g_decim_buffer_pos++;
				if ( g_decim_buffer_pos >= DECIM_BUFFER_SAMPLES ) {
				    g_decim_buffer_pos = 0;
				}

			    // 今はまだ次段へ渡さない
			    // 64 kS/s IQがここにできている
				const float power =	out_i * out_i +	out_q * out_q;

				power_sum += power;
				power_count++;
				
    			g_decim_interval_samples.fetch_add( 1, std::memory_order_relaxed );
			}
        }
		TickType_t now = xTaskGetTickCount();
		if ( now - last_idle_yield >= pdMS_TO_TICKS( 2000 ) ) {
    		last_idle_yield = now;
//    		vTaskDelay( 1 );
		}        

		if ( power_count > 0 ) {
		    const float avg_power =	static_cast<float>( power_sum / power_count );
		    g_decim_power.store( static_cast<uint32_t>( avg_power * 1000000.0f ), std::memory_order_relaxed );
    		power_sum = 0.0;
    		power_count = 0;
		}

        g_dsp_interval_bytes.fetch_add( static_cast<uint32_t>( block.bytes ), std::memory_order_relaxed );
        g_dsp_interval_blocks.fetch_add( 1, std::memory_order_relaxed );

        (void)xQueueSend( g_free_q, &block.slot, portMAX_DELAY );
    }
}

// ============================================================
// HRO radio control task
// Core 1
// ============================================================
static void hro_radio_task( void * )
{
    ESP_LOGI( TAG, "Waiting for RTL-SDR READY..." );

    while ( !g_rtl_ready.load( std::memory_order_acquire ) ) {
        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }

    ESP_LOGI( TAG, "RTL ready" );

    if ( !esp_rtl_sdr_is_rate_supported( RTL_SAMPLE_RATE ) ) {
        ESP_LOGE( TAG, "Sample rate %u not supported", static_cast<unsigned>( RTL_SAMPLE_RATE ) );
        vTaskDelete( nullptr );
        return;
    }

    esp_rtl_sdr_stream_config_t stream;
    esp_rtl_sdr_stream_config_default( &stream );

	stream.preset = ESP_RTL_SDR_PRESET_CUSTOM_HZ;
	stream.frequency_hz = g_hro_lo_frequency_hz;
	stream.sample_rate_sps = RTL_SAMPLE_RATE;

    // 連続受信
    stream.max_bytes = 0;
    stream.timeout_ms = 0;

    esp_err_t err = esp_rtl_sdr_stream_config_validate( &stream );

    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "stream config error: %s", esp_rtl_sdr_err_to_name( err ) );
        vTaskDelete( nullptr );
        return;
    }

    ESP_LOGI(TAG,
             "Starting stream RF=%u Hz rate=%u sps",
             static_cast<unsigned>( g_hro_lo_frequency_hz ),
             static_cast<unsigned>( RTL_SAMPLE_RATE ) );

    err = esp_rtl_sdr_start( g_rtl, &stream );
    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "RTL start failed: %s", esp_rtl_sdr_err_to_name( err ) );
        vTaskDelete( nullptr );
        return;
    }

	esp_err_t gain_err = esp_rtl_sdr_set_tuner_gain(g_rtl, 402);
	if (gain_err == ESP_OK) {
    	ESP_LOGI(TAG, "RTL tuner gain = 40.2 dB (manual)");
	} else {
    	ESP_LOGE(TAG, "RTL gain set failed: %s", esp_rtl_sdr_err_to_name( gain_err ) );
	}

    ESP_LOGI(TAG, "HRO IQ stream running");

	uint32_t log_seconds = 0;
	int64_t resamp_test_start_us = esp_timer_get_time();
	
    while ( true ) {
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
		const uint32_t rx_bytes = g_interval_bytes.exchange( 0, std::memory_order_relaxed );
		const uint32_t rx_blocks = g_interval_blocks.exchange( 0, std::memory_order_relaxed );
		const uint32_t dsp_bytes = g_dsp_interval_bytes.exchange( 0, std::memory_order_relaxed );
		const uint32_t dsp_blocks = g_dsp_interval_blocks.exchange( 0, std::memory_order_relaxed );
		const uint32_t drops = g_pipeline_drops.load( std::memory_order_relaxed );
		const uint32_t decim_samples = g_decim_interval_samples.exchange(0, std::memory_order_relaxed );
		const uint32_t decim_power = g_decim_power.load( std::memory_order_relaxed );
		const uint32_t resamp_samples = g_resamp_interval_samples.exchange( 0, std::memory_order_relaxed );
		const uint32_t resamp_power = g_resamp_power.load( std::memory_order_relaxed );
		const uint32_t fft_frames = g_fft_frames.exchange( 0, std::memory_order_relaxed );
		const uint32_t fft_processed = g_fft_processed.exchange( 0, std::memory_order_relaxed );
		const uint32_t fft_drops = g_fft_drops.load( std::memory_order_relaxed );
		ESP_LOGI(
    		TAG,
	    	"RX=%u B/s %u blk  "
		    "DSP=%u B/s %u blk  "
		    "DECIM=%u  RESAMP=%u  "
		    "FFT=%u/%u  "
		    "drops=%u fftdrops=%u",
		    rx_bytes,
		    rx_blocks,
		    dsp_bytes,
		    dsp_blocks,
		    decim_samples,
		    resamp_samples,
		    fft_frames,
		    fft_processed,
		    drops,
    		fft_drops );
    		    		
		if ( ++log_seconds >= 10 ) {
		    log_seconds = 0;
		    const int64_t now_us = esp_timer_get_time();
		    const uint32_t count = g_resamp_10sec_samples.exchange( 0, std::memory_order_relaxed );
		    const double elapsed_sec = static_cast<double>( now_us - resamp_test_start_us ) / 1000000.0;
		    const double actual_rate = count / elapsed_sec;

		    ESP_LOGI( TAG, "RESAMP test: %u samples / %.3f sec = %.2f sps", count, elapsed_sec, actual_rate );
		    resamp_test_start_us = now_us;
		}
		if ( g_hro_history_count >= 10 ) {
		    const float *oldest = get_hro_history( 0 );
		    const float *newest = get_hro_history( g_hro_history_count - 1 );
		    if ( oldest != nullptr && newest != nullptr ) {
		        ESP_LOGI(
        	    TAG,
            	"HISTORY READ: count=%u  "
            	"oldest780=%.1f  newest780=%.1f",
            	static_cast<unsigned>(g_hro_history_count), oldest[780 - HRO_BIN_MIN], newest[780 - HRO_BIN_MIN] );
		    }
		}
    }
}

static void hro_audio_task( void *arg )
{
    ESP_LOGI( TAG, "HRO Audio task started" );

    int play_index = 0;
    uint32_t block_count = 0;

    for ( ;; ) {
        AudioBlock incoming;
        if ( xQueueReceive( g_audio_queue, &incoming, portMAX_DELAY ) != pdTRUE ) {
            continue;
        }

        block_count++;

		// 最大絶対値
		int16_t peak = 0;
		for ( size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i ) {
    		int32_t v = incoming.samples[i];
		    if ( v < 0 ) {
        		v = -v;
    		}
		    if ( v > peak ) {
        		peak = static_cast<int16_t>(v);
    		}
		}

		if ( ( block_count % 100 ) == 0 ) {
    		ESP_LOGI( TAG, "AUDIO: received %lu blocks peak=%d", (unsigned long)block_count, (int)peak );
		}

        AudioBlock &play = g_audio_play_buffer[play_index];
        const bool mute = g_audio_mute.load( std::memory_order_relaxed );
        const int volume = g_audio_volume.load( std::memory_order_relaxed );
        constexpr float AUDIO_GAIN = 4.0f;
		const float gain = mute ? 0.0f : AUDIO_GAIN * static_cast<float>(volume) / 100.0f;

        for ( size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i ) {
            float s = static_cast<float>( incoming.samples[i] ) * gain;
			// Beeper
			int beep_samples = g_touch_beep_samples.load( std::memory_order_relaxed );
			if ( beep_samples > 0 ) {
			    // 1200 Hz の短いタッチ音
			    constexpr float BEEP_FREQ = 1200.0f;
    			constexpr float BEEP_LEVEL = 3000.0f;
			    s += sinf(g_touch_beep_phase) * BEEP_LEVEL;
			    g_touch_beep_phase += 2.0f * M_PI * BEEP_FREQ / static_cast<float>( AUDIO_SAMPLE_RATE );
			    if ( g_touch_beep_phase >= 2.0f * M_PI ) {
        			g_touch_beep_phase -= 2.0f * M_PI;
    			}
			    g_touch_beep_samples.store( beep_samples - 1, std::memory_order_relaxed );
			}
            if ( s > 32767.0f ) {
                s = 32767.0f;
            } else if ( s < -32768.0f ) {
                s = -32768.0f;
            }
            play.samples[i] = static_cast<int16_t>( s );
        }
        bool ok = M5.Speaker.playRaw(
            play.samples,
            AUDIO_BLOCK_SAMPLES,
            AUDIO_SAMPLE_RATE,
            false,      // mono
            1,          // repeat
            0,          // channel 0
            false       // 現在音を強制停止しない
        );

        if ( !ok ) {
            ESP_LOGW( TAG, "Speaker playRaw queue full" );
        }

        play_index++;

        if ( play_index >= AUDIO_BUFFER_COUNT ) {
            play_index = 0;
        }
    }
}

// --------------------------------------------------
// FFT waterfall周波数目盛
// --------------------------------------------------
static void draw_waterfall_frequency_axis( void )
{
    constexpr int DIVISIONS = 4;

    const int center_hz = g_hro_config.fft_center_hz;
    const int range_hz  = g_hro_config.fft_range_hz;

    const int min_hz = center_hz - range_hz;
    const int max_hz = center_hz + range_hz;

    M5.Display.setFont( &fonts::Font0 );
    M5.Display.setTextColor( CYAN );
    M5.Display.setTextDatum( middle_right );

    for ( int i = 0; i <= DIVISIONS; ++i ) {
        const int y = WF_Y + static_cast<int>( static_cast<int64_t>( i ) * ( WF_H - 1 ) / DIVISIONS );
        const int hz = max_hz - static_cast<int>( static_cast<int64_t>( i ) * ( max_hz - min_hz ) / DIVISIONS );

        char buf[16];
        snprintf( buf, sizeof( buf ), "%d", hz );
        M5.Display.drawString( buf, WF_X - 7, y );
        M5.Display.drawFastHLine( WF_X - 5, y, 5, CYAN );
        M5.Display.drawFastHLine( WF_X + WF_W + 1, y, 5, CYAN );
    }

	// Waterfallの下に時刻目盛を描く
	const int axis_y = WF_Y + WF_H + 1;
    for ( int sec = 0; sec <= TOTAL_SEC; sec += INTERVAL_SEC ) {
        const int x = WF_X + WF_W - static_cast<int>( static_cast<int64_t>( sec ) * WF_W / TOTAL_SEC );
		M5.Display.drawFastVLine( x, axis_y, 5, CYAN );
    }

}

// --------------------------------------------------
// PNG上部の情報表示領域
//   1: Observer / Location
//   2: Longitude / Latitude
//   3: Receiver / Antenna
//   4: RF / FFT
// --------------------------------------------------
static void draw_station_info( void )
{
    // PNG上部の情報表示領域
    //   1: Observer / Location
    //   2: Longitude / Latitude
    //   3: Receiver / Antenna
    //   4: RF / FFT

    const uint32_t WHITE = 0xFFFFFF;
    const uint32_t CYAN  = 0x00E5FF;

    // --------------------------------------------------
    // Decimal degrees -> DMS
    // --------------------------------------------------

    DmsValue lon = longitude_to_dms( g_hro_config.longitude );
    DmsValue lat = latitude_to_dms( g_hro_config.latitude );

    char line1[128];
    char line2[128];

    M5.Display.setTextDatum( middle_left );
    M5.Display.setFont( &fonts::Font2 );
//	M5.Display.setTextSize( 1.2f );	// フォント 1.2倍

    // --------------------------------------------------
    // 1. Observer / Location
    // x ≈ 490 .. 743
    // --------------------------------------------------

    snprintf( line1, sizeof( line1 ), "%s", g_hro_config.observer );
    snprintf( line2, sizeof( line2 ), "%s", g_hro_config.location );

	M5.Display.setTextColor( WHITE );
	M5.Display.drawString( line1, 474, 31 );

	M5.Display.setTextColor( CYAN );
	M5.Display.drawString( line2, 474, 58 );

    // --------------------------------------------------
    // 2. Longitude / Latitude
    // x ≈ 745 .. 892
    // --------------------------------------------------

    snprintf( line1, sizeof( line1 ),
             "%c %d %02d %05.2f",
             lon.direction,
             lon.degrees,
             lon.minutes,
             lon.seconds);

    snprintf( line2, sizeof( line2 ),
             "%c %d %02d %05.2f",
             lat.direction,
             lat.degrees,
             lat.minutes,
             lat.seconds);

    M5.Display.setTextColor( WHITE );
    M5.Display.drawString( line1, 780, 31 );
    M5.Display.drawString( line2, 780, 58 );


    // --------------------------------------------------
    // 3. Receiver / Antenna
    // x ≈ 894 .. 1108
    // --------------------------------------------------

    snprintf( line1, sizeof( line1 ), "%s", g_hro_config.receiver );
    snprintf(line2, sizeof( line2 ), "%s", g_hro_config.antenna );

    M5.Display.setTextColor( WHITE );
    M5.Display.drawString( line1, 905, 31 );

    M5.Display.setTextColor( CYAN );
    M5.Display.drawString( line2, 905, 58 );

    // --------------------------------------------------
    // 4. RF / FFT
    // x ≈ 1110 .. 1268
    // --------------------------------------------------

    const double rf_mhz = static_cast<double>( g_hro_config.frequency_hz ) / 1000000.0;
    snprintf( line1, sizeof( line1 ), "%.6f MHz", rf_mhz );
	snprintf( line2, sizeof( line2 ), "FFT %ld +/-%ld Hz", static_cast<long>( g_hro_config.fft_center_hz ),static_cast<long>( g_hro_config.fft_range_hz ) );

    M5.Display.setTextColor( WHITE );
    M5.Display.drawString( line1, 1130, 31 );

    M5.Display.setTextColor( CYAN );
    M5.Display.drawString( line2, 1130, 58 );
}

static void test_lcd_snapshot_speed( void )
{
    const int w = CAP_W;
    const int h = CAP_H;
    const int x = CAP_X;
    const int y = CAP_Y;

    const size_t bytes =
        static_cast<size_t>( w ) *
        static_cast<size_t>( h ) *
        sizeof( uint16_t );

    ESP_LOGI( TAG,
              "LCD snapshot: allocating %u bytes PSRAM",
              (unsigned)bytes );

    uint16_t* buffer =
        static_cast<uint16_t*>(
            heap_caps_malloc(
                bytes,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            )
        );

    if ( buffer == nullptr ) {
        ESP_LOGE( TAG, "LCD snapshot: PSRAM allocation failed" );
        return;
    }

    const int64_t start_us = esp_timer_get_time();

    M5.Display.readRect(
        x,
        y,
        w,
        h,
        buffer
    );

    const int64_t end_us = esp_timer_get_time();

    ESP_LOGI(
        TAG,
        "LCD snapshot: %dx%d RGB565 %u bytes time=%lld us (%.2f ms)",
        w,
        h,
        (unsigned)bytes,
        (long long)( end_us - start_us ),
        (double)( end_us - start_us ) / 1000.0
    );

    free( buffer );
}


// ============================================================
// app_main
// ============================================================
extern "C" void app_main( void )
{
    ESP_LOGI( TAG, "" );
    ESP_LOGI( TAG, "==========================" );
    ESP_LOGI( TAG, "      Tab5-HRO" );
    ESP_LOGI( TAG, "==========================" );

	auto m5cfg = M5.config();
	M5.begin( m5cfg );

	M5.Speaker.begin();
	M5.Speaker.setVolume( 128 );

    M5.Power.setExtOutput( true, m5::ext_USB );
	M5.Display.setRotation( 3 );
	M5.Display.setBrightness( 200 );

//	show_system_ready();

	esp_err_t ret = nvs_flash_init();    // RTC → System Clock
    restore_system_time_from_rtc();

	if ( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) {
	    ESP_ERROR_CHECK( nvs_flash_erase() );
	    ESP_ERROR_CHECK( nvs_flash_init() );
	} else {
    	ESP_ERROR_CHECK( ret );
	}

	char saved_ssid[33] = {};
	char saved_password[65] = {};

	if ( load_wifi_settings( saved_ssid, sizeof( saved_ssid ), saved_password, sizeof( saved_password ) ) ) {
	    ESP_LOGI( TAG, "Wi-Fi configuration restored from NVS" );
	}
	
	g_waterfall.setPsram( true );
	g_waterfall.setColorDepth( 16 );
	if ( g_waterfall.createSprite( WF_W, WF_H ) == nullptr ) {
	    ESP_LOGE( TAG, "Failed to create combined waterfall sprite" );
	} else {
	    ESP_LOGI( TAG, "Combined sprite created: %d x %d", WF_W, WF_SPRITE_H );
	    // Waterfall部分
    	g_waterfall.fillRect( 0, WF_SPRITE_Y, WF_W, WF_H, M5.Display.color565( 0, 10, 18 ) );
		ESP_LOGI( TAG, "Waterfall sprite created: %d x %d", WF_W, WF_H );
	}

	g_level_graph.setPsram( false );
	g_level_graph.setColorDepth( 8 );
	if ( g_level_graph.createSprite( LEVEL_W, LEVEL_H ) == nullptr ) {
    	ESP_LOGE( TAG, "Level graph sprite allocation failed" );
	} else {
    	g_level_graph.fillSprite( BLACK );
    	g_level_graph.pushSprite( LEVEL_X, LEVEL_Y );
		ESP_LOGI( TAG, "Level graph sprite created: %d x %d", LEVEL_W, LEVEL_H );
	}

    // OrcSDRの現在の起動方法と同じ。
    // RTL-SDR接続中にUSB railをOFF→ONしない。
    ESP_LOGI( TAG, "Enabling USB-A VBUS" );

	if ( init_sdcard() == ESP_OK ) {
    	list_sdcard_root();
    	load_base_screen();
		ensure_hro_config();
		draw_station_info();
		draw_waterfall_frequency_axis();
		draw_shutdown_button( false );
	}
#if 0
	// 保存用PSRAMバッファとQueueを初期化
	if ( init_hro_save_buffers() ) {
	    xTaskCreatePinnedToCore( hro_save_writer_task, "hro_save", 4096, nullptr, 2, nullptr, 0 );       // CPU0
	} else {
	    ESP_LOGE( TAG, "HRO data save initialization failed" );
	}
#endif

	// Audio設定を実際のAudio制御へ反映
	g_audio_volume.store( g_hro_config.audio_volume, std::memory_order_relaxed );
	g_audio_mute.store( g_hro_config.audio_mute, std::memory_order_relaxed );
	draw_audio_controls();

	HroTuning tuning = make_hro_tuning();
	g_hro_lo_frequency_hz = tuning.actual_lo_hz;
	g_hro_freq_shift = tuning.nco_shift_hz;

	ESP_LOGI(
    	TAG,
    	"HRO tuning: target=%lu ideal_LO=%lu actual_LO=%lu IF=%ld NCO=%.1f Hz",
    	static_cast<unsigned long>(tuning.target_rf_hz),
    	static_cast<unsigned long>(tuning.ideal_lo_hz),
    	static_cast<unsigned long>(tuning.actual_lo_hz),
    	static_cast<long>(tuning.actual_if_hz),
    	tuning.nco_shift_hz
	);
	
	
    // OrcSDRでは350ms待っている。
    // 最小試験では少し余裕を持たせ500ms。
    vTaskDelay( pdMS_TO_TICKS( 500 ) );

    // --------------------------------------------------------
    // Audio queues
    // --------------------------------------------------------
	g_audio_queue = xQueueCreate( 4, sizeof( AudioBlock ) );
	if ( g_audio_queue == nullptr ) {
	    ESP_LOGE( TAG, "Failed to create audio queue" );
	    return;
	}

    // --------------------------------------------------------
    // IQ ring buffer / queues
    // --------------------------------------------------------
    g_free_q = xQueueCreate( IQ_SLOT_COUNT, sizeof( uint8_t ) );
    g_filled_q = xQueueCreate( IQ_SLOT_COUNT, sizeof( RtlIqBlock ) );

    if ( g_free_q == nullptr || g_filled_q == nullptr ) {
        ESP_LOGE( TAG, "Failed to create IQ queues" );
        return;
    }

    for ( uint8_t i = 0; i < IQ_SLOT_COUNT; ++i ) {
        g_iq_slots[i] = static_cast<uint8_t *>( heap_caps_malloc( IQ_SLOT_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );
        if ( g_iq_slots[i] == nullptr ) {
            ESP_LOGE( TAG, "Failed to allocate IQ slot %u", static_cast<unsigned>( i ) );
            return;
        }
        xQueueSend( g_free_q, &i, portMAX_DELAY );
    }

    ESP_LOGI( TAG,
             "IQ ring allocated: %u slots x %u bytes = %u bytes",
             static_cast<unsigned>( IQ_SLOT_COUNT ),
             static_cast<unsigned>( IQ_SLOT_BYTES ),
             static_cast<unsigned>( IQ_SLOT_COUNT * IQ_SLOT_BYTES ) );
         
    // ========================================
    // FFT queue 作成
    // ========================================
	g_fft_free_q = xQueueCreate( FFT_BUFFER_COUNT, sizeof( uint8_t ) );
	g_fft_ready_q = xQueueCreate( FFT_BUFFER_COUNT, sizeof( FftFrame ) );

	if ( g_fft_free_q == nullptr || g_fft_ready_q == nullptr ) {
	    ESP_LOGE( TAG, "Failed to create FFT queues" );
	    return;
	}

    // ========================================
    // FFT double buffer 確保
    // ========================================
	for ( uint8_t i = 0; i < FFT_BUFFER_COUNT; ++i ) {
	    g_fft_i[i] = static_cast<float *>( heap_caps_malloc( FFT_SIZE * sizeof( float ), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );
	    g_fft_q[i] = static_cast<float *>( heap_caps_malloc( FFT_SIZE * sizeof( float ), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );

	    if ( g_fft_i[i] == nullptr || g_fft_q[i] == nullptr) {
	        ESP_LOGE( TAG, "Failed to allocate FFT buffer %u", static_cast<unsigned>( i ) );
    	    return;
	    }
    	xQueueSend( g_fft_free_q, &i, portMAX_DELAY );
	}

	ESP_LOGI(
	    TAG,
    	"FFT double buffer allocated: "
    	"%u x %u complex samples",
    	FFT_BUFFER_COUNT,
    	static_cast<unsigned>( FFT_SIZE ) );
    
    // ========================================
    // 最初の書き込み用FFT bufferを1個取得
    // ========================================
    if ( xQueueReceive( g_fft_free_q, &g_fft_write_slot, portMAX_DELAY) != pdTRUE ) {
        ESP_LOGE( TAG, "Failed to get first FFT buffer" );
        return;
    }
    g_fft_pos = 0;

    // ========================================
    // FFT専用の作業バッファ取得
    // ========================================
	g_fft_work = static_cast<float *>( heap_caps_malloc( FFT_SIZE * 2 * sizeof( float ), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );
	if ( g_fft_work == nullptr ) {
    	ESP_LOGE( TAG, "Failed to allocate FFT work buffer" );
	    return;
	}

    // ========================================
    // FFTテーブルを初期化
    // ========================================
	esp_err_t fft_ret = dsps_fft2r_init_fc32( nullptr, FFT_SIZE );
	if ( fft_ret != ESP_OK ) {
	    ESP_LOGE( TAG, "FFT init failed: %s", esp_err_to_name( fft_ret ) );
	    return;
	}
	ESP_LOGI( TAG, "8192-point FFT initialized" );

	const size_t history_bytes = HRO_HISTORY_SECONDS * HRO_BIN_COUNT * sizeof( float );
	g_hro_history = static_cast<float *>( heap_caps_malloc( history_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT ) );
	if ( g_hro_history == nullptr ) {
	    ESP_LOGE( TAG, "Failed to allocate HRO history buffer" );
	    return;
	}
	memset( g_hro_history, 0, history_bytes );
	ESP_LOGI(
    	TAG,
    	"HRO history allocated: %d sec x %d bins = %u bytes",
    	HRO_HISTORY_SECONDS,
    	HRO_BIN_COUNT,
    	static_cast<unsigned>( history_bytes ) );
    
    // ========================================
    // FFT task 起動
    // ========================================
    BaseType_t fft_task_ok = xTaskCreate( hro_fft_task, "hro_fft", 8192, nullptr, 4, nullptr );
    if ( fft_task_ok != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create FFT task" );
        return;
    }

    // --------------------------------------------------------
    // esp_rtl_sdr
    // --------------------------------------------------------
    esp_rtl_sdr_config_t cfg;
    esp_rtl_sdr_config_default( &cfg );

    cfg.event_cb = rtl_event_callback;

    // OrcSDRで実績のある設定
    cfg.transfer_bytes = 32768;
    cfg.transfer_count = 3;

    // USB owner task
    cfg.usb_task_core_id = 0;

    // pull ringは不要
    cfg.delivery_mode = ESP_RTL_SDR_DELIVERY_CALLBACK;

    esp_err_t err = esp_rtl_sdr_config_validate( &cfg );

    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "RTL config invalid: %s", esp_rtl_sdr_err_to_name( err ) );
        return;
    }
    
    BaseType_t task_ok = xTaskCreatePinnedToCore(
        hro_dsp_task,
        "hro_dsp",
        4096,
        nullptr,
        5,
        nullptr,
        1);

    if ( task_ok != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create HRO DSP task" );
        return;
    }
    
    err = esp_rtl_sdr_install( &cfg, &g_rtl );

    if ( err != ESP_OK ) {
        ESP_LOGE( TAG, "RTL install failed: %s", esp_rtl_sdr_err_to_name( err ) );
        return;
    }

    ESP_LOGI( TAG, "esp_rtl_sdr installed v%s", esp_rtl_sdr_get_version_string() );

    // RTL制御/DSP側はCore 1
    BaseType_t hro_radio_task_result =
        xTaskCreatePinnedToCore(
            hro_radio_task,
            "hro_radio",
            8192,
            nullptr,
            5,
            nullptr,
            1 );	// CPU1

    if ( hro_radio_task_result != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create hro_radio task" );
        return;
    }
    
    // スペクトルグラフ表示側はCore 0
    BaseType_t hro_display_task_result =
		xTaskCreatePinnedToCore(
    		hro_display_task,
    		"hro_display",
    		8192,
    		nullptr,
    		2,
    		nullptr,
    		0 );	// CPU0    

    if ( hro_display_task_result != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create hro_display task" );
        return;
    }

    // AudioタスクはCore 0
    BaseType_t hro_audio_task_result =
    	xTaskCreatePinnedToCore(
    	hro_audio_task,
    	"hro_audio",
    	4096,
    	nullptr,
    	3,
    	nullptr,
    	0
	);
    
    if ( hro_audio_task_result != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create hro_audio task" );
        return;
    }

	wifi_start_ap( saved_ssid, saved_password );
}

#include "hro_config.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>

#include "esp_log.h"

//------------------------------------------------------------------------------
// Module constants
//------------------------------------------------------------------------------

static const char *TAG = "Tab5-HRO";

const char *const HRO_CONFIG_DIR = "/sdcard/tab5-hro";
const char *const HRO_CONFIG_FILE = "/sdcard/tab5-hro/config.ini";
const char *const HRO_CONFIG_TEMP_FILE = "/sdcard/tab5-hro/config.tmp";

//------------------------------------------------------------------------------
// Global HRO configuration
//------------------------------------------------------------------------------

HroConfig g_hro_config = {
    .observer      = "",
    .location      = "",
    .latitude      = 0.0,
    .longitude     = 0.0,

    .receiver      = "RTL-SDR Blog V4",
    .antenna       = "",

    .frequency_hz  = 53750000,
    .fft_center_hz = 780,
    .fft_range_hz  = 250,
    .level_average_range_hz = 5,

    .audio_volume = 20,
    .audio_mute   = false,

    .screenshot_prefix = "HRO",
};

//------------------------------------------------------------------------------
// trim_string
//
// 文字列の先頭と末尾にある空白文字を取り除く。
// 戻り値は入力バッファ内を指す。
//------------------------------------------------------------------------------
static char *trim_string( char *s )
{
    while ( *s && std::isspace( static_cast<unsigned char>( *s ) ) ) {
        ++s;
    }

    if ( *s == '\0' ) {
        return s;
    }

    char *end = s + strlen( s ) - 1;

    while ( end > s && std::isspace( static_cast<unsigned char>( *end ) ) ) {
        *end-- = '\0';
    }

    return s;
}

//------------------------------------------------------------------------------
// hro_fft_min_hz
//------------------------------------------------------------------------------
int32_t hro_fft_min_hz( void )
{
    return g_hro_config.fft_center_hz - g_hro_config.fft_range_hz;
}

//------------------------------------------------------------------------------
// hro_fft_max_hz
//------------------------------------------------------------------------------
int32_t hro_fft_max_hz( void )
{
    return g_hro_config.fft_center_hz + g_hro_config.fft_range_hz;
}

//------------------------------------------------------------------------------
// hro_fft_bin_count
//
// 現在のFFT表示範囲に必要な1 Hz bin数を返す。
//------------------------------------------------------------------------------
int32_t hro_fft_bin_count( void )
{
    return hro_fft_max_hz() - hro_fft_min_hz() + 1;
}

//------------------------------------------------------------------------------
// is_valid_file_prefix
//
// Screenshotファイル名に使用するprefixを検証する。
// 1～8文字のASCII英数字のみ許可する。
//------------------------------------------------------------------------------
bool is_valid_file_prefix( const char *prefix )
{
    if ( prefix == nullptr ) {
        return false;
    }

    const size_t len = strlen( prefix );

    if ( len < 1 || len > 8 ) {
        return false;
    }

    for ( size_t i = 0; i < len; ++i ) {
        const char c = prefix[i];

        if ( !( ( c >= 'A' && c <= 'Z' ) ||
                ( c >= 'a' && c <= 'z' ) ||
                ( c >= '0' && c <= '9' ) ) ) {
            return false;
        }
    }

    return true;
}

//------------------------------------------------------------------------------
// validate_hro_config
//
// HRO設定値が現在のDSP・表示条件の範囲内か検証する。
//------------------------------------------------------------------------------
bool validate_hro_config( const HroConfig &cfg, char *error_msg, size_t error_msg_size )
{
    auto set_error = [&]( const char *msg ) {
        if ( error_msg && error_msg_size > 0 ) {
            snprintf( error_msg, error_msg_size, "%s", msg );
        }

        return false;
    };

    // Station position
    if ( cfg.latitude < -90.0 || cfg.latitude > 90.0 ) {
        return set_error( "Latitude must be between -90 and +90 degrees" );
    }

    if ( cfg.longitude < -180.0 || cfg.longitude > 180.0 ) {
        return set_error( "Longitude must be between -180 and +180 degrees" );
    }

    // Receiving frequency
    if ( cfg.frequency_hz < 1000000UL || cfg.frequency_hz > 2000000000UL ) {
        return set_error( "Receiving frequency must be between 1 MHz and 2000 MHz" );
    }

    // FFT settings
    if ( cfg.fft_center_hz < 0 ) {
        return set_error( "FFT center frequency must be 0 Hz or greater" );
    }

    if ( cfg.fft_range_hz < 1 ) {
        return set_error( "FFT display range must be 1 Hz or greater" );
    }

    const int64_t fft_min = static_cast<int64_t>( cfg.fft_center_hz ) - static_cast<int64_t>( cfg.fft_range_hz );
    const int64_t fft_max = static_cast<int64_t>( cfg.fft_center_hz ) + static_cast<int64_t>( cfg.fft_range_hz );

    if ( fft_min < 0 ) {
        return set_error( "FFT display lower frequency must be 0 Hz or greater" );
    }

    // 現在のresampler LPFは約1800 Hz
    if ( fft_max > 1800 ) {
        return set_error( "FFT display upper frequency must not exceed 1800 Hz" );
    }

    // 現在は1 Hz/bin
    const int64_t bin_count = fft_max - fft_min + 1;

    if ( bin_count < 1 || bin_count > 3601 ) {
        return set_error( "Invalid FFT display bin count" );
    }

    // Level Graph範囲
    if ( cfg.level_average_range_hz < 0 || cfg.level_average_range_hz > cfg.fft_range_hz ) {
        return set_error( "Level Average Range must be between 0 and FFT Range." );
    }

    // Screenshot settings
    if ( !is_valid_file_prefix( cfg.screenshot_prefix ) ) {
        return set_error( "Screenshot File Prefix must be 1 to 8 alphanumeric characters" );
    }

    if ( error_msg && error_msg_size > 0 ) {
        error_msg[0] = '\0';
    }

    return true;
}

//------------------------------------------------------------------------------
// write_hro_config_file
//
// 指定されたHroConfigをINI形式で保存する。
//------------------------------------------------------------------------------
bool write_hro_config_file( const char *filename, const HroConfig &cfg )
{
    FILE *fp = fopen( filename, "w" );
    if ( !fp ) {
        ESP_LOGE( TAG, "Cannot open config file: %s", filename );
        return false;
    }

    fprintf( fp, "[station]\n" );
    fprintf( fp, "observer=%s\n", cfg.observer );
    fprintf( fp, "location=%s\n", cfg.location );
    fprintf( fp, "latitude=%.6f\n", cfg.latitude );
    fprintf( fp, "longitude=%.6f\n", cfg.longitude );
    fprintf( fp, "\n" );

    fprintf( fp, "[receiver]\n" );
    fprintf( fp, "receiver=%s\n", cfg.receiver );
    fprintf( fp, "frequency_hz=%lu\n", static_cast<unsigned long>( cfg.frequency_hz ) );
    fprintf( fp, "fft_center_hz=%ld\n", static_cast<long>( cfg.fft_center_hz ) );
    fprintf( fp, "fft_range_hz=%ld\n", static_cast<long>( cfg.fft_range_hz ) );
    fprintf( fp, "antenna=%s\n", cfg.antenna );
    fprintf( fp, "level_average_range_hz=%ld\n", static_cast<long>( cfg.level_average_range_hz ) );
    fprintf( fp, "\n" );

    fprintf( fp, "[audio]\n" );
    fprintf( fp, "volume=%d\n", cfg.audio_volume );
    fprintf( fp, "mute=%d\n", cfg.audio_mute ? 1 : 0 );
    fprintf( fp, "\n" );

    fprintf( fp, "[screenshot]\n" );
    fprintf( fp, "prefix=%s\n", cfg.screenshot_prefix );

    fflush( fp );
    fclose( fp );

    return true;
}

//------------------------------------------------------------------------------
// save_hro_config
//
// 現在のg_hro_configをSDカードへ保存する。
//------------------------------------------------------------------------------
bool save_hro_config( void )
{
    mkdir( HRO_CONFIG_DIR, 0775 );

    if ( !write_hro_config_file( HRO_CONFIG_FILE, g_hro_config ) ) {
        ESP_LOGE( TAG, "Failed to save HRO config: %s", HRO_CONFIG_FILE );
        return false;
    }
    ESP_LOGI( TAG, "HRO config saved: %s", HRO_CONFIG_FILE );

    return true;
}

//------------------------------------------------------------------------------
// load_hro_config
//
// SDカード上のconfig.iniを読み込み、g_hro_configへ反映する。
//------------------------------------------------------------------------------
bool load_hro_config( void )
{
    FILE *fp = fopen( HRO_CONFIG_FILE, "r" );

    if ( !fp ) {
        ESP_LOGW( TAG, "HRO config not found: %s", HRO_CONFIG_FILE );
        return false;
    }

    char line[256];
    char section[32] = "";

    while ( fgets( line, sizeof( line ), fp ) ) {
        char *p = trim_string( line );

        // 空行・コメント
        if ( *p == '\0' ||
             *p == '#' ||
             *p == ';' ) {
            continue;
        }

        // [section]
        if ( *p == '[' ) {
            char *end = strchr( p, ']' );

            if ( end ) {
                *end = '\0';
                snprintf( section, sizeof( section ), "%s", p + 1 );
            }
            continue;
        }

        // key=value
        char *eq = strchr( p, '=' );

        if ( !eq ) {
            continue;
        }

        *eq = '\0';

        char *key = trim_string( p );

        char *value = trim_string( eq + 1 );

        if ( strcmp( section, "station" ) == 0 ) {
            if ( strcmp( key, "observer" ) == 0 ) {
                snprintf( g_hro_config.observer, sizeof( g_hro_config.observer ), "%s", value );
            } else if ( strcmp( key, "location" ) == 0 ) {
                snprintf( g_hro_config.location, sizeof( g_hro_config.location ), "%s", value );
            } else if ( strcmp( key, "latitude" ) == 0 ) {
                g_hro_config.latitude = strtod( value, nullptr );
            } else if ( strcmp( key, "longitude" ) == 0 ) {
                g_hro_config.longitude = strtod( value, nullptr );
            }
        } else if ( strcmp( section, "receiver" ) == 0 ) {
            if ( strcmp( key, "receiver" ) == 0 ) {
                snprintf( g_hro_config.receiver, sizeof( g_hro_config.receiver ), "%s", value );
            } else if ( strcmp( key, "antenna" ) == 0 ) {
                snprintf( g_hro_config.antenna, sizeof( g_hro_config.antenna ), "%s", value );
            } else if ( strcmp( key, "frequency_hz" ) == 0 ) {
                g_hro_config.frequency_hz = static_cast<uint32_t>( strtoul( value, nullptr, 10 ) );
            } else if ( strcmp( key, "fft_center_hz" ) == 0 ) {
                g_hro_config.fft_center_hz = static_cast<int32_t>( strtol( value, nullptr, 10 ) );
            } else if ( strcmp( key, "fft_range_hz" ) == 0 ) {
                g_hro_config.fft_range_hz = static_cast<int32_t>( strtol( value, nullptr, 10 ) );
            } else if ( strcmp( key, "level_average_range_hz" ) == 0 ) {
                g_hro_config.level_average_range_hz = atoi( value );
            }
        } else if ( strcmp( section, "audio" ) == 0 ) {
            if ( strcmp( key, "volume" ) == 0 ) {
                g_hro_config.audio_volume = atoi( value );
            } else if ( strcmp( key, "mute" ) == 0 ) {
                g_hro_config.audio_mute = ( atoi( value ) != 0 );
            }
        } else if ( strcmp( section, "screenshot" ) == 0 ) {
            if ( strcmp( key, "prefix" ) == 0 ) {
                snprintf( g_hro_config.screenshot_prefix, sizeof( g_hro_config.screenshot_prefix ), "%s", value );
            }
        }
    }

    fclose( fp );

    ESP_LOGI( TAG, "HRO config loaded" );

    char error_msg[128];

    if ( !validate_hro_config( g_hro_config, error_msg, sizeof( error_msg ) ) ) {
        ESP_LOGE( TAG, "Invalid HRO config: %s", error_msg );
        return false;
    }

    ESP_LOGI( TAG, "HRO config validation OK" );
    ESP_LOGI( TAG, "Station: observer='%s' location='%s'", g_hro_config.observer, g_hro_config.location );
    ESP_LOGI( TAG, "Position: lat=%.6f lon=%.6f", g_hro_config.latitude, g_hro_config.longitude );
    ESP_LOGI( TAG, "Receiver: '%s' antenna='%s'", g_hro_config.receiver, g_hro_config.antenna );
    ESP_LOGI( TAG, "HRO: RF=%lu Hz FFT center=%ld Hz range=+/- %ld Hz", static_cast<unsigned long>( g_hro_config.frequency_hz ), static_cast<long>( g_hro_config.fft_center_hz ), static_cast<long>( g_hro_config.fft_range_hz ) );
    ESP_LOGI( TAG, "FFT display: %ld .. %ld Hz (%ld bins)", static_cast<long>( hro_fft_min_hz() ), static_cast<long>( hro_fft_max_hz() ), static_cast<long>( hro_fft_bin_count() ) );
    ESP_LOGI( TAG, "Screenshot: prefix='%s'", g_hro_config.screenshot_prefix );

    return true;
}

//------------------------------------------------------------------------------
// ensure_hro_config
//
// config.iniが存在すれば読み込む。
// 存在しなければデフォルト設定で作成してから読み込む。
//------------------------------------------------------------------------------
bool ensure_hro_config( void )
{
    FILE *fp = fopen( HRO_CONFIG_FILE, "r" );
    if ( fp ) {
        fclose( fp );

        ESP_LOGI( TAG, "HRO config found: %s", HRO_CONFIG_FILE );
        return load_hro_config();
    }

    ESP_LOGI( TAG, "HRO config not found, creating default" );
    if ( !save_hro_config() ) {
        return false;
    }

    return load_hro_config();
}

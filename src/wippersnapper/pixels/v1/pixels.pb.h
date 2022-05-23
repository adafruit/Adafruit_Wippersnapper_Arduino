/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5-dev at Mon May 23 12:17:12 2022. */

#ifndef PB_WIPPERSNAPPER_PIXELS_V1_WIPPERSNAPPER_PIXELS_V1_PIXELS_PB_H_INCLUDED
#define PB_WIPPERSNAPPER_PIXELS_V1_WIPPERSNAPPER_PIXELS_V1_PIXELS_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _wippersnapper_pixels_v1_NeoPixelType {
    wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_UNSPECIFIED = 0,
    wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_RGB = 1,
    wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_GRB = 2,
    wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_RGBW = 3
} wippersnapper_pixels_v1_NeoPixelType;

typedef enum _wippersnapper_pixels_v1_NeoPixelSpeed {
    wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_UNSPECIFIED = 0,
    wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_KHZ800 = 1,
    wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_KHZ400 = 2
} wippersnapper_pixels_v1_NeoPixelSpeed;

/* Struct definitions */
typedef struct _wippersnapper_pixels_v1_DotStarConfig {
    uint32_t pixel_num;
    bool use_hardware_spi;
    uint32_t pin_data;
    uint32_t pin_clock;
} wippersnapper_pixels_v1_DotStarConfig;

typedef struct _wippersnapper_pixels_v1_NeoPixelConfig {
    uint32_t neo_pixel_pin;
    wippersnapper_pixels_v1_NeoPixelType neo_pixel_type;
    wippersnapper_pixels_v1_NeoPixelSpeed neo_pixel_speed;
} wippersnapper_pixels_v1_NeoPixelConfig;

typedef struct _wippersnapper_pixels_v1_PixelsCreate {
    uint32_t pixel_num;
    uint32_t pixel_brightness;
    bool has_neo_pixel_init;
    wippersnapper_pixels_v1_NeoPixelConfig neo_pixel_init;
    bool has_dot_star_init;
    wippersnapper_pixels_v1_DotStarConfig dot_star_init;
} wippersnapper_pixels_v1_PixelsCreate;

typedef struct _wippersnapper_pixels_v1_PixelsDelete {
    bool has_neo_pixel_config;
    wippersnapper_pixels_v1_NeoPixelConfig neo_pixel_config;
    bool has_dot_star_config;
    wippersnapper_pixels_v1_DotStarConfig dot_star_config;
} wippersnapper_pixels_v1_PixelsDelete;

typedef struct _wippersnapper_pixels_v1_PixelsFillAll {
    uint32_t color;
    bool has_neo_pixel_config;
    wippersnapper_pixels_v1_NeoPixelConfig neo_pixel_config;
    bool has_dot_star_config;
    wippersnapper_pixels_v1_DotStarConfig dot_star_config;
} wippersnapper_pixels_v1_PixelsFillAll;

typedef struct _wippersnapper_pixels_v1_PixelsUpdate {
    uint32_t pixel_brightness;
    bool has_neo_pixel_config;
    wippersnapper_pixels_v1_NeoPixelConfig neo_pixel_config;
    bool has_dot_star_config;
    wippersnapper_pixels_v1_DotStarConfig dot_star_config;
} wippersnapper_pixels_v1_PixelsUpdate;


/* Helper constants for enums */
#define _wippersnapper_pixels_v1_NeoPixelType_MIN wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_UNSPECIFIED
#define _wippersnapper_pixels_v1_NeoPixelType_MAX wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_RGBW
#define _wippersnapper_pixels_v1_NeoPixelType_ARRAYSIZE ((wippersnapper_pixels_v1_NeoPixelType)(wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_RGBW+1))

#define _wippersnapper_pixels_v1_NeoPixelSpeed_MIN wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_UNSPECIFIED
#define _wippersnapper_pixels_v1_NeoPixelSpeed_MAX wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_KHZ400
#define _wippersnapper_pixels_v1_NeoPixelSpeed_ARRAYSIZE ((wippersnapper_pixels_v1_NeoPixelSpeed)(wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_KHZ400+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define wippersnapper_pixels_v1_PixelsCreate_init_default {0, 0, false, wippersnapper_pixels_v1_NeoPixelConfig_init_default, false, wippersnapper_pixels_v1_DotStarConfig_init_default}
#define wippersnapper_pixels_v1_PixelsUpdate_init_default {0, false, wippersnapper_pixels_v1_NeoPixelConfig_init_default, false, wippersnapper_pixels_v1_DotStarConfig_init_default}
#define wippersnapper_pixels_v1_PixelsDelete_init_default {false, wippersnapper_pixels_v1_NeoPixelConfig_init_default, false, wippersnapper_pixels_v1_DotStarConfig_init_default}
#define wippersnapper_pixels_v1_PixelsFillAll_init_default {0, false, wippersnapper_pixels_v1_NeoPixelConfig_init_default, false, wippersnapper_pixels_v1_DotStarConfig_init_default}
#define wippersnapper_pixels_v1_DotStarConfig_init_default {0, 0, 0, 0}
#define wippersnapper_pixels_v1_NeoPixelConfig_init_default {0, _wippersnapper_pixels_v1_NeoPixelType_MIN, _wippersnapper_pixels_v1_NeoPixelSpeed_MIN}
#define wippersnapper_pixels_v1_PixelsCreate_init_zero {0, 0, false, wippersnapper_pixels_v1_NeoPixelConfig_init_zero, false, wippersnapper_pixels_v1_DotStarConfig_init_zero}
#define wippersnapper_pixels_v1_PixelsUpdate_init_zero {0, false, wippersnapper_pixels_v1_NeoPixelConfig_init_zero, false, wippersnapper_pixels_v1_DotStarConfig_init_zero}
#define wippersnapper_pixels_v1_PixelsDelete_init_zero {false, wippersnapper_pixels_v1_NeoPixelConfig_init_zero, false, wippersnapper_pixels_v1_DotStarConfig_init_zero}
#define wippersnapper_pixels_v1_PixelsFillAll_init_zero {0, false, wippersnapper_pixels_v1_NeoPixelConfig_init_zero, false, wippersnapper_pixels_v1_DotStarConfig_init_zero}
#define wippersnapper_pixels_v1_DotStarConfig_init_zero {0, 0, 0, 0}
#define wippersnapper_pixels_v1_NeoPixelConfig_init_zero {0, _wippersnapper_pixels_v1_NeoPixelType_MIN, _wippersnapper_pixels_v1_NeoPixelSpeed_MIN}

/* Field tags (for use in manual encoding/decoding) */
#define wippersnapper_pixels_v1_DotStarConfig_pixel_num_tag 1
#define wippersnapper_pixels_v1_DotStarConfig_use_hardware_spi_tag 2
#define wippersnapper_pixels_v1_DotStarConfig_pin_data_tag 3
#define wippersnapper_pixels_v1_DotStarConfig_pin_clock_tag 4
#define wippersnapper_pixels_v1_NeoPixelConfig_neo_pixel_pin_tag 1
#define wippersnapper_pixels_v1_NeoPixelConfig_neo_pixel_type_tag 2
#define wippersnapper_pixels_v1_NeoPixelConfig_neo_pixel_speed_tag 3
#define wippersnapper_pixels_v1_PixelsCreate_pixel_num_tag 1
#define wippersnapper_pixels_v1_PixelsCreate_pixel_brightness_tag 2
#define wippersnapper_pixels_v1_PixelsCreate_neo_pixel_init_tag 3
#define wippersnapper_pixels_v1_PixelsCreate_dot_star_init_tag 4
#define wippersnapper_pixels_v1_PixelsDelete_neo_pixel_config_tag 1
#define wippersnapper_pixels_v1_PixelsDelete_dot_star_config_tag 2
#define wippersnapper_pixels_v1_PixelsFillAll_color_tag 1
#define wippersnapper_pixels_v1_PixelsFillAll_neo_pixel_config_tag 2
#define wippersnapper_pixels_v1_PixelsFillAll_dot_star_config_tag 3
#define wippersnapper_pixels_v1_PixelsUpdate_pixel_brightness_tag 1
#define wippersnapper_pixels_v1_PixelsUpdate_neo_pixel_config_tag 2
#define wippersnapper_pixels_v1_PixelsUpdate_dot_star_config_tag 3

/* Struct field encoding specification for nanopb */
#define wippersnapper_pixels_v1_PixelsCreate_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   pixel_num,         1) \
X(a, STATIC,   SINGULAR, UINT32,   pixel_brightness,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  neo_pixel_init,    3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  dot_star_init,     4)
#define wippersnapper_pixels_v1_PixelsCreate_CALLBACK NULL
#define wippersnapper_pixels_v1_PixelsCreate_DEFAULT NULL
#define wippersnapper_pixels_v1_PixelsCreate_neo_pixel_init_MSGTYPE wippersnapper_pixels_v1_NeoPixelConfig
#define wippersnapper_pixels_v1_PixelsCreate_dot_star_init_MSGTYPE wippersnapper_pixels_v1_DotStarConfig

#define wippersnapper_pixels_v1_PixelsUpdate_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   pixel_brightness,   1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  neo_pixel_config,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  dot_star_config,   3)
#define wippersnapper_pixels_v1_PixelsUpdate_CALLBACK NULL
#define wippersnapper_pixels_v1_PixelsUpdate_DEFAULT NULL
#define wippersnapper_pixels_v1_PixelsUpdate_neo_pixel_config_MSGTYPE wippersnapper_pixels_v1_NeoPixelConfig
#define wippersnapper_pixels_v1_PixelsUpdate_dot_star_config_MSGTYPE wippersnapper_pixels_v1_DotStarConfig

#define wippersnapper_pixels_v1_PixelsDelete_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  neo_pixel_config,   1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  dot_star_config,   2)
#define wippersnapper_pixels_v1_PixelsDelete_CALLBACK NULL
#define wippersnapper_pixels_v1_PixelsDelete_DEFAULT NULL
#define wippersnapper_pixels_v1_PixelsDelete_neo_pixel_config_MSGTYPE wippersnapper_pixels_v1_NeoPixelConfig
#define wippersnapper_pixels_v1_PixelsDelete_dot_star_config_MSGTYPE wippersnapper_pixels_v1_DotStarConfig

#define wippersnapper_pixels_v1_PixelsFillAll_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   color,             1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  neo_pixel_config,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  dot_star_config,   3)
#define wippersnapper_pixels_v1_PixelsFillAll_CALLBACK NULL
#define wippersnapper_pixels_v1_PixelsFillAll_DEFAULT NULL
#define wippersnapper_pixels_v1_PixelsFillAll_neo_pixel_config_MSGTYPE wippersnapper_pixels_v1_NeoPixelConfig
#define wippersnapper_pixels_v1_PixelsFillAll_dot_star_config_MSGTYPE wippersnapper_pixels_v1_DotStarConfig

#define wippersnapper_pixels_v1_DotStarConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   pixel_num,         1) \
X(a, STATIC,   SINGULAR, BOOL,     use_hardware_spi,   2) \
X(a, STATIC,   SINGULAR, UINT32,   pin_data,          3) \
X(a, STATIC,   SINGULAR, UINT32,   pin_clock,         4)
#define wippersnapper_pixels_v1_DotStarConfig_CALLBACK NULL
#define wippersnapper_pixels_v1_DotStarConfig_DEFAULT NULL

#define wippersnapper_pixels_v1_NeoPixelConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   neo_pixel_pin,     1) \
X(a, STATIC,   SINGULAR, UENUM,    neo_pixel_type,    2) \
X(a, STATIC,   SINGULAR, UENUM,    neo_pixel_speed,   3)
#define wippersnapper_pixels_v1_NeoPixelConfig_CALLBACK NULL
#define wippersnapper_pixels_v1_NeoPixelConfig_DEFAULT NULL

extern const pb_msgdesc_t wippersnapper_pixels_v1_PixelsCreate_msg;
extern const pb_msgdesc_t wippersnapper_pixels_v1_PixelsUpdate_msg;
extern const pb_msgdesc_t wippersnapper_pixels_v1_PixelsDelete_msg;
extern const pb_msgdesc_t wippersnapper_pixels_v1_PixelsFillAll_msg;
extern const pb_msgdesc_t wippersnapper_pixels_v1_DotStarConfig_msg;
extern const pb_msgdesc_t wippersnapper_pixels_v1_NeoPixelConfig_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define wippersnapper_pixels_v1_PixelsCreate_fields &wippersnapper_pixels_v1_PixelsCreate_msg
#define wippersnapper_pixels_v1_PixelsUpdate_fields &wippersnapper_pixels_v1_PixelsUpdate_msg
#define wippersnapper_pixels_v1_PixelsDelete_fields &wippersnapper_pixels_v1_PixelsDelete_msg
#define wippersnapper_pixels_v1_PixelsFillAll_fields &wippersnapper_pixels_v1_PixelsFillAll_msg
#define wippersnapper_pixels_v1_DotStarConfig_fields &wippersnapper_pixels_v1_DotStarConfig_msg
#define wippersnapper_pixels_v1_NeoPixelConfig_fields &wippersnapper_pixels_v1_NeoPixelConfig_msg

/* Maximum encoded size of messages (where known) */
#define wippersnapper_pixels_v1_PixelsCreate_size 46
#define wippersnapper_pixels_v1_PixelsUpdate_size 40
#define wippersnapper_pixels_v1_PixelsDelete_size 34
#define wippersnapper_pixels_v1_PixelsFillAll_size 40
#define wippersnapper_pixels_v1_DotStarConfig_size 20
#define wippersnapper_pixels_v1_NeoPixelConfig_size 10

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

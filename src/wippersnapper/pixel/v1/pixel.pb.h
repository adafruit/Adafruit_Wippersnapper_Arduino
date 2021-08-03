/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5-dev at Tue Aug  3 19:48:47 2021. */

#ifndef PB_WIPPERSNAPPER_PIXEL_V1_WIPPERSNAPPER_PIXEL_V1_PIXEL_PB_H_INCLUDED
#define PB_WIPPERSNAPPER_PIXEL_V1_WIPPERSNAPPER_PIXEL_V1_PIXEL_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _wippersnapper_pixel_v1_ConfigurePixels_PixelType {
    wippersnapper_pixel_v1_ConfigurePixels_PixelType_PIXEL_TYPE_UNSPECIFIED = 0,
    wippersnapper_pixel_v1_ConfigurePixels_PixelType_PIXEL_TYPE_WS2812 = 1,
    wippersnapper_pixel_v1_ConfigurePixels_PixelType_PIXEL_TYPE_APA201 = 2
} wippersnapper_pixel_v1_ConfigurePixels_PixelType;

typedef enum _wippersnapper_pixel_v1_ConfigurePixels_PixelOrder {
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_UNSPECIFIED = 0,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_NEOPIXEL_RGB = 1,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_NEOPIXEL_RBG = 2,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_NEOPIXEL_RGBW = 3,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_NEOPIXEL_GRB = 4,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_NEOPIXEL_GRBW = 5,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_RGB = 6,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_RBG = 7,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_GRB = 8,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_GBR = 9,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_BGR = 10,
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_BRG = 11
} wippersnapper_pixel_v1_ConfigurePixels_PixelOrder;

/* Struct definitions */
typedef struct _wippersnapper_pixel_v1_ConfigurePixels {
    pb_callback_t pixel_pin;
    wippersnapper_pixel_v1_ConfigurePixels_PixelType pixel_type;
    wippersnapper_pixel_v1_ConfigurePixels_PixelOrder pixel_order;
    uint32_t pixel_number;
} wippersnapper_pixel_v1_ConfigurePixels;

typedef struct _wippersnapper_pixel_v1_PixelEvent {
    uint32_t color;
    uint32_t idx_first_pixel;
    uint32_t pixel_count;
} wippersnapper_pixel_v1_PixelEvent;


/* Helper constants for enums */
#define _wippersnapper_pixel_v1_ConfigurePixels_PixelType_MIN wippersnapper_pixel_v1_ConfigurePixels_PixelType_PIXEL_TYPE_UNSPECIFIED
#define _wippersnapper_pixel_v1_ConfigurePixels_PixelType_MAX wippersnapper_pixel_v1_ConfigurePixels_PixelType_PIXEL_TYPE_APA201
#define _wippersnapper_pixel_v1_ConfigurePixels_PixelType_ARRAYSIZE ((wippersnapper_pixel_v1_ConfigurePixels_PixelType)(wippersnapper_pixel_v1_ConfigurePixels_PixelType_PIXEL_TYPE_APA201+1))

#define _wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_MIN wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_UNSPECIFIED
#define _wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_MAX wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_BRG
#define _wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_ARRAYSIZE ((wippersnapper_pixel_v1_ConfigurePixels_PixelOrder)(wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_PIXEL_ORDER_DOTSTAR_BRG+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define wippersnapper_pixel_v1_ConfigurePixels_init_default {{{NULL}, NULL}, _wippersnapper_pixel_v1_ConfigurePixels_PixelType_MIN, _wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_MIN, 0}
#define wippersnapper_pixel_v1_PixelEvent_init_default {0, 0, 0}
#define wippersnapper_pixel_v1_ConfigurePixels_init_zero {{{NULL}, NULL}, _wippersnapper_pixel_v1_ConfigurePixels_PixelType_MIN, _wippersnapper_pixel_v1_ConfigurePixels_PixelOrder_MIN, 0}
#define wippersnapper_pixel_v1_PixelEvent_init_zero {0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define wippersnapper_pixel_v1_ConfigurePixels_pixel_pin_tag 1
#define wippersnapper_pixel_v1_ConfigurePixels_pixel_type_tag 2
#define wippersnapper_pixel_v1_ConfigurePixels_pixel_order_tag 3
#define wippersnapper_pixel_v1_ConfigurePixels_pixel_number_tag 4
#define wippersnapper_pixel_v1_PixelEvent_color_tag 1
#define wippersnapper_pixel_v1_PixelEvent_idx_first_pixel_tag 2
#define wippersnapper_pixel_v1_PixelEvent_pixel_count_tag 3

/* Struct field encoding specification for nanopb */
#define wippersnapper_pixel_v1_ConfigurePixels_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   pixel_pin,         1) \
X(a, STATIC,   SINGULAR, UENUM,    pixel_type,        2) \
X(a, STATIC,   SINGULAR, UENUM,    pixel_order,       3) \
X(a, STATIC,   SINGULAR, UINT32,   pixel_number,      4)
#define wippersnapper_pixel_v1_ConfigurePixels_CALLBACK pb_default_field_callback
#define wippersnapper_pixel_v1_ConfigurePixels_DEFAULT NULL

#define wippersnapper_pixel_v1_PixelEvent_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   color,             1) \
X(a, STATIC,   SINGULAR, UINT32,   idx_first_pixel,   2) \
X(a, STATIC,   SINGULAR, UINT32,   pixel_count,       3)
#define wippersnapper_pixel_v1_PixelEvent_CALLBACK NULL
#define wippersnapper_pixel_v1_PixelEvent_DEFAULT NULL

extern const pb_msgdesc_t wippersnapper_pixel_v1_ConfigurePixels_msg;
extern const pb_msgdesc_t wippersnapper_pixel_v1_PixelEvent_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define wippersnapper_pixel_v1_ConfigurePixels_fields &wippersnapper_pixel_v1_ConfigurePixels_msg
#define wippersnapper_pixel_v1_PixelEvent_fields &wippersnapper_pixel_v1_PixelEvent_msg

/* Maximum encoded size of messages (where known) */
/* wippersnapper_pixel_v1_ConfigurePixels_size depends on runtime parameters */
#define wippersnapper_pixel_v1_PixelEvent_size   18

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

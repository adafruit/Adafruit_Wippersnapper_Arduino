/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8 at Tue Aug 20 16:38:43 2024. */

#ifndef PB_WIPPERSNAPPER_DIGITALIO_DIGITALIO_PB_H_INCLUDED
#define PB_WIPPERSNAPPER_DIGITALIO_DIGITALIO_PB_H_INCLUDED
#include <pb.h>
#include "sensor.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* *
 DigitalIOSampleMode specifies the pin's sample mode. */
typedef enum _wippersnapper_digitalio_DigitalIOSampleMode {
    wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_UNSPECIFIED = 0, /* * Invalid Sample Mode from Broker. */
    wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_TIMER = 1, /* * Periodically sample the pin's value. */
    wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_EVENT = 2 /* * Sample the pin's value when an event occurs. */
} wippersnapper_digitalio_DigitalIOSampleMode;

/* *
 DigitalIODirection specifies the pin's direction, INPUT/INPUT_PULL_UP/OUTPUT. */
typedef enum _wippersnapper_digitalio_DigitalIODirection {
    wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_UNSPECIFIED = 0, /* * Invalid Direction from Broker. */
    wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT = 1, /* * Set the pin to behave as an input. */
    wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT_PULL_UP = 2, /* * Set the pin to behave as an input. */
    wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_OUTPUT = 3 /* * Set the pin to behave as an output. */
} wippersnapper_digitalio_DigitalIODirection;

/* Struct definitions */
/* *
 DigitalIOAdd adds a digital pin to the device. */
typedef struct _wippersnapper_digitalio_DigitalIOAdd {
    pb_callback_t pin_name; /* * The pin's name. */
    wippersnapper_digitalio_DigitalIODirection gpio_direction; /* * The pin's direction. */
    wippersnapper_digitalio_DigitalIOSampleMode sample_mode; /* * Specifies the pin's sample mode. */
    float period; /* * Time between measurements in seconds, if MODE_TIMER. */
    bool value; /* * Re-sync only - send the pin's value. */
} wippersnapper_digitalio_DigitalIOAdd;

/* *
 DigitalIORemove removes a digital pin from the device. */
typedef struct _wippersnapper_digitalio_DigitalIORemove {
    pb_callback_t pin_name; /* * The pin's name. */
} wippersnapper_digitalio_DigitalIORemove;

/* *
 DigitalIOEvent is sent from the device to the broker when a digital pin's value changes. */
typedef struct _wippersnapper_digitalio_DigitalIOEvent {
    pb_callback_t pin_name; /* * The pin's name. */
    bool has_value;
    wippersnapper_sensor_SensorEvent value; /* * The pin's value. */
} wippersnapper_digitalio_DigitalIOEvent;

/* *
 DigitalIOWrite writes a boolean value to a digital pin. */
typedef struct _wippersnapper_digitalio_DigitalIOWrite {
    pb_callback_t pin_name; /* * The pin's name. */
    bool has_value;
    wippersnapper_sensor_SensorEvent value; /* * The pin's value. */
} wippersnapper_digitalio_DigitalIOWrite;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _wippersnapper_digitalio_DigitalIOSampleMode_MIN wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_UNSPECIFIED
#define _wippersnapper_digitalio_DigitalIOSampleMode_MAX wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_EVENT
#define _wippersnapper_digitalio_DigitalIOSampleMode_ARRAYSIZE ((wippersnapper_digitalio_DigitalIOSampleMode)(wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_EVENT+1))

#define _wippersnapper_digitalio_DigitalIODirection_MIN wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_UNSPECIFIED
#define _wippersnapper_digitalio_DigitalIODirection_MAX wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_OUTPUT
#define _wippersnapper_digitalio_DigitalIODirection_ARRAYSIZE ((wippersnapper_digitalio_DigitalIODirection)(wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_OUTPUT+1))

#define wippersnapper_digitalio_DigitalIOAdd_gpio_direction_ENUMTYPE wippersnapper_digitalio_DigitalIODirection
#define wippersnapper_digitalio_DigitalIOAdd_sample_mode_ENUMTYPE wippersnapper_digitalio_DigitalIOSampleMode





/* Initializer values for message structs */
#define wippersnapper_digitalio_DigitalIOAdd_init_default {{{NULL}, NULL}, _wippersnapper_digitalio_DigitalIODirection_MIN, _wippersnapper_digitalio_DigitalIOSampleMode_MIN, 0, 0}
#define wippersnapper_digitalio_DigitalIORemove_init_default {{{NULL}, NULL}}
#define wippersnapper_digitalio_DigitalIOEvent_init_default {{{NULL}, NULL}, false, wippersnapper_sensor_SensorEvent_init_default}
#define wippersnapper_digitalio_DigitalIOWrite_init_default {{{NULL}, NULL}, false, wippersnapper_sensor_SensorEvent_init_default}
#define wippersnapper_digitalio_DigitalIOAdd_init_zero {{{NULL}, NULL}, _wippersnapper_digitalio_DigitalIODirection_MIN, _wippersnapper_digitalio_DigitalIOSampleMode_MIN, 0, 0}
#define wippersnapper_digitalio_DigitalIORemove_init_zero {{{NULL}, NULL}}
#define wippersnapper_digitalio_DigitalIOEvent_init_zero {{{NULL}, NULL}, false, wippersnapper_sensor_SensorEvent_init_zero}
#define wippersnapper_digitalio_DigitalIOWrite_init_zero {{{NULL}, NULL}, false, wippersnapper_sensor_SensorEvent_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define wippersnapper_digitalio_DigitalIOAdd_pin_name_tag 1
#define wippersnapper_digitalio_DigitalIOAdd_gpio_direction_tag 2
#define wippersnapper_digitalio_DigitalIOAdd_sample_mode_tag 3
#define wippersnapper_digitalio_DigitalIOAdd_period_tag 4
#define wippersnapper_digitalio_DigitalIOAdd_value_tag 5
#define wippersnapper_digitalio_DigitalIORemove_pin_name_tag 1
#define wippersnapper_digitalio_DigitalIOEvent_pin_name_tag 1
#define wippersnapper_digitalio_DigitalIOEvent_value_tag 2
#define wippersnapper_digitalio_DigitalIOWrite_pin_name_tag 1
#define wippersnapper_digitalio_DigitalIOWrite_value_tag 2

/* Struct field encoding specification for nanopb */
#define wippersnapper_digitalio_DigitalIOAdd_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   pin_name,          1) \
X(a, STATIC,   SINGULAR, UENUM,    gpio_direction,    2) \
X(a, STATIC,   SINGULAR, UENUM,    sample_mode,       3) \
X(a, STATIC,   SINGULAR, FLOAT,    period,            4) \
X(a, STATIC,   SINGULAR, BOOL,     value,             5)
#define wippersnapper_digitalio_DigitalIOAdd_CALLBACK pb_default_field_callback
#define wippersnapper_digitalio_DigitalIOAdd_DEFAULT NULL

#define wippersnapper_digitalio_DigitalIORemove_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   pin_name,          1)
#define wippersnapper_digitalio_DigitalIORemove_CALLBACK pb_default_field_callback
#define wippersnapper_digitalio_DigitalIORemove_DEFAULT NULL

#define wippersnapper_digitalio_DigitalIOEvent_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   pin_name,          1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  value,             2)
#define wippersnapper_digitalio_DigitalIOEvent_CALLBACK pb_default_field_callback
#define wippersnapper_digitalio_DigitalIOEvent_DEFAULT NULL
#define wippersnapper_digitalio_DigitalIOEvent_value_MSGTYPE wippersnapper_sensor_SensorEvent

#define wippersnapper_digitalio_DigitalIOWrite_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   pin_name,          1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  value,             2)
#define wippersnapper_digitalio_DigitalIOWrite_CALLBACK pb_default_field_callback
#define wippersnapper_digitalio_DigitalIOWrite_DEFAULT NULL
#define wippersnapper_digitalio_DigitalIOWrite_value_MSGTYPE wippersnapper_sensor_SensorEvent

extern const pb_msgdesc_t wippersnapper_digitalio_DigitalIOAdd_msg;
extern const pb_msgdesc_t wippersnapper_digitalio_DigitalIORemove_msg;
extern const pb_msgdesc_t wippersnapper_digitalio_DigitalIOEvent_msg;
extern const pb_msgdesc_t wippersnapper_digitalio_DigitalIOWrite_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define wippersnapper_digitalio_DigitalIOAdd_fields &wippersnapper_digitalio_DigitalIOAdd_msg
#define wippersnapper_digitalio_DigitalIORemove_fields &wippersnapper_digitalio_DigitalIORemove_msg
#define wippersnapper_digitalio_DigitalIOEvent_fields &wippersnapper_digitalio_DigitalIOEvent_msg
#define wippersnapper_digitalio_DigitalIOWrite_fields &wippersnapper_digitalio_DigitalIOWrite_msg

/* Maximum encoded size of messages (where known) */
/* wippersnapper_digitalio_DigitalIOAdd_size depends on runtime parameters */
/* wippersnapper_digitalio_DigitalIORemove_size depends on runtime parameters */
/* wippersnapper_digitalio_DigitalIOEvent_size depends on runtime parameters */
/* wippersnapper_digitalio_DigitalIOWrite_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

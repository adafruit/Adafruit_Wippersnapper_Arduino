/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5-dev at Thu May 19 14:02:54 2022. */

#ifndef PB_WIPPERSNAPPER_DESCRIPTION_V1_WIPPERSNAPPER_DESCRIPTION_V1_DESCRIPTION_PB_H_INCLUDED
#define PB_WIPPERSNAPPER_DESCRIPTION_V1_WIPPERSNAPPER_DESCRIPTION_V1_DESCRIPTION_PB_H_INCLUDED
#include <pb.h>
#include "nanopb/nanopb.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _wippersnapper_description_v1_CreateDescriptionResponse_Response {
    wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_UNSPECIFIED = 0,
    wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK = 1,
    wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND = 2
} wippersnapper_description_v1_CreateDescriptionResponse_Response;

/* Struct definitions */
typedef struct _wippersnapper_description_v1_CreateDescriptionRequest_Version {
    char dummy_field;
} wippersnapper_description_v1_CreateDescriptionRequest_Version;

typedef struct _wippersnapper_description_v1_GetDefinitionRequest {
    char dummy_field;
} wippersnapper_description_v1_GetDefinitionRequest;

typedef struct _wippersnapper_description_v1_GetDefinitionResponse {
    char dummy_field;
} wippersnapper_description_v1_GetDefinitionResponse;

typedef struct _wippersnapper_description_v1_CreateDescriptionRequest {
    char machine_name[64];
    int32_t mac_addr;
    int32_t usb_vid;
    int32_t usb_pid;
    char str_version[20];
} wippersnapper_description_v1_CreateDescriptionRequest;

typedef struct _wippersnapper_description_v1_CreateDescriptionResponse {
    wippersnapper_description_v1_CreateDescriptionResponse_Response response;
    int32_t total_gpio_pins;
    int32_t total_analog_pins;
    float reference_voltage;
    int32_t total_i2c_ports;
} wippersnapper_description_v1_CreateDescriptionResponse;

typedef struct _wippersnapper_description_v1_RegistrationComplete {
    bool is_complete;
} wippersnapper_description_v1_RegistrationComplete;


/* Helper constants for enums */
#define _wippersnapper_description_v1_CreateDescriptionResponse_Response_MIN wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_UNSPECIFIED
#define _wippersnapper_description_v1_CreateDescriptionResponse_Response_MAX wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND
#define _wippersnapper_description_v1_CreateDescriptionResponse_Response_ARRAYSIZE ((wippersnapper_description_v1_CreateDescriptionResponse_Response)(wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define wippersnapper_description_v1_CreateDescriptionRequest_init_default {"", 0, 0, 0, ""}
#define wippersnapper_description_v1_CreateDescriptionRequest_Version_init_default {0}
#define wippersnapper_description_v1_CreateDescriptionResponse_init_default {_wippersnapper_description_v1_CreateDescriptionResponse_Response_MIN, 0, 0, 0, 0}
#define wippersnapper_description_v1_RegistrationComplete_init_default {0}
#define wippersnapper_description_v1_GetDefinitionRequest_init_default {0}
#define wippersnapper_description_v1_GetDefinitionResponse_init_default {0}
#define wippersnapper_description_v1_CreateDescriptionRequest_init_zero {"", 0, 0, 0, ""}
#define wippersnapper_description_v1_CreateDescriptionRequest_Version_init_zero {0}
#define wippersnapper_description_v1_CreateDescriptionResponse_init_zero {_wippersnapper_description_v1_CreateDescriptionResponse_Response_MIN, 0, 0, 0, 0}
#define wippersnapper_description_v1_RegistrationComplete_init_zero {0}
#define wippersnapper_description_v1_GetDefinitionRequest_init_zero {0}
#define wippersnapper_description_v1_GetDefinitionResponse_init_zero {0}

/* Field tags (for use in manual encoding/decoding) */
#define wippersnapper_description_v1_CreateDescriptionRequest_machine_name_tag 1
#define wippersnapper_description_v1_CreateDescriptionRequest_mac_addr_tag 2
#define wippersnapper_description_v1_CreateDescriptionRequest_usb_vid_tag 3
#define wippersnapper_description_v1_CreateDescriptionRequest_usb_pid_tag 4
#define wippersnapper_description_v1_CreateDescriptionRequest_str_version_tag 15
#define wippersnapper_description_v1_CreateDescriptionResponse_response_tag 1
#define wippersnapper_description_v1_CreateDescriptionResponse_total_gpio_pins_tag 2
#define wippersnapper_description_v1_CreateDescriptionResponse_total_analog_pins_tag 3
#define wippersnapper_description_v1_CreateDescriptionResponse_reference_voltage_tag 4
#define wippersnapper_description_v1_CreateDescriptionResponse_total_i2c_ports_tag 5
#define wippersnapper_description_v1_RegistrationComplete_is_complete_tag 1

/* Struct field encoding specification for nanopb */
#define wippersnapper_description_v1_CreateDescriptionRequest_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   machine_name,      1) \
X(a, STATIC,   SINGULAR, INT32,    mac_addr,          2) \
X(a, STATIC,   SINGULAR, INT32,    usb_vid,           3) \
X(a, STATIC,   SINGULAR, INT32,    usb_pid,           4) \
X(a, STATIC,   SINGULAR, STRING,   str_version,      15)
#define wippersnapper_description_v1_CreateDescriptionRequest_CALLBACK NULL
#define wippersnapper_description_v1_CreateDescriptionRequest_DEFAULT NULL

#define wippersnapper_description_v1_CreateDescriptionRequest_Version_FIELDLIST(X, a) \

#define wippersnapper_description_v1_CreateDescriptionRequest_Version_CALLBACK NULL
#define wippersnapper_description_v1_CreateDescriptionRequest_Version_DEFAULT NULL

#define wippersnapper_description_v1_CreateDescriptionResponse_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    response,          1) \
X(a, STATIC,   SINGULAR, INT32,    total_gpio_pins,   2) \
X(a, STATIC,   SINGULAR, INT32,    total_analog_pins,   3) \
X(a, STATIC,   SINGULAR, FLOAT,    reference_voltage,   4) \
X(a, STATIC,   SINGULAR, INT32,    total_i2c_ports,   5)
#define wippersnapper_description_v1_CreateDescriptionResponse_CALLBACK NULL
#define wippersnapper_description_v1_CreateDescriptionResponse_DEFAULT NULL

#define wippersnapper_description_v1_RegistrationComplete_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     is_complete,       1)
#define wippersnapper_description_v1_RegistrationComplete_CALLBACK NULL
#define wippersnapper_description_v1_RegistrationComplete_DEFAULT NULL

#define wippersnapper_description_v1_GetDefinitionRequest_FIELDLIST(X, a) \

#define wippersnapper_description_v1_GetDefinitionRequest_CALLBACK NULL
#define wippersnapper_description_v1_GetDefinitionRequest_DEFAULT NULL

#define wippersnapper_description_v1_GetDefinitionResponse_FIELDLIST(X, a) \

#define wippersnapper_description_v1_GetDefinitionResponse_CALLBACK NULL
#define wippersnapper_description_v1_GetDefinitionResponse_DEFAULT NULL

extern const pb_msgdesc_t wippersnapper_description_v1_CreateDescriptionRequest_msg;
extern const pb_msgdesc_t wippersnapper_description_v1_CreateDescriptionRequest_Version_msg;
extern const pb_msgdesc_t wippersnapper_description_v1_CreateDescriptionResponse_msg;
extern const pb_msgdesc_t wippersnapper_description_v1_RegistrationComplete_msg;
extern const pb_msgdesc_t wippersnapper_description_v1_GetDefinitionRequest_msg;
extern const pb_msgdesc_t wippersnapper_description_v1_GetDefinitionResponse_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define wippersnapper_description_v1_CreateDescriptionRequest_fields &wippersnapper_description_v1_CreateDescriptionRequest_msg
#define wippersnapper_description_v1_CreateDescriptionRequest_Version_fields &wippersnapper_description_v1_CreateDescriptionRequest_Version_msg
#define wippersnapper_description_v1_CreateDescriptionResponse_fields &wippersnapper_description_v1_CreateDescriptionResponse_msg
#define wippersnapper_description_v1_RegistrationComplete_fields &wippersnapper_description_v1_RegistrationComplete_msg
#define wippersnapper_description_v1_GetDefinitionRequest_fields &wippersnapper_description_v1_GetDefinitionRequest_msg
#define wippersnapper_description_v1_GetDefinitionResponse_fields &wippersnapper_description_v1_GetDefinitionResponse_msg

/* Maximum encoded size of messages (where known) */
#define wippersnapper_description_v1_CreateDescriptionRequest_size 119
#define wippersnapper_description_v1_CreateDescriptionRequest_Version_size 0
#define wippersnapper_description_v1_CreateDescriptionResponse_size 40
#define wippersnapper_description_v1_RegistrationComplete_size 2
#define wippersnapper_description_v1_GetDefinitionRequest_size 0
#define wippersnapper_description_v1_GetDefinitionResponse_size 0

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

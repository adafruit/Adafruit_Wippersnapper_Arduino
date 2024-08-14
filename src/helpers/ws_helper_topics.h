#ifndef WS_HELPER_TOPICS_H
#define WS_HELPER_TOPICS_H

// Reserved Adafruit IO MQTT topics
#define TOPIC_IO_THROTTLE "/throttle" ///< Adafruit IO Throttle MQTT Topic
#define TOPIC_IO_ERRORS "/errors"     ///< Adafruit IO Error MQTT Topic

// Reserved Wippersnapper topics
#define TOPIC_WS "/wprsnpr/"      ///< WipperSnapper topic
#define TOPIC_INFO "/info/"       ///< Registration sub-topic
#define TOPIC_SIGNALS "/signals/" ///< Signals sub-topic
#define TOPIC_I2C "/i2c"          ///< I2C sub-topic
#define MQTT_TOPIC_PIXELS_DEVICE                                               \
  "/signals/device/pixel" ///< Pixels device->broker topic
#define MQTT_TOPIC_PIXELS_BROKER                                               \
  "/signals/broker/pixel" ///< Pixels broker->device topic

#endif // WS_HELPER_TOPICS_H
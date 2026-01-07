#ifndef WS_HELPER_MACROS_H
#define WS_HELPER_MACROS_H

#define WS_DEBUG          ///< Define to enable debugging to serial terminal
#define WS_PRINTER Serial ///< Where debug messages will be printed

// Define actual debug output functions when necessary.
#ifdef WS_DEBUG
#define WS_DEBUG_PRINT(...)                                                    \
  { WS_PRINTER.print(__VA_ARGS__); } ///< Prints debug output.
#define WS_DEBUG_PRINTLN(...)                                                  \
  { WS_PRINTER.println(__VA_ARGS__); } ///< Prints line from debug output.
#define WS_DEBUG_PRINTHEX(...)                                                 \
  { WS_PRINTER.print(__VA_ARGS__, HEX); } ///< Prints debug output.
#else
#define WS_DEBUG_PRINT(...)                                                    \
  {} ///< Prints debug output
#define WS_DEBUG_PRINTLN(...)                                                  \
  {} ///< Prints line from debug output.
#endif

#define WS_DELAY_WITH_WDT(timeout)                                             \
  {                                                                            \
    unsigned long start = millis();                                            \
    while (millis() - start < timeout) {                                       \
      delay(10);                                                               \
      yield();                                                                 \
      Ws.feedWDTV2();                                                        \
      if (millis() < start) {                                                  \
        start = millis(); /* if rollover */                                    \
      }                                                                        \
    }                                                                          \
  } ///< Delay function

/*!
    @brief  Retry a function until a condition is met or a timeout is reached.
    @param  func
            The function to retry.
    @param  result_type
            The type of the result of the function.
    @param  result_var
            The variable to store the last result of the function.
    @param  condition
            The condition to check the result against.
    @param  timeout
            The maximum time to retry the function.
    @param  interval
            The time to wait between retries.
    @param  ...
            The arguments to pass to the function.
*/
#define RETRY_FUNCTION_UNTIL_TIMEOUT(func, result_var, condition, timeout,     \
                                     interval, ...)                            \
  {                                                                            \
    unsigned long startTime = millis();                                        \
    while (millis() - startTime < timeout) {                                   \
      result_var = func(__VA_ARGS__);                                          \
      if (condition(result_var)) {                                             \
        break;                                                                 \
      }                                                                        \
      if (startTime > millis()) {                                              \
        startTime = millis(); /* if rollover */                                \
      }                                                                        \
      WS_DELAY_WITH_WDT(interval);                                             \
    }                                                                          \
  } ///< Retry a function until a condition is met or a timeout is reached.

#endif // WS_HELPER_MACROS_H
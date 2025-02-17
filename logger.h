

#define LOG_LEVEL_TRACE      // // LOG_LEVEEL_TRACE,LOG_LEVEEL_DEBUG, LOG_LEVEEL_INFO, LOG_LEVEEL_WARN and ERROR.


#if defined LOG_LEVEL_TRACE
#define TRACE(x, ...) std::printf("[TRACE: %s:%d]"x"", __FILE__, __LINE__, ##__VA_ARGS__);
#define DEBUG(x, ...) std::printf("[DEBUG: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define INFO(x, ...) std::printf("[INFO: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define WARN(x, ...) std::printf("[WARN: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define ERROR(x, ...) std::printf("[ERROR: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);

#elif defined LOG_LEVEL_DEBUG
#define TRACE(x, ...)
#define DEBUG(x, ...) std::printf("[DEBUG: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define INFO(x, ...) std::printf("[INFO: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define WARN(x, ...) std::printf("[WARN: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define ERROR(x, ...) std::printf("[ERROR: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);

#elif defined LOG_LEVEL_INFO
#define TRACE(x, ...)
#define DEBUG(x, ...)
#define INFO(x, ...) printf("[INFO: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define WARN(x, ...) std::printf("[WARN: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define ERROR(x, ...) std::printf("[ERROR: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);

#elif defined LOG_LEVEL_WARN
#define TRACE(x, ...)
#define DEBUG(x, ...)
#define INFO(x, ...)
#define WARN(x, ...) std::printf("[WARN: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define ERROR(x, ...) std::printf("[ERROR: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);

#elif defined LOG_LEVEL_ERROR
#define TRACE(x, ...)
#define DEBUG(x, ...)
#define INFO(x, ...)
#define WARN(x, ...)
#define ERROR(x, ...) std::printf("[ERROR: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);

#else
#define TRACE(x, ...)
#define DEBUG(x, ...)
#define INFO(x, ...) printf("[INFO: %s:%d]"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define WARN(x, ...)
#define ERROR(x, ...)
#endif

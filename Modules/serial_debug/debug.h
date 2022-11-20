/******************************************************************************
 * @file      debug.h
 * @brief
 * @version   1.0
 * @date      Apr 17, 2022
 * @copyright
 *****************************************************************************/
#ifndef __DEBUG_DEBUG_H__
#define __DEBUG_DEBUG_H__

#define DEBUG_LEVEL_DEBUG       "DEBUG"
#define DEBUG_LEVEL_INFO        "INFO"
#define DEBUG_LEVEL_ERROR       "ERROR"
#define DEBUG_LEVEL_CRITICAL    "CRITICAL"

#if defined(DEBUG)

/* ------------------------------------------------------------------------- */

/**
 * printf function prototype, used for debugging,
 * so that modules using Debug() macro do not have to include the whole stdio library
 * */
extern int  printf (const char *__restrict, ...) __attribute__ ((__format__ (__printf__, 1, 2)));

/* ------------------------------------------------------------------------- */

/**
 * Define to enable debug to used non-blocking transmission
 * */
#define DEBUG_NON_BLOCK

/**
 * @brief Initialize debug UART
 *
 * @param void
 *
 * @return void
 * */
void Debug_vidInitialize(void);


/**
 * Flush debug output
 *
 * @param void
 *
 * @return void
 * */
void Debug_vidFlushOutput(void);

/**
 * @brief De-initialize debug UART
 *
 * @return void
 * */
void Debug_vidDeInitialize(void);

/**
 * @brief Print message on debug interface in the format `FILE LINE#: TAG[LEVEL]: msg`
 *
 * @param message debug tag, to distinguish messages from different sources
 * @param debug level
 * @param message format
 * @param format arguments
 * */
#define Debug(TAG, LEVEL, msg_fmt, ...)      printf("%s %d: [%s] %s: " msg_fmt, __FILE__, __LINE__, (LEVEL), (TAG), ## __VA_ARGS__)


/**
 * @brief Dump byte contents of a given buffer in hex format
 *
 * @param buffer pointer to buffer of byts
 * @param len buffer length
 * @param width line width (ho many bytes / line)
 * */
#define DebugHexDump(buffer, len, width)  { for(uint32_t idx = 0; idx < (len); idx++){ if(idx % (width) == 0){ printf("\n0x%04X : ", (uint16_t)idx); } printf("0x%02X ", ((uint8_t *)(buffer))[idx]); } }


#else

#define Debug_vidInitialize()

#define Debug_vidDeInitialize()

#define Debug(TAG, LEVEL, msg_fmt, ...)

#define DebugHexDump(buffer, len, width)

#endif /*   DEBUG   */

#endif /* __DEBUG_DEBUG_H__ */

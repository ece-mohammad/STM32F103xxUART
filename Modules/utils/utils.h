/******************************************************************************
 * @file      utils.h
 * @brief
 * @version   1.0
 * @date      Jul 13, 2022
 * @copyright
 *****************************************************************************/

#ifndef _UTILS_H_
#define _UTILS_H_

#ifndef NULL
/**
 * @brief NULL pointer
 * */
#define NULL                ((void *)0)
#endif  /*  NULL  */

#ifndef TRUE
/**
 * @brief True
 * */
#define TRUE                (1 == 1)
#endif  /* TRUE */

#ifndef FALSE
/**
 * @brief False
 * */
#define FALSE               (!TRUE)
#endif  /* FALSE*/

/**
 * @brief Check if pointer is @p NULL
 *
 * @param [in] ptr
 *
 * @return #TRUE if @p ptr is #NULL. Otherwise #FALSE
 * */
#define IS_NULLPTR(ptr)     ((ptr) == NULL)

/**
 * @brief Check if value is 0
 *
 * @param [in] val
 *
 * @return #TRUE if @p val is 0. Otherwise #FALSE
 * */
#define IS_ZERO(val)        ((val) == 0)

/**
 * @brief Get minimum of 2 values
 *
 * @param [in] a
 * @param [in] b
 *
 * @return @p b if @p a > @p b. Otherwise @p a
 *
 * */
#define MIN(a, b)           (((a) < (b)) ? (a) : (b))

/**
 * @brief Get maximum of 2 values
 *
 * @param [in] a
 * @param [in] b
 *
 * @return @p a if @p a > @p b. Otherwise @p b
 *
 * */
#define MAX(a, b)           (((a) > (b)) ? (a) : (b))

#endif /* _UTILS_H_ */

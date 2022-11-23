/******************************************************************************
 * @file        utils.h
 * @brief       C utility MACROs
 * @version     1.0
 * @date        Jul 13, 2022
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
 * @brief Check if an expression is TRUE
 * 
 * @param [in] expr expression to check 
 * 
 * @return @ref TRUE if expression evaluates to TRUE, otherwise returns @ref FALSE
 */
#define IS_TRUE(expr)       ((expr) == TRUE)

/**
 * @brief Check if an expression is FALSE
 * 
 * @param [in] expr expression to check 
 * 
 * @return @ref TRUE if expression evaluates to FALSE, otherwise returns @ref TRUE
 */
#define IS_FALSE(expr)      ((expr) == FALSE)

/**
 * @brief Check if pointer is @p NULL
 *
 * @param [in] ptr pointer to check
 *
 * @return @ref TRUE if @p ptr is @ref NULL. Otherwise @ref FALSE
 * */
#define IS_NULLPTR(ptr)     ((ptr) == NULL)

/**
 * @brief Check if a value is 0
 *
 * @param [in] val value to check
 *
 * @return @ref TRUE if @p val is 0. Otherwise @ref FALSE
 * */
#define IS_ZERO(val)        ((val) == 0x00)

/**
 * @brief Check if a number is even
 *
 * @param [in] num number to check
 *
 * @return @ref TRUE if @p num is even. Otherwise @ref FALSE
 * */
#define IS_EVEN(num)        (((num) & 0x01) == 0)

/**
 * @brief Check if a number is odd
 *
 * @param [in] num number to check
 *
 * @return @ref TRUE if @p num is odd. Otherwise @ref FALSE
 * */
#define IS_ODD(num)        (((num) & 0x01) == 0)

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

/**
 * @brief Macro to disable compiler warnings about unused variables
 */
#define UNUSED(var)         ((void)var)

#endif /* _UTILS_H_ */

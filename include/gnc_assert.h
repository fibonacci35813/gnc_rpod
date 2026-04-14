/**
 * @file    gnc_assert.h
 * @brief   GNC assertion macro — NASA Power of 10, Rule 5
 *
 * GNC_ASSERT evaluates a condition at runtime.
 * On failure it records the error code and executes the recovery action.
 * Assertions are side-effect free: the condition expression must not
 * modify any state.
 *
 * Usage:
 *   GNC_ASSERT(ptr != NULL,   ERR_NULL_PTR,  return ERR_NULL_PTR);
 *   GNC_ASSERT(n > 0,         ERR_BAD_PARAM, return ERR_BAD_PARAM);
 */

#ifndef GNC_ASSERT_H
#define GNC_ASSERT_H

#include <stdio.h>
#include "gnc_types.h"

/* GNC_ASSERT(condition, error_code, recovery_action)
 * - condition     : boolean expression, must be side-effect free
 * - error_code    : GncStatus value to report
 * - recovery_action: a complete statement, e.g. "return ERR_NULL_PTR"
 *
 * Expands to a complete if-statement (Rule 8: complete syntactic unit).
 */
#define GNC_ASSERT(cond, err, action)                          \
    if (!(cond)) {                                             \
        (void)fprintf(stderr,                                  \
            "GNC_ASSERT FAIL [%s:%d] cond=(" #cond ") err=%d\n", \
            __FILE__, __LINE__, (int)(err));                   \
        action;                                                \
    }

#endif /* GNC_ASSERT_H */

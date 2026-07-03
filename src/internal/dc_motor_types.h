#ifndef DC_MOTOR_INTERNAL_TYPES_H
#define DC_MOTOR_INTERNAL_TYPES_H

/**
 * @file  dc_motor_types.h  (internal)
 * @brief Re-exports the public dc_motor_types.h so that all internal
 *        .c files can simply do:
 *
 *            #include "internal/dc_motor_types.h"
 *
 *        and get both the public types AND any future internal-only
 *        extensions added below.
 */

#include "../../include/dc_motor_types.h"

/* Add internal-only types/macros here if needed in the future. */

#endif /* DC_MOTOR_INTERNAL_TYPES_H */

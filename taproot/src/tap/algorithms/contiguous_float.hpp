/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_CONTIGUOUS_FLOAT_HPP_
#define TAPROOT_CONTIGUOUS_FLOAT_HPP_

namespace tap
{
namespace algorithms
{
/**
 * Wraps a float to allow easy comparison and manipulation of sensor readings
 * that wrap (e.g. -180 to 180).
 *
 * For bounds 0 - 10, logically:
 *   - 10 + 1 == 1
 *   - 0 - 1 == 9
 *   - 0 == 10
 *
 * Credit to: https://github.com/Team488/SeriouslyCommonLib/blob/af2ce83a830299a8ab3773bec9b8ccc6ab
 *            5a3367/src/main/java/xbot/common/math/ContiguousDouble.java
 */
class ContiguousFloat
{
public:
    ContiguousFloat(const float value, const float lowerBound, const float upperBound);

    /**
     * Shifts the value so that it still represents the same position but is
     * within the current bounds.
     *
     * @return the new value for chaining functions.
     */
    float reboundValue();

    /**
     * Computes the difference between two values (other - this), accounting for
     * wrapping. Treats the given 'other' value as a number within the same bounds
     * as the current instance.
     *
     * @param[in] otherValue the other value to compare against.
     * @return the computed difference.
     */
    float difference(const float otherValue) const;

    /**
     * Computes the difference between two values (other - this), accounting for
     * wrapping.
     *
     * @param[in] otherValue the other value to compare against (must have the same bounds
     *      as the current instance).
     * @return the computed difference.
     */
    float difference(const ContiguousFloat& otherValue) const;

    /**
     * Shifts both bounds by the specified amount
     *
     * @param[in] shiftMagnitude the amount to add to each bound.
     */
    void shiftBounds(const float shiftMagnitude);

    /**
     * Shifts value by the specified amount (addition).
     *
     * @param[in] shiftMagnitude The amount to add to the current value.
     */
    void shiftValue(const float shiftMagnitude);

    /**
     * Limits the passed in contiguous float between the closest of the
     * min or max value if outside the min and max value's wrapped range.
     *
     * The min and max must have the same wrapped bounds as the valueToLimit.
     *
     *
     * For example given a value wrapped from -10 to 10, with the following
     * conditions:
     * - valueToLimit: 5, min: 1, max: 4, returns 4.
     * - valueToLimit: 9, min: 1, max: 3, returns 1 (since valueToLimit is closest to 1).
     * - valueToLimit: 9, min: 2, max: 1, returns 9 (since the range between min and max
     *                 starts at 2, goes up to 9, then wraps around to 1).
     *
     * @param[in] valueToLimit the ContigousFloat whose value it is to limit
     * @param[in] min the ContiguousFloat with the same bounds as valueToLimit that
     *      valueToLimit will be limited below.
     * @param[in] max the ContiguousFloat with the same bounds as valueToLimit that
     *      valueToLimit will be limited above.
     * @param[out] status the status result (what operation the limitValue function performed). The
     * status codes are described below:
     *  - 0: No limiting performed
     *  - 1: Limited to min value
     *  - 2: Limited to max value
     * @return the limited value.
     */
    static float limitValue(
        const ContiguousFloat& valueToLimit,
        const ContiguousFloat& min,
        const ContiguousFloat& max,
        int* status);

    /**
     * Runs the limitValue function from above, wrapping the min and max passed in to
     * the same bounds as those of valueToLimit's.
     *
     * @see limitValue.
     * @param[in] valueToLimit the ContigousFloat whose value it is to limit
     * @param[in] min the ContiguousFloat with the same bounds as valueToLimit that
     *      valueToLimit will be limited below.
     * @param[in] max the ContiguousFloat with the same bounds as valueToLimit that
     *      valueToLimit will be limited above.
     * @param[out] status the status result (what operation the limitValue function performed). The
     * status codes are described below:
     *  - 0: No limiting performed
     *  - 1: Limited to min value
     *  - 2: Limited to max value
     * @return the limited value.
     */
    static float limitValue(
        const ContiguousFloat& valueToLimit,
        const float min,
        const float max,
        int* status);

    // Getters/Setters ----------------

    /**
     * Returns the wrapped value.
     */
    float getValue() const;

    void setValue(const float newValue);

    /**
     * Returns the value's upper bound.
     */
    float getUpperBound() const;

    /**
     * Sets the upper bound to newValue.
     */
    void setUpperBound(const float newValue);

    /**
     * Returns the value's lower bound.
     */
    float getLowerBound() const;

    /**
     * Sets the lower bound to newValue.
     */
    void setLowerBound(const float newValue);

private:
    /**
     * The wrapped value.
     */
    float value;

    /**
     * The lower bound to wrap around.
     */
    float lowerBound;
    /**
     * The upper bound to wrap around.
     */
    float upperBound;

    /**
     * Flips the lower and upper bounds if the lower bound is larger than the
     * upper bound.
     */
    void validateBounds();

    /**
     * Calculates a number representing the current value that is higher than
     * (or equal to) the upper bound. Used to make normal numerical comparisons
     * without needing to handle wrap cases.
     *
     * @return the computed value
     */
    float unwrapAbove() const;

    /**
     * Calculates a number representing the current value that is lower than (or
     * equal to) the lower bound. Used to make normal numerical comparisons
     * without needing to handle wrap cases.
     *
     * @return the computed value
     */
    float unwrapBelow() const;
};  // class ContiguousFloat

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_CONTIGUOUS_FLOAT_HPP_

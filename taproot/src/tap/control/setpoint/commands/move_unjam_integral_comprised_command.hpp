/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_MOVE_UNJAM_INTEGRAL_COMPRISED_COMMAND_HPP_
#define TAPROOT_MOVE_UNJAM_INTEGRAL_COMPRISED_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "../interfaces/integrable_setpoint_subsystem.hpp"

#include "move_integral_command.hpp"
#include "unjam_integral_command.hpp"

namespace tap::control::setpoint
{
/**
 * A comprised command that combines the integral unjam and rotate commands. Will schedule the move
 * command when not jammed. When jammed, will switch to the unjam command.
 *
 * @see UnjamIntegralCommand
 * @see MoveIntegralCommand
 */
class MoveUnjamIntegralComprisedCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * @param[in] drivers A reference to the `Drivers` struct.
     * @param[in] subsystem The IntegrableSetpointSubsystem that will be rotated or unjammed.
     * @param[in] moveIntegralCommand A command that rotates the agitator forward.
     * @param[in] unjamIntegralCommand A command that unjams the agitator.
     *
     * @note The move integral and unjam integral commands must have the same subsystem requirement.
     * This subsystem requirement must be the subsystem passed in.
     */
    MoveUnjamIntegralComprisedCommand(
        tap::Drivers &drivers,
        IntegrableSetpointSubsystem &subsystem,
        MoveIntegralCommand &moveIntegralCommand,
        UnjamIntegralCommand &unjamIntegralCommand);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "move unjam integral cc"; }

private:
    IntegrableSetpointSubsystem &subsystem;

    MoveIntegralCommand &moveIntegralCommand;

    UnjamIntegralCommand &unjamCommand;

    /// True if the agitator is being unjammed
    bool unjamSequenceCommencing;
};

}  // namespace tap::control::setpoint

#endif  // TAPROOT_MOVE_UNJAM_INTEGRAL_COMPRISED_COMMAND_HPP_
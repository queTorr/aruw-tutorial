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

#ifndef TAPROOT_COMMAND_MOCK_HPP_
#define TAPROOT_COMMAND_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/control/command.hpp"

namespace tap
{
namespace mock
{
class CommandMock : public control::Command
{
public:
    CommandMock();
    virtual ~CommandMock();

    MOCK_METHOD(
        control::subsystem_scheduler_bitmap_t,
        getRequirementsBitwise,
        (),
        (const override));
    MOCK_METHOD(void, addSubsystemRequirement, (control::Subsystem * requirement), (override));
    MOCK_METHOD(const char*, getName, (), (const override));
    MOCK_METHOD(bool, isReady, (), (override));
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, execute, (), (override));
    MOCK_METHOD(void, end, (bool interrupted), (override));
    MOCK_METHOD(bool, isFinished, (), (const override));
};  // class CommandMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_COMMAND_MOCK_HPP_

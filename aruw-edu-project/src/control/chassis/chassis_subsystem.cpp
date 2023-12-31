/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

using tap::algorithms::limitVal;

namespace control::chassis
{
// STEP 1 (Tank Drive): create constructor

ChassisSubsystem::ChassisSubsystem(Drivers& drivers, const ChassisConfig& config):
        tap::control::Subsystem(&drivers),
        desiredOutput{0,0,0,0},
        motors{
            Motor(&drivers, config.leftFrontId, config.canBus, false, "LF"),
            Motor(&drivers, config.leftBackId, config.canBus, false, "LB"),
            Motor(&drivers, config.rightBackId, config.canBus, true, "RB"),
            Motor(&drivers, config.rightFrontId, config.canBus, true, "RF"),
        } 
    {modm::Pid<float>::Parameter param = config.wheelVelocityPidConfig;

    pidControllers = {Pid(param), Pid(param), Pid(param), Pid(param)} ;
    }  

// STEP 2 (Tank Drive): initialize function

void ChassisSubsystem::initialize() {
    for(Motor& motor : motors) {
        motor.initialize();
    }
}

// STEP 3 (Tank Drive): setVelocityTankDrive function

void ChassisSubsystem::setVelocityTankDrive(float left, float right) {
    
    left = mpsToRpm(left);
    right = mpsToRpm(right);

    left = limitVal(left, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    right = limitVal(right, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

    desiredOutput[static_cast<uint8_t>(MotorId::LF)] = left;
    desiredOutput[static_cast<uint8_t>(MotorId::LB)] = left;
    desiredOutput[static_cast<uint8_t>(MotorId::RB)] = right;
    desiredOutput[static_cast<uint8_t>(MotorId::RF)] = right;

}


// STEP 4 (Tank Drive): refresh function

void ChassisSubsystem::refresh() {

    for (size_t i = 0; i < motors.size(); i++)
    {
        pidControllers[i].update(desiredOutput[i] - motors[i].getShaftRPM());
        motors[i].setDesiredOutput(pidControllers[i].getValue());
    }


}  // namespace control::chassis
}
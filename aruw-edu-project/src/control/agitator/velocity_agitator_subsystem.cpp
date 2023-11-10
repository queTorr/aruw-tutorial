/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "velocity_agitator_subsystem.hpp"

#include "tap/architecture/clock.hpp"

#include "modm/math/geometry/angle.hpp"

#include "drivers.hpp"

using tap::arch::clock::getTimeMilliseconds;
using tap::motor::DjiMotor;

namespace control::agitator
{
// STEP 1 (Agitator Control): create constructor
VelocityAgitatorSubsystem::VelocityAgitatorSubsystem(
        Drivers& drivers, const control::algorithms::EduPidConfig& pidConfig, DjiMotor& agitator)
        :tap::control::Subsystem(&drivers), agitator(agitator), velocityPid(pidConfig)
        {}
        
       

// STEP 2 (Agitator Control): initialize function

void VelocityAgitatorSubsystem::initialize()
{
    agitator.initialize();
}

// STEP 6 (Agitator Control): refresh function

void VelocityAgitatorSubsystem::refresh()
{
    if(isOnline())                        
    {
        float error = velocitySetpoint - getCurrentValue();  
          // desired agitator velocity - its actual velocity

        float currTime = tap::arch::clock::getTimeMilliseconds();
        float dt = currTime - prevTime; 
        float prevTime = currTime;

        velocityPid.runControllerDerivateError(error, dt);  //use PID to correct output
        velocitySetpoint = velocityPid.getOutput();     
    }
    if(!isOnline())
        calibrated = false;
    else 
        calibrateHere();
}

// STEP 3 (Agitator Control): getSetpoint function
 float VelocityAgitatorSubsystem::getSetpoint() const
{
   return velocitySetpoint;   //in rad/s
}
// STEP 4 (Agitator Control): getCurrentValue function
float VelocityAgitatorSubsystem::getCurrentValue() const
{
    //get actual agitator velocity in RPM then convert into rad/s
    float currentValue = (agitator.getShaftRPM()/AGITATOR_GEAR_RATIO_M2006)*(M_2_PI/60.0f);

    return currentValue; 
}

// STEP 5 (Agitator Control): calibrateHere function

bool VelocityAgitatorSubsystem::calibrateHere()
{
    if(isOnline())
    {
        agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
        calibrated = true;
        velocitySetpoint = 0.0f;
        return true;
    }
    else 
        return false;
}


// STEP 7 (Agitator Control): isOnline function

bool VelocityAgitatorSubsystem::isOnline()
{
    return agitator.isMotorOnline();
}

// STEP 8 (Agitator Control): getCurrentValueIntegral function

 float VelocityAgitatorSubsystem::getCurrentValueIntegral() const
 {
    return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
 }

float VelocityAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    return (2.0f * M_PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
           agitator.getEncoderUnwrapped() / AGITATOR_GEAR_RATIO_M2006;
}
}  // namespace control::agitator

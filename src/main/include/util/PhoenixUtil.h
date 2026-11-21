// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>

#include <ctre/phoenix6/TalonFX.hpp>

/**
 * Helpers for reliably configuring CTRE devices (based on 254's). Only the TalonFX versions were
 * ported, since that's all this robot uses. CANcoder/CANdi/CANrange versions follow the same
 * pattern if you need them later.
 */
namespace PhoenixUtil {

/** Calls `function` until it returns OK or we run out of attempts. */
bool SpamGetStatusCode(const std::function<ctre::phoenix::StatusCode()>& function,
                       int numTries = 5);

/** Reads the config back off the Talon and checks it matches what we meant to send. */
bool ReadAndVerifyConfiguration(ctre::phoenix6::hardware::TalonFX& talon,
                                const ctre::phoenix6::configs::TalonFXConfiguration& config);

/** The main one to use: apply, read back, verify, retry. Skips verification in sim. */
bool ApplyMotorConfigs(ctre::phoenix6::hardware::TalonFX& motor,
                       const ctre::phoenix6::configs::TalonFXConfiguration& config,
                       int numTries);

/** True if two TalonFX configs match (within a small tolerance). */
bool IsEqual(const ctre::phoenix6::configs::TalonFXConfiguration& a,
             const ctre::phoenix6::configs::TalonFXConfiguration& b);

}  // namespace PhoenixUtil

// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <string_view>

/**
 * Sags the simulated battery using the current from EVERY simulated mechanism at once.
 *
 * Each mechanism sim calls Report() with its current draw every loop, and Robot calls Update()
 * once per loop. (If each sim set the battery voltage itself, the last one to run would overwrite
 * all the others.)
 */
namespace SimBattery {

void Report(std::string_view mechanism, double amps);
void Update();

}  // namespace SimBattery

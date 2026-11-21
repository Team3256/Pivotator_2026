// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "sim/Reefscape2025.h"

void reefscape::BuildField(mayhem::sim::Obstacles& obstacles) {
  obstacles.SetPerimeter(0, 0, kFieldLength, kFieldWidth, 0, 0.5);
  // The reefs: hexagonal bases, solid to well above any robot.
  obstacles.AddPolygon("reef_blue", {{5.321, 4.506},
                                     {4.489, 4.986},
                                     {3.658, 4.506},
                                     {3.658, 3.546},
                                     {4.489, 3.066},
                                     {5.321, 3.546}},
                       0, 2.5);
  obstacles.AddPolygon("reef_red", {{12.227, 3.546},
                                    {13.059, 3.066},
                                    {13.890, 3.546},
                                    {13.890, 4.506},
                                    {13.059, 4.987},
                                    {12.227, 4.506}},
                       0, 2.5);
  // Coral stations cut the four corners.
  obstacles.AddPolygon("station_blue_right", {{0, 0}, {1.703, 0}, {0, 1.312}}, 0, 2.0);
  obstacles.AddPolygon("station_blue_left", {{0, 8.052}, {0, 6.740}, {1.703, 8.052}}, 0, 2.0);
  obstacles.AddPolygon("station_red_left", {{17.548, 8.052}, {15.845, 8.052}, {17.548, 6.740}},
                       0, 2.0);
  obstacles.AddPolygon("station_red_right", {{17.548, 0}, {17.548, 1.312}, {15.845, 0}}, 0, 2.0);
}

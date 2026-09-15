/*
   Copyright (C) 2026 valtec.ai

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// ArduPilot's SITL models a pack the same way, in SIM_Battery, and this file
// deliberately does not reuse it: ArduPilot is GPL-3.0 while this repository
// is LGPL-3.0, so copying that code would force the whole combined work to
// GPL-3.0 and shut the door on contributing any of this back upstream. Read
// it for reference if you like -- the maths below (coulomb counting, an OCV
// table, an I*R drop, a single-pole filter) is the textbook model both arrive
// at -- but write the code here, from cell data we own.

#include "BatteryModel.hh"

#include <algorithm>
#include <cmath>

namespace
{
/// Open-circuit voltage of one Fullymax cell against state of charge, as
/// measured for the aircraft's own pack. Same 14 points the companion
/// computer's estimator uses (marlin-drone:
/// drone/lua/battery-soc-soh/ekf/mod/bat_fullymax.lua), so the simulated pack
/// and the estimator agree on what the cell is.
///
/// State of charge ascending. An empty pack rests at 3.333 V/cell rather than
/// falling to zero -- that is the bottom of the measured curve, and it sits
/// below every flight-controller failsafe threshold on this airframe (6S: ARM
/// 22.5 V, LOW 22 V, CRIT 21.6 V), so failsafes still fire.
constexpr std::size_t kOcvPoints = 14;

constexpr double kOcvSoc[kOcvPoints] = {
    0.0000, 0.0159, 0.0337, 0.1514, 0.2720, 0.4456, 0.4949,
    0.5461, 0.6125, 0.8767, 0.9814, 0.9995, 0.9998, 1.0000};

constexpr double kOcvVolt[kOcvPoints] = {
    3.3330, 3.3866, 3.4081, 3.5020, 3.5665, 3.6411, 3.6726,
    3.7118, 3.7745, 4.0534, 4.1544, 4.1748, 4.1848, 4.2000};

/// Cutoff for the current filter. The current follows rotor speed, which steps
/// sharply on spin-up; without this the pack would see spikes no real ESC
/// could draw.
constexpr double kCurrentFilterHz = 10.0;

constexpr double kPi = 3.14159265358979323846;

/// Floor on the reported terminal voltage. Below zero the model would latch,
/// and at or below zero SITL stops believing the FDM has a battery at all.
constexpr double kMinVoltage = 0.1;
}  // namespace

double BatteryModel::OcvPerCell(double _soc)
{
  const double soc = std::min(std::max(_soc, 0.0), 1.0);

  for (std::size_t i = 1; i < kOcvPoints; ++i)
  {
    if (soc <= kOcvSoc[i])
    {
      const double span = kOcvSoc[i] - kOcvSoc[i - 1];
      if (span <= 0.0)
      {
        return kOcvVolt[i];
      }
      const double frac = (soc - kOcvSoc[i - 1]) / span;
      return kOcvVolt[i - 1] + frac * (kOcvVolt[i] - kOcvVolt[i - 1]);
    }
  }

  return kOcvVolt[kOcvPoints - 1];
}

void BatteryModel::Setup(
    double _cells,
    double _capacityAh,
    double _resistance,
    double _maxCurrent)
{
  this->cells = std::max(_cells, 1.0);
  this->capacityAh = std::max(_capacityAh, 0.0);
  this->resistance = std::max(_resistance, 0.0);
  this->maxCurrent = std::max(_maxCurrent, 0.0);
  this->Reset();
}

void BatteryModel::Reset()
{
  this->remainingAh = this->capacityAh;
  this->current = 0.0;
  this->voltage = this->cells * OcvPerCell(1.0);
}

void BatteryModel::Update(double _elecPowerW, double _dt)
{
  // A paused or rewound simulation must not discharge the pack.
  if (!(_dt > 0.0))
  {
    return;
  }

  const double elecPowerW = std::max(_elecPowerW, 0.0);
  double target = elecPowerW / std::max(this->voltage, 0.1);
  if (this->maxCurrent > 0.0)
  {
    target = std::min(target, this->maxCurrent);
  }

  const double rc = 1.0 / (2.0 * kPi * kCurrentFilterHz);
  const double alpha = _dt / (_dt + rc);
  this->current += (target - this->current) * alpha;

  // Zero capacity means "unlimited pack", matching ArduPilot's SITL: the
  // voltage is pinned and nothing sags.
  if (this->capacityAh <= 0.0)
  {
    this->voltage = this->cells * OcvPerCell(1.0);
    return;
  }

  this->remainingAh =
      std::max(0.0, this->remainingAh - this->current * _dt / 3600.0);

  const double soc = this->remainingAh / this->capacityAh;

  // Keep the terminal voltage positive whatever resistance and current clamp
  // the model was given. A non-positive reading would latch -- it feeds back
  // as the divisor on the next step, pinning the current at its clamp -- and
  // SITL reads voltage <= 0 as "the FDM has no battery", quietly falling back
  // to its own model.
  this->voltage = std::max(
      this->cells * OcvPerCell(soc) - this->current * this->resistance,
      kMinVoltage);
}

double BatteryModel::Voltage() const
{
  return this->voltage;
}

double BatteryModel::Current() const
{
  return this->current;
}

double BatteryModel::SocPct() const
{
  if (this->capacityAh <= 0.0)
  {
    return 100.0;
  }
  return 100.0 * this->remainingAh / this->capacityAh;
}

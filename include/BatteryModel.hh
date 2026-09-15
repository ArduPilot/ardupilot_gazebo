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

#ifndef BATTERYMODEL_HH_
#define BATTERYMODEL_HH_

#include <cstddef>

/// \brief Lumped model of a LiPo pack, driven by the electrical power drawn.
///
/// ArduPilot's SITL runs an equivalent model inside the Frame backend
/// (SIM_Battery), but only for frame-based backends. With --model JSON there
/// is no Frame, so the pack has to be simulated on this side and handed over
/// in the state packet as battery.voltage / battery.current.
///
/// The open-circuit-voltage curve and the pack resistance are the ones the
/// companion computer's own state-of-charge estimator assumes
/// (marlin-drone: drone/lua/battery-soc-soh, bat_fullymax), so the simulated
/// pack and the estimator describe the same battery.
///
/// Note this makes the simulation agree with the estimator by construction: it
/// exercises the plumbing and the energy bookkeeping, not the estimator's
/// ability to identify an unknown cell.
class BatteryModel
{
  /// \brief Configure the pack. Safe to call again to reconfigure.
  /// \param[in] _cells         cells in series
  /// \param[in] _capacityAh    usable capacity, Ah. Zero disables discharge:
  ///                           the voltage then stays at its initial value.
  /// \param[in] _resistance    whole-pack internal resistance, ohms
  /// \param[in] _maxCurrent    clamp on the reported current, amps
  public: void Setup(
              double _cells,
              double _capacityAh,
              double _resistance,
              double _maxCurrent);

  /// \brief Return the pack to fully charged.
  public: void Reset();

  /// \brief Advance the model.
  /// \param[in] _elecPowerW  electrical power drawn from the pack, W
  /// \param[in] _dt          simulation step, seconds. Non-positive is ignored,
  ///                         so a paused simulation does not discharge.
  public: void Update(double _elecPowerW, double _dt);

  /// \brief Terminal voltage, volts.
  public: double Voltage() const;

  /// \brief Pack current, amps. Positive is discharge.
  public: double Current() const;

  /// \brief Remaining charge as a percentage of capacity.
  public: double SocPct() const;

  /// \brief Open-circuit voltage of one cell at a given state of charge.
  /// \param[in] _soc  state of charge, 0.0 to 1.0
  public: static double OcvPerCell(double _soc);

  private: double cells{6.0};
  private: double capacityAh{0.0};
  private: double resistance{0.0};
  private: double maxCurrent{0.0};

  private: double remainingAh{0.0};
  private: double current{0.0};
  private: double voltage{0.0};
};

#endif  // BATTERYMODEL_HH_

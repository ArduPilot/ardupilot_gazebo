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

#include "BatteryModel.hh"

#include <cmath>
#include <cstdio>
#include <string>

namespace
{
int failures = 0;

void Check(bool _ok, const std::string &_what)
{
  if (!_ok)
  {
    std::printf("FAIL: %s\n", _what.c_str());
    ++failures;
  }
}

void CheckNear(double _got, double _want, double _tol,
               const std::string &_what)
{
  if (std::fabs(_got - _want) > _tol)
  {
    std::printf("FAIL: %s (got %.6f, want %.6f +/- %.6f)\n",
                _what.c_str(), _got, _want, _tol);
    ++failures;
  }
}

// The aircraft's pack: 6S Fullymax, 20.898 Ah, ~0.013 ohm.
BatteryModel MakePack()
{
  BatteryModel battery;
  battery.Setup(6.0, 20.89770474, 0.013, 200.0);
  return battery;
}

// Run the model for a while at a fixed electrical load, returning the charge
// drawn by integrating the reported current -- what a coulomb counter sees.
double RunSeconds(BatteryModel &_battery, double _elecPowerW, double _seconds)
{
  const double dt = 0.0025;  // the simulation step gz runs at
  double drawnAh = 0.0;
  for (double t = 0.0; t < _seconds; t += dt)
  {
    _battery.Update(_elecPowerW, dt);
    drawnAh += _battery.Current() * dt / 3600.0;
  }
  return drawnAh;
}

void TestFullPackAtRest()
{
  BatteryModel battery = MakePack();
  CheckNear(battery.Voltage(), 25.2, 1e-9, "full pack rests at 6 * 4.2 V");
  CheckNear(battery.Current(), 0.0, 1e-9, "idle pack draws nothing");
  CheckNear(battery.SocPct(), 100.0, 1e-9, "full pack reads 100%");

  RunSeconds(battery, 0.0, 60.0);
  CheckNear(battery.Voltage(), 25.2, 1e-6,
            "60 s of no load leaves the voltage alone");
  CheckNear(battery.SocPct(), 100.0, 1e-6,
            "60 s of no load draws no charge");
}

void TestCoulombCounting()
{
  BatteryModel battery = MakePack();

  // 525 W is the ~21 A hover draw measured on this airframe in non-gazebo SITL.
  const double drawnAh = RunSeconds(battery, 525.0, 600.0);

  Check(battery.Current() > 18.0 && battery.Current() < 25.0,
        "a hover-sized load draws a hover-sized current");

  const double lostAh =
      20.89770474 * (100.0 - battery.SocPct()) / 100.0;
  CheckNear(lostAh, drawnAh, 1e-4,
            "charge removed equals the integral of the reported current");

  Check(battery.Voltage() < 25.2, "the pack sags under load");
  Check(battery.Voltage() > 23.0, "10 minutes does not flatten the pack");
}

void TestSagIsResistive()
{
  BatteryModel battery = MakePack();
  RunSeconds(battery, 525.0, 5.0);

  const double soc = battery.SocPct() / 100.0;
  const double resting = 6.0 * BatteryModel::OcvPerCell(soc);
  CheckNear(resting - battery.Voltage(), battery.Current() * 0.013, 1e-6,
            "terminal voltage is the resting voltage less I*R");
}

void TestCurrentIsClamped()
{
  BatteryModel battery = MakePack();
  RunSeconds(battery, 1.0e6, 5.0);
  CheckNear(battery.Current(), 200.0, 1e-3,
            "an absurd shaft power is clamped to the configured maximum");
}

void TestEmptyPackStaysSane()
{
  BatteryModel battery;
  // A small pack, so it empties inside the test.
  battery.Setup(6.0, 0.05, 0.013, 200.0);
  RunSeconds(battery, 525.0, 600.0);

  CheckNear(battery.SocPct(), 0.0, 1e-9, "an emptied pack reads 0%");
  Check(battery.Voltage() > 0.0,
        "an emptied pack still reports a positive voltage");
  Check(battery.Voltage() < 6.0 * 3.34 + 0.1,
        "an emptied pack rests at the bottom of the OCV curve");
}

void TestZeroCapacityPinsVoltage()
{
  BatteryModel battery;
  battery.Setup(6.0, 0.0, 0.013, 200.0);
  RunSeconds(battery, 525.0, 600.0);

  CheckNear(battery.Voltage(), 25.2, 1e-9,
            "zero capacity means an unlimited pack, as in ArduPilot's SITL");
  Check(battery.Current() > 0.0,
        "an unlimited pack still reports the current drawn");
}

void TestVoltageStaysPositive()
{
  // A pack configured with an implausible resistance: 200 A through 0.5 ohm
  // is 100 V of sag against a 25 V pack.
  BatteryModel battery;
  battery.Setup(6.0, 20.89770474, 0.5, 200.0);
  RunSeconds(battery, 5000.0, 30.0);

  Check(battery.Voltage() > 0.0,
        "terminal voltage never reaches zero, whatever the configuration");
  Check(battery.SocPct() > 0.0,
        "the pack is not flattened by the voltage floor alone");
}

void TestPausedSimulationDoesNotDischarge()
{
  BatteryModel battery = MakePack();
  RunSeconds(battery, 525.0, 10.0);

  const double soc = battery.SocPct();
  const double voltage = battery.Voltage();
  for (int i = 0; i < 1000; ++i)
  {
    battery.Update(525.0, 0.0);
    battery.Update(525.0, -0.01);
  }
  CheckNear(battery.SocPct(), soc, 1e-12, "a stopped clock draws no charge");
  CheckNear(battery.Voltage(), voltage, 1e-12,
            "a stopped clock does not move the voltage");
}

void TestOcvCurve()
{
  CheckNear(BatteryModel::OcvPerCell(1.0), 4.2, 1e-9, "full cell is 4.2 V");
  CheckNear(BatteryModel::OcvPerCell(0.0), 3.333, 1e-9,
            "empty cell is 3.333 V");
  CheckNear(BatteryModel::OcvPerCell(2.0), 4.2, 1e-9, "SoC above 1 clamps");
  CheckNear(BatteryModel::OcvPerCell(-1.0), 3.333, 1e-9,
            "SoC below 0 clamps");

  double previous = 0.0;
  for (int i = 0; i <= 100; ++i)
  {
    const double v = BatteryModel::OcvPerCell(i / 100.0);
    Check(v >= previous, "the OCV curve never falls as SoC rises");
    previous = v;
  }
}
}  // namespace

int main()
{
  TestOcvCurve();
  TestFullPackAtRest();
  TestCoulombCounting();
  TestSagIsResistive();
  TestCurrentIsClamped();
  TestEmptyPackStaysSane();
  TestZeroCapacityPinsVoltage();
  TestVoltageStaysPositive();
  TestPausedSimulationDoesNotDischarge();

  if (failures != 0)
  {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("battery_model_test: all checks passed\n");
  return 0;
}

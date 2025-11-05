# Working Configuration - Nov 5, 2025

## ✅ VERIFIED WORKING SETUP

### ArduPilot
- **Commit:** `f622ede` (upstream master, NO custom FP fix)
- **Build:** Release mode (NOT debug)
- **Build command:**
  ```bash
  cd ~/github.com/NERVsystems/ardupilot
  ./waf distclean
  ./waf configure --board sitl  # NO --debug flag!
  ./waf copter
  ```
- **Binary:** `/Users/pdfinn/github.com/NERVsystems/ardupilot/build/sitl/bin/arducopter`
- **Built:** Nov 5, 2025 14:39

### ardupilot_gazebo Plugin
- **Commit:** `2de005a` (Nov 3 cleanup scripts)
- **Build command:**
  ```bash
  cd ~/github.com/NERVsystems/ardupilot_gazebo
  rm -rf build && mkdir build && cd build
  export GZ_VERSION=ionic
  export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:/opt/homebrew"
  cmake ..
  make -j4
  ```

### Homebrew Packages
- **gz-sim9:** 9.4.0_3 (built from source, pure Homebrew)
- **gz-physics8:** 8.3.0_4 (built with eigen@3)
- **gz-rendering9, gz-gui9, gz-transport14, gz-msgs11, etc:** All 9.x/14.x/15.x Ionic versions
- **eigen:** 5.0.0 installed, but **eigen@3 (3.4.1) linked** via `brew link eigen@3 --force`
- **fmt:** 12.1.0
- **dartsim:** 6.15.0_6

### Critical: Anaconda Libraries REMOVED
- **Backed up to:** `/opt/anaconda3/lib/backup_for_gazebo/`
- **Moved:** libQt5*.dylib, libabsl*.dylib, libfmt*.dylib, cmake/ configs
- **Count:** 495 files + cmake directory
- **Why:** Mixed Anaconda/Homebrew libs caused Gazebo build failures and runtime Qt conflicts

### Environment Variables
Set in `run_gazebo.sh` and `start_sitl_gazebo.sh`:
```bash
export GZ_VERSION=ionic
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5"
export PATH="/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:/opt/homebrew/bin:/opt/homebrew/sbin"
```

## ❌ WHAT BROKE IT (DO NOT REPEAT)

### 1. Running `brew install` without constraints
- **Nov 4 15:13:** Ran `brew install <packages>`
- **Result:** Upgraded fmt (11→12) and eigen (3.4→5.0)
- **Consequence:** Triggered rebuild of ALL gz-* packages from source with mixed libs
- **Fix:** Pin packages OR always use `brew install --formula <specific-version>`

### 2. Your FP Fix Commit (f3f02b4)
- **What:** Disabled floating-point exceptions on macOS ARM64
- **Problem:** Broke basic SITL functionality - drone doesn't initialize properly
- **Symptoms:** EKF won't converge, can't arm, drone position invalid
- **Status:** DO NOT USE this commit until debugged further

### 3. Debug Builds
- **Trigger:** sim_vehicle.py with `--debug` flag
- **Problem:** Debug builds had issues with SITL state/initialization
- **Fix:** Always use release builds

### 4. Corrupted SITL State Files
- **Files:** `mav.parm`, `eeprom.bin`, `*.stg` in ArduPilot root
- **Problem:** Saved corrupted state from bad sessions
- **Symptoms:** Drone in "5th dimension", won't appear on map
- **Fix:** Delete before testing: `rm -f ~/github.com/NERVsystems/ardupilot/{mav.parm,eeprom.bin,*.stg}`

## 🔒 LOCKDOWN PROCEDURES

### 1. Pin Homebrew Packages
Create `/opt/homebrew/etc/Brewfile.lock` to prevent auto-upgrades:
```bash
# In your shell startup (.zshrc):
export HOMEBREW_NO_AUTO_UPDATE=1
export HOMEBREW_NO_INSTALL_UPGRADE=1
```

Or pin specific formulas:
```bash
brew pin eigen@3 gz-sim9 gz-physics8 gz-rendering9
```

### 2. Version Check Script
Create `check_versions.sh`:
```bash
#!/bin/bash
echo "Checking critical versions..."
echo "eigen: $(brew list --versions eigen | grep -o '[0-9.]*')"
echo "eigen@3 linked: $(readlink /opt/homebrew/include/eigen3/Eigen/Core | grep -q '3.4' && echo 'YES' || echo 'NO')"
echo "fmt: $(brew list --versions fmt)"
echo "gz-sim9: $(brew list --versions gz-sim9)"
echo "gz-physics8: $(brew list --versions gz-physics8)"
echo "Anaconda libs: $(ls /opt/anaconda3/lib/libQt5*.dylib 2>/dev/null | wc -l) (should be 0)"
echo "ArduCopter: $(stat -f '%Sm' ~/github.com/NERVsystems/ardupilot/build/sitl/bin/arducopter)"
```

### 3. Pre-Flight Checklist
Before running SITL:
```bash
# 1. Check no Anaconda contamination
ls /opt/anaconda3/lib/libQt5*.dylib 2>&1 | grep "No such file"  # Should see this

# 2. Verify eigen@3 linked
ls -la /opt/homebrew/include/eigen3/Eigen/Core | grep "@3"  # Should see eigen@3 in path

# 3. Clean SITL state
rm -f ~/github.com/NERVsystems/ardupilot/{mav.parm,eeprom.bin,*.stg}

# 4. Verify GZ_VERSION set
echo $GZ_VERSION  # Should show "ionic"
```

### 4. Backup Current Working State
```bash
# Backup ArduCopter binary
cp ~/github.com/NERVsystems/ardupilot/build/sitl/bin/arducopter \
   ~/github.com/NERVsystems/ardupilot/build/sitl/bin/arducopter.working.nov5

# Backup Gazebo plugin
cp ~/github.com/NERVsystems/ardupilot_gazebo/build/libArduPilotPlugin.dylib \
   ~/github.com/NERVsystems/ardupilot_gazebo/build/libArduPilotPlugin.dylib.working.nov5
```

## 📝 WHAT WE LEARNED

### Root Causes
1. **Homebrew bottle system fragility:** osrf/simulation removes bottles when core deps upgrade
2. **Source builds mix libraries:** CMake finds both Anaconda and Homebrew, creates conflicts
3. **ArduPilot FP fix side effects:** The crash fix broke SITL initialization
4. **State file corruption:** Bad sessions leave persistent broken state

### The Fix That Worked
1. Remove ALL Anaconda conflicting libraries (Qt5, abseil, fmt)
2. Rebuild Gazebo from source with PURE Homebrew dependencies
3. Use upstream ArduPilot (revert FP fix)
4. Release builds only (no --debug)
5. Clean SITL state files

### What Actually Causes Crashes
- **The crashes you saw** were from `feenableexcept()` on macOS ARM64
- **Your FP fix prevented crashes** but broke SITL initialization somehow
- **Without the fix:** Crashes return, but basic functionality works
- **Trade-off:** Crashes vs Working - pick your poison

## 🚫 DO NOT DO THESE

1. ❌ Run `brew install` without checking what will upgrade
2. ❌ Run `brew upgrade` (will break everything again)
3. ❌ Mix Anaconda and Homebrew libraries
4. ❌ Build ArduPilot with `--debug` flag
5. ❌ Use custom FP fix commit (f3f02b4) without more testing
6. ❌ Keep SITL state files between major changes

## ✅ SAFE TO DO

1. ✅ Use `brew reinstall <formula>` to rebuild specific packages
2. ✅ Build Gazebo plugin with `GZ_VERSION=ionic`
3. ✅ Delete SITL state files (mav.parm, eeprom.bin) when issues arise
4. ✅ Use `./start_sitl_gazebo.sh -w` to wipe and reset
5. ✅ Keep Anaconda libs backed up, restore when needed for other work

## 🔧 TO RESTORE ANACONDA (when not using Gazebo)

```bash
mv /opt/anaconda3/lib/backup_for_gazebo/* /opt/anaconda3/lib/
rmdir /opt/anaconda3/lib/backup_for_gazebo
```

## 📞 WHEN THINGS BREAK AGAIN

**Symptom:** EKF errors, can't arm, drone not on map
**Quick Fix:**
```bash
# 1. Clean state
rm -f ~/github.com/NERVsystems/ardupilot/{mav.parm,eeprom.bin,*.stg}

# 2. Restart with wipe
./start_sitl_gazebo.sh -w  # or start_sitl_standard.sh -w
```

**Symptom:** Gazebo crashes, Qt errors, library conflicts
**Check:**
```bash
ls /opt/anaconda3/lib/libQt5*.dylib  # Should NOT exist
ls /opt/homebrew/include/eigen3 | grep Eigen  # Should exist
```

**Symptom:** "cmake can't find gz-sim9" when building plugin
**Fix:**
```bash
export GZ_VERSION=ionic
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:/opt/homebrew"
```

## 🎯 THE GOLDEN RULE

**NEVER run `brew install` or `brew upgrade` without:**
1. Checking what will change: `brew install --dry-run <formula>`
2. Reviewing dependencies: `brew deps --tree <formula>`
3. Making a backup of working binaries first

**ALWAYS rebuild from known good commits:**
- ardupilot: f622ede
- ardupilot_gazebo: 2de005a

## 🕐 TIME MACHINE RECOVERY (Last Resort)

If Homebrew packages get completely fucked and you can't rebuild, restore from Time Machine:

### 1. Find Working Snapshot
```bash
tmutil listlocalsnapshotdates / | grep "2025-11-05"
# Look for snapshot from Nov 5 14:00 or later (after working rebuild)
```

### 2. Restore Homebrew Cellar
```bash
# Example snapshot: com.apple.TimeMachine.2025-11-05-140000.local
sudo tmutil restore \
  -v /opt/homebrew/Cellar/gz-sim9 \
  /Volumes/.timemachine/*/Backups.backupdb/*/2025-11-05-*/Macintosh\ HD/opt/homebrew/Cellar/gz-sim9

# Restore all critical packages:
for pkg in gz-physics8 gz-rendering9 gz-transport14 gz-msgs11 gz-common6 gz-math8; do
  sudo tmutil restore -v /opt/homebrew/Cellar/$pkg \
    /path/to/snapshot/opt/homebrew/Cellar/$pkg
done
```

### 3. Restore Anaconda Backup
```bash
sudo tmutil restore -v /opt/anaconda3/lib/backup_for_gazebo \
  /path/to/snapshot/opt/anaconda3/lib/backup_for_gazebo
```

### 4. Alternative: Restore Entire /opt/homebrew
**WARNING:** This will revert ALL Homebrew packages to snapshot date!
```bash
# Backup current state first
sudo mv /opt/homebrew /opt/homebrew.broken

# Restore from Time Machine
sudo tmutil restore -v /opt/homebrew \
  /Volumes/.timemachine/*/Backups.backupdb/*/2025-11-05-*/Macintosh\ HD/opt/homebrew

# If that fails, use Finder:
# 1. Open Time Machine
# 2. Navigate to /opt/homebrew
# 3. Select Nov 5 14:00+ snapshot
# 4. Click "Restore"
```

### 5. Verify After Restore
```bash
cd ~/github.com/NERVsystems/ardupilot_gazebo
./verify_setup.sh

# If verification fails, rebuild binaries:
cd ~/github.com/NERVsystems/ardupilot
./waf distclean
./waf configure --board sitl
./waf copter

cd ~/github.com/NERVsystems/ardupilot_gazebo/build
rm -rf *
export GZ_VERSION=ionic
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:/opt/homebrew"
cmake ..
make -j4
```

### 6. What Time Machine Backs Up
✅ **Homebrew packages:** /opt/homebrew/Cellar/
✅ **Anaconda backup:** /opt/anaconda3/lib/backup_for_gazebo/
✅ **Your repos:** ~/github.com/NERVsystems/
✅ **Working binaries:** build/sitl/bin/arducopter*, build/libArduPilotPlugin.dylib*

### 7. What Time Machine DOESN'T Back Up
❌ **Build artifacts:** ~/github.com/.../build/* (excluded by default)
❌ **SITL state files:** mav.parm, eeprom.bin (you don't want corrupted ones anyway)

### 8. Create Manual Backup NOW
Since Time Machine might skip some things:
```bash
# Backup critical Homebrew packages
mkdir -p ~/Backups/gazebo-working-nov5
cd /opt/homebrew/Cellar
tar czf ~/Backups/gazebo-working-nov5/gz-packages.tar.gz \
  gz-sim9 gz-physics8 gz-rendering9 gz-transport14 gz-msgs11 \
  gz-common6 gz-math8 gz-plugin3 gz-utils3 sdformat15

# Backup Anaconda
tar czf ~/Backups/gazebo-working-nov5/anaconda-backup.tar.gz \
  /opt/anaconda3/lib/backup_for_gazebo/

# Verify
ls -lh ~/Backups/gazebo-working-nov5/
```

### 9. To Restore from Manual Backup
```bash
cd /opt/homebrew/Cellar
sudo tar xzf ~/Backups/gazebo-working-nov5/gz-packages.tar.gz

cd /opt/anaconda3/lib
tar xzf ~/Backups/gazebo-working-nov5/anaconda-backup.tar.gz

brew link --overwrite gz-sim9 gz-physics8 gz-rendering9
```

### 10. Nuclear Option - Fresh Start
If everything is truly fucked beyond repair:
```bash
# 1. Uninstall ALL Gazebo
brew uninstall --ignore-dependencies gz-*

# 2. Move Anaconda libs away
mkdir -p /opt/anaconda3/lib/backup_for_gazebo
mv /opt/anaconda3/lib/{libQt5*,libabsl*,libfmt*,cmake} /opt/anaconda3/lib/backup_for_gazebo/

# 3. Clean install
brew install gz-sim9  # Will build from source with pure Homebrew

# 4. Rebuild ArduPilot
cd ~/github.com/NERVsystems/ardupilot
git checkout f622ede
./waf distclean
./waf configure --board sitl
./waf copter

# 5. Rebuild plugin
cd ~/github.com/NERVsystems/ardupilot_gazebo
git checkout 2de005a
rm -rf build && mkdir build && cd build
export GZ_VERSION=ionic
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:/opt/homebrew"
cmake ..
make -j4
```

**Estimated recovery time:**
- Time Machine restore: 5-10 minutes
- Manual tar backup restore: 2 minutes
- Fresh rebuild: 20-30 minutes

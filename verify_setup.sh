#!/bin/bash
# Verify Working Configuration - Run before flight tests

echo "🔍 Checking NERV UAS Gazebo Setup..."
echo ""

ERRORS=0

# Check ArduPilot commit
cd ~/github.com/NERVsystems/ardupilot
COMMIT=$(git rev-parse HEAD | cut -c1-7)
if [ "$COMMIT" != "f622ede" ]; then
    echo "❌ ArduPilot commit is $COMMIT (expected f622ede)"
    ERRORS=$((ERRORS + 1))
else
    echo "✅ ArduPilot at working commit (f622ede)"
fi

# Check ardupilot_gazebo commit
cd ~/github.com/NERVsystems/ardupilot_gazebo
COMMIT=$(git rev-parse HEAD | cut -c1-7)
if [ "$COMMIT" != "2de005a" ]; then
    echo "❌ ardupilot_gazebo commit is $COMMIT (expected 2de005a)"
    ERRORS=$((ERRORS + 1))
else
    echo "✅ ardupilot_gazebo at working commit (2de005a)"
fi

# Check Anaconda libraries NOT present
ANACONDA_QT=$(ls /opt/anaconda3/lib/libQt5*.dylib 2>/dev/null | wc -l | tr -d ' ')
if [ "$ANACONDA_QT" != "0" ]; then
    echo "❌ Anaconda Qt libraries found ($ANACONDA_QT files) - will cause conflicts!"
    ERRORS=$((ERRORS + 1))
else
    echo "✅ No Anaconda library conflicts"
fi

# Check backup exists
BACKUP_COUNT=$(ls /opt/anaconda3/lib/backup_for_gazebo/ 2>/dev/null | wc -l | tr -d ' ')
if [ "$BACKUP_COUNT" -lt "400" ]; then
    echo "⚠️  Warning: Anaconda backup seems incomplete ($BACKUP_COUNT files)"
else
    echo "✅ Anaconda libraries safely backed up ($BACKUP_COUNT files)"
fi

# Check eigen@3 linked
if readlink /opt/homebrew/include/eigen3/Eigen/Core 2>/dev/null | grep -q "3.4"; then
    echo "✅ eigen@3 (3.4.1) correctly linked"
else
    echo "❌ eigen@3 NOT linked - run: brew link eigen@3 --force"
    ERRORS=$((ERRORS + 1))
fi

# Check Gazebo packages installed
for pkg in gz-sim9 gz-physics8 gz-rendering9; do
    if brew list $pkg &>/dev/null; then
        echo "✅ $pkg installed"
    else
        echo "❌ $pkg NOT installed"
        ERRORS=$((ERRORS + 1))
    fi
done

# Check working binaries backed up
if [ -f ~/github.com/NERVsystems/ardupilot/build/sitl/bin/arducopter.working.nov5.f622ede ]; then
    echo "✅ ArduCopter backup exists"
else
    echo "⚠️  Warning: No ArduCopter backup found"
fi

if [ -f ~/github.com/NERVsystems/ardupilot_gazebo/build/libArduPilotPlugin.dylib.working.nov5.2de005a ]; then
    echo "✅ ArduPilot plugin backup exists"
else
    echo "⚠️  Warning: No plugin backup found"
fi

# Check GZ_VERSION
if [ -z "$GZ_VERSION" ]; then
    echo "⚠️  Warning: GZ_VERSION not set - export GZ_VERSION=ionic before building"
else
    if [ "$GZ_VERSION" = "ionic" ]; then
        echo "✅ GZ_VERSION=ionic"
    else
        echo "❌ GZ_VERSION=$GZ_VERSION (expected ionic)"
        ERRORS=$((ERRORS + 1))
    fi
fi

# Summary
echo ""
if [ $ERRORS -eq 0 ]; then
    echo "✅ All checks passed - setup is correct!"
    echo ""
    echo "To launch:"
    echo "  Terminal 1: cd ~/github.com/NERVsystems/ardupilot_gazebo && export GZ_VERSION=ionic && ./run_gazebo.sh"
    echo "  Terminal 2: cd ~/github.com/NERVsystems/ardupilot_gazebo && ./start_sitl_gazebo.sh"
    exit 0
else
    echo "❌ Found $ERRORS error(s) - review WORKING_CONFIG.md"
    exit 1
fi

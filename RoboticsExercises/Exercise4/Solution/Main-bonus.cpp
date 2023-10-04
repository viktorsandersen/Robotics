

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/Path.hpp>

#include <filesystem>

using namespace rw::loaders;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::kinematics;

bool withinLimit (Device::Ptr device, LinearInterpolator< Q > interp)
{
    Q velLimit   = device->getVelocityLimits ();
    Q accelLimit = device->getAccelerationLimits ();

    for (double t = 0; t < interp.duration (); t += 0.05) {
        for (size_t i = 0; i < device->getDOF (); i++) {
            if (abs (interp.dx (t)[i]) > velLimit[i]) {
                return false;
            }
            if (abs (interp.ddx (t)[i]) > accelLimit[i]) {
                return false;
            }
        }
    }
    return true;
}

TimedStatePath linInterp (Device::Ptr device, State state, Q from, Q to)
{
    TimedStatePath res;

    double minDur = 0.0001;
    double maxDur = 10;

    while (!withinLimit (device, LinearInterpolator< Q > (from, to, maxDur))) {
        maxDur += 10;
    }

    double duration = (maxDur + minDur) / 2;
    for (size_t i = 0; i < 100; i++) {
        if (withinLimit (device, LinearInterpolator< Q > (from, to, duration))) {
            maxDur = duration;
        }
        else {
            minDur = duration;
        }
        duration = (maxDur + minDur) / 2;
    }

    LinearInterpolator< Q > interp (from, to, maxDur);

    for (double i = 0; i < duration; i += 0.05) {
        device->setQ (interp.x (i), state);
        res.push_back (TimedState (i, state));
    }

    return res;
}

int main (int argc, char** argv)
{
    // load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (
        std::filesystem::path (__FILE__).parent_path ().parent_path () / "WorkCell/Scene.wc.xml");
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // find Device
    Device::Ptr robotUR5 = wc->findDevice ("UR5");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }

    // define movement
    Q start (-0.957365, -2.06061, -1.34591, -4.48342, -1.62694, 2.18569);
    Q end (-0.213454, -1.98838, -2.32209, 0.332119, -2.68517, 5.9914);

    TimedStatePath linearQMotion = linInterp (robotUR5, wc->getDefaultState (), start, end);
    PathLoader::storeTimedStatePath (*wc, linearQMotion, "./visu-ex4bonus.rwplay");
}
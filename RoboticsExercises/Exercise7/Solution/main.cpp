#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/Path.hpp>

#include <filesystem>
#include <iostream>
#include <string>

using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::math;
using namespace rw::kinematics;

#define STEPSIZE 1 * Deg2Rad

struct PathChecker
{
    WorkCell::Ptr wc;
    Device::Ptr dev;
    CollisionDetector::Ptr col;

    bool inCollision (Q from, Q to)
    {
        State state = wc->getDefaultState ();

        LinearInterpolator< Q > p (from, to, (from - to).norm2 ());
        for (double step = STEPSIZE; step < p.duration (); step += STEPSIZE) {
            dev->setQ (p.x (step), state);
            if (col->inCollision (state)) {
                return true;
            }
        }
        return false;
    }

    double pathLength (TimedStatePath p)
    {
        double d = 0;
        Q last   = dev->getQ (p.front ().getValue ());
        for (TimedState ts : p) {
            Q current = dev->getQ (ts.getValue ());
            d += (current - last).norm2 ();
            last = current;
        }
        return d;
    }
};

int main (int argc, char** argv)
{
    std::string wc_file;
    std::string path_file;

    if (argc != 3) {
        std::cout << "PathPruning </path/to/workcell.wc.xml> </path/to/playback.rwplay> "
                  << std::endl;

        wc_file = std::filesystem::path (__FILE__).parent_path () / "Kr16WallWorkCell/Scene.wc.xml";
        path_file = std::filesystem::path (__FILE__).parent_path () / "ToBePruned.rwplay";
    }
    else {
        wc_file   = argv[1];
        path_file = argv[2];
    }

    rw::models::WorkCell::Ptr wc = WorkCellLoader::Factory::load (wc_file);
    if (wc.isNull ()) {
        RW_THROW ("WorkCell Not Found at: " << wc_file);
    }

    rw::trajectory::TimedStatePath path = PathLoader::loadTimedStatePath (wc, path_file);
    if (path.empty ())
        RW_THROW ("Path Not Found at: " << path_file);

    rw::models::Device::Ptr dev = wc->findDevice ("KukaKr16");
    if (dev.isNull ())
        RW_THROW ("Device Not Found");

    wc->setDefaultState (path.front ().getValue ());

    PathChecker check = {wc,
                         dev,
                         rw::core::ownedPtr (new CollisionDetector (
                             wc, CollisionStrategy::Factory::makeStrategy ("PQP")))};

    std::cout << "PathLength: " << check.pathLength (path) << std::endl;

    int i = 0;
    while (i < (int) path.size () - 2) {
        Q from = dev->getQ (path[i].getValue ());
        Q to   = dev->getQ (path[i + 2].getValue ());

        if (check.inCollision (from, to)) {
            i++;
        }
        else {
            path.erase (path.begin () + i + 1);
            if (i > 0) {
                i--;
            }
        }
        std::cout << "Progress: " << i << " / " << path.size () - 2 << "                       \r";
    }
    std::cout << "\n Done" << std::endl;

    std::cout << "PathLength: " << check.pathLength (path) << std::endl;

    rw::loaders::PathLoader::storeTimedStatePath (*wc, path, "PrunedPath.rwplay");

    return 0;
}

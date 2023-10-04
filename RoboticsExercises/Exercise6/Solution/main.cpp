#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 60.

bool checkCollisions (Device::Ptr device, const State& state, const CollisionDetector& detector,
                      const Q& q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ (q, testState);
    colFrom = detector.inCollision (testState, &data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin (); it != fps.end (); it++) {
            cerr << (*it).first->getName () << " " << (*it).second->getName () << endl;
        }
        return false;
    }
    return true;
}

#define EXTEND 0.1
#define TRIALS 3

int main (int argc, char** argv)
{
    ofstream mydata;
    mydata.open ("ROBDATA.dat");
    mydata << "time\tdistance\teps\tsteps"
           << "\n";
    mydata.close ();

    mydata.open ("ROBDATA.dat", std::ios_base::app);
    int index = 0;
    std::cout << "Starting" << std::endl;

    for (double extend = 0.02; extend <= 1.0; extend += EXTEND) {
        for (int trial = 0; trial < TRIALS; trial++) {
            std::cout << "Progress: " << (extend + trial * EXTEND / TRIALS) * 100
                      << " %          \r" << std::flush;
            const string deviceName = "KukaKr16";
            rw::math::Math::seed ();
            WorkCell::Ptr wc = WorkCellLoader::Factory::load (
                std::filesystem::path (__FILE__).parent_path () / "Kr16WallWorkCell/Scene.wc.xml");
                
            Frame* tool_frame   = wc->findFrame ("Tool");
            Frame* bottle_frame = wc->findFrame ("Bottle");

            Device::Ptr device = wc->findDevice (deviceName);
            if (device == NULL) {
                cerr << "Device: " << deviceName << " not found!" << endl;
                return 0;
            }

            State state = wc->getDefaultState ();
            Q from (6, -3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
            Q to (6, 1.571, 0.006, 0.030, 0.153, 0.762, 4.490);
            device->setQ (from, state);
            Kinematics::gripFrame (bottle_frame, tool_frame, state);
            CollisionDetector detector (wc,
                                        ProximityStrategyFactory::makeDefaultCollisionStrategy ());
            PlannerConstraint constraint = PlannerConstraint::make (&detector, device, state);

            QSampler::Ptr sampler    = QSampler::makeConstrained (QSampler::makeUniform (device),
                                                               constraint.getQConstraintPtr ());
            QMetric::Ptr metric      = MetricFactory::makeEuclidean< Q > ();
            QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner (
                constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

            if (!checkCollisions (device, state, detector, from))
                return 0;
            if (!checkCollisions (device, state, detector, to))
                return 0;

            // cout << "Planning from " << from << " to " << to << endl;
            QPath path;
            Timer t;
            t.resetAndResume ();
            planner->query (from, to, path, MAXTIME);
            t.pause ();

            // cout << "Path of length " << path.size() << " found in " << t.getTime() << "
            // seconds." << endl;
            if (t.getTime () >= MAXTIME) {
                cout << "\nNotice: max time of " << MAXTIME << " seconds reached." << endl;
            }
            // visualize them
            if (path.size () >= 2) {
                double distance = 0;
                Q last (path.front ());
                for (Q q : path) {
                    distance += (q - last).norm2 ();
                }

                mydata << t.getTime () << "\t" << distance << "\t" << extend << "\t" << path.size ()
                       << "\n";

                double time = 0.0;
                TimedStatePath tStatePath;
                for (size_t i = 0; i < path.size (); i += 1) {
                    device->setQ (path.at (i), state);
                    tStatePath.push_back (TimedState (time, state));
                    time += 0.01;
                }

                rw::loaders::PathLoader::storeTimedStatePath (
                    *wc, tStatePath, "visu" + std::to_string (index++) + ".rwplay");
            }
        }
    }
    std::cout << "Progress: " << 100 << " %          " << std::endl;

    mydata.close ();
    return 0;
}

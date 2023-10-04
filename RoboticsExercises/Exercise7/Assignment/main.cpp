#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/Path.hpp>

#include <iostream>
#include <string>

using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::math;
using namespace rw::kinematics;


int main (int argc, char** argv)
{
    if (argc != 3) {
        std::cout << "PathPruning </path/to/workcell.wc.xml> </path/to/playback.rwplay> "
                  << std::endl;
        return 1;
    }

    std::string wc_file   = argv[1];
    std::string path_file = argv[2];

    rw::models::WorkCell::Ptr wc        = WorkCellLoader::Factory::load (wc_file);
    if(wc.isNull()) RW_THROW("WorkCell Not Found");

    rw::trajectory::TimedStatePath path = PathLoader::loadTimedStatePath (wc, path_file);
    if(path.empty()) RW_THROW("Path Not Found");

    rw::models::Device::Ptr dev         = wc->findDevice ("KukaKr16");
    if(dev.isNull()) RW_THROW("Device Not Found");

    // Implement the Pruning Algorithm here

    //How to erase element from path
    //path.erase(path.begin()+index);


    // Save Result
    rw::loaders::PathLoader::storeTimedStatePath (*wc, path, "PrunedPath.rwplay");

    return 0;
}

#include <iostream>
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

using namespace std;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

int main(int argc, char** argv) {
	WorkCell::Ptr wc =rw::loaders::WorkCellLoader::Factory::load("/home/kalor/Workspace/roviexercises/Robotics Exercises/Exercise 2/Exercise1P3-Solution/Scene.wc.xml");
	Device::Ptr device = wc->findDevice("UR-6-85-5-A");
	State state = wc->getDefaultState();
	Q q = device->getQ(state);

	return 0;
}

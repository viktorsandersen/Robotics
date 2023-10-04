#include <iostream>
#include <vector>
#include <rw/rw.hpp>
#include <numeric>
#include <functional>
#include <filesystem>

rw::math::Transform3D<> forwardKinematics(const std::vector<rw::math::Transform3D<>>&  trefs,
					  const unsigned int idx, const rw::math::Q&  q) {
  if(trefs.size() != q.size()) {
    RW_THROW("The number of local transformations must be equal to the length of the configuration vector");
  }

  rw::math::Transform3D<> baseTi;

  for(unsigned int i = 0; i < idx; ++i) {
    rw::math::Transform3D<> Tz(rw::math::RPY<>(q[i], 0, 0).toRotation3D());
    baseTi = baseTi*trefs[i]*Tz;
  }

  return baseTi;
}

std::vector<rw::math::Transform3D<>> generateTrefs();

int main() {
  // Manual version
  // Create Tref vector
  const std::vector<rw::math::Transform3D<>> Trefs = generateTrefs();

  // The transformation from Joint 5 to the TCP
  // TCP: <RPY> 270 0 0 </RPY> <Pos> 0 0 0.082 </Pos>
  const rw::math::Vector3D<> VTCP(0, 0, 0.082);
  const rw::math::RPY<> RTCP(270*rw::math::Deg2Rad, 0, 0);
  const rw::math::Transform3D<> TTCP(VTCP, RTCP.toRotation3D());

  // The configuration to calculate the forward dynamics for
  const rw::math::Q q(6, 0.859, 0.208, -0.825, -0.746,  -1.632, 1.527);
  const rw::math::Transform3D<> baseTtcp_manual = forwardKinematics(Trefs, 6, q)*TTCP;

  std::cout << "Manual: \n" << baseTtcp_manual.P() << " " << rw::math::RPY<>(baseTtcp_manual.R()) << std::endl;

  // RobWork version using the result from Programming Exercise 3.3
  // Load RW workcell and compare
  // Load workcell and get device
  
  const std::string workcell_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "Exercise1/Solution/Scene.wc.xml";
  const std::string device_name = "UR-6-85-5-A";

  rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(workcell_path);
  if(wc.isNull()){
    RW_THROW("WorkCell not Found");
  }
  rw::models::Device::Ptr device = wc->findDevice(device_name);
  if(device == nullptr) {
      RW_THROW("Device " << device_name << " was not found!");
  }

  // Set robot state, q
  rw::kinematics::State state = wc->getDefaultState();
  device->setQ(q, state);

  // Compute baseTtcp
  rw::kinematics::Frame* tcp_frame = wc->findFrame(device_name + ".TCP");
  if(tcp_frame == nullptr) {
      RW_THROW("TCP frame not found!");
  }

  const rw::math::Transform3D<> baseTtool_rw = device->baseTframe(tcp_frame, state);

  std::cout << "RW:\n" << baseTtool_rw.P() << " " << rw::math::RPY<>(baseTtool_rw.R()) << std::endl; 

  return 0;
}

std::vector<rw::math::Transform3D<>> generateTrefs() {
  // Create Tref vector
  // Joint 0: <RPY> 0 0 0 </RPY> <Pos> 0 0 0 </Pos>
  rw::math::Vector3D<> V0(0, 0, 0);
  rw::math::RPY<> R0(0, 0, 0);
  rw::math::Transform3D<> T0(V0, R0.toRotation3D());

  // Joint 1: <RPY> 90 0 90 </RPY> <Pos> 0 0 0.08920 </Pos>
  rw::math::Vector3D<> V1(0, 0, 0.08920);
  rw::math::RPY<> R1(90*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
  rw::math::Transform3D<> T1(V1, R1.toRotation3D());

  // Joint 2: <RPY> 270 0 0 </RPY> <Pos> 0 0.425 0 </Pos>
  rw::math::Vector3D<> V2(0, 0.425, 0);
  rw::math::RPY<> R2(270*rw::math::Deg2Rad, 0, 0);
  rw::math::Transform3D<> T2(V2, R2.toRotation3D());

  // Joint 3: <RPY> 0 0 0 </RPY> <Pos> -0.39243 0 0 </Pos>
  rw::math::Vector3D<> V3(-0.39243, 0, 0);
  rw::math::RPY<> R3(0, 0, 0);
  rw::math::Transform3D<> T3(V3, R3.toRotation3D());

  // Joint 4: <RPY> 270 0 90 </RPY> <Pos> 0 0 0.109 </Pos>
  rw::math::Vector3D<> V4(0, 0, 0.109);
  rw::math::RPY<> R4(270*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
  rw::math::Transform3D<> T4(V4, R4.toRotation3D());
  
  // Joint 5: <RPY> 0 0 270 </RPY> <Pos> 0 0 0.093 </Pos>
  rw::math::Vector3D<> V5(0, 0, 0.093);
  rw::math::RPY<> R5(0, 0, 270*rw::math::Deg2Rad);
  rw::math::Transform3D<> T5(V5, R5.toRotation3D());
  
  return {T0, T1, T2, T3, T4, T5};
}

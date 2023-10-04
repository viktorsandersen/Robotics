# RobWorkCheatSheet

- `rw::<*>::Ptr` : a smart pointer definition of the given Type, but can also handle raw pointers.
  - the type can be found as `rw::core::Ptr<T>`
  - to create one use `rw::core::ownedPtr(new T())`;
- `rw::models::WorkCell` : contains information of what is in our World
- `rw::models::Device`   : a robotiq device that can be moved around by giving it a Q vector
- `rw::models::SerialDevice`: a specialization of device for serial devices
- `rw::kinematics::Frame`: a named position and orientation in the world
- `rw::kinematics::MovableFrame`: a named position and orientation in the world, but this one can be moved from its default position
- `rw::kinematics::State`: the state of the world. ie. where everything is in the world
  - Often created by calling getDefaultState() from a workcell
- `rw::kinematics::Kinematics`: a class containing a many useful static functions for calculating kinematics
- `rw::math::Jacobian`: a representation of the Jacobian Matrix
- `rw::math::Transform3D<T>`: a Transformation matrix in 3D
- `rw::math::Rotation3D<T>`: a Rotation matrix in 3D
- `rw::math::Vector3D<T>`: a 3D Vector
- `rw::math::Vector2D<T>`: a 2D Vector
- `rw::math::RPY<T>`: Roll Pitch Yaw angles with rotations given as {z,y,x}
  - use function .toRotation3D() to convert to a Rotation3D
- `rw::math::Q`: a list of double most often used to symbolize the robot configuration, but sometimes used as a generic list of doubles.
- `rw::math::Deg2Rad`: a converter constant from degrees to radians
- `rw::math::Rad2Deg`: a converter constant from radians to degrees
- `rw::loaders::WorkCellLoader::Factory::load`: a static function for loading a workcell form file
- `rw::loaders::PathLoader` a class of static functions to load and save playback files for RobWorkStudio
- `rw::invkin::ClosedFormIKSolverUR`: inverse kinematic solver for URRobots
- `rw::trajectory::Path<T>`: a renamed std::vector
- `rw::trajectory::Timed<T>`: a pair of values, time and T
- `rw::trajectory::TimedStatePath`: typedef of `Path<Timed<State>>`
- `rw::trajectory::QPath` : typedef of `Path<Q>`
- `rw::trajectory::LinearInterpolator<T>`: RobWork implementation of Y=a*X+b using type T
- `rw::trajectory::ParabolicBlend<T>`: a Parabolic blend, between two LinearInterpolators
- `rw::proximity::CollisionDetector`: used to detect if a rw::kinematic::State is in collision
  - use `rwlibs::proximitystrategies::ProximityStrategyFactory` to get one of the build in Collision Strategies or use `rw::proximity::CollisionStrategy::Factory::makeStrategy`
- `rw::pathplanning::PlannerConstraint` : this class is used to constrain the pathplanning in what it can and can't do.
  - often created with PlannerConstraint::make (detector, device, state), which just limits the path planning to be in a collision free state
- `rw::pathplanning::QSampler`: used to declare how the pathplanner should sample the workspace.
  - `QSampler::makeUniform(device)`: create a sampler that creates uniform random positions within the bounds of the device.
  - `QSampler::makeConstrained(sample,constrained)`: used to constrain another sampler, ie. with the PlannerConstraint, so it won't find a position in collision
- `rw::pathplanning::QMetric`: how should the distance between two samples be measured
  - you can use `MetricFactory::makeEuclidean< Q > ();` to make it use Euclidean distance for type Q
- `rw::pathplanning::QToQPlanner` abstract class for any pathplanner
  - you can use `rwlibs::pathplanners::RRTPlanner` to create different RRTPlanners using its makeQToQPlanner function

## Template Classes in RobWork

In RobWork you will often encounter Template Classes. These are often used to choose how the data in the class should be stored.

For the Math classes:

- `rw::math::Transform3D<T>`
- `rw::math::Rotation3D<T>`
- `rw::math::Vector3D<T>`
- `rw::math::Vector2D<T>`
- `rw::math::RPY<T>`

The underlying data is most often used as double, but it could also be int, char or any other data type that has the common math operator +-*/= if in doubt you will get a compiler error if it is wrong.

For the Trajectory Classes it is most common to use one of RobWorks math classes, but some of them like path and timed don't care at all what the data type is as they are only storing it and not working with it

- `rw::trajectory::Timed<T>`
- `rw::trajectory::Path<T>`
- `rw::trajectory::QPath`
- `rw::trajectory::LinearInterpolator<T>` : any type that can handle the this math expression T = T * double + T
- `rw::trajectory::ParabolicBlend<T>` : T depends on which T has been chosen for the LinearInterpolators it should blend.

## Eigen library

The RobWork Math library is build on top of an external library called Eigen. So when the math functions has a .e() function it means that it is returning it's internal Eigen matrix/Vector.

read more about eigen library here <https://eigen.tuxfamily.org/>

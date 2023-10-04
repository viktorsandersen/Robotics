#include "Tree.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/invkin.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders.hpp>
#include <rw/math.hpp>
#include <rw/models.hpp>
#include <rw/proximity.hpp>
#include <rw/trajectory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

using namespace rw::core;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;

CollisionDetector::Ptr detector;
WorkCell::Ptr wc;
SerialDevice::Ptr device;
State state;
Node::Ptr closest;
double dist;

double compute_task_error (Q q_s, Q q_near)
{
    //(C, Tt0) ← RETRIEVE CONSTRAINT(qs, qnear);
    Eigen::Matrix< double, 6, 6 > C = Eigen::Matrix< double, 6, 6 >::Zero ();
    C (3, 3)                        = 1;
    C (4, 4)                        = 1;
    C (5, 5)                        = 1;

    // 0Te ← FORWARD KINEMATICS(qs);
    Frame::Ptr base_frame   = wc->findFrame ("UR-6-85-5-A.Base");
    Frame::Ptr bottle_frame = wc->findFrame ("Bottle");
    device->setQ (q_s, state);

    // tTe ← tT0 0Te;
    Transform3D< double > base_t_task (RPY< double > (-Pi / 2, 0, Pi / 2).toRotation3D ());
    Transform3D< double > base_t_bottle = Kinematics::frameTframe (base_frame, bottle_frame, state);
    Transform3D< double > task_t_bottle = inverse (base_t_task) * base_t_bottle;

    //∆x ← TASK COORDINATES(Tt);
    RPY< double > rpy (task_t_bottle.R ());
    Eigen::MatrixXd dx (6, 1);

    dx << task_t_bottle.P ().e (), Vector3Dd (rpy[2], rpy[1], rpy[0]).e ();

    //∆xerr ← C∆x
    Eigen::Matrix< double, 6, 1 > dx_error = C * dx;

    // return ∆xerr ;
    return dx_error.norm ();
}

bool rgd_new_config (Q& q_s, Q q_near, double eps = 2e-3)
{
    // i ← 0; j ← 0;
    int i = 0;
    int I = 1000;
    int j = 0;
    int J = 100;

    //∆xerr ← COMPUTE TASK ERROR(qs, qnear );

    // while i < I and j < J and |∆xerr | > eps
    
        // do i ← i + 1; j ← j + 1;
      

        // q`s = qs + RANDOM DISPLACEMENT(dmax);
       
        //∆x′err ← COMPUTE TASK ERROR(qs, qnear );
        
        // if ∆xerr < ∆xerr
       
            // then j ← 0; qs = qs; ∆xerr = ∆xerr ;
          
        // if ∆xerr ≤ eps
       
            // then if IN COLLISION(qs)
            
                // then return false;
               
                // else return true;
                
    // return false
}

Tree task_constrained_rrt (Node::Ptr q_init, Node::Ptr q_goal, double dt)
{
    // T .init(qinit);
    Tree T (q_init);
    closest = q_init;
    dist    = (q_init->q - q_goal->q).norm2 ();

    double A    = 1e6;
    Q joint_max = Q (180.0, 90.0, 180.0, 90.0, 180.0, 180.0) * Deg2Rad;
    Q joint_min = Q (-180.0, -270.0, -180.0, -270.0, -180.0, -180.0) * Deg2Rad;

    // for a = 1 to A
    
        // do qrand ← RANDOM CONFIG;
        
        // qnear ← NEAREST NEIGHBOR(qrand, T );
        
        // qdir ← (qrand − qnear )/|qrand − qnear |;
        
        // qs = qnear + qdir ∆t;
        
        // if *CONSTRAINED* NEW CONFIG(qs, qnear )
            // then T . add vertex (qs);
            
            // T . add edge (qnear , qs);
            

            // Progress Info
            double dist_t_goal = 0 ;//(sQ_n->q - q_goal->q).norm2 ();
            if (dist_t_goal < dist) {
                //closest =  sQ_n;
                dist    = dist_t_goal;
                //std::cout << "[" << a << "] Distance to goal:" << dist_t_goal << " at: " << sQ_n->q
                //          << std::endl;
            }
            // Make it exit early when the solution is close enough
            if (dist_t_goal < dt) {
                //T.add_edge (sQ_n, q_goal);
                std::cout << "Found a route" << std::endl;
                //break;
            }
        
    

    // return T
    return T;
}

std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                    rw::models::SerialDevice::Ptr robot,
                                    rw::models::WorkCell::Ptr wc, State& state)
{
    // Get, make and print name of frames
    const std::string robotName     = robot->getName ();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp  = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame::Ptr goal_f      = wc->findFrame (nameGoal);
    Frame::Ptr tcp_f       = wc->findFrame (nameTcp);
    Frame::Ptr robotBase_f = wc->findFrame (nameRobotBase);
    Frame::Ptr robotTcp_f  = wc->findFrame (nameRobotTcp);
    if (goal_f.isNull () || tcp_f.isNull () || robotBase_f.isNull () || robotTcp_f.isNull ()) {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (goal_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (tcp_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameRobotBase
                  << "\": " << (robotBase_f.isNull () ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp
                  << "\": " << (robotTcp_f.isNull () ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    Transform3D<> baseTGoal    = Kinematics::frameTframe (robotBase_f, goal_f, state);
    Transform3D<> tcpTRobotTcp = Kinematics::frameTframe (tcp_f, robotTcp_f, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = baseTGoal * tcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
        rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (robot, state));

    return closedFormSovler->solve (targetAt, state);
}

int main (int argc, char** argv)
{
    std::string wcFile = "/home/kalor/Workspace/roviexercises/RoboticsExercises/Exercise8/Solution/"
                         "WorkCellObstacle/Scene.wc.xml";
    std::string deviceName = "UR-6-85-5-A";
    Math::seed ();

    wc = WorkCellLoader::Factory::load (wcFile);
    if (wc.isNull ()) {
        std::cout << "WorkCell: \"" << wcFile << "\" not found!" << std::endl;
        return 1;
    }

    Frame::Ptr tool_frame   = wc->findFrame ("GraspTCP");
    Frame::Ptr bottle_frame = wc->findFrame ("Bottle");

    device = wc->findDevice< SerialDevice > (deviceName);

    if (device.isNull ()) {
        std::cout << "Device: " << deviceName << " not found!" << std::endl;
        return 1;
    }
    detector = ownedPtr (
        new CollisionDetector (wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ()));

    state                      = wc->getDefaultState ();
    std::vector< Q > solutions = getConfigurations ("Bottle", "GraspTCP", device, wc, state);

    Q from;
    for (size_t i = 0; i < solutions.size (); i++) {
        // set the robot in that configuration and check if it is in collision
        device->setQ (solutions[i], state);
        if (!detector->inCollision (state)) {
            from = solutions[i];    // save it
            break;                  // we only need one
        }
    }
    std::cout << "From:" << from << std::endl;
    device->setQ (from, state);
    Kinematics::gripFrame (bottle_frame, tool_frame, state);

    solutions = getConfigurations ("BottleGoal", "Bottle", device, wc, state);
    Q to;

    for (size_t i = 0; i < solutions.size (); i++) {
        // set the robot in that configuration and check if it is in collision
        device->setQ (solutions[i], state);
        if (!detector->inCollision (state)) {
            to = solutions[i];    // save it
            break;                // we only need one
        }
    }
    std::cout << "To:" << to << std::endl;
    Node::Ptr from_n = ownedPtr (new Node (from));
    Node::Ptr to_n   = ownedPtr (new Node (to));

    Tree tree = task_constrained_rrt (from_n, to_n, 0.05);

    std::cout << "From:    " << from << std::endl;
    std::cout << "To:      " << to << std::endl;
    std::cout << "Closest: " << closest->q << " dist to goal: " << (to - closest->q).norm2 ()
              << std::endl;

    // visualize them
    TimedStatePath tStatePath;
    double time = 0;
    for (Node::Ptr& n : tree.get_route (to_n)) {
        device->setQ (n->q, state);
        tStatePath.push_back (TimedState (time, state));
        time += 0.01;
    }
    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "visu.rwplay");

    TimedStatePath tStatePathClose;
    time = 0;
    for (Node::Ptr& n : tree.get_route (closest)) {
        std::cout << "Node: " << n->q << std::endl;
        device->setQ (n->q, state);
        tStatePathClose.push_back (TimedState (time, state));
        time += 0.01;
    }
    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePathClose, "visuClosest.rwplay");
}
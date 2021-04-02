/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <mutex>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Conversions.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/PID.hh>

#include "RobotiqController.hh"

#define WHEEL_NUM 4
using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace std;


enum class GripperStatus { keep_close,
    keep_open,
    move_close,
    move_open };

class ignition::gazebo::systems::RobotiqControllerPrivate
{
public:

public:
    void OnCmd(const ignition::msgs::Boolean  &_msg);

public:
    transport::Node node;
    //model
    Model model{kNullEntity};
    //gripper joint
    vector<string> jointNames{"finger_joint","left_inner_knuckle_joint","left_inner_finger_joint",
                    "right_outer_knuckle_joint","right_inner_knuckle_joint","right_inner_finger_joint"};
    vector<Entity> jointEntities;
    vector<int> jointMultipliers{1,-1,1,-1,-1,1};
    vector<ignition::math::PID> jointPids;
    //base link and grasp link(for contact detection and effort control)
    Entity gripperFingerLink[2];
    Entity gripperBaseLink;
    //gripper cmd
    std::mutex statusMutex;
    GripperStatus gripperStatus = GripperStatus::keep_open;
};

/******************implementation for RobotiqController************************/
RobotiqController::RobotiqController() : dataPtr(std::make_unique<RobotiqControllerPrivate>())
{

}

void RobotiqController::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "RobotiqController plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    // Get BaseLink
    this->dataPtr->gripperBaseLink=this->dataPtr->model.LinkByName(_ecm, "robotiq_arg2f_base_link");
    this->dataPtr->gripperFingerLink[0]==this->dataPtr->model.LinkByName(_ecm, "left_inner_finger");
    this->dataPtr->gripperFingerLink[1]==this->dataPtr->model.LinkByName(_ecm, "right_inner_finger");
    // Get joint Entity
    for(auto &joint_name:this->dataPtr->jointNames){
        auto entity=this->dataPtr->model.JointByName(_ecm, joint_name);
        if(entity == kNullEntity){
            ignerr << "joint with name[" << joint_name<< "] not found. " << std::endl;
            return;
        }
        this->dataPtr->jointEntities.push_back(entity);
    }
    // Subscribe to commands
    std::string topic{this->dataPtr->model.Name(_ecm) + "/gripper"};
    this->dataPtr->node.Subscribe(topic, &RobotiqControllerPrivate::OnCmd, this->dataPtr.get());
    ignmsg << "RobotiqController subscribing to twist messages on [" << topic << "]" << std::endl;

}

void RobotiqController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
    //current state
    GripperStatus gripperStatus;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        gripperStatus = this->dataPtr->gripperStatus;
    }
    //current state
    //const auto chassisPose = _ecm.Component<components::WorldPose>(this->dataPtr->chassisLink)->Data();

    //for angular velocity control
    //double wErr = angularVel.Z() - targetVel.angular().z();
    //double wCmd = this->dataPtr->wPid.Update(wErr, _info.dt);

}
void RobotiqController::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                              const ignition::gazebo::EntityComponentManager &_ecm)
{
    //do nothing
}

/******************implementation for RobotiqControllerPrivate******************/

void RobotiqControllerPrivate::OnCmd(const ignition::msgs::Boolean  &_msg)
{
    std::lock_guard<std::mutex> lock(this->statusMutex);

    //is grasp?
    if (_msg.data()){
        //to close gripper (grasp it!)
        if (this->gripperStatus == GripperStatus::keep_open || this->gripperStatus == GripperStatus::move_open) {
            this->gripperStatus = GripperStatus::move_close;
            //last_check_time_ = model_->GetWorld()->SimTime();
            //last_check_position_ = joints_[0]->Position();
            //InitPidControl();
        }
    } else {
        //to open gripper (don't grasp!)
        if (this->gripperStatus == GripperStatus::keep_close || this->gripperStatus == GripperStatus::move_close) {
            this->gripperStatus = GripperStatus::move_open;
            //last_check_time_ = model_->GetWorld()->SimTime();
            //InitPidControl();
        }
    }
}


/******************register*************************************************/
IGNITION_ADD_PLUGIN(RobotiqController,
                    ignition::gazebo::System,
                    RobotiqController::ISystemConfigure,
                    RobotiqController::ISystemPreUpdate,
                    RobotiqController::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RobotiqController, "ignition::gazebo::systems::RobotiqController")
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
#include "RobotiqController.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/common/Util.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <mutex>

#define WHEEL_NUM 4
using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace std;

enum class GripperStatus {
    keep_close,
    keep_open,
    move_close,
    move_open
};

class ignition::gazebo::systems::RobotiqControllerPrivate {
   public:
   public:
    double getJointPosition(ignition::gazebo::EntityComponentManager &_ecm);
    void setJointVelocity(ignition::gazebo::EntityComponentManager &_ecm, double vel);
    bool checkGrasp(ignition::gazebo::EntityComponentManager &_ecm);
    void OnCmd(const ignition::msgs::Boolean &_msg);

   public:
    transport::Node node;
    //model
    Model model{kNullEntity};
    //gripper joint
    vector<string> jointNames{"finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint",
                              "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"};
    vector<Entity> jointEntities;
    vector<int> jointMultipliers{1, -1, 1, -1, -1, 1};
    vector<ignition::math::PID> jointPids;
    //grasp link(for contact detection,not used currently)
    Entity gripperFingerLink[2];
    //gripper cmd
    std::mutex statusMutex;
    GripperStatus gripperStatus = GripperStatus::keep_open;
};

/******************implementation for RobotiqController************************/
RobotiqController::RobotiqController() : dataPtr(std::make_unique<RobotiqControllerPrivate>()) {
}

void RobotiqController::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager & /*_eventMgr*/) {
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm)) {
        ignerr << "RobotiqController plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    // Get Gripper Finger Link (not used!)
    //this->dataPtr->gripperFingerLink[0] == this->dataPtr->model.LinkByName(_ecm, "left_inner_finger");
    //this->dataPtr->gripperFingerLink[1] == this->dataPtr->model.LinkByName(_ecm, "right_inner_finger");
    // Get joint Entity
    for (auto &joint_name : this->dataPtr->jointNames) {
        auto entity = this->dataPtr->model.JointByName(_ecm, joint_name);
        if (entity == kNullEntity) {
            ignerr << "joint with name[" << joint_name << "] not found. " << std::endl;
            return;
        }
        this->dataPtr->jointEntities.push_back(entity);
    }
    // Subscribe to commands
    std::string topic{this->dataPtr->model.Name(_ecm) + "/gripper"};
    this->dataPtr->node.Subscribe(topic, &RobotiqControllerPrivate::OnCmd, this->dataPtr.get());
    ignmsg << "RobotiqController subscribing to twist messages on [" << topic << "]" << std::endl;
    this->dataPtr->gripperStatus = GripperStatus::move_close;
}

void RobotiqController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm) {
    //do nothing if it's paused or not initialized.
    if (_info.paused) {
        return;
    }
    //current state
    GripperStatus gripperStatus;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        gripperStatus = this->dataPtr->gripperStatus;
    }
    //FSM
    if (gripperStatus == GripperStatus::move_close) {
        //check and set velocity
        if (this->dataPtr->checkGrasp(_ecm)) {
            std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
            this->dataPtr->gripperStatus = GripperStatus::keep_close;
            this->dataPtr->setJointVelocity(_ecm, 0);
        } else {
            this->dataPtr->setJointVelocity(_ecm, 0.1);
        }
    } else if (gripperStatus == GripperStatus::move_open) {
        //check and set velocity
        if (this->dataPtr->getJointPosition(_ecm) < 0.001) {
            std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
            this->dataPtr->gripperStatus = GripperStatus::keep_open;
            this->dataPtr->setJointVelocity(_ecm, 0);
        } else {
            this->dataPtr->setJointVelocity(_ecm, -0.1);
        }
    } else if (gripperStatus == GripperStatus::keep_open || gripperStatus == GripperStatus::keep_close) {
        this->dataPtr->setJointVelocity(_ecm, 0);
    }
}
void RobotiqController::PostUpdate(const ignition::gazebo::UpdateInfo &/*_info*/,
                                   const ignition::gazebo::EntityComponentManager &/*_ecm*/) {
    //do nothing
}

/******************implementation for RobotiqControllerPrivate******************/
void RobotiqControllerPrivate::setJointVelocity(ignition::gazebo::EntityComponentManager &_ecm, double vel) {
    for (size_t i = 0; i < this->jointEntities.size(); i++) {
        auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(this->jointEntities[i]);
        double jointVel = vel * this->jointMultipliers[i];
        if (jointVelCmd == nullptr) {
            _ecm.CreateComponent(this->jointEntities[i], components::JointVelocityCmd({jointVel}));
        } else {
            jointVelCmd->Data()[0] = jointVel;
        }
    }
}

double RobotiqControllerPrivate::getJointPosition(ignition::gazebo::EntityComponentManager &_ecm) {
    auto jointPosComp = _ecm.Component<components::JointPosition>(this->jointEntities[0]);
    if (jointPosComp == nullptr) {
        _ecm.CreateComponent(this->jointEntities[0], components::JointPosition());
    }
    return fabs(jointPosComp->Data().at(0));
}

bool RobotiqControllerPrivate::checkGrasp(ignition::gazebo::EntityComponentManager &_ecm) {
    auto base = getJointPosition(_ecm);
    if (base > 0.695) {
        ignmsg << "Grasp Nothing!" << std::endl;
        return true;
    }
    double diff = 0;
    for (size_t i = 0; i < this->jointEntities.size(); i++) {
        auto jointPosComp = _ecm.Component<components::JointPosition>(this->jointEntities[i]);
        if (jointPosComp == nullptr) {
            _ecm.CreateComponent(this->jointEntities[0], components::JointPosition());
        }
        auto pos = fabs(jointPosComp->Data().at(0));
        diff = diff + fabs(pos - base);
    }
    if(diff > 0.02){
        ignmsg << "Grasp Object!" << std::endl;
        return true;
    }
    return false;
}

void RobotiqControllerPrivate::OnCmd(const ignition::msgs::Boolean &_msg) {
    std::lock_guard<std::mutex> lock(this->statusMutex);
    //is grasp?
    if (_msg.data()) {
        //to close gripper (grasp it!)
        if (this->gripperStatus == GripperStatus::keep_open || this->gripperStatus == GripperStatus::move_open) {
            this->gripperStatus = GripperStatus::move_close;
        }
    } else {
        //to open gripper (don't grasp!)
        if (this->gripperStatus == GripperStatus::keep_close || this->gripperStatus == GripperStatus::move_close) {
            this->gripperStatus = GripperStatus::move_open;
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
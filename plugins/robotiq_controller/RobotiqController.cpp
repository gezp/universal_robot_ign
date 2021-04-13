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
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
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
    Entity getGraspedLink(ignition::gazebo::EntityComponentManager &_ecm);
    void attachGraspedLink(ignition::gazebo::EntityComponentManager &_ecm, Entity entity);
    void detachGraspedLink(ignition::gazebo::EntityComponentManager &_ecm);
    //check for control
    bool checkGrasp(ignition::gazebo::EntityComponentManager &_ecm);
    bool checkCloseGripper(ignition::gazebo::EntityComponentManager &_ecm);
    bool checkOpenGripper(ignition::gazebo::EntityComponentManager &_ecm);
    void OnCmd(const ignition::msgs::Boolean &_msg);

   public:
    transport::Node node;
    bool initialized{false};
    //model
    Model model{kNullEntity};
    //gripper joint
    vector<string> jointNames{"finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint",
                              "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"};
    vector<int> jointMultipliers{1, -1, 1, -1, -1, 1};
    vector<Entity> jointEntities;
    //Gripper Data
    string gripperName{"gripper"};
    bool isFixed{false};
    double velScale{1.0};
    //grasp object by using detachable joint (contain contact detection)
    Entity gripperBaseLink;
    Entity gripperFingerCollisions[2];
    Entity detachableJointEntity{kNullEntity};
    //gripper cmd
    std::mutex statusMutex;
    GripperStatus gripperStatus = GripperStatus::keep_open;
};

/******************implementation for RobotiqController************************/
RobotiqController::RobotiqController() : dataPtr(std::make_unique<RobotiqControllerPrivate>()) {
}

void RobotiqController::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> & _sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager & /*_eventMgr*/) {
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm)) {
        ignerr << "RobotiqController plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    if (_sdf->HasElement("fixed")){
        this->dataPtr->isFixed = _sdf->Get<bool>("fixed");
    }
    if (_sdf->HasElement("vel_scale")){
        this->dataPtr->velScale=_sdf->Get<double>("vel_scale");
    }
    if (_sdf->HasElement("gripper_name")){
        this->dataPtr->gripperName=_sdf->Get<std::string>("gripper_name");
    }
    // Get Gripper Finger Link
    this->dataPtr->gripperBaseLink = this->dataPtr->model.LinkByName(_ecm, "robotiq_arg2f_base_link");
    auto linkEntity1 = this->dataPtr->model.LinkByName(_ecm, "left_inner_finger");
    this->dataPtr->gripperFingerCollisions[0] = Link(linkEntity1).CollisionByName(_ecm, "left_inner_finger_pad_collision");
    auto linkEntity2 = this->dataPtr->model.LinkByName(_ecm, "right_inner_finger");
    this->dataPtr->gripperFingerCollisions[1] = Link(linkEntity2).CollisionByName(_ecm, "right_inner_finger_pad_collision");
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
    std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->gripperName};
    this->dataPtr->node.Subscribe(topic, &RobotiqControllerPrivate::OnCmd, this->dataPtr.get());
    ignmsg << "RobotiqController subscribing to twist messages on [" << topic << "]" << std::endl;
}

void RobotiqController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm) {
    //do nothing if it's paused or not initialized.
    if (_info.paused) {
        return;
    }
    //initialize
    if(!this->dataPtr->initialized){
        for (int i = 0; i < 2; i++) {
            auto &colEntity = this->dataPtr->gripperFingerCollisions[i];
            _ecm.CreateComponent(colEntity,components::ContactSensorData());
        }
        for (size_t i = 0; i < this->dataPtr->jointEntities.size(); i++) {
            _ecm.CreateComponent(this->dataPtr->jointEntities[i], components::JointPosition());
            _ecm.CreateComponent(this->dataPtr->jointEntities[i], components::JointVelocityCmd({0}));
        }
        this->dataPtr->initialized=true;
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
        if (this->dataPtr->checkCloseGripper(_ecm)) {
            std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
            this->dataPtr->gripperStatus = GripperStatus::keep_close;
            this->dataPtr->setJointVelocity(_ecm, 0);
        } else {
            this->dataPtr->setJointVelocity(_ecm, 0.3 * this->dataPtr->velScale);
        }
    } else if (gripperStatus == GripperStatus::move_open) {
        //check and set velocity
        if (this->dataPtr->checkOpenGripper(_ecm)) {
            std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
            this->dataPtr->gripperStatus = GripperStatus::keep_open;
            this->dataPtr->setJointVelocity(_ecm, 0);
        } else {
            this->dataPtr->setJointVelocity(_ecm, -0.6 * this->dataPtr->velScale);
        }
    } else if (gripperStatus == GripperStatus::keep_open || gripperStatus == GripperStatus::keep_close) {
        //this->dataPtr->setJointVelocity(_ecm, 0);
    }
}
void RobotiqController::PostUpdate(const ignition::gazebo::UpdateInfo & /*_info*/,
                                   const ignition::gazebo::EntityComponentManager & /*_ecm*/) {
    //do nothing
}

/******************implementation for RobotiqControllerPrivate******************/
void RobotiqControllerPrivate::setJointVelocity(ignition::gazebo::EntityComponentManager &_ecm, double vel) {
    for (size_t i = 0; i < this->jointEntities.size(); i++) {
        auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(this->jointEntities[i]);
        double jointVel = vel * this->jointMultipliers[i];
        jointVelCmd->Data()[0] = jointVel;
    }
}

double RobotiqControllerPrivate::getJointPosition(ignition::gazebo::EntityComponentManager &_ecm) {
    auto jointPosComp = _ecm.Component<components::JointPosition>(this->jointEntities[0]);
    return fabs(jointPosComp->Data().at(0));
}

Entity RobotiqControllerPrivate::getGraspedLink(ignition::gazebo::EntityComponentManager &_ecm) {
    for (int i = 0; i < 2; i++) {
        auto &colEntity = this->gripperFingerCollisions[i];
        auto contacts = _ecm.Component<components::ContactSensorData>(colEntity);
        for (const auto &contact : contacts->Data().contact()) {
            auto targetColEntity = (contact.collision1().id() != colEntity) ? contact.collision1().id() : contact.collision2().id();
            auto *parentEntity = _ecm.Component<components::ParentEntity>(targetColEntity);
            if (nullptr == parentEntity)
                continue;
            return parentEntity->Data();
        }
    }
    return kNullEntity;
}

void RobotiqControllerPrivate::attachGraspedLink(ignition::gazebo::EntityComponentManager &_ecm, Entity entity) {
    if (this->detachableJointEntity != kNullEntity) {
        return;
    }
    this->detachableJointEntity = _ecm.CreateEntity();
    _ecm.CreateComponent(this->detachableJointEntity,components::DetachableJoint({this->gripperBaseLink, entity, "fixed"}));
}

void RobotiqControllerPrivate::detachGraspedLink(ignition::gazebo::EntityComponentManager &_ecm) {
    if (this->detachableJointEntity == kNullEntity) {
        return;
    }
    _ecm.RequestRemoveEntity(this->detachableJointEntity);
    this->detachableJointEntity = kNullEntity;
}

bool RobotiqControllerPrivate::checkOpenGripper(ignition::gazebo::EntityComponentManager &_ecm) {
    if(this->isFixed){
        detachGraspedLink(_ecm);
    }
    auto base = getJointPosition(_ecm);
    if (base > 0.001) {
        return false;
    }
    double diff = 0;
    for (size_t i = 0; i < this->jointEntities.size(); i++) {
        auto jointPosComp = _ecm.Component<components::JointPosition>(this->jointEntities[i]);
        auto pos = fabs(jointPosComp->Data().at(0));
        diff = diff + fabs(pos - base);
    }
    if (diff > 0.01) {
        return false;
    }
    return true;
}

bool RobotiqControllerPrivate::checkCloseGripper(ignition::gazebo::EntityComponentManager &_ecm) {
    auto base = getJointPosition(_ecm);
    if (base > 0.695) {
        ignmsg << "Grasp Nothing!" << std::endl;
        return true;
    }
    double diff = 0;
    for (size_t i = 0; i < this->jointEntities.size(); i++) {
        auto jointPosComp = _ecm.Component<components::JointPosition>(this->jointEntities[i]);
        auto pos = fabs(jointPosComp->Data().at(0));
        diff = diff + fabs(pos - base);
    }
    if (diff > 0.02) {
        if(this->isFixed){
            //use fixed joint to fix Grasped Object
            auto link=getGraspedLink(_ecm);
            if(link != kNullEntity){
                attachGraspedLink(_ecm,link);
                ignmsg << "Grasp Object!" << std::endl;
                return true;
            }
        }else{
            ignmsg << "Grasp Object!" << std::endl;
            return true;
        }
    }
    return false;
}

void RobotiqControllerPrivate::OnCmd(const ignition::msgs::Boolean &_msg) {
    std::lock_guard<std::mutex> lock(this->statusMutex);
    //is grasp?
    if (_msg.data()) {
        //to close gripper (grasp it!)
        if (this->gripperStatus == GripperStatus::keep_open) {
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
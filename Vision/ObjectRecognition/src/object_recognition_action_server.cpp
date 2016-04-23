#include <object_recognition_action.hpp>

ObjectRecognitionAction::ObjectRecognitionAction(nh, name) : node_handle_(nh), action_name_(name) {
    action_server_.registerGoalCallback(boost::bind(&ObjectRecognitionAction::goalCallback, this));
    action_server_.registerPreemptCallback(boost::bind(&ObjectRecognitionAction::preemptCallback, this));
    action_server_.start();
}

ObjectRecognitionAction::goalCallback() {
}

ObjectRecognitionAction::preemptCallback() {
}

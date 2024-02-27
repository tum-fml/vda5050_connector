/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/action_client.h"

using namespace std;
using namespace connector_utils;

// TODO: Send orderCancel to order manager (2 cases: instantAction, failed action)
// TODO: Implement instantAction routine
// TODO: Implement orderAction routine
// TODO: Implement difference between paused by instantAction and paused by AGV
// TODO: Check if last action still running (new action blocking type hard)
// TODO: Sort instant actions by blocking type (hard least)???
// TODO: Implement topic to cancel actions on AGV

/*--------------------------------ActionElement--------------------------------------------------------------*/

ActionElement::ActionElement(
    const vda5050_msgs::Action* incomingAction, string incomingOrderId, string newState) {
  orderId = incomingOrderId;
  actionId = incomingAction->actionId;
  blockingType = incomingAction->blockingType;
  actionType = incomingAction->actionType;
  actionDescription = incomingAction->actionDescription;
  actionParameters = incomingAction->actionParameters;
  state = newState;
}

bool ActionElement::compareActionId(string actionId2comp) { return actionId == actionId2comp; }

string ActionElement::getActionId() const { return actionId; }

string ActionElement::getActionType() const { return actionType; }

vda5050_msgs::Action ActionElement::packAction() {
  vda5050_msgs::Action msg;
  msg.actionId = actionId;
  msg.blockingType = blockingType;
  msg.actionType = actionType;
  msg.actionDescription = actionDescription;
  msg.actionParameters = actionParameters;

  return msg;
}

/*--------------------------------ActionClient--------------------------------------------------------------*/

ActionClient::ActionClient() {
  LinkPublishTopics(&(this->nh));
  LinkSubscriptionTopics(&(this->nh));
}

void ActionClient::LinkPublishTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList =
      GetTopicList(ros::this_node::getName() + "/publish_topics");

  for (const auto& elem : topicList) {
    // ROS_INFO("topic_index = %s",topic_index.c_str());
    if (CheckParamIncludes(elem.first, "actionToAgv"))
      messagePublisher[elem.first] =
          make_shared<ros::Publisher>(nh->advertise<vda5050_msgs::Action>(elem.second, 1000));
    if (CheckParamIncludes(elem.first, "agvActionCancel"))
      messagePublisher[elem.first] =
          make_shared<ros::Publisher>(nh->advertise<std_msgs::String>(elem.second, 1000));
    if (CheckParamIncludes(elem.first, "prActions"))
      messagePublisher[elem.first] =
          make_shared<ros::Publisher>(nh->advertise<std_msgs::String>(elem.second, 1000));
    if (CheckParamIncludes(elem.first, "prDriving"))
      messagePublisher[elem.first] =
          make_shared<ros::Publisher>(nh->advertise<std_msgs::String>(elem.second, 1000));
  }
}

void ActionClient::LinkSubscriptionTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList =
      GetTopicList(ros::this_node::getName() + "/subscribe_topics");
  for (const auto& elem : topicList) {
    if (CheckParamIncludes(elem.first, "instantAction"))
      nh->subscribe(elem.second, 1000, &ActionClient::InstantActionsCallback, this);
    if (CheckParamIncludes(elem.first, "agvActionState"))
      nh->subscribe(elem.second, 1000, &ActionClient::AgvActionStateCallback, this);
    if (CheckParamIncludes(elem.first, "driving"))
      nh->subscribe(elem.second, 1000, &ActionClient::DrivingCallback, this);
  }
}

void ActionClient::OrderTriggerCallback(const std_msgs::String& msg) {
  shared_ptr<ActionElement> activeAction = FindAction(msg.data);

  // Sort out duplicates?

  if (activeAction) {
    // Push action to queue
    vda5050_msgs::Action triggeredOrder = activeAction->packAction();
    orderActionQueue.push_back(triggeredOrder);
    ROS_INFO("Found Action to trigger: %s", msg.data.c_str());
  } else
    ROS_WARN("Action to trigger not found!");
}

void ActionClient::OrderCancelCallback(const std_msgs::String& msg) {
  ordersSucCancelled.push_back(msg.data);
}

void ActionClient::InstantActionsCallback(const vda5050_msgs::InstantAction::ConstPtr& msg) {
  // Iterate over all actions in the instantActions msg
  for (auto& iaction : msg->instantActions) {
    // Add action to active actions list
    ActionClient::AddActionToList(&iaction, "Instant", "WAITING");
    // Initialize order ID variable
    string orderIdToCancel;

    // Decide if the action contains an order cancel
    if (iaction.actionType == "cancelOrder") {
      // New actions to cancel
      vector<shared_ptr<ActionElement>> newActionsToCancel;
      // Add orderId to orderCancellations list and get all actions to cancel
      for (auto const& param : iaction.actionParameters) {
        if (param.key == "orderId") {
          orderIdToCancel = param.value;
          newActionsToCancel = GetActionsToCancel(orderIdToCancel);
        }
      }

      // Cancel actions from newActionsToCancel list
      for (std::vector<std::shared_ptr<ActionElement>>::iterator cAction =
               newActionsToCancel.begin();
           cAction != newActionsToCancel.end();) {
        // Wating actions can simply be removed as long as they have not been sent to the AGV
        if (cAction->get()->state == "WAITING") {
          // Check if Action has already been sent to AGV (in activeActionsList but not in queue)

          // action already sent to AGV
          if (cAction->get()->sentToAgv) {
            // Send action cancel request to AGV
            std_msgs::String cancel_msg;
            cancel_msg.data = string(cAction->get()->getActionId());
            messagePublisher["agvActionCancel"]->publish(cancel_msg);
            cAction++;
          }

          // action not sent to AGV yet
          else {
            // action triggered and in queue (but still not sent to AGV)
            auto queueAction = find_if(orderActionQueue.begin(), orderActionQueue.end(),
                [cAction](vda5050_msgs::Action& orderAction) {
                  return orderAction.actionId == cAction->get()->getActionId();
                });
            // delete action from queue
            if (queueAction != orderActionQueue.end()) orderActionQueue.erase(queueAction);

            // delete from newActionsToCancel
            newActionsToCancel.erase(
                remove(newActionsToCancel.begin(), newActionsToCancel.end(), *cAction));

            // send failed state to state daemon
            vda5050_msgs::ActionState state_msg;
            state_msg.actionId = cAction->get()->getActionId();
            state_msg.actionType = cAction->get()->getActionType();
            state_msg.actionStatus = "FAILED";
            state_msg.resultDescription = "order cancelled";  // Description necessary?
            actionStatesPub.publish(state_msg);

            // delete from activeActionsList
            auto actAct_it = find_if(activeActionsList.begin(), activeActionsList.end(),
                [cAction](shared_ptr<ActionElement>& activeAction) {
                  return cAction->get()->compareActionId(cAction->get()->getActionId());
                });

            if (actAct_it != activeActionsList.end()) {
              activeActionsList.erase(actAct_it);
            }
          }
        }
        // Running/Initializing/Paused actions must be stopped
        else {
          // Send action cancel request to AGV
          std_msgs::String cancel_msg;
          cancel_msg.data = string(cAction->get()->getActionId());
          messagePublisher["agvActionCancel"]->publish(cancel_msg);
          cAction++;
        }
      }

      // Create new order to cancel
      orderToCancel newOrderToCancel;
      newOrderToCancel.orderIdToCancel = orderIdToCancel;
      newOrderToCancel.iActionId = iaction.actionId;
      newOrderToCancel.allActionsCancelledSent = false;

      // Add all remaining new actions to cancel to the list
      if (!newActionsToCancel.empty())
        newOrderToCancel.actionsToCancel.insert(newOrderToCancel.actionsToCancel.end(),
            newActionsToCancel.begin(), newActionsToCancel.end());
      // If no action to cancel remains (i.e. no action has been sent to the AGV already)
      // -> remove order ID from cancellation list in update loop

      // Add new order to cancel to list
      orderCancellations.push_back(newOrderToCancel);

      // Send cancel request to order daemon
      std_msgs::String cancelOrderMsg;
      cancelOrderMsg.data = orderIdToCancel;
      orderCancelPub.publish(cancelOrderMsg);
    }

    // if the action contains no order cancel
    else {
      // Push to instant action queue
      instantActionQueue.push_back(iaction);
      // Create and publish action state msg
      vda5050_msgs::ActionState state_msg;
      state_msg.actionId = iaction.actionId;
      state_msg.actionType = iaction.actionType;
      state_msg.actionStatus = "WAITING";
      state_msg.resultDescription = "";  // Description necessary?
      actionStatesPub.publish(state_msg);
    }
  }
}

void ActionClient::AgvActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg) {
  shared_ptr<ActionElement> actionToUpdate = FindAction(msg->actionId);
  actionStatesPub.publish(msg);

  if (actionToUpdate) {
    actionStatesPub.publish(*msg);
    if ((msg->actionStatus == "WAITING") || (msg->actionStatus == "INITIALIZING") ||
        (msg->actionStatus == "RUNNING") || (msg->actionStatus == "PAUSED")) {
      actionToUpdate->state = msg->actionStatus;
      // actionStatesPub.publish(msg);
    } else if (msg->actionStatus == "FINISHED") {
      if (actionToUpdate->blockingType != "NONE") {
        std_msgs::String resumeMsg;
        resumeMsg.data = "RESUME";
        messagePublisher["prDriving"]->publish(resumeMsg);
      }
      // actionStatesPub.publish(msg);
      activeActionsList.erase(
          remove(activeActionsList.begin(), activeActionsList.end(), actionToUpdate));
    } else if (msg->actionStatus == "FAILED") {
      if (actionToUpdate->blockingType != "NONE") {
        std_msgs::String resumeMsg;
        resumeMsg.data = "RESUME";
        messagePublisher["prDriving"]->publish(resumeMsg);
      }
      // actionStatesPub.publish(msg);

      activeActionsList.erase(
          remove(activeActionsList.begin(), activeActionsList.end(), actionToUpdate));

      std_msgs::String cancelMsg;
      cancelMsg.data = "CANCEL ORDER";
      orderCancelPub.publish(cancelMsg);
    }
  } else
    ROS_WARN("Action to update not found!");
}

void ActionClient::DrivingCallback(const std_msgs::Bool::ConstPtr& msg) { isDriving = msg->data; }

void ActionClient::AddActionToList(
    const vda5050_msgs::Action* incomingAction, string orderId, string state) {
  shared_ptr<ActionElement> newAction = make_shared<ActionElement>(incomingAction, orderId, state);
  activeActionsList.push_back(newAction);
}

bool ActionClient::CheckDriving() {
  if (isDriving) {
    std_msgs::String pauseMsg;
    pauseMsg.data = "PAUSE";
    messagePublisher["prDriving"]->publish(pauseMsg);
    return false;
  } else
    return true;
}

vector<shared_ptr<ActionElement>> ActionClient::GetRunningActions() {
  vector<shared_ptr<ActionElement>> runningActions;
  for (auto const& action_it : activeActionsList) {
    if (action_it->state == "RUNNING") {
      runningActions.push_back(action_it);
    }
  }
  return runningActions;
}

vector<shared_ptr<ActionElement>> ActionClient::GetRunningPausedActions() {
  vector<shared_ptr<ActionElement>> runningPausedActions;
  for (auto const& action_it : activeActionsList) {
    if (action_it->state == "RUNNING" || action_it->state == "PAUSED") {
      runningPausedActions.push_back(action_it);
    }
  }
  return runningPausedActions;
}

vector<shared_ptr<ActionElement>> ActionClient::GetActionsToCancel(string orderIdToCancel) {
  vector<shared_ptr<ActionElement>> actionsToCancel;
  auto it = activeActionsList.begin();
  while ((it = find_if(
              it, activeActionsList.end(), [&orderIdToCancel](shared_ptr<ActionElement> const& p) {
                return p->orderId == orderIdToCancel;
              })) != activeActionsList.end()) {
    actionsToCancel.push_back(*it);
    it++;
  }

  return actionsToCancel;
}

shared_ptr<ActionElement> ActionClient::FindAction(string actionId) {
  vector<shared_ptr<ActionElement>>::iterator it =
      find_if(activeActionsList.begin(), activeActionsList.end(),
          [&actionId](shared_ptr<ActionElement> const& p) { return p->compareActionId(actionId); });
  if (it == activeActionsList.end())
    return nullptr;
  else
    return *it;
}

void ActionClient::UpdateActions() {
  // check if orders must be cancelled -> block all actions.
  if (!orderCancellations.empty()) {
    vector<orderToCancel> orderCancellationsFinished;
    for (auto& orderCan_it : orderCancellations) {
      // Remove failed/finished actions from observing list.
      // FIXME: Does not work <-----------------------------------------------------------------.
      if (!orderCan_it.actionsToCancel.empty()) {
        orderCan_it.actionsToCancel.erase(
            remove_if(orderCan_it.actionsToCancel.begin(), orderCan_it.actionsToCancel.end(),
                [&](weak_ptr<ActionElement> const& p) { return p.expired(); }),
            orderCan_it.actionsToCancel.end());
      }
      // FIXME: Does not work
      // Only check the order cancel state, if all actions are cancelled.
      if (orderCan_it.actionsToCancel.empty()) {
        // send all actions cancelled signal to order daemon.
        if (!orderCan_it.allActionsCancelledSent) {
          std_msgs::String allActionsCancelledMsg;
          allActionsCancelledMsg.data = orderCan_it.orderIdToCancel;
          allActionsCancelledPub.publish(allActionsCancelledMsg);
        }

        // Check if order has been cancelled by order daemon.
        auto orderCancelled = find_if(ordersSucCancelled.begin(), ordersSucCancelled.end(),
            [orderCan_it](string orderCanc) { return orderCan_it.orderIdToCancel == orderCanc; });
        // If order has been cancelled by order deamon.
        if (orderCancelled != ordersSucCancelled.end()) {
          // Send action state finished.
          auto actAct_it = find_if(activeActionsList.begin(), activeActionsList.end(),
              [&orderCan_it](shared_ptr<ActionElement>& activeAction) {
                return activeAction->compareActionId(orderCan_it.iActionId);
              });
          if (actAct_it != activeActionsList.end()) {
            // Create and publish action state msg.
            vda5050_msgs::ActionState state_msg;
            state_msg.actionId = (**actAct_it).getActionId();
            state_msg.actionType = (**actAct_it).getActionType();
            state_msg.actionStatus = "FINISHED";
            state_msg.resultDescription = "";  // Description necessary?.
            actionStatesPub.publish(state_msg);

            // Remove instant action from active actions list.
            activeActionsList.erase(actAct_it);
          } else {
            ROS_ERROR_STREAM("ACTION NOT FOUND IN ACTIVE ACTIONS!");
          }

          // Remove order to cancel from orderSucCancelled list.
          ordersSucCancelled.erase(orderCancelled);
          // Remove order to cancel from orderCancellations list.
          orderCancellationsFinished.push_back(orderCan_it);
        }
      }
    }
    // Delete from List.
    for (auto const& finishedOrderCancel : orderCancellationsFinished) {
      auto ordCan_it = find_if(orderCancellations.begin(), orderCancellations.end(),
          [&finishedOrderCancel](orderToCancel& orderCancel) {
            return orderCancel.iActionId == finishedOrderCancel.iActionId;
          });
      if (ordCan_it != orderCancellations.end()) orderCancellations.erase(ordCan_it);
    }
  }

  // Instant action routine -> block order actions.
  else if (!instantActionQueue.empty()) {
    // get running actions.
    vector<shared_ptr<ActionElement>> runningPausedActions = GetRunningPausedActions();

    if (!runningPausedActions.empty()) {
      // hard blocking action running?.
      bool RunningActionHardBlocking = false;
      for (auto& elem : runningPausedActions) {
        if (elem->state == "RUNNING" && elem->blockingType == "HARD")
          RunningActionHardBlocking = true;
      }
      if (!RunningActionHardBlocking) {
        // new instant action blocking hard.
        string& nextBlockType = instantActionQueue.front().blockingType;

        if (nextBlockType == "HARD") {
          if (CheckDriving()) {
            // set sentToAgv to true.
            auto sentAction = FindAction(instantActionQueue.front().actionId);
            sentAction->sentToAgv = true;

            // send action.
            vda5050_msgs::Action instantActionMsg = instantActionQueue.front();
            messagePublisher["actionToAgv"]->publish(instantActionMsg);
            instantActionQueue.pop_front();
          }
          // Pause all actions.
          std_msgs::String pause_msg;
          pause_msg.data = "PAUSE";
          messagePublisher["prActions"]->publish(pause_msg);
        } else if (nextBlockType == "SOFT") {
          if (CheckDriving()) {
            // set sentToAgv to true.
            auto sentAction = FindAction(instantActionQueue.front().actionId);
            sentAction->sentToAgv = true;

            // send action.
            vda5050_msgs::Action instantActionMsg = instantActionQueue.front();
            messagePublisher["actionToAgv"]->publish(instantActionMsg);
            instantActionQueue.pop_front();
          }
        } else if (nextBlockType == "NONE") {
          // set sentToAgv to true.
          auto sentAction = FindAction(instantActionQueue.front().actionId);
          sentAction->sentToAgv = true;

          // send action.
          vda5050_msgs::Action instantActionMsg = instantActionQueue.front();
          messagePublisher["actionToAgv"]->publish(instantActionMsg);

          instantActionQueue.pop_front();
        }
      } else {
        // Pause all actions.
        std_msgs::String pause_msg;
        pause_msg.data = "PAUSE";
        messagePublisher["prActions"]->publish(pause_msg);
      }
    }

    // no action running.
    else {
      // set sentToAgv to true.
      auto sentAction = FindAction(instantActionQueue.front().actionId);
      sentAction->sentToAgv = true;

      // send action to AGV.
      vda5050_msgs::Action instantActionMsg = instantActionQueue.front();
      messagePublisher["actionToAgv"]->publish(instantActionMsg);
      instantActionQueue.pop_front();
    }
  }

  // Order action routine.
  else if (!orderActionQueue.empty()) {
    // get running actions.
    vector<shared_ptr<ActionElement>> runningPausedActions = GetRunningPausedActions();

    if (!runningPausedActions.empty()) {
      // hard blocking action running?.
      bool RunningActionHardBlocking = false;
      for (auto& elem : runningPausedActions) {
        if (elem->state == "RUNNING" && elem->blockingType == "HARD")
          RunningActionHardBlocking = true;
      }

      if (!RunningActionHardBlocking) {
        for (auto const& action_it : runningPausedActions) {
          // resume actions paused by instant actions.
          if (action_it->state == "PAUSED") {
            std_msgs::String resume_msg;
            resume_msg.data = "RESUME";
            messagePublisher["prActions"]->publish(resume_msg);
          }
          // no actions to resume.
          else {
            // new action blocking hard.
            string& nextBlockType = orderActionQueue.front().blockingType;
            if (nextBlockType == "HARD") {
              // TODO: Check if last action still running.
              // If driving -> stop, else publish action.
              if (ActionClient::CheckDriving()) {
                // set sentToAgv to true.
                auto sentAction = FindAction(orderActionQueue.front().actionId);
                sentAction->sentToAgv = true;

                // send action to AGV.
                vda5050_msgs::Action orderActionMsg = orderActionQueue.front();
                messagePublisher["actionToAgv"]->publish(orderActionMsg);
                orderActionQueue.pop_front();
              }
            }
            // new action blocking soft.
            else if (nextBlockType == "SOFT") {
              // If driving -> stop, else publish action.
              if (CheckDriving()) {
                // set sentToAgv to true.
                auto sentAction = FindAction(orderActionQueue.front().actionId);
                sentAction->sentToAgv = true;

                // send action to AGV.
                vda5050_msgs::Action orderActionMsg = orderActionQueue.front();
                messagePublisher["actionToAgv"]->publish(orderActionMsg);
                orderActionQueue.pop_front();
              }
            }
            // new action not blocking.
            else if (nextBlockType == "NONE") {
              // set sentToAgv to true.
              auto sentAction = FindAction(orderActionQueue.front().actionId);
              sentAction->sentToAgv = true;

              // send action to AGV.
              vda5050_msgs::Action orderActionMsg = orderActionQueue.front();
              messagePublisher["actionToAgv"]->publish(orderActionMsg);
              orderActionQueue.pop_front();
            }
          }
        }
      }
    }

    // no action running.
    else {
      // set sentToAgv to true.
      auto sentAction = FindAction(orderActionQueue.front().actionId);
      sentAction->sentToAgv = true;

      // send action to AGV.
      vda5050_msgs::Action orderActionMsg = orderActionQueue.front();
      messagePublisher["actionToAgv"]->publish(orderActionMsg);
      orderActionQueue.pop_front();
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_deamon");

  ActionClient ActionClient;

  ros::Rate rate(0.05);

  while (ros::ok()) {
    ActionClient.UpdateActions();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
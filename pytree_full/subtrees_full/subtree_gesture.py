import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions

policyvar = 3

def subtree_gesture() -> py_trees.behaviour.Behaviour:

    gesture_sequence = py_trees.composites.Sequence("Gestenerkennung",memory=False)

    gesture_height_selector = py_trees.composites.Selector("Höhencheck",memory=False)

    gesture_height_condition = py_trees_ros.subscribers.CheckData(
        name="Zielhöhe Gestenerkennung erreicht?", 
        topic_name="gesture_height_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )

    gesture_height_action = py_trees_ros.action_clients.FromConstant(  
        name="Höhe reduzieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    gesture_active_selector = py_trees.composites.Selector("Gestenerkennung eingeschalten?",memory=False)

    gesture_active_condition = py_trees_ros.subscribers.CheckData(
        name="Gestenerkennung aktiv?", 
        topic_name="gesture_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )

    gesture_active_action = py_trees_ros.action_clients.FromConstant(  
        name="Gestenerkennung aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    gesture_sequence.add_children([gesture_height_selector,gesture_active_selector])
    gesture_height_selector.add_children([gesture_height_condition,gesture_height_action])
    gesture_active_selector.add_children([gesture_active_condition,gesture_active_action])

    return gesture_sequence


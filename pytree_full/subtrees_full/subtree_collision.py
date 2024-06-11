import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions

def subtree_collision() -> py_trees.behaviour.Behaviour:

    collision_selector = py_trees.composites.Selector("Kollisionscheck",memory=False)

    collision_condition = py_trees_ros.subscribers.CheckData(
        name="Kollision möglich?", 
        topic_name="collision_possible_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    collision_action = py_trees_ros.action_clients.FromConstant(  
        name="Hindernis umfliegen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    collision_selector.add_children([collision_condition,collision_action])

    return collision_selector
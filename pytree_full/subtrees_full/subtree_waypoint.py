import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions

def subtree_waypoint() -> py_trees.behaviour.Behaviour:

    waypoint_selector = py_trees.composites.Selector("Waypoint-Mission",memory=False)

    waypoint_condition = py_trees_ros.subscribers.CheckData(
        name="Letzter WP erreicht?", 
        topic_name="waypoint_reached_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    waypoint_action = py_trees_ros.action_clients.FromConstant(  
        name="Zum letzten WP fliegen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    waypoint_selector.add_children([waypoint_condition,waypoint_action])

    return waypoint_selector
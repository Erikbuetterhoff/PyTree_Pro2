import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions

def subtree_zone() -> py_trees.behaviour.Behaviour:

    zone_selector = py_trees.composites.Selector("Flugumgebung Check",memory=False)

    zone_envcheck_condition = py_trees_ros.subscribers.CheckData(
        name="Flugumgebung okay?", 
        topic_name="zone_environment_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    zone_sub_selector = py_trees.composites.Selector("",memory=False)

    zone_leavezone_action = py_trees_ros.action_clients.FromConstant(  
        name="Flugverbotszone verlassen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    zone_hpl_action = py_trees_ros.action_clients.FromConstant(  
        name="Hotpoint Landing aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    zone_rth_action = py_trees_ros.action_clients.FromConstant(  
        name="RTH aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    zone_selector.add_children([zone_envcheck_condition,zone_sub_selector])
    zone_sub_selector.add_children([zone_leavezone_action,zone_hpl_action,zone_rth_action])

    return zone_selector
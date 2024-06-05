import rclpy
import py_trees
import py_trees_ros.trees
import sys
import py_trees.console as console
from std_msgs.msg import Bool
import action_pkg.action as actions

def create_subtree_landing() -> py_trees.behaviour.Behaviour:

    landing_selector = py_trees.composites.Selector("Drohne okay oder landen?",memory=False)

    wait_goal = actions.Wait.Goal()
    wait_goal.timer = 5

    landing_check_selector = py_trees.composites.Sequence("Check",memory=False)
    landing_battery_condition = py_trees_ros.subscribers.CheckData(
        name="Batterie okay?", 
        topic_name="landing_battery_test_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )
    landing_dronecheck_condition = py_trees_ros.subscribers.CheckData(
        name="Drohne okay?", 
        topic_name="landing_landing_test_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    landing_sub_sequence = py_trees.composites.Sequence(name="Landung",memory=False)

    landing_rth_sequence = py_trees.composites.Sequence(name="RTH Landung",memory=False)
    landing_rth_condition = py_trees_ros.subscribers.CheckData(  
        name="RTH möglich?", 
        topic_name="landing_rth_test_topic",
        topic_type=Bool, variable_name="rth_available",
        expected_value= True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
        )
    landing_rth_action = py_trees_ros.action_clients.FromConstant(  
        name="RTH aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )
    
    landing_hpl_sequence = py_trees.composites.Sequence(name="HPL Landung",memory=False)
    landing_hpl_condition = py_trees_ros.subscribers.CheckData(  
        name="HPL möglich?", 
        topic_name="landing_hpl_test_topic",
        topic_type=Bool, variable_name="rth_available",
        expected_value= True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
        )
    landing_hpl_action = py_trees_ros.action_clients.FromConstant(  
        name="HPL aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )
    
    landing_para_action = py_trees_ros.action_clients.FromConstant(  
        name="Fallschirm aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    



    landing_selector.add_children([landing_check_selector,landing_sub_sequence])
    landing_check_selector.add_children([landing_battery_condition,landing_dronecheck_condition])

    landing_sub_sequence.add_children([landing_rth_sequence,landing_hpl_sequence,landing_para_action])
    landing_rth_sequence.add_children([landing_rth_condition,landing_rth_action])
    landing_hpl_sequence.add_children([landing_hpl_condition,landing_hpl_action])

    return landing_selector

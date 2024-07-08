import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions
import operator
import sensor_msgs.msg

policyvar = 3

def subtree_landing() -> py_trees.behaviour.Behaviour:

    landing_selector = py_trees.composites.Selector("Drohne okay oder landen?",memory=False)

    landing_check_sequence = py_trees.composites.Sequence("Check",memory=False)
    
    landing_battery_condition = py_trees_ros.subscribers.CheckData(
        name="Batterie okay?", 
        topic_name="/wrapper/psdk_ros2/battery", 
        topic_type=sensor_msgs.msg.BatteryState, 
        variable_name="percentage", 
        expected_value=0.3, 
        comparison_operator= operator.ge,
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )
    landing_dronecheck_condition = py_trees_ros.subscribers.CheckData(
        name="Drohne okay?", 
        topic_name="landing_drone_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )

    landing_sub_selector = py_trees.composites.Selector(name="Landung",memory=False)

    landing_rth_sequence = py_trees.composites.Sequence(name="RTH Landung",memory=False)
    
    landing_rth_condition = py_trees_ros.subscribers.CheckData(  
        name="RTH möglich?", 
        topic_name="landing_rth_test_topic",
        topic_type=Bool, 
        variable_name="rth_available",
        expected_value= True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
        )
    landing_rth_action = py_trees_ros.action_clients.FromConstant(  
        name="RTH aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
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
        clearing_policy=policyvar
        )
    
    landing_hpl_action = py_trees_ros.action_clients.FromConstant(  
        name="HPL aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )
    
    landing_para_action = py_trees_ros.action_clients.FromConstant(  
        name="Fallschirm aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    landing_selector.add_children([landing_check_sequence,landing_sub_selector])
    landing_check_sequence.add_children([landing_battery_condition,landing_dronecheck_condition])
    landing_sub_selector.add_children([landing_rth_sequence,landing_hpl_sequence,landing_para_action])
    landing_rth_sequence.add_children([landing_rth_condition,landing_rth_action])
    landing_hpl_sequence.add_children([landing_hpl_condition,landing_hpl_action])

    return landing_selector

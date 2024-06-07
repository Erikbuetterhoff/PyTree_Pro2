import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions

def subtree_lidar() -> py_trees.behaviour.Behaviour:

    lidar_sequence = py_trees.composites.Sequence("LIDAR Check",memory=False)

    lidar_checkactive_selector = py_trees.composites.Selector("LIDAR eingeschaltet?",memory=False)

    lidar_checkactive_condition = py_trees_ros.subscribers.CheckData(
        name="Lidar aktiv?", 
        topic_name="lidar_active_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )


    lidar_checkactive_action = py_trees_ros.action_clients.FromConstant(  
        name="Lidar verbinden",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    lidar_rec_selector = py_trees.composites.Selector("Scan vorhanden?",memory=False)

    lidar_rec_condition = py_trees_ros.subscribers.CheckData(
        name="LIDAR Aufnahme erfolgreich?", 
        topic_name="lidar_rec_success_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )


    lidar_scan_sequence = py_trees.composites.Sequence("Scannen",memory=False)

    lidar_scan_height_action = py_trees_ros.action_clients.FromConstant(  
        name="Höhe reduzieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    lidar_scan_action = py_trees_ros.action_clients.FromConstant(  
        name="LIDAR scan durchführen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    lidar_sequence.add_children([lidar_checkactive_selector,lidar_rec_selector])
    lidar_checkactive_selector.add_children([lidar_checkactive_condition,lidar_checkactive_action])
    lidar_rec_selector.add_children([lidar_rec_condition,lidar_scan_sequence])
    lidar_scan_sequence.add_children([lidar_scan_height_action,lidar_scan_action])

    return lidar_sequence
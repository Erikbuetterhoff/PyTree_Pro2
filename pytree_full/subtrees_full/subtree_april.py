import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions

def subtree_april() -> py_trees.behaviour.Behaviour:

    april_sequence = py_trees.composites.Sequence("Apirltaglandung",memory=False)

    april_gesture_selector = py_trees.composites.Selector("Geste erkannt?",memory=False)

    april_gesture_condition = py_trees_ros.subscribers.CheckData(
        name="Keine Abbruchgeste erkannt", 
        topic_name="april_gesture_check_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    april_gesture_sub_selector = py_trees.composites.Selector("Landegeste Check",memory=False)

    april_gesture_hold_condition = py_trees_ros.subscribers.CheckData(
        name="Landegeste Corehelper?", 
        topic_name="april_gesture_landing_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    april_gesture_hold_action = py_trees_ros.action_clients.FromConstant(  
        name="Höhe halten",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    april_height_selector = py_trees.composites.Selector("Höhe Check",memory=False)

    april_height_condition = py_trees_ros.subscribers.CheckData(
        name="Zielhöhe Apriltag erreicht?", 
        topic_name="april_height_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    april_height_action = py_trees_ros.action_clients.FromConstant(  
        name="Höhe reduzieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    april_tag_selector = py_trees.composites.Selector("Apriltag erkennen",memory=False)

    april_tag_condition = py_trees_ros.subscribers.CheckData(
        name="Apriltag erkannt?", 
        topic_name="april_tag_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    april_tag_sub_sequence = py_trees.composites.Sequence("Apriltag finden",memory=False)

    april_tag_cam_selector = py_trees.composites.Selector("Kamera Check",memory=False)

    april_tag_cam_condition = py_trees_ros.subscribers.CheckData(
        name="Kamera aktiv?", 
        topic_name="april_camera_active_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    april_tag_cam_action = py_trees_ros.action_clients.FromConstant(  
        name="Kamera aktivieren",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    april_tag_action = py_trees_ros.action_clients.FromConstant(  
        name="Apriltag suchen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    april_landing_selector = py_trees.composites.Selector("Landung Apriltag",memory=False)

    april_landing_condition = py_trees_ros.subscribers.CheckData(
        name="Gelandet (Apriltag)?", 
        topic_name="april_landed_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    april_landing_action = py_trees_ros.action_clients.FromConstant(  
        name="Landung Apriltag",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=wait_goal,
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    april_sequence.add_children([april_gesture_selector,april_height_selector,april_tag_selector,april_landing_selector])
    april_gesture_selector.add_children([april_gesture_condition,april_gesture_sub_selector])
    april_gesture_sub_selector.add_children([april_gesture_hold_condition,april_gesture_hold_action])
    april_height_selector.add_children([april_height_condition,april_height_action])
    april_tag_selector.add_children([april_tag_condition,april_tag_sub_sequence])
    april_tag_sub_sequence.add_children([april_tag_cam_selector,april_tag_action])
    april_tag_cam_selector.add_children([april_tag_cam_condition,april_tag_cam_action])
    april_landing_selector.add_children([april_landing_condition,april_landing_action])

    return april_sequence
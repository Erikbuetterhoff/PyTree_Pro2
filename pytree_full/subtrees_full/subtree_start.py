import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import std_msgs.msg
import action_pkg.action as actions
import operator
import sensor_msgs.msg
import psdk_interfaces.msg

policyvar = 3

def subtree_start() -> py_trees.behaviour.Behaviour:

    start_sequence = py_trees.composites.Sequence("Startsequenz einleiten",memory=False)

    start_systemcheck_sequence = py_trees.composites.Sequence("Systemcheck, EDGE und WPM ready?",memory=False)


    start_systemcheck_battery = py_trees_ros.subscribers.CheckData(
        name="Batterie okay?", 
        topic_name="/wrapper/psdk_ros2/battery", 
        topic_type=sensor_msgs.msg.BatteryState, 
        variable_name="percentage", 
        expected_value=0.3,         # b 
        comparison_operator= operator.ge,
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )

    start_systemcheck_rc = py_trees_ros.subscribers.CheckData(
        name="Remote verbunden?", 
        topic_name="/wrapper/psdk_ros2/rc_connection_status", 
        topic_type=psdk_interfaces.msg.RCConnectionStatus, ### oder doch std_msgs.msg.Bool??
        variable_name="ground_connection", 
        expected_value=1,         # b
        #comparison_operator= operator.ge,
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )

    start_systemcheck_hp_sub = py_trees.composites.Selector("Homepoint Subtree",memory=False)

    start_systemcheck_hp_check = py_trees_ros.subscribers.CheckData(
        name="Homepoint gesetzt?", 
        topic_name="/wrapper/psdk_ros2/home_point_status", 
        topic_type=std_msgs.msg.Bool, 
        variable_name="data", ## finde nur Message-Definitionen zu HomePosition
        expected_value=False,         # b 
        # comparison_operator= operator.ge,
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )

    start_systemcheck_hp_set = py_trees_ros.action_clients.FromConstant(  
        name="Neuen Homepoint setzen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )


    start_systemcheck_hms = py_trees_ros.subscribers.CheckData(
        name="HMS aktiv??", 
        topic_name="/wrapper/psdk_ros2/hms_info_table", 
        topic_type=psdk_interfaces.msg.HmsInfoTable,
        variable_name="table", #################################### Su
        expected_value=None,         # b 
        comparison_operator= operator.ne,
        fail_if_no_data=True,
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )

    converter_hms = py_trees.decorators.FailureIsSuccess(
        name="converter_hms",
        child=start_systemcheck_hms
    )

    start_systemcheck_edge_sub = py_trees.composites.Sequence("Systemcheck, EDGE und WPM ready?",memory=False)
    
    start_systemcheck_edge_ping_send = py_trees_ros.action_clients.FromConstant(  
        name="Ping an Edge senden",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    start_systemcheck_edge_ping_check = py_trees_ros.subscribers.CheckData(
        name="Mit EDGE verbunden?", 
        topic_name="start_edge_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )


    start_systemcheck_wpm_sub = py_trees.composites.Sequence("Systemcheck, EDGE und WPM ready?",memory=False)


    start_systemcheck_wpm_ping_check = py_trees_ros.subscribers.CheckData(
        name="Ping an WPM senden", 
        topic_name="start_wpm_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=policyvar
    )


    start_systemcheck_wpm_ping_send = py_trees_ros.action_clients.FromConstant(  
        name="WPM geladen?",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    start_wpm_action = py_trees_ros.action_clients.FromConstant(  
        name="WPM ausführen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )


    running_succes_start = py_trees.decorators.RunningIsSuccess(    ## Namen richtig machen Malte
        name = "Start_Running_to_success",
        child = start_wpm_action
        )

    start_sequence.add_children([start_systemcheck_sequence, running_succes_start])     ## Namen richtig machen Malte
    start_systemcheck_sequence.add_children([start_systemcheck_battery,start_systemcheck_rc,start_systemcheck_hp_sub, converter_hms,start_systemcheck_edge_sub,start_systemcheck_wpm_sub])
    start_systemcheck_hp_sub.add_children([start_systemcheck_hp_check,start_systemcheck_hp_set])
    start_systemcheck_edge_sub.add_children([start_systemcheck_edge_ping_send, start_systemcheck_edge_ping_check])
    start_systemcheck_wpm_sub.add_children([start_systemcheck_wpm_ping_send,start_systemcheck_wpm_ping_check])
    return start_sequence


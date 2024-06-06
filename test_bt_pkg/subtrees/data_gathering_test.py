import py_trees
import py_trees_ros.tree
from std_msgs.msg import Bool

def create_data_gatherer():

    sys_check= py_trees_ros.subscribers.ToBlackboard(topic_name="systemcheck_test",
                                          topic_type=Bool,
                                          blackboard_variables={"Systemcheck_erfolgreich": None}
                                          )
    
    edge_check= py_trees_ros.subscribers.ToBlackboard(topic_name="edge_test",
                                          topic_type=Bool,
                                          blackboard_variables={"EDGE_verbunden": None}
                                          )
    
    wpm_check = py_trees_ros.subscribers.ToBlackboard(topic_name="wpm_test",
                                          topic_type=Bool,
                                          blackboard_variables={"WPM_geladen": None}
                                          )
    
    root = py_trees.composites.Parallel(name="Data Gatherer")

    root.add_children([sys_check, edge_check, wpm_check])

    return root
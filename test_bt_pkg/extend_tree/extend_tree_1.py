import py_trees


def extend_tree_1() -> py_trees.behaviour.Behaviour:
    
    root = py_trees.composites.Parallel(name="Parallel Dronecheck")
    
    hpl_ok = py_trees.behaviours.Success("HPL möglich")
    hotpoint_landing = py_trees.behaviours.Success("hotpoint")
    para_landing = py_trees.behaviours.Success("para_landing")
    
    root.add_child(hpl_ok)
    root.add_child(hotpoint_landing)
    root.add_child(para_landing)

    return root
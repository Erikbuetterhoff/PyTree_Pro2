import py_trees
import py_trees_ros.trees
import action_pkg.action as actions


def extend_tree_1() -> py_trees.behaviour.Behaviour:
    
    root = py_trees.composites.Sequenz(name="Start")
    
    start_check = py_trees.behaviours.Selector("Start überprüfen")
    
    wpm_start= py_trees_ros.action_clients.FromConstant( #Action die die WPM ausführt // py_trees_ros_interface
        name="WPM ausführen",
        action_type= actions.WPM,
        action_name= "wpm_action",
        action_goal=actions.WMP.Goal(),
        generate_feedback_message=lambda msg: actions.WPM.Feedback()
    )

    check_start_on_bb = py_trees.decorators.EthernalGuard(
        name="Sequenz abgeschlossen?",
        condition= True, #aus Beispiel Batterycheck, keine Ahung ob vlt noch Funktion nötig
        blackboard_keys={"Systemcheck_erfolgreich", "EDGE_verbunden", "WPM_geladen"}, #abfragen ob alle Werte True vielleicht ?
        child = startsequenz
    )

    startsequenz= py_trees.behaviours.Sequenz("Startsequenz")
    
    check_systemcheck_on_bb = py_trees.decorators.EthernalGuard(
        name="Systemcheck erfolgreich?",
        condition= True, 
        blackboard_keys={"Systemcheck_erfolgreich"},
        child = systemcheck
    )

    # systemcheck-> irgendwie werte von PSDK auslesen (actionclient)

    check_EDGE_on_bb = py_trees.decorators.EthernalGuard(
        name="Mit EDGE verbunden?",
        condition= True, 
        blackboard_keys={"EDGE_verbunden"},
        child = edge_verbinden
    )

    # edge_verbinden(actionclient)

    check_WPM_on_bb = py_trees.decorators.EthernalGuard(
        name="WPM geladen ?",
        condition= True, 
        blackboard_keys={"WPM_geladen"},
        child = wpm_laden
    )

    # wpm_laden (actionclient)

    root.add_children([check_start_on_bb, wpm_start])
    startsequenz.add_children([check_systemcheck_on_bb, check_EDGE_on_bb, check_WPM_on_bb]) #hier fehlen zuweisungen aber bereits in Ethernalguard entahlten
   

    return root
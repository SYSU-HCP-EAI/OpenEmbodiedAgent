from PhyAgentOS.agent.loop import AgentLoop


def test_infer_named_waypoint_detects_explicit_navigation() -> None:
    assert AgentLoop._infer_named_waypoint_from_text("go to the table") == "table"
    assert AgentLoop._infer_named_waypoint_from_text("前往桌子") == "table"


def test_infer_named_waypoint_ignores_robot_id_go_substring() -> None:
    assert AgentLoop._infer_named_waypoint_from_text("pipergo2 看到桌子上有什么") is None

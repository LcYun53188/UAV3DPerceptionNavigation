import nav_local.local_map_builder
import nav_local.local_planner
import nav_local.safety_monitor


def test_nav_local_wrappers_import():
    assert callable(nav_local.local_map_builder.main)
    assert callable(nav_local.local_planner.main)
    assert callable(nav_local.safety_monitor.main)

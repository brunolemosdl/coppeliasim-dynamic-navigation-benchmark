from algorithms import DWAPlanner, ORCAPlanner, PIDPlanner, PlannerConfig, TEBPlanner

def create_planner(planner_type: str, config: PlannerConfig):
    if planner_type.lower() == "pid":
        return PIDPlanner(config)
    elif planner_type.lower() == "dwa":
        return DWAPlanner(config)
    elif planner_type.lower() == "teb":
        return TEBPlanner(config)
    elif planner_type.lower() == "orca":
        return ORCAPlanner(config)
    else:
        raise ValueError(f"Unknown planner type: {planner_type}")

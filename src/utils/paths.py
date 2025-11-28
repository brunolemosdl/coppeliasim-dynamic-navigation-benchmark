from pathlib import Path

def resolve_scene_path(scene_path: str, scene_name: str) -> Path:
    project_root = Path.cwd()

    scenes_dir = project_root / scene_path
    scene_file = scenes_dir / f"{scene_name}.ttt"

    scene_file = scene_file.resolve()

    if not scene_file.exists():
        raise FileNotFoundError(f"Scene file not found: {scene_file}")

    return scene_file

def validate_scene_file(scene_path: str, scene_name: str) -> bool:
    try:
        scene_file = resolve_scene_path(scene_path, scene_name)
        return scene_file.is_file()
    except FileNotFoundError:
        return False

def get_organized_output_dir(
    base_output_dir: str, scene: str, algorithm: str, timestamp: str | None = None
) -> Path:
    scene_name = Path(scene).stem
    organized_dir = Path(base_output_dir) / scene_name / algorithm.lower()
    if timestamp:
        organized_dir = organized_dir / timestamp
    organized_dir.mkdir(parents=True, exist_ok=True)
    return organized_dir

def get_scenario_number(scene_name: str) -> int:
    try:
        if "scene_1" in scene_name or "1" in scene_name:
            return 1
        elif "scene_2" in scene_name or "2" in scene_name:
            return 2
        elif "scene_3" in scene_name or "3" in scene_name:
            return 3
    except Exception:
        pass
    return 1

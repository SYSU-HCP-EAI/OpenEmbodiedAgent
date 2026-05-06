import os


def validate_scene_file(file_path: str, prim_path_root: str = 'background'):
    """
    Validate scene file.
    Args:
        file_path (str): path to scene config file(use to be a .usd file)
        prim_path_root (str): path to root prim

    Returns:
        config_json_path (str): path to config file
        world_prim_path (str): path to world prim
    """
    world_prim_path = '/' + prim_path_root
    if file_path.endswith('usd') or file_path.endswith('usda') or file_path.endswith('usdc'):
        if not os.path.exists(file_path):
            raise FileNotFoundError('File not found: ' + file_path)
        # Add usd directly
        return file_path, world_prim_path
    raise RuntimeError('Env file path needs to end with .usd, .usda or .usdc .')

import importlib
import os

from internutopia.core.util import log

ALL_MODULES = []
MODEL_MODULES = [
    'controllers',
    'objects',
    'metrics',
    'robots',
    'sensors',
    'tasks',
    'interactions',
    'agents',
]

DEFAULT_EXTENSION_PATH = os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../../internutopia_extension')


def _handle_errors(errors):
    """
    Log out and possibly reraise errors during import.

    Args:
        errors: errors dict to be logged
    """
    if not errors:
        return
    for name, err in errors:
        log.warning('Module {} import failed: {}'.format(name, err))


def import_all_modules_for_register(custom_module_paths=None, extension_path=None):
    """
    Import all modules for register.

    Args:
        custom_module_paths: custom module paths, e.g. ['xxx.lib1', 'xxx.lib2', 'xxx.lib3']
        extension_path: Extension path(integrated in internutopia_extension as default)
    """
    if extension_path is None:
        extension_path = DEFAULT_EXTENSION_PATH
    for _mod in MODEL_MODULES:
        # internutopia_extension's default path
        path = os.path.join(extension_path, _mod)
        m = [m.split('.py')[0] for m in os.listdir(path) if m.endswith('.py') and m != '__init__.py']
        ALL_MODULES.append((_mod, m))
    modules = []
    for base_dir, mods in ALL_MODULES:
        for name in mods:
            full_name = 'internutopia_extension.' + base_dir + '.' + name
            modules.append(full_name)
    if isinstance(custom_module_paths, list):
        modules += custom_module_paths
    errors = []
    for module in modules:
        try:
            importlib.import_module(module)
        except ImportError as error:
            errors.append((module, error))
    _handle_errors(errors)

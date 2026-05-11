"""
A set of utility functions for general python usage
"""
import inspect
import re
from abc import ABCMeta
from collections.abc import Iterable
from copy import deepcopy
from functools import wraps
from importlib import import_module

import numpy as np

# Global dictionary storing all unique names
NAMES = set()
CLASS_NAMES = set()


class ClassProperty:
    def __init__(self, f_get):
        self.f_get = f_get

    def __get__(self, owner_self, owner_cls):
        return self.f_get(owner_cls)


def subclass_factory(name, base_classes, __init__=None, **kwargs):
    """
    Programmatically generates a new class type with name @name, subclassing from base classes @base_classes, with
    corresponding __init__ call @__init__.

    NOTE: If __init__ is None (default), the __init__ call from @base_classes will be used instead.

    cf. https://stackoverflow.com/questions/15247075/how-can-i-dynamically-create-derived-classes-from-a-base-class

    Args:
        name (str): Generated class name
        base_classes (type, or list of type): Base class(es) to use for generating the subclass
        __init__ (None or function): Init call to use for the base class when it is instantiated. If None if specified,
            the newly generated class will automatically inherit the __init__ call from @base_classes
        **kwargs (any): keyword-mapped parameters to override / set in the child class, where the keys represent
            the class / instance attribute to modify and the values represent the functions / value to set
    """
    # Standardize base_classes
    base_classes = tuple(base_classes if isinstance(base_classes, Iterable) else [base_classes])

    # Generate the new class
    if __init__ is not None:
        kwargs['__init__'] = __init__
    return type(name, base_classes, kwargs)


def save_init_info(func):
    """
    Decorator to save the init info of an objects to objects._init_info.

    _init_info contains class name and class constructor's input args.
    """
    sig = inspect.signature(func)

    @wraps(func)  # preserve func name, docstring, arguments list, etc.
    def wrapper(self, *args, **kwargs):
        values = sig.bind(self, *args, **kwargs)

        # Prevent args of super init from being saved.
        if hasattr(self, '_init_info'):
            func(*values.args, **values.kwargs)
            return

        # Initialize class's self._init_info.
        self._init_info = {'class_module': self.__class__.__module__, 'class_name': self.__class__.__name__, 'args': {}}

        # Populate class's self._init_info.
        for k, p in sig.parameters.items():
            if k == 'self':
                continue
            if k in values.arguments:
                val = values.arguments[k]
                if p.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY):
                    self._init_info['args'][k] = val
                elif p.kind == inspect.Parameter.VAR_KEYWORD:
                    for kwarg_k, kwarg_val in values.arguments[k].items():
                        self._init_info['args'][kwarg_k] = kwarg_val

        # Call the original function.
        func(*values.args, **values.kwargs)

    return wrapper


class RecreatableMeta(type):
    """
    Simple metaclass that automatically saves __init__ args of the instances it creates.
    """

    def __new__(cls, clsname, bases, clsdict):
        if '__init__' in clsdict:
            clsdict['__init__'] = save_init_info(clsdict['__init__'])
        return super().__new__(cls, clsname, bases, clsdict)


class RecreatableAbcMeta(RecreatableMeta, ABCMeta):
    """
    A composite metaclass of both RecreatableMeta and ABCMeta.

    Adding in ABCMeta to resolve metadata conflicts.
    """

    pass


class Recreatable(metaclass=RecreatableAbcMeta):
    """
    Simple class that provides an abstract interface automatically saving __init__ args of
    the classes inheriting it.
    """

    def get_init_info(self):
        """
        Grabs relevant initialization information for this class instance. Useful for directly
        reloading an objects from this information, using @create_object_from_init_info.

        Returns:
            dict: Nested dictionary that contains this objects' initialization information
        """
        # Note: self._init_info is procedurally generated via @save_init_info called in metaclass
        return self._init_info


def create_object_from_init_info(init_info):
    """
    Create a new objects based on given init info.

    Args:
        init_info (dict): Nested dictionary that contains an objects's init information.

    Returns:
        any: Newly created objects.
    """
    module = import_module(init_info['class_module'])
    cls = getattr(module, init_info['class_name'])
    return cls(**init_info['args'], **init_info.get('kwargs', {}))


def merge_nested_dicts(base_dict, extra_dict, inplace=False, verbose=False):
    """
    Iteratively updates @base_dict with values from @extra_dict. Note: This generates a new dictionary!

    Args:
        base_dict (dict): Nested base dictionary, which should be updated with all values from @extra_dict
        extra_dict (dict): Nested extra dictionary, whose values will overwrite corresponding ones in @base_dict
        inplace (bool): Whether to modify @base_dict in place or not
        verbose (bool): If True, will print when keys are mismatched

    Returns:
        dict: Updated dictionary
    """
    # Loop through all keys in @extra_dict and update the corresponding values in @base_dict
    base_dict = base_dict if inplace else deepcopy(base_dict)
    for k, v in extra_dict.items():
        if k not in base_dict:
            base_dict[k] = v
        else:
            if isinstance(v, dict) and isinstance(base_dict[k], dict):
                base_dict[k] = merge_nested_dicts(base_dict[k], v)
            else:
                not_equal = base_dict[k] != v
                if isinstance(not_equal, np.ndarray):
                    not_equal = not_equal.any()
                if not_equal and verbose:
                    print(f'Different values for key {k}: {base_dict[k]}, {v}\n')
                base_dict[k] = np.array(v) if isinstance(v, list) else v

    # Return new dict
    return base_dict


def get_class_init_kwargs(cls):
    """
    Helper function to return a list of all valid keyword arguments (excluding "self") for the given @cls class.

    Args:
        cls (object): Class from which to grab __init__ kwargs

    Returns:
        list: All keyword arguments (excluding "self") specified by @cls __init__ constructor method
    """
    return list(inspect.signature(cls.__init__).parameters.keys())[1:]


def extract_subset_dict(dic, keys, copy=False):
    """
    Helper function to extract a subset of dictionary key-values from a current dictionary. Optionally (deep)copies
    the values extracted from the original @dic if @copy is True.

    Args:
        dic (dict): Dictionary containing multiple key-values
        keys (Iterable): Specific keys to extract from @dic. If the key doesn't exist in @dic, then the key is skipped
        copy (bool): If True, will deepcopy all values corresponding to the specified @keys

    Returns:
        dict: Extracted subset dictionary containing only the specified @keys and their corresponding values
    """
    subset = {k: dic[k] for k in keys if k in dic}
    return deepcopy(subset) if copy else subset


def extract_class_init_kwargs_from_dict(cls, dic, copy=False):
    """
    Helper function to return a dictionary of key-values that specifically correspond to @cls class's __init__
    constructor method, from @dic which may or may not contain additional, irrelevant kwargs.
    Note that @dic may possibly be missing certain kwargs as specified by cls.__init__. No error will be raised.

    Args:
        cls (object): Class from which to grab __init__ kwargs that will be be used as filtering keys for @dic
        dic (dict): Dictionary containing multiple key-values
        copy (bool): If True, will deepcopy all values corresponding to the specified @keys

    Returns:
        dict: Extracted subset dictionary possibly containing only the specified keys from cls.__init__ and their
            corresponding values
    """
    # extract only relevant kwargs for this specific backbone
    return extract_subset_dict(
        dic=dic,
        keys=get_class_init_kwargs(cls),
        copy=copy,
    )


def assert_valid_key(key, valid_keys, name=None):
    """
    Helper function that asserts that @key is in dictionary @valid_keys keys. If not, it will raise an error.

    Args:
        key (any): key to check for in dictionary @dic's keys
        valid_keys (Iterable): contains keys should be checked with @key
        name (str or None): if specified, is the name associated with the key that will be printed out if the
            key is not found. If None, default is "value"
    """
    if name is None:
        name = 'value'
    assert key in valid_keys, 'Invalid {} received! Valid options are: {}, got: {}'.format(
        name, valid_keys.keys() if isinstance(valid_keys, dict) else valid_keys, key
    )


def create_class_from_registry_and_config(cls_name, cls_registry, cfg, cls_type_descriptor):
    """
    Helper function to create a class with str type @cls_name, which should be a valid entry in @cls_registry, using
    kwargs in dictionary form @cfg to pass to the constructor, with @cls_type_name specified for debugging

    Args:
        cls_name (str): Name of the class to create. This should correspond to the actual class type, in string form
        cls_registry (dict): Class registry. This should map string names of valid classes to create to the
            actual class type itself
        cfg (dict): Any keyword arguments to pass to the class constructor
        cls_type_descriptor (str): Description of the class type being created. This can be any string and is used
            solely for debugging purposes

    Returns:
        any: Created class instance
    """
    # Make sure the requested class type is valid
    assert_valid_key(key=cls_name, valid_keys=cls_registry, name=f'{cls_type_descriptor} type')

    # Grab the kwargs relevant for the specific class
    cls = cls_registry[cls_name]
    cls_kwargs = extract_class_init_kwargs_from_dict(cls=cls, dic=cfg, copy=False)

    # Create the class
    return cls(**cls_kwargs)


def get_uuid(name, n_digits=8):
    """
    Helper function to create a unique @n_digits uuid given a unique @name

    Args:
        name (str): Name of the objects or class
        n_digits (int): Number of digits of the uuid, default is 8

    Returns:
        int: uuid
    """
    return abs(hash(name)) % (10**n_digits)


def camel_case_to_snake_case(camel_case_text):
    """
    Helper function to convert a camel case text to snake case, e.g. "StrawberrySmoothie" -> "strawberry_smoothie"

    Args:
        camel_case_text (str): Text in camel case

    Returns:
        str: snake case text
    """
    return re.sub(r'(?<!^)(?=[A-Z])', '_', camel_case_text).lower()


def snake_case_to_camel_case(snake_case_text):
    """
    Helper function to convert a snake case text to camel case, e.g. "strawberry_smoothie" -> "StrawberrySmoothie"

    Args:
        snake_case_text (str): Text in snake case

    Returns:
        str: camel case text
    """
    return ''.join(item.title() for item in snake_case_text.split('_'))


def meets_minimum_version(test_version, minimum_version):
    """
    Verify that @test_version meets the @minimum_version

    Args:
        test_version (str): Python package version. Should be, e.g., 0.26.1
        minimum_version (str): Python package version to test against. Should be, e.g., 0.27.2

    Returns:
        bool: Whether @test_version meets @minimum_version
    """
    test_nums = [int(num) for num in test_version.split('.')]
    minimum_nums = [int(num) for num in minimum_version.split('.')]
    assert len(test_nums) == 3
    assert len(minimum_nums) == 3

    for test_num, minimum_num in zip(test_nums, minimum_nums):
        if test_num > minimum_num:
            return True
        elif test_num < minimum_num:
            return False
        # Otherwise, we continue through all sub-versions

    # If we get here, that means test_version == threshold_version, so this is a success
    return True


class UniquelyNamed:
    """
    Simple class that implements a name property, that must be implemented by a subclass. Note that any @Named
    entity must be UNIQUE!
    """

    def __init__(self):
        global NAMES
        # Register this objects, making sure it's name is unique
        assert self.name not in NAMES, f'UniquelyNamed objects with name {self.name} already exists!'
        NAMES.add(self.name)

    # def __del__(self):
    #     # Remove this objects name from the registry if it's still there
    #     self.remove_names(include_all_owned=True)

    def remove_names(self, include_all_owned=True, skip_ids=None):
        """
        Checks if self.name exists in the global NAMES registry, and deletes it if so. Possibly also iterates through
        all owned member variables and checks for their corresponding names if @include_all_owned is True.

        Args:
            include_all_owned (bool): If True, will iterate through all owned members of this instance and remove their
                names as well, if they are UniquelyNamed

            skip_ids (None or set of int): If specified, will skip over any ids in the specified set that are matched
                to any attributes found (this compares id(attr) to @skip_ids).
        """
        # Make sure skip_ids is a set so we can pass this into the method, and add the dictionary so we don't
        # get infinite recursive loops
        skip_ids = set() if skip_ids is None else skip_ids
        skip_ids.add(id(self))

        # Check for this name, possibly remove it if it exists
        if self.name in NAMES:
            NAMES.remove(self.name)

        # Also possibly iterate through all owned members and check if those are instances of UniquelyNamed
        if include_all_owned:
            self._remove_names_recursively_from_dict(dic=self.__dict__, skip_ids=skip_ids)

    def _remove_names_recursively_from_dict(self, dic, skip_ids=None):
        """
        Checks if self.name exists in the global NAMES registry, and deletes it if so

        Args:
            skip_ids (None or set): If specified, will skip over any objects in the specified set that are matched
                to any attributes found.
        """
        # Make sure skip_ids is a set so we can pass this into the method, and add the dictionary so we don't
        # get infinite recursive loops
        skip_ids = set() if skip_ids is None else skip_ids
        skip_ids.add(id(dic))

        # Loop through all values in the inputted dictionary, and check if any of the values are UniquelyNamed
        for name, val in dic.items():
            if id(val) not in skip_ids:
                # No need to explicitly add val to skip objects because the methods below handle adding it
                if isinstance(val, UniquelyNamed):
                    val.remove_names(include_all_owned=True, skip_ids=skip_ids)
                elif isinstance(val, dict):
                    # Recursively iterate
                    self._remove_names_recursively_from_dict(dic=val, skip_ids=skip_ids)
                elif hasattr(val, '__dict__'):
                    # Add the attribute and recursively iterate
                    skip_ids.add(id(val))
                    self._remove_names_recursively_from_dict(dic=val.__dict__, skip_ids=skip_ids)
                else:
                    # Otherwise we just add the value to skip_ids so we don't check it again
                    skip_ids.add(id(val))

    @property
    def name(self):
        """
        Returns:
            str: Name of this instance. Must be unique!
        """
        raise NotImplementedError


class UniquelyNamedNonInstance:
    """
    Identical to UniquelyNamed, but intended for non-instanceable classes
    """

    def __init_subclass__(cls, **kwargs):
        global CLASS_NAMES
        # Register this objects, making sure it's name is unique
        assert cls.name not in CLASS_NAMES, f'UniquelyNamed class with name {cls.name} already exists!'
        CLASS_NAMES.add(cls.name)

    @ClassProperty
    def name(self):
        """
        Returns:
            str: Name of this instance. Must be unique!
        """
        raise NotImplementedError


class Registerable:
    """
    Simple class template that provides an abstract interface for registering classes.
    """

    def __init_subclass__(cls, **kwargs):
        """
        Registers all subclasses as part of this registry. This is useful to decouple internal codebase from external
        user additions. This way, users can add their custom subclasses by simply extending this class,
        and it will automatically be registered internally. This allows users to then specify their classes
        directly in string-form in e.g., their config files, without having to manually set the str-to-class mapping
        in our code.
        """
        cls._register_cls()

    @classmethod
    def _register_cls(cls):
        """
        Register this class. Can be extended by subclass.
        """
        # print(f"registering: {cls.__name__}")
        # print(f"registry: {cls._cls_registry}", cls.__name__ not in cls._cls_registry)
        # print(f"do not register: {cls._do_not_register_classes}", cls.__name__ not in cls._do_not_register_classes)
        # input()
        if cls.__name__ not in cls._cls_registry and cls.__name__ not in cls._do_not_register_classes:
            cls._cls_registry[cls.__name__] = cls

    @ClassProperty
    def _do_not_register_classes(self):
        """
        Returns:
            set of str: Name(s) of classes that should not be registered. Default is empty set.
                Subclasses that shouldn't be added should call super() and then add their own class name to the set
        """
        return set()

    @ClassProperty
    def _cls_registry(self):
        """
        Returns:
            dict: Mapping from all registered class names to their classes. This should be a REFERENCE
                to some external, global dictionary that will be filled-in at runtime.
        """
        raise NotImplementedError()


class Serializable:
    """
    Simple class that provides an abstract interface to dump / load states, optionally with serialized functionality
    as well.
    """

    @property
    def state_size(self):
        """
        Returns:
            int: Size of this objects's serialized state
        """
        raise NotImplementedError()

    def _dump_state(self):
        """
        Dumps the state of this objects in dictionary form (can be empty). Should be implemented by subclass.

        Returns:
            dict: Keyword-mapped states of this objects
        """
        raise NotImplementedError()

    def dump_state(self, serialized=False):
        """
        Dumps the state of this objects in either dictionary of flattened numerical form.

        Args:
            serialized (bool): If True, will return the state of this objects as a 1D numpy array. Otherwise,
            will return a (potentially nested) dictionary of states for this objects

        Returns:
            dict or n-array: Either:
                - Keyword-mapped states of these objects, or
                - encoded + serialized, 1D numerical np.array \
                  capturing this objects' state, where n is @self.state_size
        """
        state = self._dump_state()
        return self.serialize(state=state) if serialized else state

    def _load_state(self, state):
        """
        Load the internal state to this objects as specified by @state. Should be implemented by subclass.

        Args:
            state (dict): Keyword-mapped states of this objects to set
        """
        raise NotImplementedError()

    def load_state(self, state, serialized=False):
        """
        Deserializes and loads this objects' state based on @state

        Args:
            state (dict or n-array): Either:
                - Keyword-mapped states of these objects, or
                - encoded + serialized, 1D numerical np.array capturing this objects' state,
                  where n is @self.state_size
            serialized (bool): If True, will interpret @state as a 1D numpy array. Otherwise,
                  will assume the input is a (potentially nested) dictionary of states for this objects
        """
        state = self.deserialize(state=state) if serialized else state
        self._load_state(state=state)

    def _serialize(self, state):
        """
        Serializes nested dictionary state @state into a flattened 1D numpy array for encoding efficiency.
        Should be implemented by subclass.

        Args:
            state (dict): Keyword-mapped states of this objects to encode. Should match structure of output from
                self._dump_state()

        Returns:
            n-array: encoded + serialized, 1D numerical np.array capturing this objects's state
        """
        raise NotImplementedError()

    def serialize(self, state):
        """
        Serializes nested dictionary state @state into a flattened 1D numpy array for encoding efficiency.
        Should be implemented by subclass.

        Args:
            state (dict): Keyword-mapped states of this objects to encode. Should match structure of output from
                self._dump_state()

        Returns:
            n-array: encoded + serialized, 1D numerical np.array capturing this objects's state
        """
        # Simply returns self._serialize() for now. this is for future proofing
        return self._serialize(state=state)

    def _deserialize(self, state):
        """
        De-serializes flattened 1D numpy array @state into nested dictionary state.
        Should be implemented by subclass.

        Args:
            state (n-array): encoded + serialized, 1D numerical np.array capturing this objects's state

        Returns:
            2-tuple:
                - dict: Keyword-mapped states of this objects. Should match structure of output from
                    self._dump_state()
                - int: current index of the flattened state vector that is left off. This is helpful for subclasses
                    that inherit partial deserializations from parent classes, and need to know where the
                    deserialization left off before continuing.
        """
        raise NotImplementedError

    def deserialize(self, state):
        """
        De-serializes flattened 1D numpy array @state into nested dictionary state.
        Should be implemented by subclass.

        Args:
            state (n-array): encoded + serialized, 1D numerical np.array capturing this objects's state

        Returns:
            dict: Keyword-mapped states of these objects. Should match structure of output from
                self._dump_state()
        """
        # Sanity check the idx with the expected state size
        state_dict, idx = self._deserialize(state=state)
        assert idx == self.state_size, (
            f'Invalid state deserialization occurred! Expected {self.state_size} total '
            f'values to be deserialized, only {idx} were.'
        )

        return state_dict


class SerializableNonInstance:
    """
    Identical to Serializable, but intended for non-instance classes
    """

    @ClassProperty
    def state_size(self):
        """
        Returns:
            int: Size of this objects's serialized state
        """
        raise NotImplementedError()

    @classmethod
    def _dump_state(cls):
        """
        Dumps the state of this objects in dictionary form (can be empty). Should be implemented by subclass.

        Returns:
            dict: Keyword-mapped states of this objects
        """
        raise NotImplementedError()

    @classmethod
    def dump_state(cls, serialized=False):
        """
        Dumps the state of this objects in either dictionary of flattened numerical form.

        Args:
            serialized (bool): If True, will return the state of this objects as a 1D numpy array. Otherwise,
            will return a (potentially nested) dictionary of states for this objects

        Returns:
            dict or n-array: Either:
                - Keyword-mapped states of these objects, or
                - encoded + serialized, 1D numerical np.array capturing this objects' state, where n is @self.state_size
        """
        state = cls._dump_state()
        return cls.serialize(state=state) if serialized else state

    @classmethod
    def _load_state(cls, state):
        """
        Load the internal state to this objects as specified by @state. Should be implemented by subclass.

        Args:
            state (dict): Keyword-mapped states of these objects to set
        """
        raise NotImplementedError()

    @classmethod
    def load_state(cls, state, serialized=False):
        """
        Deserializes and loads this objects' state based on @state

        Args:
            state (dict or n-array): Either:
                - Keyword-mapped states of these objects, or
                - encoded + serialized, 1D numerical np.array capturing this objects' state,
                  where n is @self.state_size
            serialized (bool): If True, will interpret @state as a 1D numpy array. Otherwise, will assume the input is
                a (potentially nested) dictionary of states for this objects
        """
        state = cls.deserialize(state=state) if serialized else state
        cls._load_state(state=state)

    @classmethod
    def _serialize(cls, state):
        """
        Serializes nested dictionary state @state into a flattened 1D numpy array for encoding efficiency.
        Should be implemented by subclass.

        Args:
            state (dict): Keyword-mapped states of this objects to encode. Should match structure of output from
                self._dump_state()

        Returns:
            n-array: encoded + serialized, 1D numerical np.array capturing this objects's state
        """
        raise NotImplementedError()

    @classmethod
    def serialize(cls, state):
        """
        Serializes nested dictionary state @state into a flattened 1D numpy array for encoding efficiency.
        Should be implemented by subclass.

        Args:
            state (dict): Keyword-mapped states of these objects to encode. Should match structure of output from
                self._dump_state()

        Returns:
            n-array: encoded + serialized, 1D numerical np.array capturing this objects's state
        """
        # Simply returns self._serialize() for now. this is for future proofing
        return cls._serialize(state=state)

    @classmethod
    def _deserialize(cls, state):
        """
        De-serializes flattened 1D numpy array @state into nested dictionary state.
        Should be implemented by subclass.

        Args:
            state (n-array): encoded + serialized, 1D numerical np.array capturing this objects's state

        Returns:
            2-tuple:
                - dict: Keyword-mapped states of this objects. Should match structure of output from
                    self._dump_state()
                - int: current index of the flattened state vector that is left off. This is helpful for subclasses
                    that inherit partial deserializations from parent classes, and need to know where the
                    deserialization left off before continuing.
        """
        raise NotImplementedError

    @classmethod
    def deserialize(cls, state):
        """
        De-serializes flattened 1D numpy array @state into nested dictionary state.
        Should be implemented by subclass.

        Args:
            state (n-array): encoded + serialized, 1D numerical np.array capturing this objects's state

        Returns:
            dict: Keyword-mapped states of this objects. Should match structure of output from
                self._dump_state()
        """
        # Sanity check the idx with the expected state size
        state_dict, idx = cls._deserialize(state=state)
        assert idx == cls.state_size, (
            f'Invalid state deserialization occurred! Expected {cls.state_size} total '
            f'values to be deserialized, only {idx} were.'
        )

        return state_dict


class Wrapper:
    """
    Base class for all wrapper in OmniGibson

    Args:
        obj (any): Arbitrary python objects instance to wrap
    """

    def __init__(self, obj):
        # Set the internal attributes -- store wrapped obj
        self.wrapped_obj = obj

    @classmethod
    def class_name(cls):
        return cls.__name__

    def _warn_double_wrap(self):
        """
        Utility function that checks if we're accidentally trying to double wrap an scenes
        Raises:
            Exception: [Double wrapping scenes]
        """
        obj = self.wrapped_obj
        while True:
            if isinstance(obj, Wrapper):
                if obj.class_name() == self.class_name():
                    raise Exception('Attempted to double wrap with Wrapper: {}'.format(self.__class__.__name__))
                obj = obj.wrapped_obj
            else:
                break

    @property
    def unwrapped(self):
        """
        Grabs unwrapped objects

        Returns:
            any: The unwrapped objects instance
        """
        return self.wrapped_obj.unwrapped if hasattr(self.wrapped_obj, 'unwrapped') else self.wrapped_obj

    # this method is a fallback option on any methods the original scenes might support
    def __getattr__(self, attr):
        # If we're querying wrapped_obj, raise an error
        if attr == 'wrapped_obj':
            raise AttributeError('wrapped_obj attribute not initialized yet!')

        # Sanity check to make sure wrapped obj is not None -- if so, raise error
        assert self.wrapped_obj is not None, f'Cannot access attribute {attr} since wrapped_obj is None!'

        # using getattr ensures that both __getattribute__ and __getattr__ (fallback) get called
        # (see https://stackoverflow.com/questions/3278077/difference-between-getattr-vs-getattribute)
        orig_attr = getattr(self.wrapped_obj, attr)
        if callable(orig_attr):

            def hooked(*args, **kwargs):
                result = orig_attr(*args, **kwargs)
                # prevent wrapped_class from becoming unwrapped
                if id(result) == id(self.wrapped_obj):
                    return self
                return result

            return hooked
        else:
            return orig_attr

    def __setattr__(self, key, value):
        # Call setattr on wrapped obj if it has the attribute, otherwise, operate on this objects
        if hasattr(self, 'wrapped_obj') and self.wrapped_obj is not None and hasattr(self.wrapped_obj, key):
            setattr(self.wrapped_obj, key, value)
        else:
            super().__setattr__(key, value)


def clear():
    """
    Clear state tied to singleton classes
    """
    NAMES.clear()
    CLASS_NAMES.clear()

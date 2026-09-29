"""Compatibility helpers for older Python runtimes."""

try:
    from dataclasses import dataclass
except ImportError:
    def dataclass(_cls=None, **kwargs):
        frozen = bool(kwargs.get("frozen", False))

        def wrap(cls):
            field_names = list(getattr(cls, "__annotations__", {}).keys())
            defaults = {}
            for name in field_names:
                if hasattr(cls, name):
                    defaults[name] = getattr(cls, name)

            def __init__(self, *args, **named):
                if len(args) > len(field_names):
                    raise TypeError(
                        "{} expected at most {} arguments".format(
                            cls.__name__, len(field_names)
                        )
                    )

                values = {}
                for name, value in zip(field_names, args):
                    values[name] = value

                for name in field_names[len(args):]:
                    if name in named:
                        values[name] = named.pop(name)
                    elif name in defaults:
                        values[name] = defaults[name]
                    else:
                        raise TypeError(
                            "__init__ missing required argument: '{}'".format(name)
                        )

                if named:
                    unknown = sorted(named.keys())[0]
                    raise TypeError(
                        "__init__ got an unexpected keyword argument '{}'".format(
                            unknown
                        )
                    )

                for name in field_names:
                    object.__setattr__(self, name, values[name])

            def __repr__(self):
                parts = ", ".join(
                    "{}={!r}".format(name, getattr(self, name))
                    for name in field_names
                )
                return "{}({})".format(cls.__name__, parts)

            def __eq__(self, other):
                if self.__class__ is not other.__class__:
                    return NotImplemented
                return all(
                    getattr(self, name) == getattr(other, name)
                    for name in field_names
                )

            cls.__init__ = __init__
            cls.__repr__ = __repr__
            cls.__eq__ = __eq__

            if frozen:
                def __setattr__(self, name, value):
                    raise AttributeError(
                        "cannot assign to field '{}'".format(name)
                    )

                cls.__setattr__ = __setattr__

            return cls

        if _cls is None:
            return wrap
        return wrap(_cls)

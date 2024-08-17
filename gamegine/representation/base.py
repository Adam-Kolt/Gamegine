class NamedObject(object):
    """Base class for objects used representation of game objects.

    :param name: Name of the object.
    :type name: str
    """

    def __init__(self, name: str) -> None:
        self.name = name
        pass

    def prefix(self, prefix: str) -> "NamedObject":
        self.name = prefix + self.name
        return self

    def suffix(self, suffix: str) -> "NamedObject":
        self.name = self.name + suffix
        return self

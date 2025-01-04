from typing import Dict, Generic, TypeVar

T = TypeVar("T")


class ValueEntry(Generic[T]):
    """A generic class for storing a value in a state space.

    :param value: The value to store.
    :type value: T
    """

    def __init__(self, value: T, name: str = "") -> None:
        self.value = value
        self.name = name

    def get(self) -> T:
        """Returns the value stored in the entry.

        :return: The value stored in the entry.
        :rtype: T
        """
        return self.value

    def set(self, value: T):
        """Sets the value stored in the entry.

        :param value: The value to store.
        :type value: T
        """
        self.value = value

    def __str__(self):
        return self.name + ": " + str(self.value)

    def __repr__(self):
        return self.name + ": " + str(self.value)


class StateSpace(object):
    """A class for representing a state space, which is a collection of values.

    :param values: A dictionary of values in the state space.
    :type values: Dict[str, ValueEntry]
    :param spaces: A dictionary of sub-state spaces.
    :type spaces: Dict[str, StateSpace]
    """

    def __init__(self) -> None:
        self.values: Dict[str, ValueEntry] = {}
        self.spaces: Dict[str, "StateSpace"] = {}

    def get(self, space: str) -> "StateSpace":
        """Returns the sub-state space with the given name.

        :param space: The name of the sub-state
        :type space: str
        :return: The sub-state space with the given name.
        :rtype: :class:`StateSpace`
        """
        return self.spaces[space]

    def createSpace(self, name: str) -> "StateSpace":
        """Creates a new sub-state space with the given name.

        :param name: The name of the new sub-state
        :type name: str
        :return: The new sub-state space.
        :rtype: :class:`StateSpace`
        """
        self.spaces[name] = StateSpace()
        return self.spaces[name]

    def registerSpace(self, name: str, space: "StateSpace") -> "StateSpace":
        """Registers a sub-state space with the given name.

        :param name: The name of the sub-state
        :type name: str
        :param space: The sub-state space to register.
        :type space: :class:`StateSpace
        :return: The sub-state space that was registered.
        :rtype: :class:`StateSpace`
        """
        self.spaces[name] = space
        return self.spaces[name]

    def getValue(self, name: str) -> ValueEntry:
        """Returns the value entry with the given name.

        :param name: The name of the value entry.
        :type name: str
        :return: The value entry with the given name.
        :rtype: :class:`ValueEntry`
        """

        return self.values[name]

    def setValue(self, name: str, value) -> ValueEntry:
        """Sets the value of the value entry with the given name.

        :param name: The name of the value entry.
        :type name: str
        :param value: The value to set.
        :return: The value entry with the given name.
        :rtype: :class:`ValueEntry`
        """
        if name not in self.values:
            self.values[name] = ValueEntry(value, name)
        else:
            self.values[name].set(value)
        return self.values[name]

    def __getitem__(self, key: str) -> ValueEntry:
        """Returns the value entry with the given name.

        :param key: The name of the value entry.
        :type key: str
        :return: The value entry with the given name.
        :rtype: :class:`ValueEntry`
        """
        return self.getValue(key)

    def __setitem__(self, key: str, value) -> ValueEntry:
        """Sets the value of the value entry with the given name.

        :param key: The name of the value entry.
        :type key: str
        :param value: The value to set.
        :return: The value entry with the given name.
        :rtype: :class:`ValueEntry`
        """
        return self.setValue(key, value)

    def __str__(self):
        return f"StateSpace({self.values}, {self.spaces})"

    def __repr__(self):
        return f"StateSpace({self.values}, {self.spaces})"


class ValueChange(object):
    """A class for representing a change to a value in a state space. Used as outputs of interactable objects.

    :param entry: The value entry to change.
    :type entry: ValueEntry
    :param value: The value to change the entry to.
    """

    def __init__(self, entry: ValueEntry, value) -> None:
        self.entry = entry
        self.value = value
        self.previous_value = entry.get()

    def __str__(self):
        return f"{self.entry} -> {self.value}"

    def apply(self) -> ValueEntry:
        """Applies the value change to the state space.

        :return: The new updated value entry.
        :rtype: ValueEntry
        """
        self.entry.set(self.value)
        return self.entry.get()

    def requested(self):
        """Returns the value that the entry is requested to be changed to.

        :return: The value that the entry is requested to be changed to.
        """
        return self.value

    def current(self):
        """Returns the current value of the entry.

        :return: The current value of the entry.
        """
        return self.entry.get()

    def __repr__(self):
        return f"{self.previous_value} -> {self.value}"


class ValueDecrease(ValueChange):
    """A class for representing a decrease in a value in a state space. Used as outputs of interactable objects.

    :param entry: The value entry to decrease.
    :type entry: ValueEntry
    :param value: The value to decrease the entry by.
    """

    def apply(self) -> ValueEntry:
        self.entry.set(self.entry.get() - self.value)
        return self.entry.get()

    def requested(self):
        return self.entry.get() - self.value

    def __str__(self):
        return f"{self.previous_value} -= {self.value}"

    def __repr__(self):
        return f"{self.previous_value} -= {self.value}"


class ValueIncrease(ValueChange):
    """A class for representing an increase in a value in a state space. Used as outputs of interactable objects.

    :param entry: The value entry to increase.
    :type entry: ValueEntry
    :param value: The value to increase the entry by.
    """

    def apply(self) -> ValueEntry:
        self.entry.set(self.entry.get() + self.value)
        return self.entry.get()

    def requested(self):
        return self.entry.get() + self.value

    def __str__(self):
        return f"{self.previous_value} += {self.value}"

    def __repr__(self):
        return f"{self.previous_value} += {self.value}"

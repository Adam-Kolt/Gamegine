from typing import Dict, Generic, TypeVar

T = TypeVar("T")


class ValueEntry(Generic[T]):
    def __init__(self, value: T) -> None:
        self.value = value

    def get(self) -> T:
        return self.value

    def set(self, value: T):
        self.value = value


class StateSpace(object):
    def __init__(self) -> None:
        self.values: Dict[str, ValueEntry] = {}
        self.spaces: Dict[str, "StateSpace"] = {}

    def get(self, space: str) -> "StateSpace":
        return self.spaces[space]

    def createSpace(self, name: str) -> "StateSpace":
        self.spaces[name] = StateSpace()
        return self.spaces[name]

    def registerSpace(self, name: str, space: "StateSpace") -> "StateSpace":
        self.spaces[name] = space
        return self.spaces[name]

    def getValue(self, name: str) -> ValueEntry:
        return self.values[name]

    def setValue(self, name: str, value) -> ValueEntry:
        if name not in self.values:
            self.values[name] = ValueEntry(value)
        else:
            self.values[name].set(value)
        return self.values[name]

    def __getitem__(self, key: str) -> ValueEntry:
        return self.getValue(key)

    def __setitem__(self, key: str, value) -> ValueEntry:
        return self.setValue(key, value)

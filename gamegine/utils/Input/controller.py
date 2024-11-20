from abc import ABC, abstractmethod
from typing import Callable, Tuple


class AnalogCallback(ABC):
    @abstractmethod
    def on_change(self, callback: Callable[[float], None]):
        pass


class DigitalCallback(ABC):
    @abstractmethod
    def on_change(self, callback: Callable[[bool], None]):
        pass

    @abstractmethod
    def on_press(self, callback: Callable[[], None]):
        pass

    @abstractmethod
    def on_release(self, callback: Callable[[], None]):
        pass


class ControllerInterface(ABC):
    @abstractmethod
    def get_axis(self, axis: int) -> float:
        pass

    @abstractmethod
    def get_left_joystick(self) -> Tuple[float, float]:
        pass

    @abstractmethod
    def get_right_joystick(self) -> Tuple[float, float]:
        pass

    @abstractmethod
    def get_button(self, button: int) -> bool:
        pass

    @abstractmethod
    def get_left_trigger(self) -> float:
        pass

    @abstractmethod
    def get_right_trigger(self) -> float:
        pass

    @abstractmethod
    def get_dpad(self) -> Tuple[float, float]:
        pass

    @abstractmethod
    def get_connected(self) -> bool:
        pass

    @abstractmethod
    def get_name(self) -> str:
        pass

    @abstractmethod
    def get_id(self) -> int:
        pass

    @abstractmethod
    def set_light_color(self, color: Tuple[int, int, int]):
        pass


class Controller:
    def __init__(self, controller: ControllerInterface):
        self.controller = controller

"""
Animation utilities for smooth visual effects.

Provides easing functions, transitions, and animation state management.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, Optional, Any, Dict
from enum import Enum


class EasingFunction(Enum):
    """Common easing functions."""
    
    LINEAR = "linear"
    EASE_IN = "ease_in"
    EASE_OUT = "ease_out"
    EASE_IN_OUT = "ease_in_out"
    BOUNCE = "bounce"
    ELASTIC = "elastic"


def _linear(t: float) -> float:
    return t


def _ease_in(t: float) -> float:
    return t * t


def _ease_out(t: float) -> float:
    return 1 - (1 - t) * (1 - t)


def _ease_in_out(t: float) -> float:
    return 3 * t * t - 2 * t * t * t


def _bounce(t: float) -> float:
    if t < 0.5:
        return 4 * t * t
    else:
        return 1 - pow(-2 * t + 2, 2) / 2


def _elastic(t: float) -> float:
    c4 = (2 * math.pi) / 3
    if t == 0:
        return 0
    elif t == 1:
        return 1
    else:
        return pow(2, -10 * t) * math.sin((t * 10 - 0.75) * c4) + 1


EASING_FUNCTIONS: Dict[EasingFunction, Callable[[float], float]] = {
    EasingFunction.LINEAR: _linear,
    EasingFunction.EASE_IN: _ease_in,
    EasingFunction.EASE_OUT: _ease_out,
    EasingFunction.EASE_IN_OUT: _ease_in_out,
    EasingFunction.BOUNCE: _bounce,
    EasingFunction.ELASTIC: _elastic,
}


def get_easing(easing: EasingFunction) -> Callable[[float], float]:
    """Get the easing function for the given type."""
    return EASING_FUNCTIONS.get(easing, _linear)


def lerp(start: float, end: float, t: float) -> float:
    """Linear interpolation between two values."""
    return start + (end - start) * t


def ease(start: float, end: float, t: float, easing: EasingFunction = EasingFunction.LINEAR) -> float:
    """Interpolate with easing between two values."""
    easing_func = get_easing(easing)
    return lerp(start, end, easing_func(t))


@dataclass
class Animation:
    """
    A single animation that interpolates a value over time.
    """
    
    start_value: float
    end_value: float
    duration: float  # seconds
    easing: EasingFunction = EasingFunction.EASE_IN_OUT
    
    # Internal state
    elapsed: float = field(default=0.0, init=False)
    _completed: bool = field(default=False, init=False)
    
    @property
    def value(self) -> float:
        """Get the current animated value."""
        if self._completed:
            return self.end_value
        t = min(1.0, self.elapsed / self.duration) if self.duration > 0 else 1.0
        return ease(self.start_value, self.end_value, t, self.easing)
    
    @property
    def progress(self) -> float:
        """Get animation progress (0.0 to 1.0)."""
        return min(1.0, self.elapsed / self.duration) if self.duration > 0 else 1.0
    
    @property
    def completed(self) -> bool:
        """Check if the animation has completed."""
        return self._completed
    
    def update(self, delta_time: float):
        """Update the animation state."""
        if self._completed:
            return
        self.elapsed += delta_time
        if self.elapsed >= self.duration:
            self._completed = True
    
    def reset(self):
        """Reset the animation to start."""
        self.elapsed = 0.0
        self._completed = False


class AnimationManager:
    """
    Manages multiple named animations.
    """
    
    def __init__(self):
        self._animations: Dict[str, Animation] = {}
    
    def add(
        self,
        name: str,
        start_value: float,
        end_value: float,
        duration: float,
        easing: EasingFunction = EasingFunction.EASE_IN_OUT,
    ) -> Animation:
        """Create and add a new animation."""
        anim = Animation(start_value, end_value, duration, easing)
        self._animations[name] = anim
        return anim
    
    def get(self, name: str) -> Optional[Animation]:
        """Get an animation by name."""
        return self._animations.get(name)
    
    def get_value(self, name: str, default: float = 0.0) -> float:
        """Get the current value of an animation, or default if not found."""
        anim = self._animations.get(name)
        return anim.value if anim else default
    
    def update(self, delta_time: float):
        """Update all animations."""
        for anim in self._animations.values():
            anim.update(delta_time)
    
    def remove(self, name: str):
        """Remove an animation."""
        self._animations.pop(name, None)
    
    def remove_completed(self):
        """Remove all completed animations."""
        self._animations = {
            name: anim for name, anim in self._animations.items()
            if not anim.completed
        }
    
    def clear(self):
        """Remove all animations."""
        self._animations.clear()


# Global animation manager
_animation_manager: Optional[AnimationManager] = None


def get_animation_manager() -> AnimationManager:
    """Get the global animation manager."""
    global _animation_manager
    if _animation_manager is None:
        _animation_manager = AnimationManager()
    return _animation_manager

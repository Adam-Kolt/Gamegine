"""
Gamegine Renderer - Apple-like simplicity.

Just add objects. The renderer handles everything.

Example:
    renderer = Renderer.create(game=MyGame)
    renderer.add(trajectory)
    renderer.run()
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import (
    Any,
    Callable,
    Dict,
    List,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
    TYPE_CHECKING,
)
import math

import arcade
from arcade.types import Color as ArcadeColor

from gamegine.render.canvas import Canvas, CanvasSettings, get_canvas, set_canvas
from gamegine.render.style import Theme, get_theme, set_theme, ROYAL_BLUE, WHITE, GRAY_FAINT

if TYPE_CHECKING:
    from gamegine.representation.game import Game
    from gamegine.utils.NCIM.ncim import SpatialMeasurement


T = TypeVar("T")


# =============================================================================
# Display Levels
# =============================================================================

class DisplayLevel(Enum):
    """Controls detail verbosity in rendering."""
    
    SHOWCASE = auto()  # Clean visuals: trajectories, obstacles, robot
    DEBUG = auto()     # Full detail: swerve states, velocities, modules


class AlertType(Enum):
    """Types of alerts with different styling."""
    
    INFO = auto()      # Blue - general info
    SUCCESS = auto()   # Green - success actions
    WARNING = auto()   # Orange - warnings
    ERROR = auto()     # Red - errors


@dataclass
class Alert:
    """A slide-in/out notification alert."""
    
    message: str
    alert_type: AlertType = AlertType.INFO
    duration: float = 2.0  # seconds to display
    elapsed: float = 0.0   # time since created
    
    # Animation: 0.3s slide in, hold, 0.3s slide out
    SLIDE_TIME: float = 0.3
    
    @property
    def progress(self) -> float:
        """Returns animation progress 0-1 (0=hidden, 1=fully visible)."""
        if self.elapsed < self.SLIDE_TIME:
            # Sliding in
            return self.elapsed / self.SLIDE_TIME
        elif self.elapsed < self.duration - self.SLIDE_TIME:
            # Fully visible
            return 1.0
        elif self.elapsed < self.duration:
            # Sliding out
            return (self.duration - self.elapsed) / self.SLIDE_TIME
        else:
            return 0.0
    
    @property
    def is_expired(self) -> bool:
        return self.elapsed >= self.duration


# =============================================================================
# Auto-Layer Types
# =============================================================================

# Type-to-layer mapping for automatic layer assignment
_TYPE_LAYERS: Dict[str, int] = {
    "grid": 0,
    "safety_padding": 5,  # Below obstacles so they show through
    "obstacles": 10,
    "mesh": 20,
    "path": 25,
    "trajectory": 30,
    "gamepiece": 35,
    "robot": 40,
    "ui": 100,
}


def _get_layer_for_type(obj: Any) -> str:
    """Determine layer for an object based on its type."""
    type_name = type(obj).__name__.lower()
    
    # Known types
    if "trajectory" in type_name:
        return "trajectory"
    elif "path" in type_name:
        return "path"
    elif "robot" in type_name:
        return "robot"
    elif "obstacle" in type_name or "boundary" in type_name:
        return "obstacles"
    elif "mesh" in type_name or "map" in type_name or "graph" in type_name:
        return "mesh"
    elif "gamepiece" in type_name:
        return "gamepiece"
    
    return "objects"


# =============================================================================
# Handler Registry
# =============================================================================

RenderHandler = Callable[[Any, Canvas, Theme, DisplayLevel], None]


class ObjectRendererRegistry:
    """Registry mapping object types to their render handlers."""
    
    _handlers: Dict[Type, RenderHandler] = {}
    
    @classmethod
    def register(cls, obj_type: Type[T]) -> Callable[[RenderHandler], RenderHandler]:
        """Decorator to register a render handler for a type."""
        def decorator(handler: RenderHandler) -> RenderHandler:
            cls._handlers[obj_type] = handler
            return handler
        return decorator
    
    @classmethod
    def register_handler(cls, obj_type: Type, handler: RenderHandler):
        """Register a handler programmatically."""
        cls._handlers[obj_type] = handler
    
    @classmethod
    def get_handler(cls, obj: Any) -> Optional[RenderHandler]:
        """Get the handler for an object, checking inheritance."""
        obj_type = type(obj)
        
        if obj_type in cls._handlers:
            return cls._handlers[obj_type]
        
        for base in obj_type.__mro__:
            if base in cls._handlers:
                return cls._handlers[base]
        
        return None
    
    @classmethod
    def has_handler(cls, obj_type: Type) -> bool:
        return obj_type in cls._handlers


# =============================================================================
# Render Layer
# =============================================================================

class RenderLayer:
    """A layer containing objects at a specific z-order."""
    
    def __init__(self, name: str, z_order: int = 0):
        self.name = name
        self.z_order = z_order
        self._objects: List[Any] = []
    
    def add(self, obj: Any):
        if obj not in self._objects:
            self._objects.append(obj)
    
    def remove(self, obj: Any):
        if obj in self._objects:
            self._objects.remove(obj)
    
    def clear(self):
        self._objects.clear()
    
    def __iter__(self):
        return iter(self._objects)
    
    def __len__(self):
        return len(self._objects)


# =============================================================================
# Main Renderer
# =============================================================================

class Renderer(arcade.Window):
    """
    Gamegine Renderer - Just add objects, we handle the rest.
    
    Usage:
        renderer = Renderer.create(game=MyGame)
        renderer.add(trajectory)
        renderer.add_obstacles(obstacles)
        renderer.run()
    """
    
    _instance: Optional["Renderer"] = None
    
    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        title: str = "Gamegine",
        render_scale: float = 100.0,
        theme: Optional[Theme] = None,
    ):
        if Renderer._instance is not None:
            raise RuntimeError("Use Renderer.create() or Renderer.get_instance()")
        Renderer._instance = self
        
        super().__init__(width, height, title, resizable=True, update_rate=1/60)
        
        # Canvas
        self._canvas = Canvas(CanvasSettings(render_scale=render_scale))
        set_canvas(self._canvas)
        
        # Theme
        self._theme = theme or Theme()
        set_theme(self._theme)
        arcade.set_background_color(self._theme.background)
        
        # Display level
        self._display_level = DisplayLevel.SHOWCASE
        
        # Layers
        self._layers: Dict[str, RenderLayer] = {}
        for name, z in _TYPE_LAYERS.items():
            self._layers[name] = RenderLayer(name, z)
        self._layers["objects"] = RenderLayer("objects", 25)
        
        # State
        self._game: Optional["Game"] = None
        self._field_size: Optional[Tuple[float, float]] = None
        self._show_grid = True
        
        # Mouse
        self._mouse_x: float = 0
        self._mouse_y: float = 0
        self._hovered_object: Any = None
        
        # Input
        self._keys_pressed: Set[int] = set()
        
        # Time
        self._elapsed_time: float = 0.0
        
        # Callbacks
        self._on_click: List[Callable[[float, float], None]] = []
        self._on_update: List[Callable[[float], None]] = []
        self._on_key_press: List[Callable[[int, int], None]] = []
        
        # Alerts (slide-in/out notifications)
        self._alerts: List[Alert] = []
    
    # -------------------------------------------------------------------------
    # Factory Method
    # -------------------------------------------------------------------------
    
    @classmethod
    def create(
        cls,
        game: Optional["Game"] = None,
        width: int = None,
        height: int = None,
        title: str = None,
        theme: Theme = None,
    ) -> "Renderer":
        """
        Create a renderer, optionally from a Game definition.
        
        This is the preferred way to create a renderer.
        """
        if cls._instance:
            return cls._instance
        
        # Compute dimensions from game
        if game:
            from gamegine.utils.NCIM.Dimensions.spatial import Meter
            field_w, field_h = game.get_field_size()
            scale = 100  # pixels per meter
            width = width or int(field_w.to(Meter) * scale)
            height = height or int(field_h.to(Meter) * scale)
            title = title or game.name
            render_scale = scale
        else:
            width = width or 1600
            height = height or 800
            title = title or "Gamegine"
            render_scale = 100.0
        
        renderer = cls(width, height, title, render_scale, theme)
        
        if game:
            renderer._game = game
            from gamegine.utils.NCIM.Dimensions.spatial import Meter
            field_w, field_h = game.get_field_size()
            renderer._field_size = (field_w.to(Meter), field_h.to(Meter))
        
        return renderer
    
    @classmethod
    def get_instance(cls) -> Optional["Renderer"]:
        return cls._instance
    
    @classmethod
    def reset(cls):
        cls._instance = None
    
    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------
    
    @property
    def drawing_canvas(self) -> Canvas:
        return self._canvas
    
    @property
    def theme(self) -> Theme:
        return self._theme
    
    @property
    def display_level(self) -> DisplayLevel:
        return self._display_level
    
    @display_level.setter
    def display_level(self, value: DisplayLevel):
        self._display_level = value
    
    @property
    def elapsed_time(self) -> float:
        return self._elapsed_time
    
    @property
    def mouse_world_pos(self) -> Tuple[float, float]:
        """Mouse position in world coordinates (meters)."""
        return (
            self._mouse_x / self._canvas.render_scale,
            self._mouse_y / self._canvas.render_scale,
        )
    
    # -------------------------------------------------------------------------
    # Adding Objects (Simple API)
    # -------------------------------------------------------------------------
    
    def add(self, obj: Any):
        """Add any object - automatically placed in correct layer."""
        layer_name = _get_layer_for_type(obj)
        self._get_or_create_layer(layer_name).add(obj)
    
    def add_obstacles(self, obstacles: List[Any]):
        """Add obstacles with proper styling."""
        for obs in obstacles:
            self._layers["obstacles"].add(obs)
    
    def add_safety_padding(self, padding: List[Any]):
        """Add safety padding (expanded bounds)."""
        for p in padding:
            self._layers["safety_padding"].add(p)
    
    def remove(self, obj: Any):
        """Remove an object from all layers."""
        for layer in self._layers.values():
            layer.remove(obj)
    
    def clear_objects(self):
        """Clear all renderable objects."""
        for name, layer in self._layers.items():
            if name not in ["grid"]:
                layer.clear()
    
    def _get_or_create_layer(self, name: str) -> RenderLayer:
        if name not in self._layers:
            max_z = max((l.z_order for l in self._layers.values()), default=0)
            self._layers[name] = RenderLayer(name, max_z + 5)
        return self._layers[name]
    
    # -------------------------------------------------------------------------
    # Event Registration (Decorator-style)
    # -------------------------------------------------------------------------
    
    def on_click(self, callback: Callable[[float, float], None] = None):
        """Register click handler. Can be used as decorator."""
        if callback:
            self._on_click.append(callback)
            return callback
        def decorator(fn):
            self._on_click.append(fn)
            return fn
        return decorator
    
    def on_update_callback(self, callback: Callable[[float], None]):
        """Register update callback."""
        self._on_update.append(callback)
    
    def on_key_press_callback(self, callback: Callable[[int, int], None]):
        """Register key press callback."""
        self._on_key_press.append(callback)
    
    def is_key_pressed(self, key: int) -> bool:
        return key in self._keys_pressed
    
    def show_alert(self, message: str, alert_type: AlertType = AlertType.INFO, duration: float = 2.0):
        """Show a slide-in/out notification alert."""
        self._alerts.append(Alert(message, alert_type, duration))
    
    # -------------------------------------------------------------------------
    # Rendering
    # -------------------------------------------------------------------------
    
    def on_draw(self):
        """Render the scene."""
        try:
            # Clear to white background
            self.clear(color=self._theme.background)
            
            # Grid
            if self._show_grid and self._field_size:
                self._draw_grid()
            
            # Field border
            if self._field_size:
                self._draw_field_border()
            
            # Layers sorted by z-order
            sorted_layers = sorted(self._layers.values(), key=lambda l: l.z_order)
            
            for layer in sorted_layers:
                # Special handling for safety_padding layer
                if layer.name == "safety_padding":
                    for obj in layer:
                        self._draw_safety_padding_object(obj)
                else:
                    for obj in layer:
                        handler = ObjectRendererRegistry.get_handler(obj)
                        if handler:
                            try:
                                handler(obj, self._canvas, self._theme, self._display_level)
                            except Exception as e:
                                print(f"Handler error for {type(obj).__name__}: {e}")
            
            # HUD
            self._draw_alerts()
        except Exception as e:
            print(f"on_draw error: {e}")
            import traceback
            traceback.print_exc()
    
    def _draw_grid(self):
        """Draw subtle grid lines."""
        if not self._field_size:
            return
        
        fw, fh = self._field_size
        fw_px = fw * self._canvas.render_scale
        fh_px = fh * self._canvas.render_scale
        color = self._theme.grid_line
        
        # Vertical lines every meter
        x = 0.0
        while x <= fw_px:
            arcade.draw_line(x, 0, x, fh_px, color, 1)
            x += self._canvas.render_scale
        
        # Horizontal lines every meter
        y = 0.0
        while y <= fh_px:
            arcade.draw_line(0, y, fw_px, y, color, 1)
            y += self._canvas.render_scale
    
    def _draw_field_border(self):
        """Draw field border."""
        fw, fh = self._field_size
        fw_px = fw * self._canvas.render_scale
        fh_px = fh * self._canvas.render_scale
        color = self._theme.field_border
        
        arcade.draw_line(0, 0, fw_px, 0, color, 2)
        arcade.draw_line(fw_px, 0, fw_px, fh_px, color, 2)
        arcade.draw_line(fw_px, fh_px, 0, fh_px, color, 2)
        arcade.draw_line(0, fh_px, 0, 0, color, 2)
    
    def _draw_safety_padding_object(self, obj: Any):
        """Draw a safety padding object with transparent fill, no outline."""
        # Try to get vertices from the object
        vertices = None
        
        if hasattr(obj, 'get_vertices'):
            vertices = obj.get_vertices()
        elif hasattr(obj, 'bounds') and hasattr(obj.bounds, 'discretized'):
            disc = obj.bounds.discretized(8)
            if hasattr(disc, 'get_vertices'):
                vertices = disc.get_vertices()
        
        if vertices and len(vertices) >= 3:
            pixel_points = [
                (self._canvas.to_pixels(p[0]), self._canvas.to_pixels(p[1]))
                for p in vertices
            ]
            # Transparent red fill, no outline
            arcade.draw_polygon_filled(pixel_points, self._theme.safety_padding)
    
    def _draw_alerts(self):
        """Draw slide-in/out notification alerts with smooth animation and fade."""
        if not self._alerts:
            return
        
        # Base colors by type (will apply alpha based on animation)
        base_colors = {
            AlertType.INFO: (65, 105, 225),      # Royal blue
            AlertType.SUCCESS: (34, 197, 94),    # Green
            AlertType.WARNING: (249, 115, 22),   # Orange
            AlertType.ERROR: (239, 68, 68),      # Red
        }
        
        # Draw alerts from top of screen
        y_offset = self.height - 50
        padding_h = 20
        padding_v = 12
        font_size = 14
        
        for alert in self._alerts:
            progress = alert.progress
            if progress <= 0:
                continue
            
            # Ease-in-out animation (smooth cubic bezier approximation)
            eased = progress * progress * (3.0 - 2.0 * progress)
            
            # Calculate text dimensions - generous width for safety
            text_width = len(alert.message) * (font_size * 0.7) + padding_h * 2
            text_height = font_size + padding_v * 2
            
            # Slide in from RIGHT (off-screen) - start position is off right edge
            off_screen_x = self.width + 50
            on_screen_x = self.width - text_width - 20
            x_pos = off_screen_x - (off_screen_x - on_screen_x) * eased
            
            # Fade effect - alpha based on eased progress
            alpha = int(230 * eased)
            
            base = base_colors.get(alert.alert_type, base_colors[AlertType.INFO])
            color = ArcadeColor(base[0], base[1], base[2], alpha)
            text_color = ArcadeColor(255, 255, 255, int(255 * eased))
            
            # Simple rectangle background (no rounded corners)
            arcade.draw_lbwh_rectangle_filled(
                x_pos, y_offset - text_height / 2,
                text_width, text_height,
                color
            )
            
            # Draw text (centered vertically)
            arcade.draw_text(
                alert.message,
                x_pos + padding_h,
                y_offset - font_size / 2 - 2,
                text_color,
                font_size,
                font_name="Arial",
            )
            y_offset -= text_height + 10
    
    # -------------------------------------------------------------------------
    # Event Handlers
    # -------------------------------------------------------------------------
    
    def on_update(self, delta_time: float):
        self._elapsed_time += delta_time
        
        # Update alerts
        for alert in self._alerts:
            alert.elapsed += delta_time
        self._alerts = [a for a in self._alerts if not a.is_expired]
        
        for cb in self._on_update:
            cb(delta_time)
    
    def on_mouse_press(self, x: float, y: float, button: int, modifiers: int):
        if button == arcade.MOUSE_BUTTON_LEFT:
            # Convert to world coords
            wx = x / self._canvas.render_scale
            wy = y / self._canvas.render_scale
            for cb in self._on_click:
                cb(wx, wy)
    
    def on_mouse_motion(self, x: float, y: float, dx: float, dy: float):
        self._mouse_x = x
        self._mouse_y = y
    
    def on_key_press(self, key: int, modifiers: int):
        self._keys_pressed.add(key)
        
        # Built-in key bindings
        if key == arcade.key.D:
            self._display_level = (
                DisplayLevel.DEBUG if self._display_level == DisplayLevel.SHOWCASE
                else DisplayLevel.SHOWCASE
            )
            mode_name = "DEBUG" if self._display_level == DisplayLevel.DEBUG else "SHOWCASE"
            self.show_alert(f"Mode: {mode_name}", AlertType.INFO, 1.5)
        elif key == arcade.key.G:
            self._show_grid = not self._show_grid
            grid_status = "ON" if self._show_grid else "OFF"
            self.show_alert(f"Grid: {grid_status}", AlertType.INFO, 1.5)
        
        # Custom callbacks
        for cb in self._on_key_press:
            cb(key, modifiers)
    
    def on_key_release(self, key: int, modifiers: int):
        self._keys_pressed.discard(key)
    
    # -------------------------------------------------------------------------
    # Utilities
    # -------------------------------------------------------------------------
    
    def screen_to_world(self, x: float, y: float) -> Tuple[float, float]:
        """Convert screen coordinates to world (meters)."""
        return (x / self._canvas.render_scale, y / self._canvas.render_scale)
    
    @staticmethod
    def to_pixels(value: "SpatialMeasurement") -> float:
        """Convert a measurement to pixels."""
        return get_canvas().to_pixels(value)


# =============================================================================
# Convenience
# =============================================================================

def run():
    """Start the render loop."""
    arcade.run()

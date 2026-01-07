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
        """Adds an object to this layer."""
        if obj not in self._objects:
            self._objects.append(obj)
    
    def remove(self, obj: Any):
        """Removes an object from this layer."""
        if obj in self._objects:
            self._objects.remove(obj)
    
    def clear(self):
        """Removes all objects from this layer."""
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
        
        # Selection & Info Card
        self._selected_object: Any = None  # Object or Callable returning object
        self._hovered_object: Any = None   # Currently hovered selectable object
        self._selectables: Dict[int, Tuple[Any, Callable[[float, float], bool]]] = {}  # id(obj) → (obj, hit_test_fn)
        self._info_card_progress: float = 0.0  # 0=hidden, 1=visible
        self._info_card_target: float = 0.0    # Target for animation
        
        # Dynamic providers - callables that return objects to render each frame
        self._dynamic_providers: List[Tuple[Callable[[], Any], str]] = []  # (provider, layer_name)
        self._tracked_robot: Optional[Callable[[], Any]] = None  # Robot state provider
    
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
        
        :param game: Optional Game instance to auto-configure dimensions.
        :param width: Window width in pixels.
        :param height: Window height in pixels.
        :param title: Window title.
        :param theme: Custom Theme instance.
        :return: A new or existing Renderer instance.
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
        """Returns the singleton instance of the Renderer if it exists."""
        return cls._instance
    
    @classmethod
    def reset(cls):
        """Resets the singleton instance."""
        cls._instance = None
    
    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------
    
    @property
    def drawing_canvas(self) -> Canvas:
        """The underlying drawing canvas."""
        return self._canvas
    
    @property
    def theme(self) -> Theme:
        """The current color theme."""
        return self._theme
    
    @property
    def display_level(self) -> DisplayLevel:
        """The current detail level (SHOWCASE or DEBUG)."""
        return self._display_level
    
    @display_level.setter
    def display_level(self, value: DisplayLevel):
        self._display_level = value
    
    @property
    def elapsed_time(self) -> float:
        """Time elapsed since renderer start."""
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
        """Add any object - automatically placed in correct layer.
        
        :param obj: The object to render.
        """
        layer_name = _get_layer_for_type(obj)
        self._get_or_create_layer(layer_name).add(obj)
    
    def add_obstacles(self, obstacles: List[Any]):
        """Add obstacles with proper styling.
        
        :param obstacles: List of obstacle objects.
        """
        for obs in obstacles:
            self._layers["obstacles"].add(obs)
    
    def add_safety_padding(self, padding: List[Any]):
        """Add safety padding (expanded bounds).
        
        :param padding: List of padding objects.
        """
        for p in padding:
            self._layers["safety_padding"].add(p)
    
    def remove(self, obj: Any):
        """Remove an object from all layers.
        
        :param obj: The object to remove.
        """
        for layer in self._layers.values():
            layer.remove(obj)
    
    def clear_objects(self):
        """Clear all renderable objects (except grid)."""
        for name, layer in self._layers.items():
            if name not in ["grid"]:
                layer.clear()
    
    def _get_or_create_layer(self, name: str) -> RenderLayer:
        if name not in self._layers:
            max_z = max((l.z_order for l in self._layers.values()), default=0)
            self._layers[name] = RenderLayer(name, max_z + 5)
        return self._layers[name]
    
    # -------------------------------------------------------------------------
    # Dynamic Data Providers
    # -------------------------------------------------------------------------
    
    def add_dynamic(self, provider: Callable[[], Any], layer: str = "objects"):
        """Add a dynamic object via provider function.
        
        The provider is called each frame to get the current object to render.
        This separates game logic from rendering - the game updates the data,
        the renderer reads it via the provider.
        
        :param provider: Callable that returns the object to render.
        :param layer: Layer name to render object in (default: "objects").
        
        Example:
            robot_state = RobotState(...)
            renderer.add_dynamic(lambda: robot_state)
            # Game logic updates robot_state, renderer reads it each frame
        """
        self._dynamic_providers.append((provider, layer))
    
    def track_robot(self, robot_provider: Callable[[], Any]):
        """Track a robot for rendering.
        
        The robot provider is called each frame to get the current robot state.
        The robot is rendered on top of other elements.
        
        :param robot_provider: Callable returning robot state object.
        
        Example:
            renderer.track_robot(lambda: game.get_robot_state("MainBot"))
        """
        self._tracked_robot = robot_provider
    
    def clear_dynamic(self):
        """Clear all dynamic providers and tracked robot."""
        self._dynamic_providers.clear()
        self._tracked_robot = None
    
    # -------------------------------------------------------------------------
    # Event Registration (Decorator-style)
    # -------------------------------------------------------------------------
    
    def on_click(self, callback: Callable[[float, float], None] = None):
        """Register click handler. Can be used as decorator.
        
        Callback receives (x, y) in world coordinates.
        """
        if callback:
            self._on_click.append(callback)
            return callback
        def decorator(fn):
            self._on_click.append(fn)
            return fn
        return decorator
    
    def on_update_callback(self, callback: Callable[[float], None]):
        """Register update callback.
        
        :param callback: Function accepting (delta_time).
        """
        self._on_update.append(callback)
    
    def on_key_press_callback(self, callback: Callable[[int, int], None]):
        """Register key press callback.
        
        :param callback: Function accepting (key, modifiers).
        """
        self._on_key_press.append(callback)
    
    def is_key_pressed(self, key: int) -> bool:
        """Check if a specific key is currently held down."""
        return key in self._keys_pressed
    
    def show_alert(self, message: str, alert_type: AlertType = AlertType.INFO, duration: float = 2.0):
        """Show a slide-in/out notification alert.
        
        :param message: The message text.
        :param alert_type: Notification type (color).
        :param duration: Seconds to display.
        """
        self._alerts.append(Alert(message, alert_type, duration))
    
    def select(self, obj_or_provider: Any):
        """Select an object to show in the info card.
        
        :param obj_or_provider: Either a static object, or a callable that returns 
                               the current object state (for dynamic updates).
        """
        self._selected_object = obj_or_provider
    
    def deselect(self):
        """Deselect the current object (hides info card)."""
        self._selected_object = None
    
    @property
    def selected_object(self) -> Any:
        """Get the currently selected object (calls provider if callable)."""
        if callable(self._selected_object):
            return self._selected_object()
        return self._selected_object
    
    @property
    def hovered_object(self) -> Any:
        """Get the currently hovered object."""
        return self._hovered_object
    
    def register_selectable(self, obj: Any, hit_test: Callable[[float, float], bool]):
        """Register an object as selectable with a hit test function.
        
        :param obj: The object to register
        :param hit_test: Function(world_x, world_y) -> bool, returns True if point is on object
        """
        self._selectables[id(obj)] = (obj, hit_test)
    
    def unregister_selectable(self, obj: Any):
        """Unregister a selectable object.
        
        :param obj: The object to unregister.
        """
        self._selectables.pop(id(obj), None)
    
    def _hit_test_selectables(self, world_x: float, world_y: float) -> Optional[Any]:
        """Find which selectable object is at the given world coordinates."""
        for obj_id, (obj, hit_test) in self._selectables.items():
            try:
                if hit_test(world_x, world_y):
                    return obj
            except:
                pass
        return None
    
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
                                handler(obj, self._canvas, self._theme, self._display_level, self)
                            except Exception as e:
                                print(f"Handler error for {type(obj).__name__}: {e}")
            
            # Dynamic providers - render objects from callables each frame
            for provider, layer_name in self._dynamic_providers:
                try:
                    obj = provider()
                    if obj is not None:
                        handler = ObjectRendererRegistry.get_handler(obj)
                        if handler:
                            handler(obj, self._canvas, self._theme, self._display_level, self)
                except Exception as e:
                    print(f"Dynamic provider error: {e}")
            
            # Tracked robot - rendered on top
            if self._tracked_robot is not None:
                try:
                    robot = self._tracked_robot()
                    if robot is not None:
                        handler = ObjectRendererRegistry.get_handler(robot)
                        if handler:
                            handler(robot, self._canvas, self._theme, self._display_level, self)
                except Exception as e:
                    print(f"Tracked robot render error: {e}")
            
            # HUD
            self._draw_alerts()
            self._draw_info_card()
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
    
    def _draw_info_card(self):
        """Draw info card sliding from left with object details."""
        if self._info_card_progress <= 0.01:
            return
        
        # Card dimensions
        card_width = 280
        card_height = 400
        padding = 16
        header_height = 40
        
        # Slide in from left (eased)
        eased = self._info_card_progress * self._info_card_progress * (3.0 - 2.0 * self._info_card_progress)
        x_pos = -card_width + (card_width + 20) * eased
        y_pos = self.height - card_height - 50
        
        # Fade effect
        alpha = int(240 * eased)
        
        # Card background (light/white)
        arcade.draw_lbwh_rectangle_filled(
            x_pos, y_pos,
            card_width, card_height,
            ArcadeColor(250, 250, 252, alpha)
        )
        
        # Shadow effect (subtle darker rectangle behind)
        arcade.draw_lbwh_rectangle_filled(
            x_pos + 3, y_pos - 3,
            card_width, card_height,
            ArcadeColor(0, 0, 0, int(30 * eased))
        )
        # Redraw card on top of shadow
        arcade.draw_lbwh_rectangle_filled(
            x_pos, y_pos,
            card_width, card_height,
            ArcadeColor(250, 250, 252, alpha)
        )
        
        # Header background
        arcade.draw_lbwh_rectangle_filled(
            x_pos, y_pos + card_height - header_height,
            card_width, header_height,
            ArcadeColor(65, 105, 225, alpha)  # Royal blue
        )
        
        # Get object info
        info_lines = self._get_object_info(self.selected_object)
        
        # Header text (object type)
        header_text = info_lines[0] if info_lines else "Selected Object"
        arcade.draw_text(
            header_text,
            x_pos + padding, y_pos + card_height - header_height + 12,
            ArcadeColor(255, 255, 255, int(255 * eased)),
            14,
            font_name="Arial",
            bold=True,
        )
        
        # Body text
        text_y = y_pos + card_height - header_height - 30
        for line in info_lines[1:]:
            if text_y < y_pos + 20:
                break
            arcade.draw_text(
                line,
                x_pos + padding, text_y,
                ArcadeColor(50, 50, 60, int(255 * eased)),
                12,
                font_name="Arial",
            )
            text_y -= 20
        
        # Draw swerve diagram if object has module states
        obj = self.selected_object
        if hasattr(obj, 'module_states') and obj.module_states and len(obj.module_states) >= 4:
            self._draw_swerve_diagram(x_pos + card_width // 2, y_pos + 80, 50, obj.module_states, eased)
    
    def _draw_swerve_diagram(self, cx: float, cy: float, size: float, module_states, alpha_factor: float):
        """Draw a mini swerve robot diagram with module states."""
        import math
        from gamegine.utils.NCIM.Dimensions.angular import Radian
        from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
        
        alpha = int(255 * alpha_factor)
        
        # Draw robot body outline
        half = size * 0.8
        arcade.draw_lbwh_rectangle_outline(
            cx - half, cy - half, half * 2, half * 2,
            ArcadeColor(100, 100, 120, alpha), 2
        )
        
        # Module positions (FL, FR, BL, BR)
        offsets = [
            (-half, half),    # FL
            (half, half),     # FR
            (-half, -half),   # BL
            (half, -half),    # BR
        ]
        module_labels = ["FL", "FR", "BL", "BR"]
        
        for i, (dx, dy) in enumerate(offsets):
            if i >= len(module_states):
                break
            mod = module_states[i]
            mx, my = cx + dx, cy + dy
            
            # Get angle and omega
            angle = mod.wheel_angle.to(Radian) if hasattr(mod.wheel_angle, 'to') else float(mod.wheel_angle)
            omega = mod.wheel_omega.to(RadiansPerSecond) if hasattr(mod.wheel_omega, 'to') else float(mod.wheel_omega)
            
            # Draw module circle
            arcade.draw_circle_filled(mx, my, 8, ArcadeColor(65, 105, 225, alpha))
            
            # Draw wheel direction arrow
            arrow_len = 15 + min(abs(omega) * 2, 15)  # Scale with velocity
            end_x = mx + math.cos(angle) * arrow_len
            end_y = my + math.sin(angle) * arrow_len
            
            # Arrow color: green for forward, red for reverse
            arrow_color = ArcadeColor(34, 197, 94, alpha) if omega >= 0 else ArcadeColor(239, 68, 68, alpha)
            arcade.draw_line(mx, my, end_x, end_y, arrow_color, 2)
            
            # Arrowhead
            head_angle = math.atan2(end_y - my, end_x - mx)
            head_len = 5
            for da in [2.5, -2.5]:  # Two sides of arrowhead
                hx = end_x - math.cos(head_angle + da) * head_len
                hy = end_y - math.sin(head_angle + da) * head_len
                arcade.draw_line(end_x, end_y, hx, hy, arrow_color, 2)
            
            # Module label
            arcade.draw_text(
                module_labels[i],
                mx - 6, my + 12,
                ArcadeColor(80, 80, 100, alpha),
                8,
                font_name="Arial",
            )
    
    def _get_object_info(self, obj: Any) -> List[str]:
        """Extract info lines for an object."""
        # Import units at top to avoid UnboundLocalError
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        from gamegine.utils.NCIM.Dimensions.temporal import Second
        from gamegine.utils.NCIM.Dimensions.angular import Degree, Radian
        from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
        from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
        
        if obj is None:
            return ["No Selection"]
        
        lines = []
        type_name = type(obj).__name__
        lines.append(f"📦 {type_name}")
        
        # Position info
        if hasattr(obj, 'x') and hasattr(obj, 'y'):
            x_val = obj.x.to(Meter) if hasattr(obj.x, 'to') else float(obj.x)
            y_val = obj.y.to(Meter) if hasattr(obj.y, 'to') else float(obj.y)
            lines.append(f"Position: ({x_val:.2f}, {y_val:.2f}) m")
        
        # Heading
        if hasattr(obj, 'theta'):
            theta = obj.theta.to(Degree) if hasattr(obj.theta, 'to') else float(obj.theta)
            lines.append(f"Heading: {theta:.1f}°")
        
        # Trajectory specific
        if hasattr(obj, 'get_length') and hasattr(obj, 'get_travel_time'):
            length = obj.get_length().to(Meter)
            time = obj.get_travel_time().to(Second)
            lines.append(f"Length: {length:.2f} m")
            lines.append(f"Travel time: {time:.2f} s")
            if hasattr(obj, 'points'):
                lines.append(f"Waypoints: {len(obj.points)}")
        
        # Velocity
        if hasattr(obj, 'vel_x') and hasattr(obj, 'vel_y'):
            vx = obj.vel_x.to(MetersPerSecond) if hasattr(obj.vel_x, 'to') else float(obj.vel_x)
            vy = obj.vel_y.to(MetersPerSecond) if hasattr(obj.vel_y, 'to') else float(obj.vel_y)
            speed = (vx**2 + vy**2)**0.5
            lines.append(f"Velocity: {speed:.2f} m/s")
        
        # Omega (angular velocity)
        if hasattr(obj, 'omega'):
            omega = obj.omega.to(RadiansPerSecond) if hasattr(obj.omega, 'to') else float(obj.omega)
            lines.append(f"Angular vel: {omega:.2f} rad/s")
        
        # Swerve module states
        if hasattr(obj, 'module_states') and obj.module_states:
            lines.append("")
            lines.append("🔧 Swerve Modules:")
            module_names = ["FL", "FR", "BL", "BR"]
            for i, mod in enumerate(obj.module_states[:4]):
                name = module_names[i] if i < len(module_names) else f"M{i}"
                angle = mod.wheel_angle.to(Degree) if hasattr(mod.wheel_angle, 'to') else float(mod.wheel_angle)
                omega_val = mod.wheel_omega.to(RadiansPerSecond) if hasattr(mod.wheel_omega, 'to') else float(mod.wheel_omega)
                lines.append(f"  {name}: {angle:.0f}° @ {omega_val:.1f} rad/s")
        
        return lines
    
    # -------------------------------------------------------------------------
    # Event Handlers
    # -------------------------------------------------------------------------
    
    def on_update(self, delta_time: float):
        self._elapsed_time += delta_time
        
        # Update alerts
        for alert in self._alerts:
            alert.elapsed += delta_time
        self._alerts = [a for a in self._alerts if not a.is_expired]
        
        # Animate info card slide (smooth lerp)
        self._info_card_target = 1.0 if self._selected_object else 0.0
        self._info_card_progress += (self._info_card_target - self._info_card_progress) * 0.12
        # Snap to target when close
        if abs(self._info_card_progress - self._info_card_target) < 0.01:
            self._info_card_progress = self._info_card_target
        
        for cb in self._on_update:
            cb(delta_time)
    
    def on_mouse_press(self, x: float, y: float, button: int, modifiers: int):
        if button == arcade.MOUSE_BUTTON_LEFT:
            # Convert to world coords
            wx = x / self._canvas.render_scale
            wy = y / self._canvas.render_scale
            
            # Automatic click selection
            hit = self._hit_test_selectables(wx, wy)
            if hit is not None:
                self.select(hit)
            elif self._selected_object is not None and not callable(self._selected_object):
                # Click on empty space - deselect (but not if using a state provider)
                self.deselect()
            
            for cb in self._on_click:
                cb(wx, wy)
    
    def on_mouse_motion(self, x: float, y: float, dx: float, dy: float):
        self._mouse_x = x
        self._mouse_y = y
        
        # Automatic hover detection
        world_x, world_y = self.screen_to_world(x, y)
        self._hovered_object = self._hit_test_selectables(world_x, world_y)
    
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

    # -------------------------------------------------------------------------
    # Frame Control API (for manual animation loops)
    # -------------------------------------------------------------------------

    @property
    def render_scale(self) -> float:
        """Returns the render scale (pixels per meter)."""
        return self._canvas.render_scale

    def begin_frame(self):
        """
        Begin a frame for manual rendering. Call this at start of render loop.
        Clears the screen and draws background/grid.
        """
        self.clear(color=self._theme.background)
        
        # Grid
        if self._show_grid and self._field_size:
            self._draw_grid()
        
        # Field border
        if self._field_size:
            self._draw_field_border()

    def end_frame(self):
        """
        End a frame and present to screen. Call this at end of render loop.
        Flips the arcade display buffer.
        """
        self.flip()

    def draw_element(self, obj: Any):
        """Draw a single element immediately using arcade."""
        handler = ObjectRendererRegistry.get_handler(obj)
        if handler:
            try:
                handler(obj, self._canvas, self._theme, self._display_level, self)
            except Exception as e:
                print(f"draw_element error for {type(obj).__name__}: {e}")

    def draw_elements(self, objs):
        """Draw multiple elements immediately using arcade."""
        for obj in objs:
            self.draw_element(obj)

    def draw_static_elements(self):
        """Draw static game elements (obstacles from game). Grid/border are drawn in begin_frame()."""
        # Draw obstacles from game if loaded
        if self._game:
            for obs in self._game.get_obstacles():
                self.draw_element(obs)

    def draw_text(self, text: str, x: float, y: float, color=(0, 0, 0), font_size: int = 18):
        """
        Draw text at screen coordinates using arcade.
        
        :param text: Text to draw
        :param x: X position in pixels
        :param y: Y position in pixels
        :param color: RGB tuple
        :param font_size: Font size
        """
        arcade.draw_text(text, x, y, color, font_size)

    def draw_rect(self, x: float, y: float, width: float, height: float, color=(255, 255, 255)):
        """
        Draw a filled rectangle using arcade.
        
        :param x: X position (left) in pixels
        :param y: Y position (bottom) in pixels
        :param width: Width in pixels
        :param height: Height in pixels
        :param color: RGB tuple
        """
        arcade.draw_lbwh_rectangle_filled(x, y, width, height, color)

    def step(self, delta_time: float = 1/60) -> bool:
        """
        Process one step of the render loop. Use for manual animation.
        Returns True to continue, False if window should close.
        
        Usage:
            while renderer.step():
                renderer.begin_frame()
                renderer.draw_elements(objects)
                renderer.end_frame()
        """
        # Process arcade dispatch events
        self.dispatch_events()
        
        # Check for close
        if not self._running:
            return False
        
        # Update time
        self._elapsed_time += delta_time
        
        # Call registered update callbacks
        for cb in self._on_update:
            cb(delta_time)
        
        return True

    def should_close(self) -> bool:
        """Check if the window should close."""
        return not getattr(self, '_running', True)

    def set_game(self, game: "Game"):
        """Set the game for rendering."""
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        self._game = game
        if game:
            field_w, field_h = game.get_field_size()
            self._field_size = (field_w.to(Meter), field_h.to(Meter))

    def init_display(self):
        """Initialize display - no-op for arcade-based renderer."""
        pass  # Arcade handles this in __init__

    def set_render_scale(self, scale):
        """Set the render scale."""
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        # Scale could be a Measurement or a float
        if hasattr(scale, 'to'):
            # It's a measurement - treat as pixels per unit
            self._canvas._settings.render_scale = 1.0 / scale.to(Meter)
        else:
            self._canvas._settings.render_scale = float(scale)

    # Legacy compatibility - loop() now uses arcade
    def loop(self) -> bool:
        """
        Manual render loop step. Processes events and returns True to continue.
        
        Usage:
            while renderer.loop():
                renderer.draw_elements(obstacles)
                renderer.render_frame()
        """
        self.dispatch_events()
        return getattr(self, '_running', True)

    def render_frame(self):
        """Flip the display buffer using arcade."""
        self.flip()


# =============================================================================
# Convenience
# =============================================================================

def run():
    """Start the render loop."""
    arcade.run()

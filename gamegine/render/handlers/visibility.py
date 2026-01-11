"""
Visibility rendering handlers for Gamegine.

Provides visualization for AprilTag visibility analysis including:
- Visibility heatmaps across the field
- AprilTag visualizations with visibility cones
- Camera sightlines and detection quality indicators
"""

from dataclasses import dataclass, field
from typing import Any, List, Optional, Dict, Tuple
import math

import arcade
from arcade.types import Color as ArcadeColor

from gamegine.render.renderer import ObjectRendererRegistry, DisplayLevel
from gamegine.render.canvas import Canvas
from gamegine.render.style import (
    Theme, Palette, lerp_color, with_alpha,
    ROYAL_BLUE, GRAY_LIGHT, WHITE,
)
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Inch


# =============================================================================
# Color Gradients for Heatmaps
# =============================================================================

# Visibility quality gradient: Red (bad) -> Yellow (medium) -> Green (good)
QUALITY_GRADIENT = [
    ArcadeColor(180, 50, 50, 180),    # 0.0 - Low quality (red)
    ArcadeColor(180, 50, 50, 180),    # 0.2
    ArcadeColor(220, 180, 60, 180),   # 0.4 - Medium (yellow)
    ArcadeColor(100, 200, 100, 180),  # 0.6
    ArcadeColor(60, 180, 80, 180),    # 0.8 - High quality (green)
    ArcadeColor(40, 200, 100, 200),   # 1.0 - Best (bright green)
]

# Tag count gradient: Blue scale
TAG_COUNT_GRADIENT = [
    ArcadeColor(200, 200, 220, 100),  # 0 tags (faint gray)
    ArcadeColor(150, 170, 220, 130),  # 1-2 tags (light blue)
    ArcadeColor(100, 140, 220, 160),  # 3-4 tags
    ArcadeColor(70, 110, 220, 180),   # 5-6 tags
    ArcadeColor(50, 90, 230, 200),    # 7-8 tags (strong blue)
    ArcadeColor(40, 70, 250, 220),    # 9+ tags (vivid blue)
]


def get_quality_color(quality: float) -> ArcadeColor:
    """Get color for a quality value (0.0 to 1.0)."""
    if quality <= 0:
        return ArcadeColor(100, 100, 100, 60)  # Gray for zero
    
    idx = min(int(quality * (len(QUALITY_GRADIENT) - 1)), len(QUALITY_GRADIENT) - 2)
    t = (quality * (len(QUALITY_GRADIENT) - 1)) - idx
    return lerp_color(QUALITY_GRADIENT[idx], QUALITY_GRADIENT[idx + 1], t)


def get_tag_count_color(count: int, max_count: int = 10) -> ArcadeColor:
    """Get color for tag count."""
    if count == 0:
        return ArcadeColor(150, 150, 150, 40)
    
    normalized = min(count / max_count, 1.0)
    idx = min(int(normalized * (len(TAG_COUNT_GRADIENT) - 1)), len(TAG_COUNT_GRADIENT) - 2)
    t = (normalized * (len(TAG_COUNT_GRADIENT) - 1)) - idx
    return lerp_color(TAG_COUNT_GRADIENT[idx], TAG_COUNT_GRADIENT[idx + 1], t)


# =============================================================================
# Visibility Grid Wrapper for Rendering
# =============================================================================

@dataclass
class VisibilityHeatmap:
    """Renderable visibility heatmap.
    
    This wrapper allows the visibility grid to be rendered using the standard
    handler pattern.
    
    :param cells: List of VisibilityGridCell from the analyzer.
    :param mode: 'quality' for quality-based color, 'count' for tag count color.
    :param cell_size: Size of each cell in the grid.
    """
    cells: List[Any]  # VisibilityGridCell
    mode: str = "quality"  # 'quality' or 'count'
    cell_size: float = 0.3  # meters
    max_tags: int = 10


@ObjectRendererRegistry.register(VisibilityHeatmap)
def render_visibility_heatmap(
    obj: VisibilityHeatmap,
    canvas: Canvas,
    theme: Theme,
    display_level: DisplayLevel,
    renderer=None,
):
    """Render visibility heatmap as colored grid cells."""
    if not obj.cells:
        return
    
    half_size = Meter(obj.cell_size / 2)
    
    # Group cells by position to get best heading per position
    positions: Dict[Tuple[float, float], Any] = {}
    for cell in obj.cells:
        key = (round(float(cell.x.to(Meter)), 2), round(float(cell.y.to(Meter)), 2))
        if key not in positions:
            positions[key] = cell
        elif cell.total_quality > positions[key].total_quality:
            positions[key] = cell
    
    for (x, y), cell in positions.items():
        if obj.mode == "quality":
            color = get_quality_color(cell.total_quality)
        else:  # count mode
            color = get_tag_count_color(cell.visible_tags, obj.max_tags)
        
        cx = canvas.to_pixels(Meter(x))
        cy = canvas.to_pixels(Meter(y))
        size = canvas.to_pixels(Meter(obj.cell_size))
        
        # Draw as polygon to avoid API version issues
        half = size / 2
        points = [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half),
        ]
        arcade.draw_polygon_filled(points, color)


# =============================================================================
# AprilTag Visualization
# =============================================================================

@dataclass
class VisualAprilTag:
    """Renderable AprilTag with visibility cone.
    
    :param tag: The AprilTag to visualize.
    :param show_cone: Whether to show visibility cone.
    :param cone_length: How far the cone extends.
    :param highlight: Whether the tag is highlighted (detected).
    """
    tag: Any  # AprilTag
    show_cone: bool = True
    cone_length: float = 2.0  # meters
    highlight: bool = False
    quality: float = 0.0  # Detection quality if visible


@ObjectRendererRegistry.register(VisualAprilTag)
def render_visual_apriltag(
    obj: VisualAprilTag,
    canvas: Canvas,
    theme: Theme,
    display_level: DisplayLevel,
    renderer=None,
):
    """Render an AprilTag with its visibility cone."""
    tag = obj.tag
    
    # Tag position
    x = canvas.to_pixels(tag.position.x)
    y = canvas.to_pixels(tag.position.y)
    
    # Tag size in pixels
    tag_size = canvas.to_pixels(tag.size)
    
    # Calculate tag orientation
    # Calculate tag orientation
    from gamegine.utils.NCIM.Dimensions.angular import Radian
    heading_rad = float(tag.heading.to(Radian))
    normal = tag.get_normal_vector_2d()
    
    # Draw visibility cone first (behind tag)
    if obj.show_cone:
        cone_length_px = canvas.to_pixels(Meter(obj.cone_length))
        arc_half = float(tag.visible_arc.to(Radian)) / 2
        
        # Cone color based on quality
        if obj.highlight:
            cone_color = get_quality_color(obj.quality)
        else:
            cone_color = ArcadeColor(100, 100, 180, 40)
        
        # Draw filled arc for cone
        start_angle = math.degrees(heading_rad - arc_half)
        end_angle = math.degrees(heading_rad + arc_half)
        
        # Draw as filled pie slice using polygon
        num_segments = 20
        points = [(x, y)]  # Center point
        for i in range(num_segments + 1):
            angle = heading_rad - arc_half + (arc_half * 2) * i / num_segments
            px = x + math.cos(angle) * cone_length_px
            py = y + math.sin(angle) * cone_length_px
            points.append((px, py))
        
        arcade.draw_polygon_filled(points, cone_color)
        
        # Cone outline
        outline_color = with_alpha(cone_color, min(255, cone_color[3] + 60))
        arcade.draw_polygon_outline(points, outline_color, 1)
    
    # Draw tag as a rectangle perpendicular to heading
    # Tag is mounted flat against surface, so draw perpendicular to normal
    perp_angle = heading_rad + math.pi / 2
    half_size = tag_size / 2
    
    # Corner points
    p1 = (x + math.cos(perp_angle) * half_size, y + math.sin(perp_angle) * half_size)
    p2 = (x - math.cos(perp_angle) * half_size, y - math.sin(perp_angle) * half_size)
    
    # Tag color
    if obj.highlight:
        tag_color = ArcadeColor(40, 180, 80, 255)  # Green for detected
        outline_color = ArcadeColor(20, 140, 60, 255)
    else:
        tag_color = ArcadeColor(80, 80, 90, 255)  # Gray
        outline_color = ArcadeColor(50, 50, 60, 255)
    
    # Draw as thick line
    arcade.draw_line(p1[0], p1[1], p2[0], p2[1], tag_color, 6)
    arcade.draw_line(p1[0], p1[1], p2[0], p2[1], outline_color, 2)
    
    # Draw normal direction indicator
    arrow_length = tag_size * 0.6
    end_x = x + normal[0] * arrow_length
    end_y = y + normal[1] * arrow_length
    arcade.draw_line(x, y, end_x, end_y, outline_color, 2)
    
    # Tag ID label
    if display_level == DisplayLevel.DEBUG:
        arcade.draw_text(
            str(tag.id),
            x + 8, y + 8,
            ArcadeColor(50, 50, 60, 255),
            10,
            anchor_x="left",
        )


# =============================================================================
# Camera Sightline Visualization
# =============================================================================

@dataclass
class CameraSightlines:
    """Collection of sightlines from camera to tags.
    
    :param camera_pos: Camera (x, y) position in meters.
    :param sightlines: List of (tag, is_visible, quality) tuples.
    """
    camera_x: float  # meters
    camera_y: float
    camera_z: float
    sightlines: List[Tuple[Any, bool, float]]  # (tag, visible, quality)
    show_blocked: bool = True


@ObjectRendererRegistry.register(CameraSightlines)
def render_camera_sightlines(
    obj: CameraSightlines,
    canvas: Canvas,
    theme: Theme,
    display_level: DisplayLevel,
    renderer=None,
):
    """Render sightlines from camera to each tag."""
    cx = canvas.to_pixels(Meter(obj.camera_x))
    cy = canvas.to_pixels(Meter(obj.camera_y))
    
    for tag, is_visible, quality in obj.sightlines:
        tx = canvas.to_pixels(tag.position.x)
        ty = canvas.to_pixels(tag.position.y)
        
        if is_visible:
            # Green line with quality-based opacity
            alpha = int(80 + quality * 175)
            color = ArcadeColor(50, 180, 80, alpha)
            line_width = 2 + quality * 2
        elif obj.show_blocked:
            # Red dashed-ish line for blocked
            color = ArcadeColor(180, 60, 60, 100)
            line_width = 1
        else:
            continue
        
        arcade.draw_line(cx, cy, tx, ty, color, line_width)
    
    # Draw camera position indicator
    arcade.draw_circle_filled(cx, cy, 8, ArcadeColor(65, 105, 225, 220))
    arcade.draw_circle_outline(cx, cy, 8, ArcadeColor(45, 85, 200, 255), 2)


# =============================================================================
# Detection Quality Legend
# =============================================================================

@dataclass
class VisibilityLegend:
    """Legend showing quality color scale.
    
    :param x: X position in pixels.
    :param y: Y position in pixels.
    :param mode: 'quality' or 'count' to match heatmap mode.
    """
    x: float = 20
    y: float = 100
    mode: str = "quality"
    width: float = 20
    height: float = 150


@ObjectRendererRegistry.register(VisibilityLegend)
def render_visibility_legend(
    obj: VisibilityLegend,
    canvas: Canvas,
    theme: Theme,
    display_level: DisplayLevel,
    renderer=None,
):
    """Render a color legend for the heatmap."""
    x, y = obj.x, obj.y
    w, h = obj.width, obj.height
    
    # Draw gradient bar
    num_steps = 20
    step_height = h / num_steps
    
    for i in range(num_steps):
        t = i / (num_steps - 1)
        if obj.mode == "quality":
            color = get_quality_color(t)
        else:
            color = get_tag_count_color(int(t * 10), 10)
        
        # Draw as polygon
        ry = y + i * step_height + step_height / 2
        rh = step_height + 1
        x_left = x
        x_right = x + w
        y_bot = ry - rh / 2
        y_top = ry + rh / 2
        
        points = [
            (x_left, y_bot),
            (x_right, y_bot),
            (x_right, y_top),
            (x_left, y_top),
        ]
        arcade.draw_polygon_filled(points, color)
    
    # Draw border
    # Draw border
    border_points = [
        (x, y),
        (x + w, y),
        (x + w, y + h),
        (x, y + h),
    ]
    arcade.draw_polygon_outline(border_points, ArcadeColor(60, 60, 70, 200), 2)
    
    # Labels
    arcade.draw_text("Low", x + w + 5, y, ArcadeColor(60, 60, 70, 255), 10)
    arcade.draw_text("High", x + w + 5, y + h - 12, ArcadeColor(60, 60, 70, 255), 10)
    
    title = "Quality" if obj.mode == "quality" else "Tags"
    arcade.draw_text(title, x, y + h + 5, ArcadeColor(40, 40, 50, 255), 12, bold=True)


# =============================================================================
# Stats Overlay
# =============================================================================

@dataclass
class VisibilityStats:
    """Overlay showing visibility statistics.
    
    :param visible_count: Number of visible tags.
    :param total_count: Total number of tags.
    :param avg_quality: Average detection quality.
    :param best_tag_id: ID of best detected tag.
    :param position: (x, y) in pixels for overlay position.
    """
    visible_count: int
    total_count: int
    avg_quality: float
    best_tag_id: Optional[int] = None
    position: Tuple[float, float] = (20, 20)


@ObjectRendererRegistry.register(VisibilityStats)
def render_visibility_stats(
    obj: VisibilityStats,
    canvas: Canvas,
    theme: Theme,
    display_level: DisplayLevel,
    renderer=None,
):
    """Render visibility statistics overlay."""
    x, y = obj.position
    
    # Background panel
    panel_w, panel_h = 180, 80
    bg_points = [
        (x + 10, y + 10),
        (x + 10 + panel_w, y + 10),
        (x + 10 + panel_w, y + 10 + panel_h),
        (x + 10, y + 10 + panel_h),
    ]
    arcade.draw_polygon_filled(bg_points, ArcadeColor(255, 255, 255, 230))
    arcade.draw_polygon_outline(bg_points, ROYAL_BLUE, 2)
    
    # Title
    arcade.draw_text(
        "Visibility",
        x + 15, y + panel_h - 8,
        ArcadeColor(65, 105, 225, 255),
        14,
        bold=True,
    )
    
    # Stats
    text_color = ArcadeColor(50, 50, 60, 255)
    arcade.draw_text(
        f"Tags: {obj.visible_count}/{obj.total_count}",
        x + 15, y + panel_h - 30,
        text_color, 11,
    )
    arcade.draw_text(
        f"Avg Quality: {obj.avg_quality:.2f}",
        x + 15, y + panel_h - 48,
        text_color, 11,
    )
    if obj.best_tag_id is not None:
        arcade.draw_text(
            f"Best Tag: #{obj.best_tag_id}",
            x + 15, y + panel_h - 66,
            text_color, 11,
        )

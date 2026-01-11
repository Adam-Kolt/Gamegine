"""
Visibility Analysis Demo with Visualization
============================================

This example demonstrates the AprilTag visibility analysis system with
visual rendering including:
- Visibility heatmap across the field
- AprilTag visualization with visibility cones
- Camera sightlines showing detection quality
"""

import arcade
from gamegine.representation.apriltag import AprilTag, AprilTagFamily
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Rectangular
from gamegine.representation.camera import CameraMount, MountConstraints
from gamegine.representation.bounds import Point
from gamegine.reference.cameras import Limelight3, Limelight3G
from gamegine.analysis.visibility import VisibilityAnalyzer
from gamegine.analysis.camera_optimizer import (
    CameraPositionOptimizer,
    OptimizationObjective,
)
from gamegine.render.renderer import Renderer, DisplayLevel
from gamegine.render.handlers.visibility import (
    VisibilityHeatmap,
    VisualAprilTag,
    CameraSightlines,
    VisibilityStats,
    VisibilityLegend,
)
from gamegine.utils.NCIM.ncim import Inch, Degree, Meter, Feet

# ============================================================================
# Field Setup - Crescendo 2024 AprilTag Layout
# ============================================================================

# ============================================================================
# Field Setup
# ============================================================================

def create_crescendo_field():
    """Load the full Crescendo field with 3D obstacles and tags."""
    try:
        from examples.crescendo.crescendo import Crescendo, apriltag_list
        
        # Ensure tags are added if not already
        if not Crescendo.get_apriltags():
            Crescendo.add_apriltags(apriltag_list)
            
        print(f"Loaded Crescendo field with {len(list(Crescendo.get_obstacles()))} obstacles")
        return Crescendo
        
    except ImportError:
        print("Warning: Could not import examples.crescendo.crescendo. Falling back to simple field.")
        # Fallback to manual creation if example module not found
        from gamegine.representation.game import Game
        game = Game("FRC Crescendo 2024 (Simple)")
        game.set_field_size(Feet(54) + Inch(3.25), Feet(26) + Inch(11.25))
        
        # ... (rest of fallback code would go here, but for brevity we'll just return the simple one)
        # Re-adding the tags manually if import fails
        tags = [
            AprilTag(Inch(593.68), Inch(313.57), Inch(53.38), Degree(-120), 1, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(637.21), Inch(288.46), Inch(53.38), Degree(-120), 2, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(652.73), Inch(127.08), Inch(57.13), Degree(-180), 3, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(652.73), Inch(104.83), Inch(57.13), Degree(-180), 4, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(578.77), Inch(0.25), Inch(53.38), Degree(-270), 5, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(72.5), Inch(0.25), Inch(53.38), Degree(-270), 6, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(-1.5), Inch(104.83), Inch(57.13), Degree(0), 7, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(-1.5), Inch(127.08), Inch(57.13), Degree(0), 8, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(14.02), Inch(288.46), Inch(53.38), Degree(-60), 9, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(57.54), Inch(313.57), Inch(53.38), Degree(-60), 10, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(468.69), Inch(177.06), Inch(52.00), Degree(-300), 11, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(468.69), Inch(146.15), Inch(52.00), Degree(-60), 12, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(441.74), Inch(161.63), Inch(52.00), Degree(-180), 13, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(209.48), Inch(161.63), Inch(52.00), Degree(0), 14, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(182.73), Inch(146.15), Inch(52.00), Degree(-120), 15, AprilTagFamily.TAG_36h11),
            AprilTag(Inch(182.73), Inch(177.06), Inch(52.00), Degree(-240), 16, AprilTagFamily.TAG_36h11),
        ]
        game.add_apriltags(tags)
        game.add_obstacle(Rectangular("Stage Left", Inch(400), Inch(135), Inch(50), Inch(55)))
        game.add_obstacle(Rectangular("Stage Right", Inch(200), Inch(135), Inch(50), Inch(55)))
        return game


class VisibilityDemo(arcade.Window):
    """Interactive visibility analysis demo with visualization."""
    
    def __init__(self, game: Game, camera=Limelight3G):
        # Calculate window size based on field
        field_w = float(game.full_field_x().to(Meter))
        field_h = float(game.full_field_y().to(Meter))
        render_scale = 80  # pixels per meter
        
        super().__init__(
            int(field_w * render_scale) + 200,  # Extra space for legend
            int(field_h * render_scale),
            "AprilTag Visibility Analysis",
        )
        arcade.set_background_color((250, 250, 252, 255))
        
        self.game = game
        self.camera_spec = camera
        self.render_scale = render_scale
        
        # Analysis components
        self.analyzer = VisibilityAnalyzer(game, self.camera_spec)
        
        # Generate visibility grid
        print("Generating visibility heatmap...")
        self.grid_cells = self.analyzer.generate_visibility_grid(
            resolution=Feet(2),
            heading_samples=8,
        )
        print(f"  Generated {len(self.grid_cells)} cells")
        
        # Find optimal camera position
        print("Finding optimal camera position...")
        optimizer = CameraPositionOptimizer(
            game, self.camera_spec,
            mount_constraints=MountConstraints(min_height=Inch(12), max_height=Feet(3)),
        )
        top_placements = optimizer.optimize(num_results=1, num_candidates=30)
        if top_placements:
            self.best_mount = top_placements[0].mount
            print(f"  Best mount: {top_placements[0].total_score:.2f}")
        else:
            self.best_mount = None
        
        # Current robot position (interactive)
        self.robot_x = game.half_field_x()
        self.robot_y = game.half_field_y()
        self.robot_heading = Degree(0)
        
        # Create canvas for coordinate conversion
        from gamegine.render.canvas import Canvas, CanvasSettings
        self.canvas = Canvas(CanvasSettings(render_scale=render_scale))
        
        # UI state
        self.show_heatmap = True
        self.show_cones = True
        self.show_sightlines = True
        self.heatmap_mode = "quality"  # or "count"
        
        print("\nControls:")
        print("  Click: Move camera position")
        print("  Arrow keys: Rotate camera")
        print("  H: Toggle heatmap")
        print("  C: Toggle visibility cones")
        print("  S: Toggle sightlines")
        print("  M: Switch heatmap mode (quality/count)")
    
    def on_draw(self):
        self.clear()
        
        # Draw field border (as polygon since draw_rectangle_outline may not exist)
        field_w = self.canvas.to_pixels(self.game.full_field_x())
        field_h = self.canvas.to_pixels(self.game.full_field_y())
        border_points = [(0, 0), (field_w, 0), (field_w, field_h), (0, field_h)]
        arcade.draw_polygon_outline(border_points, (200, 200, 210, 255), 2)
        
        # Draw heatmap
        if self.show_heatmap:
            heatmap = VisibilityHeatmap(
                cells=self.grid_cells,
                mode=self.heatmap_mode,
                cell_size=float(Feet(2).to(Meter)),
            )
            from gamegine.render.style import Theme
            from gamegine.render.style import Theme
            render_visibility_heatmap(heatmap, self.canvas, Theme(), DisplayLevel.DEBUG)
        
        # Draw obstacles
        for obs in self.game.get_obstacles():
            if hasattr(obs.bounds, 'get_vertices'):
                vertices = obs.bounds.get_vertices()
                if len(vertices) >= 3:
                    pixel_points = [
                        (self.canvas.to_pixels(p[0]), self.canvas.to_pixels(p[1]))
                        for p in vertices
                    ]
                    arcade.draw_polygon_filled(pixel_points, (70, 75, 85, 255))
                    arcade.draw_polygon_outline(pixel_points, (50, 55, 65, 255), 2)
        
        # Analyze current position
        results = self.analyzer.analyze_from_position(
            self.robot_x, self.robot_y, self.robot_heading
        )
        visible_results = [r for r in results if r.is_visible]
        
        # Draw AprilTags
        from gamegine.render.style import Theme
        theme = Theme()
        for r in results:
            tag = r.tag
            visual = VisualAprilTag(
                tag=tag,
                show_cone=self.show_cones,
                cone_length=1.5,
                highlight=r.is_visible,
                quality=r.detection_quality,
            )
            render_visual_apriltag(visual, self.canvas, theme, DisplayLevel.DEBUG)
        
        # Draw sightlines
        if self.show_sightlines:
            sightline_data = [
                (r.tag, r.is_visible, r.detection_quality) for r in results
            ]
            sightlines = CameraSightlines(
                camera_x=float(self.robot_x.to(Meter)),
                camera_y=float(self.robot_y.to(Meter)),
                camera_z=float(Inch(24).to(Meter)),
                sightlines=sightline_data,
                show_blocked=True,
            )
            render_camera_sightlines(sightlines, self.canvas, theme, DisplayLevel.DEBUG)
        
        # Draw stats overlay
        avg_quality = sum(r.detection_quality for r in visible_results) / max(len(visible_results), 1)
        best_tag = max(visible_results, key=lambda r: r.detection_quality).tag.id if visible_results else None
        
        stats = VisibilityStats(
            visible_count=len(visible_results),
            total_count=len(results),
            avg_quality=avg_quality,
            best_tag_id=best_tag,
            position=(field_w + 15, field_h - 100),
        )
        render_visibility_stats(stats, self.canvas, theme, DisplayLevel.DEBUG)
        
        # Draw legend
        legend = VisibilityLegend(
            x=field_w + 20,
            y=50,
            mode=self.heatmap_mode,
        )
        render_visibility_legend(legend, self.canvas, theme, DisplayLevel.DEBUG)
        
        # Draw heading indicator
        cx = self.canvas.to_pixels(self.robot_x)
        cy = self.canvas.to_pixels(self.robot_y)
        import math
        from gamegine.utils.NCIM.Dimensions.angular import Radian
        heading_rad = float(self.robot_heading.to(Radian))
        arrow_len = 30
        ax = cx + math.cos(heading_rad) * arrow_len
        ay = cy + math.sin(heading_rad) * arrow_len
        arcade.draw_line(cx, cy, ax, ay, (65, 105, 225, 255), 3)
        
        # Draw controls hint
        arcade.draw_text(
            "Click: Move | Arrows: Rotate | H/C/S/M: Toggle",
            10, 10, (100, 100, 110, 255), 10,
        )
    

    
    def on_mouse_press(self, x, y, button, modifiers):
        # Convert click to field coordinates
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        self.robot_x = Meter(x / self.render_scale)
        self.robot_y = Meter(y / self.render_scale)
    
    def on_key_press(self, key, modifiers):
        if key == arcade.key.LEFT:
            self.robot_heading = Degree(float(self.robot_heading.to(Degree)) + 15)
        elif key == arcade.key.RIGHT:
            self.robot_heading = Degree(float(self.robot_heading.to(Degree)) - 15)
        elif key == arcade.key.H:
            self.show_heatmap = not self.show_heatmap
        elif key == arcade.key.C:
            self.show_cones = not self.show_cones
        elif key == arcade.key.S:
            self.show_sightlines = not self.show_sightlines
        elif key == arcade.key.M:
            self.heatmap_mode = "count" if self.heatmap_mode == "quality" else "quality"


# Import handlers for their registration side effects
from gamegine.render.handlers.visibility import (
    render_visibility_heatmap,
    render_visual_apriltag,
    render_camera_sightlines,
    render_visibility_stats,
    render_visibility_legend,
)


def main():
    print("=" * 60)
    print("AprilTag Visibility Analysis Demo (Interactive)")
    print("=" * 60)
    
    game = create_crescendo_field()
    print(f"\nField: {game.name}")
    print(f"AprilTags: {len(game.get_apriltags())}")
    print(f"Obstacles: {len(list(game.get_obstacles()))}")
    
    window = VisibilityDemo(game, Limelight3G)
    arcade.run()


if __name__ == "__main__":
    main()

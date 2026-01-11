"""Rebuilt FRC Game Definition

Based on official field layout:
- Field: 26ft 3.5in x 54ft 3.25in
- TRENCH: Low clearance zones (29" height limit) along side walls
- BUMP: 12" speed bumps across field width
- HUB: Central scoring tower at (0,0)
- TOWER: Endgame climb structure
"""

from gamegine.representation.game import Game
from gamegine.representation.obstacle import Obstacle, Obstacle3D, Rectangular
from gamegine.representation.bounds import Rectangle
from gamegine.representation.zone import TraversalZone
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch


# =============================================================================
# FIELD DIMENSIONS (Origin at center)
# =============================================================================

FIELD_WIDTH = Inch(26 * 12 + 3.5)   # 26ft 3.5in = 315.5in
FIELD_LENGTH = Inch(54 * 12 + 3.25) # 54ft 3.25in = 651.25in

# For Gamegine (origin at bottom-left), we need to offset
HALF_WIDTH = FIELD_WIDTH / 2   # ~157.75in
HALF_LENGTH = FIELD_LENGTH / 2 # ~325.6in


def create_rebuilt_game() -> Game:
    """Create the Rebuilt game with accurate field layout."""
    
    game = Game("Rebuilt 2026")
    
    # Field size (convert to gamegine origin at bottom-left)
    game.set_field_size(FIELD_LENGTH, FIELD_WIDTH)
    game.enable_field_border_obstacles()
    
    # =========================================================================
    # TRENCHES - Low clearance zones along walls (29" max height)
    # Robots >29" cannot traverse
    # =========================================================================
    
    trench_length = Inch(16 * 12)  # 16 ft
    trench_width = Inch(4 * 12 + 2)  # 4ft 2in
    trench_clearance = Inch(29)
    
    # Calculate trench positions (centered on long axis)
    trench_start_x = (FIELD_LENGTH - trench_length) / 2  # Centered
    
    # Left trench (bottom wall, Y=0)
    left_trench_boundary = Rectangle(
        trench_start_x,
        Inch(0),
        trench_length,
        trench_width,
    )
    left_trench = TraversalZone(
        name="left_trench",
        boundary=left_trench_boundary,
        speed_multiplier=0.8  # Slightly slower in trench
    )
    game.add_zone(left_trench)
    
    # Add height obstacle for left trench
    left_trench_bar = Obstacle3D(
        name="Left Trench Ceiling",
        bounds_2d=left_trench_boundary,
        z_min=trench_clearance,  # 29" clearance
        z_max=Feet(6),  # Bar extends up
    )
    game.add_obstacle(left_trench_bar)
    
    # Right trench (top wall)
    right_trench_boundary = Rectangle(
        trench_start_x,
        FIELD_WIDTH - trench_width,
        trench_length,
        trench_width,
    )
    right_trench = TraversalZone(
        name="right_trench",
        boundary=right_trench_boundary,
        speed_multiplier=0.8
    )
    game.add_zone(right_trench)
    
    right_trench_bar = Obstacle3D(
        name="Right Trench Ceiling",
        bounds_2d=right_trench_boundary,
        z_min=trench_clearance,
        z_max=Feet(6),
    )
    game.add_obstacle(right_trench_bar)
    
    # =========================================================================
    # BUMPS - Speed bumps across field width
    # 12" height, 45° ramps, significant slowdown
    # =========================================================================
    
    bump_base_width = Inch(18)  # 18" base
    bump_top_width = Inch(6)    # 6" top
    bump_height = Inch(12)      # 12" tall
    
    # Bumps at X = ~±8ft from center (about 96" from center)
    bump_offset = Inch(8 * 12)  # 8 feet
    
    # Left bump (closer to red side)
    left_bump_x = HALF_LENGTH - bump_offset - bump_base_width / 2
    left_bump_boundary = Rectangle(
        left_bump_x,
        Inch(0),
        bump_base_width,
        FIELD_WIDTH,
    )
    left_bump_zone = TraversalZone(
        name="left_bump",
        boundary=left_bump_boundary,
        speed_multiplier=0.4  # Very slow over bump
    )
    game.add_zone(left_bump_zone)
    
    # Right bump (closer to blue side)
    right_bump_x = HALF_LENGTH + bump_offset - bump_base_width / 2
    right_bump_boundary = Rectangle(
        right_bump_x,
        Inch(0),
        bump_base_width,
        FIELD_WIDTH,
    )
    right_bump_zone = TraversalZone(
        name="right_bump",
        boundary=right_bump_boundary,
        speed_multiplier=0.4
    )
    game.add_zone(right_bump_zone)
    
    # =========================================================================
    # HUB - Central scoring tower at field center
    # =========================================================================
    
    hub_diameter = Inches(4 * 12)  # 4ft diameter
    hub_radius = hub_diameter / 2
    
    # Hub as rectangular approximation (obstacle)
    hub_obstacle = Rectangular(
        name="Hub",
        x=HALF_LENGTH - hub_radius,
        y=HALF_WIDTH - hub_radius,
        width=hub_diameter,
        height=hub_diameter,
    )
    game.add_obstacle(hub_obstacle)
    
    # =========================================================================
    # TOWERS - Endgame climb structures
    # 12ft from center, 8ft x 8ft base
    # =========================================================================
    
    tower_offset = Inch(12 * 12)  # 12 feet from center
    tower_size = Inch(8 * 12)     # 8ft x 8ft base
    tower_half = tower_size / 2
    
    # Red alliance tower (left side)
    red_tower = Rectangular(
        name="Red Tower",
        x=HALF_LENGTH - tower_offset - tower_half,
        y=HALF_WIDTH - tower_half,
        width=tower_size,
        height=tower_size,
    )
    game.add_obstacle(red_tower)
    
    # Blue alliance tower (right side)
    blue_tower = Rectangular(
        name="Blue Tower",
        x=HALF_LENGTH + tower_offset - tower_half,
        y=HALF_WIDTH - tower_half,
        width=tower_size,
        height=tower_size,
    )
    game.add_obstacle(blue_tower)
    
    return game


# Typo fix helper
def Inches(val):
    return Inch(val)


# =============================================================================
# GAMEPIECE SPAWN LOCATIONS
# =============================================================================

# Neutral zone pieces (5 FUEL on centerline)
NEUTRAL_ZONE_PIECES = [
    (HALF_LENGTH, HALF_WIDTH - Inch(60)),     # Center-left
    (HALF_LENGTH, HALF_WIDTH - Inch(30)),     # Near center-left
    (HALF_LENGTH, HALF_WIDTH),                 # Dead center
    (HALF_LENGTH, HALF_WIDTH + Inch(30)),     # Near center-right
    (HALF_LENGTH, HALF_WIDTH + Inch(60)),     # Center-right
]

# Alliance-side starting pieces (near each tower)
RED_ALLIANCE_PIECES = [
    (HALF_LENGTH - Inch(96), HALF_WIDTH - Inch(48)),
    (HALF_LENGTH - Inch(96), HALF_WIDTH + Inch(48)),
]

BLUE_ALLIANCE_PIECES = [
    (HALF_LENGTH + Inch(96), HALF_WIDTH - Inch(48)),
    (HALF_LENGTH + Inch(96), HALF_WIDTH + Inch(48)),
]

ALL_PIECES = NEUTRAL_ZONE_PIECES + RED_ALLIANCE_PIECES + BLUE_ALLIANCE_PIECES


if __name__ == "__main__":
    game = create_rebuilt_game()
    print(f"Created game: {game.name}")
    print(f"Field size: {game.get_field_size()}")
    print(f"Obstacles: {len(list(game.get_obstacles()))}")
    print(f"Zones: {len(game.get_zones())}")
    
    print("\nObstacles:")
    for obs in game.get_obstacles():
        print(f"  - {obs.name}")
    
    print("\nZones:")
    for zone in game.get_zones():
        print(f"  - {zone.name}: {zone.speed_multiplier*100:.0f}% speed")
    
    print(f"\nGamepiece locations: {len(ALL_PIECES)}")

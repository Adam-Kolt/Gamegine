"""Rebuilt FRC Game Definition

Based on official field layout:
- Field: 26ft 3.5in x 54ft 3.25in
- TRENCH: Low clearance zones (29" height limit) along side walls
- BUMP: 12" speed bumps across field width
- HUB: Central scoring tower (one per alliance)
- TOWER: Endgame climb structure
- DEPOT: FUEL pickup zones
- NEUTRAL ZONE: Starting FUEL positions
"""

from typing import List, Tuple
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Obstacle, Obstacle3D, Rectangular
from gamegine.representation.bounds import Rectangle, SymmetricalX
from gamegine.representation.zone import TraversalZone
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch, SpatialMeasurement
from gamegine.utils.NCIM.Dimensions.angular import Degree

# Import scoring components
from examples.Rebuilt.scoring import Hub, Tower, Depot, Fuel, NeutralZone, AllianceZone
from examples.Rebuilt.shooting_locations import ShootingLocation, create_shooting_locations_for_alliance


# =============================================================================
# FIELD DIMENSIONS
# =============================================================================

FIELD_WIDTH = Inch(317.69)   # 26ft 5.69in
FIELD_LENGTH = Inch(651.22)  # 54ft 3.22in

# Half dimensions for centering
HALF_WIDTH = FIELD_WIDTH / 2
HALF_LENGTH = FIELD_LENGTH / 2

# Trench/obstacles
TRENCH_LINE = Inch(182.11)
TRENCH_1_LENGTH = Inch(50.59)
TRENCH_2_LENGTH = Inch(50.35)
TRENCH_HEIGHT = Inch(22.25)
BUMP_LENGTH = Inch(73)
BUMP_WIDTH = Inch(47)
PILLAR_WIDTH = Feet(1)

# Hub dimensions
HUB_SIZE = BUMP_WIDTH  # 47" square

# Tower position (near alliance wall)
TOWER_OFFSET_FROM_WALL = Feet(3)

# Depot position
DEPOT_OFFSET_FROM_WALL = Feet(2)
DEPOT_WIDTH = Feet(4)


# =============================================================================
# BLUE SIDE OBSTACLES (will be mirrored for red side)
# =============================================================================

blue_side_obstacles = [
    Rectangular("Pillar 1", TRENCH_LINE - BUMP_WIDTH/2, TRENCH_1_LENGTH, BUMP_WIDTH, PILLAR_WIDTH),
    Rectangular("Pillar 2", TRENCH_LINE - BUMP_WIDTH/2, FIELD_WIDTH - TRENCH_2_LENGTH - PILLAR_WIDTH, BUMP_WIDTH, PILLAR_WIDTH),
    Obstacle3D(
        name="Trench 1 Bar",
        bounds_2d=Rectangle(TRENCH_LINE - Inch(6)/2, Inch(0), Inch(6), TRENCH_1_LENGTH),
        z_min=TRENCH_HEIGHT,
        z_max=Feet(3),
    ),
    Obstacle3D(
        name="Trench 2 Bar",
        bounds_2d=Rectangle(TRENCH_LINE - Inch(6)/2, FIELD_WIDTH - TRENCH_2_LENGTH, Inch(6), TRENCH_2_LENGTH),
        z_min=TRENCH_HEIGHT,
        z_max=Feet(3),
    ),
    # Hub obstacle (for pathfinding collision)
    Rectangular("HUB Obstacle", TRENCH_LINE - BUMP_WIDTH/2, TRENCH_1_LENGTH + PILLAR_WIDTH + BUMP_LENGTH, BUMP_WIDTH, BUMP_WIDTH),
]


# =============================================================================
# BLUE SIDE ZONES (will be mirrored for red side)
# =============================================================================

blue_side_zones = [
    TraversalZone(
        "Bump 1",
        Rectangle(TRENCH_LINE - BUMP_WIDTH/2, TRENCH_1_LENGTH + PILLAR_WIDTH, BUMP_WIDTH, BUMP_LENGTH),
        0.005  # 0.5% of normal speed - realistic bump slowdown
    ),
    TraversalZone(
        "Bump 2",
        Rectangle(TRENCH_LINE - BUMP_WIDTH/2, FIELD_WIDTH - TRENCH_2_LENGTH - BUMP_LENGTH - PILLAR_WIDTH, BUMP_WIDTH, BUMP_LENGTH),
        0.005  # 0.5% of normal speed - realistic bump slowdown
    ),
]


# =============================================================================
# FUEL SPAWN POSITIONS
# =============================================================================

def generate_neutral_zone_positions() -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
    """Generate FUEL positions in the NEUTRAL ZONE.
    
    Creates a grid of ~360 FUEL in the center of the field.
    Neutral zone is ~206" wide x 72" deep, centered on field.
    """
    positions = []
    
    # Neutral zone bounds (centered on field)
    nz_width = Inch(206)
    nz_depth = Inch(72)
    nz_x_start = HALF_LENGTH - nz_width / 2
    nz_y_start = HALF_WIDTH - nz_depth / 2
    
    # Ball spacing (5.91" diameter, add some margin)
    ball_spacing = Inch(6)
    
    # Calculate grid
    cols = int(float(nz_width.to(Inch)) / float(ball_spacing.to(Inch)))
    rows = int(float(nz_depth.to(Inch)) / float(ball_spacing.to(Inch)))
    
    for row in range(rows):
        for col in range(cols):
            x = nz_x_start + ball_spacing * col + ball_spacing / 2
            y = nz_y_start + ball_spacing * row + ball_spacing / 2
            positions.append((x, y))
    
    return positions


def generate_depot_positions(alliance_x: SpatialMeasurement) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
    """Generate 24 FUEL positions for a DEPOT.
    
    DEPOTs are near the alliance walls, arranged in a 6x4 grid.
    """
    positions = []
    ball_spacing = Inch(9)
    
    # Offset from wall
    x_start = alliance_x + DEPOT_OFFSET_FROM_WALL
    y_start = HALF_WIDTH - (2 * ball_spacing)
    
    for row in range(4):
        for col in range(6):
            x = x_start + ball_spacing * col
            y = y_start + ball_spacing * row
            positions.append((x, y))
    
    return positions


# Pre-computed fuel positions
NEUTRAL_ZONE_PIECES = generate_neutral_zone_positions()
BLUE_DEPOT_PIECES = generate_depot_positions(Inch(0))
RED_DEPOT_PIECES = generate_depot_positions(FIELD_LENGTH - Feet(8))

# All pieces combined (504 total target)
ALL_PIECES = NEUTRAL_ZONE_PIECES + BLUE_DEPOT_PIECES + RED_DEPOT_PIECES


# =============================================================================
# GAME FACTORY
# =============================================================================

class RebuiltGame(Game):
    """Rebuilt game with RL observation support."""
    
    def get_observation_space_size(self) -> int:
        # 1. Neutral Zone Fuel (1)
        # 2. Blue Depot Fuel (1)
        # 3. Red Depot Fuel (1)
        # 4. Blue Alliance Zone Fuel (1)
        # 5. Red Alliance Zone Fuel (1)
        return 5

    def get_observation(self, game_state) -> List[float]:
        # Extract interactables
        interactables = game_state.get("interactables") if game_state else None
        if not interactables:
            return [0.0] * 5
            
        # Helper to get fuel count safely
        def get_fuel(name):
            obj = interactables.get(name)
            # Access underlying Value object if it exists
            return float(obj.fuel_available.get()) if obj and hasattr(obj, "fuel_available") else 0.0

        # Normalize against typical max capacities
        nz_fuel = get_fuel("Neutral Zone") / RebuiltMatchConfig.NEUTRAL_ZONE_FUEL
        blue_depot = get_fuel("Blue Depot") / RebuiltMatchConfig.DEPOT_FUEL
        red_depot = get_fuel("Red Depot") / RebuiltMatchConfig.DEPOT_FUEL
        blue_zone = get_fuel("Blue Alliance Zone") / 50.0 # Estimate max
        red_zone = get_fuel("Red Alliance Zone") / 50.0   # Estimate max
        
        return [nz_fuel, blue_depot, red_depot, blue_zone, red_zone]


def create_rebuilt_game() -> Game:
    """Create the REBUILT 2026 game with all field elements.
    
    Includes:
    - Hubs (scoring towers)
    - Towers (endgame climbing)
    - Depots (alliance-specific fuel pickup)
    - Neutral Zone (shared fuel storage, fills from Hub scores)
    - Alliance Zones (team fuel storage, fills from shuttling/misses)
    - Shooting Locations (6 per alliance, varying accuracy)
    """
    game = RebuiltGame("Rebuilt 2026")
    
    # Field size
    game.set_field_size(FIELD_LENGTH, FIELD_WIDTH)
    game.enable_field_border_obstacles()

    # Mirror blue side objects to create red side
    all_obstacles = SymmetricalX(blue_side_obstacles, FIELD_LENGTH / 2, "Red ", "Blue ")
    all_zones = SymmetricalX(blue_side_zones, FIELD_LENGTH / 2, "Red ", "Blue ")

    # Add obstacles and zones
    game.add_obstacles(all_obstacles)
    game.add_zones(all_zones)

    # --- Add Scoring Interactables ---
    
    # Hub positions (near center, offset toward each alliance)
    blue_hub_x = Meter(4.629)
    red_hub_x = FIELD_LENGTH - blue_hub_x
    hub_y = FIELD_WIDTH / 2
    
    blue_hub = Hub(
        center=(blue_hub_x, hub_y),
        navigation_point=(blue_hub_x - Meter(1), hub_y, Degree(180)),
        alliance=Alliance.BLUE,
        name="Blue Hub",
    )
    red_hub = Hub(
        center=(red_hub_x, hub_y),
        navigation_point=(red_hub_x + Meter(1), hub_y, Degree(0)),
        alliance=Alliance.RED,
        name="Red Hub",
    )
    
    game.add_interactable(blue_hub)
    game.add_interactable(red_hub)
    
    # Tower positions (near alliance walls)
    blue_tower = Tower(
        center=(TOWER_OFFSET_FROM_WALL, HALF_WIDTH),
        navigation_point=(TOWER_OFFSET_FROM_WALL + Feet(2), HALF_WIDTH, Degree(180)),
        alliance=Alliance.BLUE,
        name="Blue Tower",
    )
    red_tower = Tower(
        center=(FIELD_LENGTH - TOWER_OFFSET_FROM_WALL, HALF_WIDTH),
        navigation_point=(FIELD_LENGTH - TOWER_OFFSET_FROM_WALL - Feet(2), HALF_WIDTH, Degree(0)),
        alliance=Alliance.RED,
        name="Red Tower",
    )
    
    game.add_interactable(blue_tower)
    game.add_interactable(red_tower)
    
    # Depot positions (FUEL pickup - starts with 24 balls each)
    blue_depot = Depot(
        center=(Meter(0.692), Meter(6.024)),
        navigation_point=(Meter(0.692), Meter(6.024), Degree(0)),
        alliance=Alliance.BLUE,
        name="Blue Depot",
        initial_fuel=24,
    )
    red_depot = Depot(
        center=(Meter(FIELD_LENGTH - Meter(0.692)), Meter(6.024)),
        navigation_point=(Meter(FIELD_LENGTH - Meter(0.692)), Meter(6.024), Degree(180)),
        alliance=Alliance.RED,
        name="Red Depot",
        initial_fuel=24,
    )
    
    game.add_interactable(blue_depot)
    game.add_interactable(red_depot)
    
    # --- Neutral Zone (center of field, shared storage) ---
    neutral_zone = NeutralZone(
        center=(HALF_LENGTH, HALF_WIDTH),
        navigation_point=(HALF_LENGTH, HALF_WIDTH, Degree(0)),
        name="Neutral Zone",
        initial_fuel=0,  # Starts empty, fills from Hub scores
    )
    game.add_interactable(neutral_zone)
    
    # --- Alliance Zones (team storage, fills from shuttling/misses) ---
    blue_alliance_zone = AllianceZone(
        center=(Meter(2.5), HALF_WIDTH),
        navigation_point=(Meter(2.5), HALF_WIDTH, Degree(0)),
        alliance=Alliance.BLUE,
        name="Blue Alliance Zone",
        initial_fuel=0,
    )
    red_alliance_zone = AllianceZone(
        center=(FIELD_LENGTH - Meter(2.5), HALF_WIDTH),
        navigation_point=(FIELD_LENGTH - Meter(2.5), HALF_WIDTH, Degree(180)),
        alliance=Alliance.RED,
        name="Red Alliance Zone",
        initial_fuel=0,
    )
    
    game.add_interactable(blue_alliance_zone)
    game.add_interactable(red_alliance_zone)
    
    # --- Shooting Locations (6 per alliance) ---
    blue_shooting_locations = create_shooting_locations_for_alliance(
        Alliance.BLUE,
        (blue_hub_x, hub_y),
        FIELD_WIDTH,
    )
    for loc in blue_shooting_locations:
        game.add_interactable(loc)
    
    red_shooting_locations = create_shooting_locations_for_alliance(
        Alliance.RED,
        (red_hub_x, hub_y),
        FIELD_WIDTH,
    )
    for loc in red_shooting_locations:
        game.add_interactable(loc)

    return game


# =============================================================================
# MATCH CONFIGURATION
# =============================================================================

class RebuiltMatchConfig:
    """Match timing configuration for REBUILT."""
    AUTO_TIME = 20.0          # 20 seconds
    TELEOP_TIME = 140.0       # 2:20 total
    ENDGAME_TIME = 30.0       # Last 30 seconds of TELEOP
    TOTAL_TIME = 160.0        # 2:40 total
    
    # Fuel counts
    NEUTRAL_ZONE_FUEL = 360
    DEPOT_FUEL = 24           # Per depot
    OUTPOST_FUEL = 24         # Per outpost
    PRELOAD_FUEL = 8          # Per robot max

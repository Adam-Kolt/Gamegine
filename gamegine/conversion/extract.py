from typing import List, Dict, Optional, TypedDict, Union, Any
import json
import logging

logger = logging.getLogger(__name__)

# --- Intermediate Representation (IR) Schema ---

class DimensionIR(TypedDict):
    value: float
    unit: str  # "inch", "feet", "meter"

class PositionIR(TypedDict):
    x: DimensionIR
    y: DimensionIR
    z: Optional[DimensionIR]

class ObstacleIR(TypedDict):
    name: str
    shape: str  # "rectangle", "circle", "polygon"
    position: PositionIR
    dimensions: Dict[str, DimensionIR] # e.g. {"width": ..., "height": ..., "radius": ...}
    is_static: bool

class ScoringRuleIR(TypedDict):
    name: str # e.g. "Note in Speaker"
    description: str
    points_auto: int
    points_teleop: int
    trigger_condition: str # Description of physical trigger

class RobotConstraintsIR(TypedDict):
    max_weight: DimensionIR
    max_frame_perimeter: DimensionIR
    max_height: DimensionIR

class GameIR(TypedDict):
    game_name: str
    year: int
    field_size: Dict[str, DimensionIR] # width, length
    obstacles: List[ObstacleIR]
    scoring_rules: List[ScoringRuleIR]
    robot_constraints: RobotConstraintsIR

from gamegine.conversion.llm import LLMClient

# --- Extractor ---

class GameExtractor:
    """
    Orchestrates the extraction of structure data from raw text using an LLM.
    """

    def __init__(self, llm_client: Optional[LLMClient] = None):
        self.llm_client = llm_client or LLMClient()

    def extract_game_ir(self, ingestor: Any) -> GameIR:
        """
        Main entry point. Runs multiple extraction passes to build the GameIR.
        Accepts an 'ingestor' instance (PDFIngestor) to query specific text regions.
        """
        logger.info("Starting IR extraction...")
        
        # 1. High Level Metadata (usually first few pages)
        # We can just grab the first 5 pages for metadata
        metadata_text = ingestor.extract_full_text() # fallback or limit?
        # A better way: just get first 3 pages
        intro_text = ingestor.get_relevant_text(["game", "manual", "season"], top_n_pages=3)
        metadata = self._extract_metadata(intro_text)
        
        # 2. Field Layout (Look for "Arena", "Field", "Dimensions")
        field_text = ingestor.get_relevant_text(["arena", "field", "dimension", "layout", "zone"], top_n_pages=5)
        field_data = self._extract_field_data(field_text)
        
        # 3. Scoring (Look for "Score", "Points", "Rule", "Match")
        scoring_text = ingestor.get_relevant_text(["score", "points", "match play", "rule", "foul"], top_n_pages=5)
        scoring_data = self._extract_scoring_rules(scoring_text)

        # 4. Robot (Look for "Robot", "Weight", "Bumper", "Frame")
        robot_text = ingestor.get_relevant_text(["robot", "weight", "perimeter", "height", "frame"], top_n_pages=3)
        
        return {
            **metadata,
            **field_data,
            "scoring_rules": scoring_data,
            "robot_constraints": self._extract_robot_constraints(robot_text)
        }

    def _call_llm_stub(self, prompt: str, schema: str) -> Dict[str, Any]:
        """
        Uses the internal LLM client to generate data.
        """
        return self.llm_client.generate_structured(prompt, schema)

    def _extract_metadata(self, text: str) -> Dict[str, Any]:
        prompt = f"""
        Extract the Game Name and Year from the following text.
        Text:
        {text[:1000]}...
        """
        return self._call_llm_stub(prompt, "metadata")

    def _extract_field_data(self, text: str) -> Dict[str, Any]:
        prompt = f"""
        Extract the field dimensions and a list of all static field obstacles/structures.
        For each obstacle, provide its approximate center position and dimensions.
        """
        return self._call_llm_stub(prompt, "field")

    def _extract_scoring_rules(self, text: str) -> List[ScoringRuleIR]:
        prompt = f"""
        Extract all scoring rules from the text. For each rule, identify:
        - Name (e.g., 'Auto Leave', 'Speaker Note')
        - Points in Auto phase
        - Points in Teleop phase
        - Trigger condition (short description)
        
        Text:
        {text[:8000]}...
        """
        res = self.llm_client.generate_structured(prompt, "List[ScoringRuleIR]")
        return res if isinstance(res, list) else res.get("scoring_rules", [])

    def _extract_robot_constraints(self, text: str) -> RobotConstraintsIR:
        prompt = f"""
        Extract robot physical constraints:
        - Max weight (e.g. 125 lbs)
        - Max frame perimeter
        - Max height
        
        Text:
        {text[:5000]}...
        """
        return self.llm_client.generate_structured(prompt, "RobotConstraintsIR")

if __name__ == "__main__":
    extractor = GameExtractor()
    ir = extractor.extract_game_ir("Mock PDF Text Content")
    print(json.dumps(ir, indent=2))

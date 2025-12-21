import os
import unittest
import shutil
from pathlib import Path
from gamegine.conversion.ingest import PDFIngestor
from gamegine.conversion.extract import GameExtractor
from tests.create_mock_pdf import create_mock_manual

class TestConversionE2E(unittest.TestCase):
    def setUp(self):
        self.test_dir = Path("tests/temp_conversion")
        self.test_dir.mkdir(exist_ok=True)
        self.pdf_path = self.test_dir / "Mock_2025_Reeftide.pdf"
        create_mock_manual(str(self.pdf_path))

    def tearDown(self):
        if self.test_dir.exists():
            shutil.rmtree(self.test_dir)

    def test_e2e_pipeline(self):
        """
        Tests the full flow: PDF -> Text -> GameIR
        """
        # 1. Ingest
        ingestor = PDFIngestor(str(self.pdf_path))
        full_text = ingestor.extract_full_text()
        ingestor.close()
        
        self.assertIn("2025 REEFTIDE", full_text)
        self.assertIn("Structure located at the center", full_text)

        # 2. Extract
        api_key = os.getenv("GEMINI_API_KEY")
        extractor = GameExtractor()
        
        # Mock LLM for test stability if no valid key is present or if using dummy key
        if not api_key or api_key == "test_key":
            print("INFO: Using Mock LLM response for E2E test due to missing/dummy API key.")
            from unittest.mock import MagicMock
            mock_ir = {
                "game_name": "2025 REEFTIDE",
                "year": 2025,
                "field_size": {"width": {"value": 54, "unit": "feet"}, "length": {"value": 26, "unit": "feet"}},
                "obstacles": [],
                "scoring_rules": [
                    {"name": "Place CORAL L1", "description": "Place coral", "points_auto": 3, "points_teleop": 2, "trigger_condition": "Place on L1"},
                    {"name": "Leave Starting Zone", "description": "Leave zone", "points_auto": 2, "points_teleop": 0, "trigger_condition": "Cross line"}
                ],
                "robot_constraints": {"max_weight": {"value": 125, "unit": "lbs"}}
            }
            extractor.llm_client.generate_structured = MagicMock(return_value=mock_ir)
            # For specific calls (like field, scoring), we might need side_effect if extract calls multiple times
            # transform extract method to returns different parts.
            # But generate_structured is general. Let's make side_effect return parts based on prompt.
            def side_effect(prompt, schema):
                if "scoring" in prompt.lower():
                    return [{"name": "Place CORAL L1", "points_auto": 3, "points_teleop": 2, "trigger_condition": "Place on L1"}]
                if "constraints" in prompt.lower():
                    return {"max_weight": {"value": 125, "unit": "lbs"}}
                if "metadata" in prompt.lower():
                    return {"game_name": "2025 REEFTIDE", "year": 2025}
                return {} # field
            extractor.llm_client.generate_structured = MagicMock(side_effect=side_effect)

        # Re-open ingestor or reuse? Ingestor was closed in step 1.
        # We need an open ingestor for the optimized extractor.
        ingestor_for_extract = PDFIngestor(str(self.pdf_path))
        ir = extractor.extract_game_ir(ingestor_for_extract)
        ingestor_for_extract.close()
        
        # 3. Assertions
        print("\n--- Extracted Game IR ---")
        import json
        print(json.dumps(ir, indent=2))
        
        # Check Metadata
        self.assertIn("Reeftide", ir.get("game_name", "FAIL")) # LLM might normalize case
        self.assertTrue(ir["year"] == 2025 or str(ir["year"]) == "2025")
        
        # Check Rules
        rules = ir.get("scoring_rules", [])
        self.assertTrue(len(rules) > 0)
        found_coral = False
        for rule in rules:
            if "CORAL" in rule["name"].upper() or "CORAL" in rule.get("description", "").upper():
                found_coral = True
                break
        self.assertTrue(found_coral, "Did not find scoring rule related to CORAL")

        # Check Constraints
        constraints = ir.get("robot_constraints", {})
        self.assertEqual(constraints.get("max_weight", {}).get("value"), 125)

if __name__ == "__main__":
    unittest.main()

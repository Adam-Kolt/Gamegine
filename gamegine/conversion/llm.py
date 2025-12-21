import os
import json
import logging
from typing import Optional, Dict, Any
import google.generativeai as genai
from google.generativeai.types import HarmCategory, HarmBlockThreshold

logger = logging.getLogger(__name__)

class LLMClient:
    """
    Wrapper for Google Generative AI (Gemini) API.
    """
    def __init__(self, api_key: Optional[str] = None):
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            logger.warning("GEMINI_API_KEY not found. LLM calls will fail unless mocked.")
        else:
            genai.configure(api_key=self.api_key)
        
        self.model_name = "gemini-1.5-flash" # Use a fast, cost-effective model
        self.model = None

    def _get_model(self):
        if not self.model:
            self.model = genai.GenerativeModel(
                model_name=self.model_name,
                generation_config={"response_mime_type": "application/json"}
            )
        return self.model

    def generate_structured(self, prompt: str, schema_description: str = "") -> Dict[str, Any]:
        """
        Generates a JSON response based on the prompt.
        """
        if not self.api_key:
            raise ValueError("API Key not configured.")

        full_prompt = f"""
        You are an expert FRC Game Manual Parser.
        Task: {prompt}
        
        Ensure the output is valid JSON strictly following this schema structure:
        {schema_description}
        
        Do not include markdown formatting (```json). Just return the raw JSON string.
        """

        try:
            model = self._get_model()
            response = model.generate_content(
                full_prompt,
                safety_settings={
                    HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: HarmBlockThreshold.BLOCK_NONE,
                    HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_NONE,
                    HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_NONE,
                    HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_NONE,
                }
            )
            
            # Clean response if necessary (Gemini usually respects JSON mode well, but sometimes adds markdown)
            text = response.text.strip()
            if text.startswith("```json"):
                text = text[7:]
            if text.endswith("```"):
                text = text[:-3]
            
            return json.loads(text)
        
        except Exception as e:
            logger.error(f"LLM Generation failed: {e}")
            # In a real app, we might retry or return a partial error object
            raise

if __name__ == "__main__":
    # Test stub
    if os.getenv("GEMINI_API_KEY"):
        client = LLMClient()
        res = client.generate_structured("Extract info about a cat", "name: str, color: str")
        print(res)
    else:
        print("Skipping test: No API Key")

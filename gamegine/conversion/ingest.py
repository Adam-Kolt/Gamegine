import fitz  # pymupdf
from typing import List, Dict, Optional
import logging

logger = logging.getLogger(__name__)

class PDFIngestor:
    """
    Handles the ingestion of FRC Game Manual PDFs.
    Extracts text and structure for downstream processing.
    """

    def __init__(self, pdf_path: str):
        self.pdf_path = pdf_path
        self.doc = None
        self._load_pdf()

    def _load_pdf(self):
        try:
            self.doc = fitz.open(self.pdf_path)
            logger.info(f"Successfully loaded PDF: {self.pdf_path} ({len(self.doc)} pages)")
        except Exception as e:
            logger.error(f"Failed to load PDF: {e}")
            raise

    def extract_full_text(self) -> str:
        """Concatenates text from all pages."""
        full_text = []
        for page_num, page in enumerate(self.doc):
            text = page.get_text()
            full_text.append(f"--- Page {page_num + 1} ---\n{text}")
        return "\n".join(full_text)

    def extract_dict_by_page(self) -> Dict[int, str]:
        """Returns a dictionary mapping page numbers (1-indexed) to text content."""
        pages = {}
        for i, page in enumerate(self.doc):
            pages[i + 1] = page.get_text()
        return pages

    def get_relevant_text(self, keywords: List[str], top_n_pages: int = 3) -> str:
        """
        Returns text from the top N pages that contain the most occurrences of the keywords.
        Acts as a 'traditional algorithmic' pre-filter.
        """
        page_scores = []
        for i, page in enumerate(self.doc):
            text = page.get_text()
            score = 0
            for kw in keywords:
                score += text.lower().count(kw.lower())
            page_scores.append((score, i, text))
        
        # Sort by score descending
        page_scores.sort(key=lambda x: x[0], reverse=True)
        
        # Take top N
        selected_pages = page_scores[:top_n_pages]
        # Sort back by page number to keep logical flow
        selected_pages.sort(key=lambda x: x[1])
        
        return "\n".join([f"--- Page {p[1] + 1} ---\n{p[2]}" for p in selected_pages])

    def search_term(self, term: str) -> List[tuple]:
        """
        Searches for a term in the PDF.
        Returns a list of (page_num, rect) tuples.
        """
        results = []
        for i, page in enumerate(self.doc):
            matches = page.search_for(term)
            if matches:
                for match in matches:
                    results.append((i + 1, match))
        return results

    def close(self):
        if self.doc:
            self.doc.close()

if __name__ == "__main__":
    # fast verify
    import sys
    if len(sys.argv) > 1:
        ingestor = PDFIngestor(sys.argv[1])
        print(ingestor.extract_full_text()[:500])
        ingestor.close()

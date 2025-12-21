from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch

def create_mock_manual(filename="Mock_2025_Reefscape_Manual.pdf"):
    c = canvas.Canvas(filename, pagesize=letter)
    width, height = letter

    # --- Page 1: Title ---
    c.setFont("Helvetica-Bold", 24)
    c.drawCentredString(width / 2, height - 2 * inch, "FIRST Robotics Competition")
    c.setFont("Helvetica-Bold", 36)
    c.drawCentredString(width / 2, height - 3 * inch, "2025 REEFTIDE") # Changed name to avoid confusion with real future if any
    c.setFont("Helvetica", 12)
    c.drawCentredString(width / 2, height - 4 * inch, "Game Manual")
    c.showPage()

    # --- Page 2: Arena ---
    c.setFont("Helvetica-Bold", 18)
    c.drawString(1 * inch, height - 1 * inch, "5. ARENA")
    
    c.setFont("Helvetica", 12)
    text_object = c.beginText(1 * inch, height - 1.5 * inch)
    text_object.setFont("Helvetica", 12)
    
    arena_text = """
    The ARENA is a 26 ft. 3 in. by 54 ft. 3.25 in. carpeted area.
    
    5.1 Field Objects
    There is a central REEF structure located at the center of the field.
    The REEF is a hexagonal prism, 4 ft. wide (flat to flat) and 6 ft. tall.
    
    There are two CORAL STATIONS located on the SOURCE wall.
    Each CORAL STATION is a rectangular zone 3 ft wide by 2 ft deep.
    """
    for line in arena_text.split('\n'):
        text_object.textLine(line.strip())
    c.drawText(text_object)
    c.showPage()

    # --- Page 3: Game Rules & Scoring ---
    c.setFont("Helvetica-Bold", 18)
    c.drawString(1 * inch, height - 1 * inch, "6. MATCH PLAY")
    
    text_object = c.beginText(1 * inch, height - 1.5 * inch)
    text_object.setFont("Helvetica", 12)
    
    rules_text = """
    6.1 Scoring
    Points are awarded for placing CORAL on the REEF.
    
    SCORING TABLE:
    --------------------------------------------------
    Action              | Auto Points | Teleop Points
    --------------------------------------------------
    Leave Starting Zone | 2           | 0
    Place CORAL L1      | 3           | 2
    Place CORAL L2      | 4           | 3
    Place CORAL L3      | 6           | 5
    --------------------------------------------------
    
    6.4 Robot Construction Rules (R101)
    - Maximum Weight: 125 lbs (excluding battery and bumpers).
    - Frame Perimeter: May not exceed 120 in.
    - Maximum Height: 4 ft. starting configuration.
    """
    for line in rules_text.split('\n'):
        text_object.textLine(line.strip())
    c.drawText(text_object)
    c.showPage()

    c.save()
    print(f"Generated mock manual: {filename}")

if __name__ == "__main__":
    create_mock_manual()

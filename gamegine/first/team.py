class TeamNumber(int):
    def __new__(cls, value):
        if value < 0 or value > 99999:
            raise ValueError("Team number must be between 0 and 9999")
        return super(TeamNumber, cls).__new__(cls, value)

    def __init__(self, value):
        self.value = value

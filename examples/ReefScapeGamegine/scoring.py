from typing import List, Tuple
from gamegine.simulation.state import (
    StateSpace,
    ValueDecrease,
    ValueIncrease,
    ValueChange,
)
from gamegine.representation.interactable import (
    RobotInteractable,
    InteractionOption,
    RobotInteractionConfig,
)
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.representation.gamepiece import Gamepiece
from gamepieces import Algae, Coral
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.ncim import (
    Inch,
    Degree,
    SpatialMeasurement,
    AngularMeasurement,
)
from gamegine.representation.bounds import Point


class L1Row(StateSpace):
    MAX_AMOUNT = 3

    def __init__(self):
        super().__init__()
        self.setValue("row_score", 0)
        self.setValue("A", 0)
        self.setValue("B", 0)
        self.setValue("C", 0)
        self.setValue("D", 0)
        self.setValue("E", 0)
        self.setValue("F", 0)
        self.setValue("G", 0)
        self.setValue("H", 0)
        self.setValue("I", 0)
        self.setValue("J", 0)
        self.setValue("K", 0)
        self.setValue("L", 0)

    @property
    def row_score(self):
        return self.getValue("row_score")

    @row_score.setter
    def row_score(self, value):
        self.setValue("row_score", value)

    def get_column(self, column):
        return self.getValue(column)

    def get_binary_list(self):
        return [
            self.getValue("A").get(),
            self.getValue("B").get(),
            self.getValue("C").get(),
            self.getValue("D").get(),
            self.getValue("E").get(),
            self.getValue("F").get(),
            self.getValue("G").get(),
            self.getValue("H").get(),
            self.getValue("I").get(),
            self.getValue("J").get(),
            self.getValue("K").get(),
            self.getValue("L").get(),
        ]


class ReefRow(StateSpace):
    def __init__(self):
        super().__init__()
        self.setValue("row_score", 0)
        self.setValue("A", False)
        self.setValue("B", False)
        self.setValue("C", False)
        self.setValue("D", False)
        self.setValue("E", False)
        self.setValue("F", False)
        self.setValue("G", False)
        self.setValue("H", False)
        self.setValue("I", False)
        self.setValue("J", False)
        self.setValue("K", False)
        self.setValue("L", False)

    @property
    def row_score(self):
        return self.getValue("row_score")

    @row_score.setter
    def row_score(self, value):
        self.setValue("row_score", value)

    def get_column(self, column):
        return self.getValue(column)

    def set_column(self, column, value):
        self.setValue(column, value)

    def get_binary_list(self):
        return [
            self.getValue("A").get(),
            self.getValue("B").get(),
            self.getValue("C").get(),
            self.getValue("D").get(),
            self.getValue("E").get(),
            self.getValue("F").get(),
            self.getValue("G").get(),
            self.getValue("H").get(),
            self.getValue("I").get(),
            self.getValue("J").get(),
            self.getValue("K").get(),
            self.getValue("L").get(),
        ]


class ReefState(StateSpace):
    def __init__(self):
        super().__init__()
        self.setValue("reef_score", 0)
        self.registerSpace("l1", L1Row())
        self.registerSpace("l2", ReefRow())
        self.registerSpace("l3", ReefRow())
        self.registerSpace("l4", ReefRow())
        self.registerSpace("algae", ReefRow())

    @property
    def reef_score(self):
        return self.getValue("reef_score")

    @reef_score.setter
    def reef_score(self, value):
        self.setValue("reef_score", value)

    @property
    def l1(self):
        return self.get("l1").getValue("row_score")

    @property
    def l2(self):
        return self.get("l2").getValue("row_score")

    @property
    def l3(self):
        return self.get("l3").getValue("row_score")

    @property
    def l4(self):
        return self.get("l4").getValue("row_score")

    def is_open(self, level, column):
        row: ReefRow = self.get(level)
        if level == "l1":
            l1row: L1Row = self.get("l1")

            return l1row.get_column(column).get() < L1Row.MAX_AMOUNT

        if level == "l2" or level == "l3":
            algae: ReefRow = self.get("algae")
            return not algae.get_column(column).get() and not row.get_column(column)

        return not row.get_column(column).get()

    def open_scoring_positions(self, level) -> int:
        """Returns the number of open scoring positions for a given level."""
        columns = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
        count = 0
        for column in columns:
            if self.is_open(level, column):
                count += 1
        return count

    def algae_count(self) -> int:
        """Returns the number of algae positions still present on the reef."""
        algae: ReefRow = self.get("algae")
        algae_pairs = ["A", "C", "E", "G", "I", "K"]  # Count only first of each pair
        count = 0
        for col in algae_pairs:
            if algae.get_column(col).get():
                count += 1
        return count


def reefNotFullCondition(level, column):
    def condition(
        interactableState: ReefState, robotState: RobotState, gameState: GameState
    ):
        return interactableState.is_open(level, column)

    return condition


def gamepieceCondition(gamepiece: Gamepiece):
    def condition(
        interactableState: StateSpace, robotState: RobotState, gameState: GameState
    ):
        return robotState.gamepieces.get().get(gamepiece, 0) > 0

    return condition


def maxGamepieceCondition(gamepiece: Gamepiece, max):
    def condition(
        interactableState: StateSpace, robotState: RobotState, gameState: GameState
    ):
        return robotState.gamepieces.get().get(gamepiece, 0) <= max

    return condition


def MultiCondition(*conditions):
    def condition(
        interactableState: StateSpace, robotState: RobotState, gameState: GameState
    ):
        return all(c(interactableState, robotState, gameState) for c in conditions)

    return condition


def is_auto(gameState: GameState) -> bool:
    return gameState.current_time.get() <= gameState.auto_time.get()


class Reef(RobotInteractable):
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[
            SpatialMeasurement, SpatialMeasurement, AngularMeasurement
        ],
        name="",
    ):
        super().__init__(Point(*center, Inch(0)), name, navigation_point)

    @staticmethod
    def initializeInteractableState() -> ReefState:
        return ReefState()

    @staticmethod
    def __generate_scoring_function(column, level, points_auto, points_teleop):
        def scoring_function(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            changes = []
            robotState.gamepieces.get()[Coral] -= 1

            row: ReefRow = interactableState.get(level)

            if is_auto(gameState):
                points = points_auto
            else:
                points = points_teleop

            changes.extend(
                [
                    ValueIncrease(interactableState.reef_score, points),
                    ValueIncrease(gameState.score, points),
                    ValueIncrease(row.row_score, points),
                    ValueChange(row.get_column(column), True),
                ]
            )

            return changes

        return scoring_function

    @staticmethod
    def __generate_l1_scoring_function(column):
        def scoring_function(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            changes = []
            robotState.gamepieces.get()[Coral] -= 1

            row: L1Row = interactableState.get("l1")

            changes.extend(
                [
                    ValueIncrease(interactableState.reef_score, 2),
                    ValueIncrease(gameState.score, 2),
                    ValueIncrease(row.row_score, 2),
                    ValueIncrease(row.get_column(column), 1),
                ]
            )

            return changes

        return scoring_function

    @staticmethod
    def __agae_left_condition(column):
        def condition(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            algae: ReefRow = interactableState.get("algae")
            return algae.get_column(column).get()

        return condition

    @staticmethod
    def __generate_dislodge_algae_function(sides: List[str], pickup: bool = False):
        def dislodge_algae(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            if pickup:
                robotState.gamepieces[Algae] = robotState.gamepieces.get(Algae, 0) + 1

            algae: ReefRow = interactableState.get("algae")
            return [
                ValueChange(algae.get_column(sides[0]), False),
                ValueChange(algae.get_column(sides[1]), False),
            ]

        return dislodge_algae

    @staticmethod
    def __any_level_open_condition(level):
        """Condition that checks if any position in the level is open."""
        def condition(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            return interactableState.open_scoring_positions(level) > 0
        return condition

    @staticmethod
    def __generate_generic_level_scoring_function(level, points_auto, points_teleop):
        """Scoring function that picks the first available column for a level."""
        columns = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
        
        def scoring_function(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            changes = []
            robotState.gamepieces.get()[Coral] -= 1
            
            # Find first open column
            chosen_column = None
            for col in columns:
                if interactableState.is_open(level, col):
                    chosen_column = col
                    break
            
            if chosen_column is None:
                return changes  # No open positions (shouldn't happen if condition passed)
            
            row = interactableState.get(level)
            
            if is_auto(gameState):
                points = points_auto
            else:
                points = points_teleop
            
            if level == "l1":
                changes.extend([
                    ValueIncrease(interactableState.reef_score, points),
                    ValueIncrease(gameState.score, points),
                    ValueIncrease(row.row_score, points),
                    ValueIncrease(row.get_column(chosen_column), 1),
                ])
            else:
                changes.extend([
                    ValueIncrease(interactableState.reef_score, points),
                    ValueIncrease(gameState.score, points),
                    ValueIncrease(row.row_score, points),
                    ValueChange(row.get_column(chosen_column), True),
                ])

            return changes

        return scoring_function

    @staticmethod
    def __any_algae_present_condition():
        """Condition that checks if any algae is present on the reef."""
        algae_pairs = [["A", "B"], ["C", "D"], ["E", "F"], ["G", "H"], ["I", "J"], ["K", "L"]]
        def condition(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            algae: ReefRow = interactableState.get("algae")
            for pair in algae_pairs:
                if algae.get_column(pair[0]).get():
                    return True
            return False
        return condition

    @staticmethod
    def __generate_generic_algae_dislodge_function():
        """Dislodge first found algae pair."""
        algae_pairs = [["A", "B"], ["C", "D"], ["E", "F"], ["G", "H"], ["I", "J"], ["K", "L"]]
        def dislodge_algae(
            interactableState: ReefState, robotState: RobotState, gameState: GameState
        ):
            algae: ReefRow = interactableState.get("algae")
            for pair in algae_pairs:
                if algae.get_column(pair[0]).get():
                    return [
                        ValueChange(algae.get_column(pair[0]), False),
                        ValueChange(algae.get_column(pair[1]), False),
                    ]
            return []
        return dislodge_algae

    @staticmethod
    def get_interactions():
        scoring_columns = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
        options = []

        for scoring_column in scoring_columns:
            options.append(
                InteractionOption(
                    f"l1_{scoring_column}",
                    f"Scores in the L1 Reef Column {scoring_column}",
                    MultiCondition(
                        gamepieceCondition(Coral),
                        reefNotFullCondition("l1", scoring_column),
                    ),
                    Reef.__generate_l1_scoring_function(scoring_column),
                )
            )

            options.append(
                InteractionOption(
                    f"l2_{scoring_column}",
                    f"Scores in the L2 Reef Column {scoring_column}",
                    MultiCondition(
                        gamepieceCondition(Coral),
                        reefNotFullCondition("l2", scoring_column),
                    ),
                    Reef.__generate_scoring_function(scoring_column, "l2", 4, 3),
                )
            )

            options.append(
                InteractionOption(
                    f"l3_{scoring_column}",
                    f"Scores in the L3 Reef Column {scoring_column}",
                    MultiCondition(
                        gamepieceCondition(Coral),
                        reefNotFullCondition("l3", scoring_column),
                    ),
                    Reef.__generate_scoring_function(scoring_column, "l3", 6, 4),
                )
            )

            options.append(
                InteractionOption(
                    f"l4_{scoring_column}",
                    f"Scores in the L4 Reef Column {scoring_column}",
                    MultiCondition(
                        gamepieceCondition(Coral),
                        reefNotFullCondition("l4", scoring_column),
                    ),
                    Reef.__generate_scoring_function(scoring_column, "l4", 7, 5),
                )
            )

        algae_sides = [
            ["A", "B"],
            ["C", "D"],
            ["E", "F"],
            ["G", "H"],
            ["I", "J"],
            ["K", "L"],
        ]

        for sides in algae_sides:
            options.append(
                InteractionOption(
                    f"dislodge_{sides[0]}{sides[1]}",
                    f"Dislodges the Algae from the {sides[0]}{sides[1]} Reef",
                    Reef.__agae_left_condition(sides[0]),
                    Reef.__generate_dislodge_algae_function(sides),
                )
            )

            options.append(
                InteractionOption(
                    f"pickup_{sides[0]}{sides[1]}",
                    f"Pickup the Algae from the {sides[0]}{sides[1]} Reef",
                    MultiCondition(
                        maxGamepieceCondition(Algae, 1),
                        Reef.__agae_left_condition(sides[0]),
                    ),
                    Reef.__generate_dislodge_algae_function(sides, True),
                )
            )

        # Generic level interactions (pick first available column)
        options.append(
            InteractionOption(
                "l1",
                "Scores in the first available L1 position",
                MultiCondition(
                    gamepieceCondition(Coral),
                    Reef.__any_level_open_condition("l1"),
                ),
                Reef.__generate_generic_level_scoring_function("l1", 2, 2),
            )
        )
        
        options.append(
            InteractionOption(
                "l2",
                "Scores in the first available L2 position",
                MultiCondition(
                    gamepieceCondition(Coral),
                    Reef.__any_level_open_condition("l2"),
                ),
                Reef.__generate_generic_level_scoring_function("l2", 4, 3),
            )
        )
        
        options.append(
            InteractionOption(
                "l3",
                "Scores in the first available L3 position",
                MultiCondition(
                    gamepieceCondition(Coral),
                    Reef.__any_level_open_condition("l3"),
                ),
                Reef.__generate_generic_level_scoring_function("l3", 6, 4),
            )
        )
        
        options.append(
            InteractionOption(
                "l4",
                "Scores in the first available L4 position",
                MultiCondition(
                    gamepieceCondition(Coral),
                    Reef.__any_level_open_condition("l4"),
                ),
                Reef.__generate_generic_level_scoring_function("l4", 7, 5),
            )
        )

        # Generic algae dislodge interactions
        options.append(
            InteractionOption(
                "l2_algae",
                "Dislodges algae blocking L2",
                Reef.__any_algae_present_condition(),
                Reef.__generate_generic_algae_dislodge_function(),
            )
        )
        
        options.append(
            InteractionOption(
                "l3_algae",
                "Dislodges algae blocking L3",
                Reef.__any_algae_present_condition(),
                Reef.__generate_generic_algae_dislodge_function(),
            )
        )

        return options


class ProcessorState(StateSpace):
    def __init__(self):
        super().__init__()
        self.setValue("processor_score", 0)
        self.setValue("processor", 0)

    @property
    def processor_score(self):
        return self.getValue("processor_score")

    @processor_score.setter
    def processor_score(self, value):
        self.setValue("processor_score", value)

    @property
    def processor(self):
        return self.getValue("processor")

    @processor.setter
    def processor(self, value):
        self.setValue("processor", value)


class Processor(RobotInteractable):
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[
            SpatialMeasurement, SpatialMeasurement, AngularMeasurement
        ],
        name="",
    ):
        super().__init__(Point(*center, Inch(0)), name, navigation_point)

    @staticmethod
    def initializeInteractableState() -> ProcessorState:
        return ProcessorState()

    @staticmethod
    def scoring_function(
        interactableState: ProcessorState, robotState: RobotState, gameState: GameState
    ):
        changes = []

        points = 6

        changes.extend(
            [
                ValueIncrease(interactableState.processor_score, points),
                ValueIncrease(gameState.score, points),
                ValueIncrease(interactableState.processor, 1),
            ]
        )

        return changes

    @staticmethod
    def get_interactions():
        return [
            InteractionOption(
                "processor",
                "Scores in the Processor",
                gamepieceCondition(Algae),
                Processor.scoring_function,
            ),
        ]


class BargeState(StateSpace):
    def __init__(self):
        super().__init__()
        self.setValue("cage_1", False)
        self.setValue("cage_2", False)
        self.setValue("cage_3", False)

    @property
    def cage_1(self):
        return self.getValue("cage_1")

    @cage_1.setter
    def cage_1(self, value):
        self.setValue("cage_1", value)

    @property
    def cage_2(self):
        return self.getValue("cage_2")

    @cage_2.setter
    def cage_2(self, value):
        self.setValue("cage_2", value)

    @property
    def cage_3(self):
        return self.getValue("cage_3")

    @cage_3.setter
    def cage_3(self, value):
        self.setValue("cage_3", value)


class Barge(RobotInteractable):
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[
            SpatialMeasurement, SpatialMeasurement, AngularMeasurement
        ],
        name="",
    ):
        super().__init__(Point(*center, Inch(0)), name, navigation_point)

    @staticmethod
    def initializeInteractableState() -> BargeState:
        return BargeState()

    @staticmethod
    def __barge_empty_condition(cage):
        def condition(
            interactableState: BargeState, robotState: RobotState, gameState: GameState
        ):
            return not interactableState.getValue(cage)

        return condition

    @staticmethod
    def generate_scoring_function(cage: str, points: int):
        def scoring_function(
            interactableState: BargeState, robotState: RobotState, gameState: GameState
        ):
            robotState.setValue("gameover", True)
            return [
                ValueIncrease(gameState.score, points),
                ValueChange(interactableState.getValue(cage), True),
            ]

    @staticmethod
    def get_interactions():
        return [
            InteractionOption(
                "Deep1",
                "Robot climbs onto the barge and scores",
                Barge.__barge_empty_condition("cage_1"),
                Barge.generate_scoring_function("cage_1", 12),
            ),
            InteractionOption(
                "Deep2",
                "Robot climbs onto the barge and scores",
                Barge.__barge_empty_condition("cage_2"),
                Barge.generate_scoring_function("cage_2", 12),
            ),
            InteractionOption(
                "Deep3",
                "Robot climbs onto the barge and scores",
                Barge.__barge_empty_condition("cage_3"),
                Barge.generate_scoring_function("cage_3", 12),
            ),
            InteractionOption(
                "Shallow1",
                "Robot climbs onto the barge and scores",
                Barge.__barge_empty_condition("cage_1"),
                Barge.generate_scoring_function("cage_1", 6),
            ),
            InteractionOption(
                "Shallow2",
                "Robot climbs onto the barge and scores",
                Barge.__barge_empty_condition("cage_2"),
                Barge.generate_scoring_function("cage_2", 6),
            ),
            InteractionOption(
                "Shallow3",
                "Robot climbs onto the barge and scores",
                Barge.__barge_empty_condition("cage_3"),
                Barge.generate_scoring_function("cage_3", 6),
            ),
        ]


class NetState(StateSpace):
    def __init__(self):
        super().__init__()
        self.setValue("ball_amount", 0)

    @property
    def ball_amount(self):
        return self.getValue("ball_amount")

    @ball_amount.setter
    def ball_amount(self, value):
        self.setValue("ball_amount", value)


class Net(RobotInteractable):
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[
            SpatialMeasurement, SpatialMeasurement, AngularMeasurement
        ],
        name="",
    ):
        super().__init__(Point(*center, Inch(0)), name, navigation_point)

    @staticmethod
    def initializeInteractableState() -> NetState:
        return NetState()

    @staticmethod
    def scoring_function(
        interactableState: NetState, robotState: RobotState, gameState: GameState
    ):
        changes = [ValueIncrease(interactableState.ball_amount, 1)]

        if robotState.alliance.get() == Alliance.BLUE:
            changes.append(ValueDecrease(gameState.score, 4))
        else:
            changes.append(ValueIncrease(gameState.score, 4))

        return changes

    @staticmethod
    def get_interactions():
        return [
            InteractionOption(
                "Net",
                "Scores in the Net",
                gamepieceCondition(Algae),
                Net.scoring_function,
            ),
        ]

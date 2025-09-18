from enum import Enum, auto
from dataclasses import dataclass

from robot.boards import ServoBoard, IOBoard, PumpBoard


class RobotFace(Enum):
    FRONT = auto()
    BACK = auto()


class RobotSide(Enum):
    LEFT = auto()
    RIGHT = auto()


# SERVOS baguette
class BaguettePosition(Enum):
    BAS = auto()
    PRISE = auto()
    RANGE_EN_HAUT = auto()


@dataclass
class BaguetteServoConfig:
    id: int
    positions_us: dict[BaguettePosition, int]


# SERVOS aimants
class AimantPosition(Enum):
    ON = auto()
    OFF = auto()


@dataclass
class AimantServoConfig:
    id: int
    positions_us: dict[AimantPosition, int]


# SERVOS bras exterieurs ascenseurs
class BrasExterieurAscenseurPosition(Enum):
    PRISE = auto()
    EXTERIEUR = auto()


@dataclass
class BrasExterieurAscenseurServoConfig:
    id: int
    positions_us: dict[BrasExterieurAscenseurPosition, int]


# SERVOS bras de platforme ascenseurs
class BrasPlateformeAscenseurPosition(Enum):
    RANGE_EN_HAUT = auto()
    PREPARE_PRISE = auto()
    PRISE = auto()


@dataclass
class BrasPlateformeAscenseurServoConfig:
    id: int
    positions_us: dict[BrasPlateformeAscenseurPosition, int]


# STEPPER ascenseur
class AscenseurPosition(Enum):
    DEPART_AVEC_BANDEROLE = auto()
    PRISE_ETAGE_1 = auto()
    DEPOSE_ETAGE_2 = auto()


@dataclass
class AscenseurStepperConfig:
    id: int
    homing_tor_id: int
    positions_steps: dict[AscenseurPosition, int]


# POMPES+ELECTROVANNES plateforme
class PompesValvesState(Enum):
    ON = auto()
    OFF = auto()


class Robot2025:
    def __init__(self, servoboard: ServoBoard, ioboard: IOBoard, pumpboard: PumpBoard) -> None:
        self.servoboard = servoboard
        self.ioboard = ioboard
        self.pumpboard = pumpboard

        self.baguettes: dict[tuple[RobotFace, RobotSide], BaguetteServoConfig] = {
            (RobotFace.FRONT, RobotSide.LEFT): BaguetteServoConfig(
                14, {BaguettePosition.BAS: 2175, BaguettePosition.PRISE: 1850, BaguettePosition.RANGE_EN_HAUT: 1100}
            ),
            (RobotFace.FRONT, RobotSide.RIGHT): BaguetteServoConfig(
                12, {BaguettePosition.BAS: 1150, BaguettePosition.PRISE: 1425, BaguettePosition.RANGE_EN_HAUT: 2175}
            ),
            (RobotFace.BACK, RobotSide.LEFT): BaguetteServoConfig(
                5, {BaguettePosition.BAS: 2275, BaguettePosition.PRISE: 1975, BaguettePosition.RANGE_EN_HAUT: 1225}
            ),
            (RobotFace.BACK, RobotSide.RIGHT): BaguetteServoConfig(
                3, {BaguettePosition.BAS: 1200, BaguettePosition.PRISE: 1475, BaguettePosition.RANGE_EN_HAUT: 2225}
            ),
        }

        self.base_aimants: dict[tuple[RobotFace, RobotSide], AimantServoConfig] = {
            (RobotFace.FRONT, RobotSide.LEFT): AimantServoConfig(15, {AimantPosition.ON: 750, AimantPosition.OFF: 2500}),
            (RobotFace.FRONT, RobotSide.RIGHT): AimantServoConfig(13, {AimantPosition.ON: 2325, AimantPosition.OFF: 500}),
            (RobotFace.BACK, RobotSide.LEFT): AimantServoConfig(7, {AimantPosition.ON: 2200, AimantPosition.OFF: 1100}),
            (RobotFace.BACK, RobotSide.RIGHT): AimantServoConfig(4, {AimantPosition.ON: 575, AimantPosition.OFF: 2500}),
        }

        self.ascenseur_aimants: dict[tuple[RobotFace, RobotSide], AimantServoConfig] = {
            (RobotFace.FRONT, RobotSide.LEFT): AimantServoConfig(9, {AimantPosition.ON: 625, AimantPosition.OFF: 2500}),
            (RobotFace.FRONT, RobotSide.RIGHT): AimantServoConfig(10, {AimantPosition.ON: 2325, AimantPosition.OFF: 500}),
            (RobotFace.BACK, RobotSide.LEFT): AimantServoConfig(0, {AimantPosition.ON: 525, AimantPosition.OFF: 2500}),
            (RobotFace.BACK, RobotSide.RIGHT): AimantServoConfig(1, {AimantPosition.ON: 2400, AimantPosition.OFF: 500}),
        }

        self.bras_exterieur_ascenseurs: dict[tuple[RobotFace, RobotSide], BrasExterieurAscenseurServoConfig] = {
            (RobotFace.FRONT, RobotSide.LEFT): BrasExterieurAscenseurServoConfig(
                11, {BrasExterieurAscenseurPosition.PRISE: 2275, BrasExterieurAscenseurPosition.EXTERIEUR: 1650}
            ),
            (RobotFace.FRONT, RobotSide.RIGHT): BrasExterieurAscenseurServoConfig(
                8, {BrasExterieurAscenseurPosition.PRISE: 1150, BrasExterieurAscenseurPosition.EXTERIEUR: 1775}
            ),
            (RobotFace.BACK, RobotSide.LEFT): BrasExterieurAscenseurServoConfig(
                6, {BrasExterieurAscenseurPosition.PRISE: 905, BrasExterieurAscenseurPosition.EXTERIEUR: 1925}
            ),
            (RobotFace.BACK, RobotSide.RIGHT): BrasExterieurAscenseurServoConfig(
                2, {BrasExterieurAscenseurPosition.PRISE: 2225, BrasExterieurAscenseurPosition.EXTERIEUR: 1525}
            ),
        }

        self.bras_plateformes_ascenseurs: dict[RobotFace, BrasPlateformeAscenseurServoConfig] = {
            RobotFace.FRONT: BrasPlateformeAscenseurServoConfig(
                16,
                {
                    BrasPlateformeAscenseurPosition.RANGE_EN_HAUT: 1090,
                    BrasPlateformeAscenseurPosition.PREPARE_PRISE: 1500,
                    BrasPlateformeAscenseurPosition.PRISE: 1730,
                },
            ),
            RobotFace.BACK: BrasPlateformeAscenseurServoConfig(
                17,
                {
                    BrasPlateformeAscenseurPosition.RANGE_EN_HAUT: 510,
                    BrasPlateformeAscenseurPosition.PREPARE_PRISE: 960,
                    BrasPlateformeAscenseurPosition.PRISE: 1130,
                },
            ),
        }

        self.ascenseurs: dict[RobotFace, AscenseurStepperConfig] = {
            RobotFace.FRONT: AscenseurStepperConfig(
                0, 15, {AscenseurPosition.DEPART_AVEC_BANDEROLE: 0, AscenseurPosition.PRISE_ETAGE_1: 0, AscenseurPosition.DEPOSE_ETAGE_2: 0}
            ),
            RobotFace.BACK: AscenseurStepperConfig(
                1, 14, {AscenseurPosition.DEPART_AVEC_BANDEROLE: 0, AscenseurPosition.PRISE_ETAGE_1: 0, AscenseurPosition.DEPOSE_ETAGE_2: 0}
            ),
        }

        self.pompes: dict[RobotFace, tuple[int, int]] = {RobotFace.FRONT: (0, 2), RobotFace.BACK: (1, 3)}

    def set_base_baguettes(self, face: RobotFace, position: BaguettePosition):
        baguette_gauche = self.baguettes[(face, RobotSide.LEFT)]
        baguette_droite = self.baguettes[(face, RobotSide.RIGHT)]
        self.servoboard.servo_set_us(baguette_gauche.id, baguette_gauche.positions_us[position])
        self.servoboard.servo_set_us(baguette_droite.id, baguette_droite.positions_us[position])

    def set_base_aimants(self, face: RobotFace, position: AimantPosition):
        aimants_gauche = self.base_aimants[(face, RobotSide.LEFT)]
        aimants_droite = self.base_aimants[(face, RobotSide.RIGHT)]
        self.servoboard.servo_set_us(aimants_gauche.id, aimants_gauche.positions_us[position])
        self.servoboard.servo_set_us(aimants_droite.id, aimants_droite.positions_us[position])

    def set_ascenseur_bras_exterieur(self, face: RobotFace, position: BrasExterieurAscenseurPosition):
        bras_gauche = self.bras_exterieur_ascenseurs[(face, RobotSide.LEFT)]
        bras_droite = self.bras_exterieur_ascenseurs[(face, RobotSide.RIGHT)]

        if position == BrasExterieurAscenseurPosition.EXTERIEUR:
            self.servoboard.servo_set_us_interp(bras_gauche.id, bras_gauche.positions_us[position], us_per_sec=500)
            self.servoboard.servo_set_us_interp(bras_droite.id, bras_droite.positions_us[position], us_per_sec=500)
        else:
            self.servoboard.servo_set_us(bras_gauche.id, bras_gauche.positions_us[position])
            self.servoboard.servo_set_us(bras_droite.id, bras_droite.positions_us[position])

    def set_ascenseur_aimants(self, face: RobotFace, position: AimantPosition):
        aimants_gauche = self.ascenseur_aimants[(face, RobotSide.LEFT)]
        aimants_droite = self.ascenseur_aimants[(face, RobotSide.RIGHT)]
        self.servoboard.servo_set_us(aimants_gauche.id, aimants_gauche.positions_us[position])
        self.servoboard.servo_set_us(aimants_droite.id, aimants_droite.positions_us[position])

    def set_ascenseur_bras_plateform(self, face: RobotFace, position: BrasPlateformeAscenseurPosition):
        bras = self.bras_plateformes_ascenseurs[face]
        self.servoboard.servo_set_us(bras.id, bras.positions_us[position])

    def set_ascenseur_position(self, face: RobotFace, position: AscenseurPosition):
        stepper = self.ascenseurs[face]
        self.ioboard.goto_abs(stepper.id, stepper.positions_steps[position], 1000, 500)

    def home_ascenseur(self, face: RobotFace):
        max_steps = 10000 * (1 if face == RobotFace.FRONT else -1)
        stepper = self.ascenseurs[face]
        self.ioboard.home(stepper.id, max_steps, stepper.homing_tor_id, True)

    def set_ascenseur_platform_suction(self, face: RobotFace, state: PompesValvesState):
        pumps_id = self.pompes[face]

        if state == PompesValvesState.ON:
            self.pumpboard.enable_pumps(pumps_id)
        else:
            self.pumpboard.disable_pump_with_auto_valve_release(pumps_id)

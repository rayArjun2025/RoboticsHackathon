package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public final class IntakeConstants {
    public static final int HALL_EFFECT_CHANNEL = 0;
    public static final boolean HALL_INVERTED = true;
    public static final double HALL_DEBOUNCE_s = 0.1;
    public static final DebounceType HALL_DEBOUNCE_TYPE = DebounceType.kRising;
    public static final double TOLERANCE_RAD = 0.1;
    public static final double HOMING_VOLTS = -0.5;

    public static final double MIN_ANGLE_DEG = 0;
    public static final double MAX_ANGLE_DEG = 180;

    public static final int ARM_MOTOR_ID = 30;
    public static final String ARM_CANBUS = TunerConstants.kCANBus.getName();
    public static final boolean ARM_INVERTED = false;
    public static final boolean ARM_BRAKE = true;
    public static final double ARM_SUPPLY_CURRENT_LIMIT_A = 30.0;
    public static final double ARM_GEAR_RATIO = 75.0;
    public static final double ARM_RADIANS_TO_ROTATIONS = ARM_GEAR_RATIO / (2.0 * Math.PI);

    public static final double ARM_kP = 8.0;
    public static final double ARM_kI = 0.0;
    public static final double ARM_kD = 0.5;
    public static final double ARM_kS = 0.12;
    public static final double ARM_kV = 0.10;
    public static final double ARM_kA = 0.01;
    public static final double ARM_kG = 0.20;

    public static final GravityType GRAVITY = GravityType.ARM;

    public static final double ARM_MM_CRUISE_VELOCITY = 2.0;
    public static final double ARM_MM_ACCELERATION = 5.0;
    public static final double ARM_MM_JERK = 50.0;

    public static final DCMotor ARM_SIM_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final double ARM_SIM_MOI = 0.08;

    public static final int ROLLER_MOTOR_ID = 31;
    public static final String ROLLER_CANBUS = TunerConstants.kCANBus.getName();
    public static final boolean ROLLER_INVERTED = false;
    public static final boolean ROLLER_BRAKE = false;

    public static final double ROLLER_MAX_STALLS = 10;
    public static final double INTAKE_VOLTS = 0.27;
    public static final double HOLDING_VOLTS = 0.15;
    public static final double ROLLER_CURRENT_LIMIT = 50;
    public static final double ROLLER_CURRENT_THRESHOLD = ROLLER_CURRENT_LIMIT - 0.5;
    public static final double ROLLER_HOLD_CURRENT = 15.0;
}

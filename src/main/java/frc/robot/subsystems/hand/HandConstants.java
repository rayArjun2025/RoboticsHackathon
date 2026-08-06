package frc.robot.subsystems.hand;


import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public final class HandConstants {

    public static final int HALL_EFFECT_CHANNEL = 0;
    public static final double MIN_ANGLE_RAD = 0;
    public static final double MAX_ANGLE_RAD = 0.52;
    public static final boolean HALL_INVERTED = true;
    public static final double HALL_DEBOUNCE_s = 0.1;
    public static final DebounceType HALL_DEBOUNCE_TYPE = DebounceType.kRising;

    public static final int HAND_MOTOR_ID = 40;
    public static final String HAND_CANBUS = TunerConstants.kCANBus.getName();

    public static final boolean HAND_INVERTED = false;
    public static final boolean HAND_BRAKE = true;

    public static final double HAND_SUPPLY_CURRENT_LIMIT_A = 20.0;

    public static final double HAND_GEAR_RATIO = 25.0;
    public static final double HAND_RADIANS_TO_ROTATIONS =
        HAND_GEAR_RATIO / (2.0 * Math.PI);

      
    public static final double HAND_kP = 6.0;
    public static final double HAND_kI = 0.0;
    public static final double HAND_kD = 0.2;


    public static final double HAND_kS = 0.08;
    public static final double HAND_kV = 0.05;
    public static final double HAND_kA = 0.0;
    public static final double HAND_kG = 0.0; 

    public static final GravityType HAND_GRAVITY =
            GravityType.NONE;

    public static final double HAND_MM_CRUISE_VELOCITY = 1.0;
    public static final double HAND_MM_ACCELERATION = 3.0;
    public static final double HAND_MM_JERK = 30.0;


    public static final DCMotor HAND_SIM_MOTOR =
        DCMotor.getKrakenX60Foc(1);

    public static final double HAND_SIM_MOI = 0.01;

    public static final int MANIPULATOR_SENSOR_ID = 25;
    public static final double TOLERANCE_RAD = 0.1;
    public static final double HOMING_VOLTS = 0.5;
}

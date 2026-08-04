package frc.robot.subsystems.arm;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig.GravityType;

public class ArmConstants {
    public static final int HALL_EFFECT_CHANNEL = 0;
    public static final boolean HALL_INVERTED = true;
    public static final double HALL_DEBOUNCE_s = 0.1;
    public static final DebounceType HALL_DEBOUNCE_TYPE = DebounceType.kRising;

    public static final double MIN_ANGLE = 20;
    public static final double MAX_ANGLE = 90;

    public static final int MOTOR_ID = 20;
    public static final int HEIGHT_FOLLOWER_ID = 21;
    public static final double HOMING_VOLTS = -0.5;
    public static final double TOLERANCE_RAD = 0.1;

    public static final int FOLLOWER_ID = 21;
    public static final String CANBUS = "rio";

    public static final boolean INVERTED = false;
    public static final MotorAlignmentValue FOLLOWER_OPPOSE = MotorAlignmentValue.Opposed;
    public static final boolean BRAKE = true;

 
    public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;


    public static final double GEAR_RATIO = 100.0;
    public static final double RADIANS_TO_ROTATIONS =
        GEAR_RATIO / (2.0 * Math.PI);

    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.8;    

    public static final double kS = 0.15;
    public static final double kV = 0.10;
    public static final double kA = 0.01;
    public static final double kG = 0.30;   

    public static final GravityType GRAVITY = GravityType.ARM;
    public static final double MM_CRUISE_VELOCITY = 2.0;
    public static final double MM_ACCELERATION = 4.0;
    public static final double MM_JERK = 40.0;

    public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
    public static final double SIM_MOI = 0.25;
}

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public final class ElevatorConstants {

    private ElevatorConstants() {}

    public static final int HEIGHT_MOTOR_ID = 15;
    
    public static final int HEIGHT_FOLLOWER_ID = 16;
    
    public static final MotorAlignmentValue FOLLOWER_OPPOSE = MotorAlignmentValue.Opposed;
    public static final String CANBUS = TunerConstants.kCANBus.getName();
    public static final int HALL_EFFECT_CHANNEL = 0;

    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;
    public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;

    public static final double kS = 0.07;
    public static final double kV = 0.75;
    public static final double kA = 0.0;
    public static final double kG = 1.0;
    public static final double kP = 18.4;
    public static final double kI = 0.0;
    public static final double kD = 1.5;
    public static final GravityType GRAVITY = GravityType.ELEVATOR;

    public static final double MM_CRUISE_VELOCITY = 3.1;
    public static final double MM_ACCELERATION = 23.5;
    public static final double MM_JERK = 150.0;

    public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(2);
    public static final double SIM_MOI = 0.1;

    public static final boolean HALL_INVERTED = true;
    public static final double HALL_DEBOUNCE_s = 0.1;
    public static final DebounceType HALL_DEBOUNCE_TYPE = DebounceType.kRising;

    public static final double MIN_HEIGHT_m = 0.2191;
    public static final double MAX_HEIGHT_m = 1.724;
    public static final double STROKE_m = MAX_HEIGHT_m - MIN_HEIGHT_m;
    public static final double TOLERANCE_m = 0.5;

    public static final double MIN_HEIGHT_R = -14.812;
    public static final double MAX_HEIGHT_R = 1.262;
    public static final double METERS_TO_ROTATIONS = (MAX_HEIGHT_R - MIN_HEIGHT_R) / STROKE_m;

    public static final double HOMING_VOLTS = -0.5;

    public static final double ARM_MIN_ANGLE_DEG = 20;
    public static final double ARM_MAX_ANGLE_DEG = 90;
    public static final double ARM_HOMING_DEG = 80;

    public static final int ARM_MOTOR_ID = 20;
    public static final double ARM_HOMING_VOLTS = 0.5;
    public static final double ARM_TOLERANCE_RAD = 0.1;
    public static final String ARM_CANBUS = TunerConstants.kCANBus.getName();

    public static final boolean ARM_INVERTED = false;
    public static final boolean ARM_BRAKE = true;

 
    public static final double ARM_SUPPLY_CURRENT_LIMIT_A = 40.0;


    public static final double ARM_GEAR_RATIO = 100.0;
    public static final double ARM_RADIANS_TO_ROTATIONS =
        ARM_GEAR_RATIO / (2.0 * Math.PI);

    public static final double ARM_kP = 10.0;
    public static final double ARM_kI = 0.0;
    public static final double ARM_kD = 0.8;    

    public static final double ARM_kS = 0.15;
    public static final double ARM_kV = 0.10;
    public static final double ARM_kA = 0.01;
    public static final double ARM_kG = 0.30;   

    public static final GravityType ARM_GRAVITY = GravityType.ARM;
    public static final double ARM_MM_CRUISE_VELOCITY = 2.0;
    public static final double ARM_MM_ACCELERATION = 4.0;
    public static final double ARM_MM_JERK = 40.0;

    public static final DCMotor ARM_SIM_MOTOR = DCMotor.getKrakenX60Foc(1);
    public static final double ARM_SIM_MOI = 0.25;
    
    public static final double L1_ANGLE_DEG = 80;
    public static final double L1_VERTICAL_HEIGHT = 1.168;
    public static final double L1_ELEVATOR_HEIGHT = L1_VERTICAL_HEIGHT / Math.sin(Math.toRadians(L1_ANGLE_DEG));
    public static final double L2_ANGLE_DEG = 55;
    public static final double L2_VERTICAL_HEIGHT = 0.597;
    public static final double L2_HEIGHT = L2_VERTICAL_HEIGHT / Math.sin(Math.toRadians(L2_ANGLE_DEG));
    public static final double L3_VERTICAL_HEIGHT_M = 0.117;
    public static final double L3_ANGLE_DEG = 30;
    public static final double L3_ELEV_HEIGHT_M = L3_VERTICAL_HEIGHT_M / Math.sin(Math.toRadians(L3_ANGLE_DEG));

    public static final double DRIVER_STATION_ANGLE_DEG = 56.0;
    public static final double DRIVER_STATION_HEIGHT_M = 0.87;

    public static final double DRIVER_STATION_EXTENSION_M =
        DRIVER_STATION_HEIGHT_M
        / Math.sin(Math.toRadians(DRIVER_STATION_ANGLE_DEG));

}

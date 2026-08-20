package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.robot.Robot;
import frc.robot.devices.halleffect.HallEffect;
import frc.robot.devices.halleffect.HallEffectConfig;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.util.Util;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase<Elevator.Command> {

    public enum Command {
        DISABLED,
        IDLE,
        HOMING,
        GO_TO_POSITION,
        MANUAL
    }

    private enum Homing {
        SEEKING,
        SETTLED
    }

    private enum Travel {
        MOVING,
        HOLDING
    }

    private static Elevator instance;

    private final Motor heightMotor;
    private final Motor angleMotor;

    private final HallEffect hallEffect = new HallEffect(
            "Elevator/HallEffect",
            new HallEffectConfig(HALL_EFFECT_CHANNEL)
                    .withInverted(HALL_INVERTED)
                    .withDebounce(HALL_DEBOUNCE_s, HALL_DEBOUNCE_TYPE));
    private final Elevator2d elevatorMeasured2d = new Elevator2d("Elevator/Measured2d", new Color8Bit(200, 0, 0));
    private final Elevator2d elevatorSet2D = new Elevator2d("Elevator/Setpoint2d", new Color8Bit(100, 100, 100));
    private final Elevator2d armMeasured2d = new Elevator2d("Elevator/Measured2d", new Color8Bit(150, 0, 50));
    private final Elevator2d armSetpoint2d = new Elevator2d("Elevator/Setpoint2d", new Color8Bit(0, 150, 50));

    private double targetHeight_m = MIN_HEIGHT_m;
    private double targetAngle_rad = Math.toRadians(ARM_MAX_ANGLE_DEG);
    private double voltsTarget = 0.0;
    private boolean hallDetected = false;
    private boolean zeroed = false;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
            System.out.println("Elevator initialized.");
        }
        return instance;
    }

    private Elevator() {
        super("Elevator");

        MotorConfig heightConfig = new MotorConfig(HEIGHT_MOTOR_ID)
                .withCanbus(CANBUS)
                .withFollower(HEIGHT_FOLLOWER_ID, FOLLOWER_OPPOSE)
                .withInverted(INVERTED)
                .withBrake(BRAKE)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_A)
                .withSensorToMechanismRatio(METERS_TO_ROTATIONS)
                .withFFGains(kS, kV, kA, kG)
                .withPIDGains(kP, kI, kD, GRAVITY)
                .withMotionMagic(MM_CRUISE_VELOCITY, MM_ACCELERATION, MM_JERK)
                .withSim(SIM_MOTOR, METERS_TO_ROTATIONS, SIM_MOI);
        
         MotorConfig angleConfig = new MotorConfig(ARM_MOTOR_ID).withCanbus(ARM_CANBUS)
                .withInverted(ARM_INVERTED)
                .withBrake(ARM_BRAKE)
                .withSupplyCurrentLimit(ARM_SUPPLY_CURRENT_LIMIT_A)
                .withSensorToMechanismRatio(ARM_RADIANS_TO_ROTATIONS)
                .withFFGains(ARM_kS, ARM_kV, ARM_kA, ARM_kG)
                .withPIDGains(ARM_kP, ARM_kI, ARM_kD, ARM_GRAVITY)
                .withMotionMagic(ARM_MM_CRUISE_VELOCITY, ARM_MM_ACCELERATION, ARM_MM_JERK)
                .withSim(ARM_SIM_MOTOR, ARM_RADIANS_TO_ROTATIONS, ARM_SIM_MOI);

        angleMotor = new Motor("Elevator/AngleMotor", angleConfig);
        heightMotor = new Motor("Elevator/HeightMotor", heightConfig);
        setCommand(Command.IDLE);
    }

    @Override
    protected void inputPeriodic() {
        heightMotor.readInputs();
        angleMotor.readInputs();
        hallEffect.readInputs();
        hallDetected = hallEffect.get();
    }

    @Override
    protected void handle() {
        switch (getCommand()) {
            case DISABLED:
                heightMotor.stop();
                angleMotor.stop();
                break;
            case IDLE:
                heightMotor.setVoltage(0.0);
                angleMotor.setVoltage(0);
                break;

            case HOMING:
                if (firstLoop()) {
                    setSubstate(zeroed ? Homing.SETTLED : Homing.SEEKING);
                }
                switch ((Homing) getSubstate()) {
                    case SEEKING:
                        heightMotor.setVoltage(HOMING_VOLTS);
                        angleMotor.setVoltage(ARM_HOMING_VOLTS);
                        if (Robot.isSimulation() || hallDetected) {
                            heightMotor.zeroPosition(MIN_HEIGHT_m);
                            angleMotor.zeroPosition(Math.toRadians(ARM_HOMING_DEG));
                            zeroed = true;
                            setSubstate(Homing.SETTLED);   
                        }
                        break;
                    case SETTLED:
                        setCommand(Command.IDLE);          
                        break;
                }
                break;

            case GO_TO_POSITION:
                if (firstLoop()) {
                    setSubstate(Travel.MOVING);
                }
                switch ((Travel) getSubstate()) {
                    case MOVING:
                        heightMotor.setMotionMagic(targetHeight_m);
                        angleMotor.setMotionMagic(targetAngle_rad);
                        if (atTargetHeight(TOLERANCE_m) && atTargetAngle(ARM_TOLERANCE_RAD)) {
                            setSubstate(Travel.HOLDING);   
                        }
                        break;
                    case HOLDING:
                        heightMotor.setMotionMagic(targetHeight_m);
                        angleMotor.setMotionMagic(targetAngle_rad);
                        if (!atTargetHeight(TOLERANCE_m) || !atTargetAngle(ARM_TOLERANCE_RAD)) {
                            setSubstate(Travel.MOVING);    
                        }
                        break;
                }
                break;

            case MANUAL:
                if(!hallDetected){
                    heightMotor.setVoltage(voltsTarget);
                    angleMotor.setVoltage(voltsTarget);
                }
                break;
        }
    }

    @Override
    protected void outputPeriodic() {
        elevatorMeasured2d.setHeight(getHeight());
        elevatorSet2D.setHeight(targetHeight_m);
        elevatorMeasured2d.periodic();
        elevatorSet2D.periodic();

        armMeasured2d.setAngle(getAngle());
        armSetpoint2d.setAngle(targetAngle_rad);
        armMeasured2d.periodic();
        armSetpoint2d.periodic();

        Logger.recordOutput("Elevator/Height_m", getHeight());
        Logger.recordOutput("Elevator/Velocity_mps", heightMotor.getVelocity());
        Logger.recordOutput("Elevator/TargetHeight_m", targetHeight_m);
        Logger.recordOutput("Elevator/Zeroed", isZeroed());

        Logger.recordOutput("Elevator/Angle_rad", Math.toDegrees(getAngle()));
        Logger.recordOutput("Elevator/Velocity_rps", angleMotor.getVelocity());
        Logger.recordOutput("Elevator/TargetAngle_rad", Math.toDegrees(targetAngle_rad));
    }


    public void idle() {
        setCommand(Command.IDLE);
    }

    public void home() {
        setCommand(Command.HOMING);
    }

    public void trackToHeight(double height_m) {
        targetHeight_m = MathUtil.clamp(height_m, MIN_HEIGHT_m, MAX_HEIGHT_m);
        if(getCommand() != Command.GO_TO_POSITION)
            setCommand(Command.GO_TO_POSITION);
    }

    public void tracktoAngle(double angle_deg) {
        angle_deg = MathUtil.clamp(angle_deg, ARM_MIN_ANGLE_DEG, ARM_MAX_ANGLE_DEG);
        targetAngle_rad = Math.toRadians(angle_deg);
        if(getCommand() != Command.GO_TO_POSITION)
            setCommand(Command.GO_TO_POSITION);
    }


    public void manual(double volts) {
        voltsTarget = volts;
        setCommand(Command.MANUAL);
    }

    public double getHeight() {
        return heightMotor.getPosition();
    }

    public double getTargetHeight() {
        return targetHeight_m;
    }

     public double getAngle() {
        return angleMotor.getPosition();
    }

    public double getTargetAngle() {
        return targetAngle_rad;
    }


    public boolean atTargetHeight(double tol) {
        return Util.inRange(targetHeight_m - getHeight(), tol);
    }

    public boolean atTargetAngle(double tol){
         return Util.inRange(targetAngle_rad - getAngle(), tol);
    }

    public boolean isZeroed() {
        return zeroed || Robot.isSimulation();
    }
}

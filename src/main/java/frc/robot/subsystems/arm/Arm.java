package frc.robot.subsystems.arm;
import static frc.robot.subsystems.arm.ArmConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.devices.halleffect.HallEffect;
import frc.robot.devices.halleffect.HallEffectConfig;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.util.Util;

public class Arm extends SubsystemBase<Arm.Command>{
    public enum Command {
        DISABLED,
        IDLE,
        HOMING,
        TRACK_TO_ANGLE,
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
    
    
    private static Arm instance;
    private final Motor motor;
    private final HallEffect hallEffect = new HallEffect(
        "Arm/HallEffect",
        new HallEffectConfig(HALL_EFFECT_CHANNEL)
        .withInverted(HALL_INVERTED)
        .withDebounce(HALL_DEBOUNCE_s, HALL_DEBOUNCE_TYPE));
        
    private final Arm2d measured2d = new Arm2d("Arm/Measured2d", new Color8Bit(200, 0, 0));
    private final Arm2d setpoint2d = new Arm2d("Arm/Setpoint2d", new Color8Bit(100, 100, 100));
    private double targetAngle_rad = Math.toRadians(MAX_ANGLE);

    private double voltsTarget = 0;
    private boolean zeroed = false;
    private boolean hallDetected = false;

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
            System.out.println("Arm initialized.");
        }

        return instance;
    }


    private Arm(){
        super("Arm");

        MotorConfig config = new MotorConfig(MOTOR_ID).withCanbus(CANBUS)
                .withInverted(INVERTED)
                .withBrake(BRAKE)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_A)
                .withSensorToMechanismRatio(RADIANS_TO_ROTATIONS)
                .withFFGains(kS, kV, kA, kG)
                .withPIDGains(kP, kI, kD, GRAVITY)
                .withMotionMagic(MM_CRUISE_VELOCITY, MM_ACCELERATION, MM_JERK)
                .withSim(SIM_MOTOR, RADIANS_TO_ROTATIONS, SIM_MOI);

        motor = new Motor("Arm/AngleMotor", config);
        setCommand(Command.IDLE);

    }

    @Override
    protected void inputPeriodic(){
        motor.readInputs();
        hallEffect.readInputs();
        hallDetected = hallEffect.get();
    }

    @Override
    protected void outputPeriodic(){
        measured2d.setAngle(getAngle());
        setpoint2d.setAngle(targetAngle_rad);
        measured2d.periodic();
        setpoint2d.periodic();

        Logger.recordOutput("Arm/Angle_rad", getAngle());
        Logger.recordOutput("Arm/Velocity_rps", motor.getVelocity());
        Logger.recordOutput("Arm/TargetAngle_rad", targetAngle_rad);
        Logger.recordOutput("Arm/Zeroed", isZeroed());
    }

    @Override
    protected void handle(){
        switch (getCommand()) {
            case DISABLED:
                motor.stop();
                break;
            case IDLE:
                motor.setVoltage(0.0);
                break;

            case HOMING:
                if (firstLoop()) {
                    setSubstate(zeroed ? Homing.SETTLED : Homing.SEEKING);
                }
                switch ((Homing) getSubstate()) {
                    case SEEKING:
                        motor.setVoltage(HOMING_VOLTS);
                        if (Robot.isSimulation() || hallDetected) {
                            motor.zeroPosition(MAX_ANGLE);
                            zeroed = true;
                            setSubstate(Homing.SETTLED);   
                        }
                        break;
                    case SETTLED:
                        setCommand(Command.IDLE);          
                        break;
                }
                break;

            case TRACK_TO_ANGLE:
                if (firstLoop()) {
                    setSubstate(Travel.MOVING);
                }
                switch ((Travel) getSubstate()) {
                    case MOVING:
                        motor.setMotionMagic(targetAngle_rad);
                        if (atTargetAngle(TOLERANCE_RAD)) {
                            setSubstate(Travel.HOLDING);   
                        }
                        break;
                    case HOLDING:
                        motor.setMotionMagic(targetAngle_rad);
                        if (!atTargetAngle(TOLERANCE_RAD)) {
                            setSubstate(Travel.MOVING);    
                        }
                        break;
                }
                break;

            case MANUAL:
                motor.setVoltage(voltsTarget);
                break;
        }
    }

    public void idle() {
        setCommand(Command.IDLE);
    }

    public void home() {
        setCommand(Command.HOMING);
    }

    public void tracktoAngle(double angle_deg) {
        angle_deg = MathUtil.clamp(angle_deg, MIN_ANGLE, MAX_ANGLE);
        targetAngle_rad = Math.toRadians(angle_deg);
        setCommand(Command.TRACK_TO_ANGLE);
    }


    public void manual(double volts) {
        voltsTarget = MathUtil.clamp(volts, -12, 12);
        setCommand(Command.MANUAL);
    }

    public double getAngle() {
        return motor.getPosition();
    }

    public double getTargetAngle() {
        return targetAngle_rad;
    }

    public boolean atTargetAngle(double tol) {
        return Util.inRange(targetAngle_rad - getAngle(), tol);
    }

    public boolean isZeroed() {
        return zeroed || Robot.isSimulation();
    }
}
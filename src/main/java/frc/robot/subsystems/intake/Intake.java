package frc.robot.subsystems.intake;
import static frc.robot.subsystems.intake.IntakeConstants.*;

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


public class Intake extends SubsystemBase<Intake.Command> {

    public enum Command {
        DISABLED, 
        IDLE,
        HOMING, 
        ADJUSTING_INTAKE_ANGLE,
        COLLECTING,
        MANUAL
    }

    private enum Homing {
        SEEKING,
        SETTLED
    }

    private enum Adjusting {
        MOVING,
        HOLDING
    }

    private enum Collecting {
        INTAKING,
        HOLDING
    }

    private static Intake instance;
    private final HallEffect hallEffect = new HallEffect(
        "Intake/HallEffect",
        new HallEffectConfig(HALL_EFFECT_CHANNEL)
        .withInverted(HALL_INVERTED)
        .withDebounce(HALL_DEBOUNCE_s, HALL_DEBOUNCE_TYPE));
        
    private final Intake2D measured2d = new Intake2D("Intake/Measured2D", new Color8Bit(150, 0, 100));
    private final Intake2D setpoint2d = new Intake2D("Intake/Set2D", new Color8Bit(150, 100, 0));

    private final Motor intakeMotor;
    private final Motor rollerMotor;
    private double targetAngle_rad = 0;
    private double targetVolts = 0;
    private boolean zeroed = false;
    private boolean hallDetected = false;
    private int rollerStallCounts = 0;


    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
            System.out.println("Intake initialized.");
        }
        return instance;
    }

    private Intake(){
        super("Intake");

        MotorConfig intakeConfig = new MotorConfig(ARM_MOTOR_ID).withCanbus(ARM_CANBUS)
                .withInverted(ARM_INVERTED)
                .withBrake(ARM_BRAKE)
                .withFFGains(ARM_kS, ARM_kV, ARM_kA, ARM_kG)
                .withPIDGains(ARM_kP, ARM_kI, ARM_kD, GRAVITY)
                .withSupplyCurrentLimit(ARM_SUPPLY_CURRENT_LIMIT_A)
                .withSensorToMechanismRatio(ARM_RADIANS_TO_ROTATIONS)
                .withMotionMagic(ARM_MM_CRUISE_VELOCITY, ARM_MM_ACCELERATION, ARM_MM_JERK)
                .withSim(ARM_SIM_MOTOR, ARM_RADIANS_TO_ROTATIONS, ARM_SIM_MOI);

        MotorConfig rollerConfig = new MotorConfig(ROLLER_MOTOR_ID).withCanbus(ROLLER_CANBUS)
                .withInverted(ROLLER_INVERTED)
                .withBrake(ROLLER_BRAKE)
                .withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT);
                

        intakeMotor = new Motor("IntakeMotor", intakeConfig);
        rollerMotor = new Motor("RollerMotor", rollerConfig);

        setCommand(Command.IDLE);
    }


    @Override
    protected void inputPeriodic(){
        rollerMotor.readInputs();
        intakeMotor.readInputs();
        hallEffect.readInputs();
        hallDetected = hallEffect.get();
    }

    @Override
    protected void handle(){
        switch (getCommand()) {
            case DISABLED:
                intakeMotor.stop();
                rollerMotor.stop();
                break;
            case IDLE:
                rollerMotor.setVoltage(0.0);
                intakeMotor.setVoltage(0);
                break;

            case HOMING:
                if (firstLoop()) {
                    setSubstate(zeroed ? Homing.SETTLED : Homing.SEEKING);
                }
                switch ((Homing) getSubstate()) {
                    case SEEKING:
                        intakeMotor.setVoltage(HOMING_VOLTS);
                        if (Robot.isSimulation() || hallDetected) {
                            intakeMotor.zeroPosition(MIN_ANGLE_DEG);
                            zeroed = true;
                            setSubstate(Homing.SETTLED);   
                        }
                        break;
                    case SETTLED:
                        setCommand(Command.IDLE);          
                        break;
                }
                break;

            case ADJUSTING_INTAKE_ANGLE:
                if (firstLoop()) {
                    setSubstate(Adjusting.MOVING);
                }
                switch ((Adjusting) getSubstate()) {
                    case MOVING:
                        intakeMotor.setMotionMagic(targetAngle_rad);
                        if (atTargetAngle(TOLERANCE_RAD)) {
                            setSubstate(Adjusting.HOLDING);   
                        }
                        break;
                    case HOLDING:
                        intakeMotor.setMotionMagic(targetAngle_rad);
                        if (!atTargetAngle(TOLERANCE_RAD)) {
                            setSubstate(Adjusting.MOVING);    
                        }
                        break;
                }
                break;

            case COLLECTING:
                if (firstLoop()) {
                    setSubstate(Collecting.INTAKING);
                }

                switch ((Collecting) getSubstate()) {

                    case INTAKING:
                        rollerMotor.setVoltage(INTAKE_VOLTS);
                        if (rollerMotor.getVoltage() > 0 && rollerMotor.getCurrent() > ROLLER_CURRENT_THRESHOLD) {
                            rollerStallCounts++;
                            if(rollerStallCounts > ROLLER_MAX_STALLS){
                                setSubstate(Collecting.HOLDING);
                            }
                        }
                        else {
                            rollerStallCounts = 0;
                        }
                        break;

                    case HOLDING:
                        rollerMotor.setVoltage(HOLDING_VOLTS);
                        break;
                }
                break;
                            

            case MANUAL:
                intakeMotor.setVoltage(targetVolts);
                rollerMotor.stop();
                break;
        }
    }

    @Override 
    protected void outputPeriodic(){
        measured2d.setAngle(getAngle());
        setpoint2d.setAngle(targetAngle_rad);
        measured2d.periodic();
        setpoint2d.periodic();

        Logger.recordOutput("Arm/Angle_rad", getAngle());
        Logger.recordOutput("Arm/Velocity_rps", intakeMotor.getVelocity());
        Logger.recordOutput("Arm/TargetAngle_rad", targetAngle_rad);
        Logger.recordOutput("Arm/Zeroed", isZeroed());
    }

    public void idle() {
        setCommand(Command.IDLE);
    }

    public void home() {
        setCommand(Command.HOMING);
    }

    public void moveToAngle(double angle_deg) {
        angle_deg = MathUtil.clamp(angle_deg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        targetAngle_rad = Math.toRadians(angle_deg);
        setCommand(Command.ADJUSTING_INTAKE_ANGLE);
    }


    public void manual(double volts) {
        targetVolts = volts;
        setCommand(Command.MANUAL);
    }

    public double getAngle() {
        return intakeMotor.getPosition();
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

package frc.robot.subsystems.hand;
import static frc.robot.subsystems.hand.HandConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.devices.halleffect.HallEffect;
import frc.robot.devices.halleffect.HallEffectConfig;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.util.Util;

public class Hand extends SubsystemBase<Hand.Command> {
    public enum Command{
        DISABLED,
        IDLE,
        MANUAL,
        HOMING,
        MANIPULATING
    }

    private enum Manipulating{
        HOLDING,
        TRAVELING
    }

    private enum Homing{
        SEEKING,
        SETTLED
    }
    
    private static Hand instance;
    private final DigitalInput handSensor;
    private final Motor handMotor;
    private final HallEffect hallEffect = new HallEffect(
            "Hand/HallEffect",
            new HallEffectConfig(HALL_EFFECT_CHANNEL)
                    .withInverted(HALL_INVERTED)
                    .withDebounce(HALL_DEBOUNCE_s, HALL_DEBOUNCE_TYPE));
    private boolean hallDetected = false;
    private double targetVolts = 0;
    private double targetAngle_rad = OPEN_ANGLE_RAD;
    private boolean zeroed = false;
    


    public static Hand getInstance() {
        if (instance == null) {
            instance = new Hand();
            System.out.println("Hand initialized.");
        }
        return instance;
    }

    private Hand(){
        super("Hand");

        MotorConfig config = new MotorConfig(HAND_MOTOR_ID).withCanbus(HAND_CANBUS)
           .withInverted(HAND_INVERTED)
            .withBrake(HAND_BRAKE)
            .withSupplyCurrentLimit(HAND_SUPPLY_CURRENT_LIMIT_A)
            .withSensorToMechanismRatio(HAND_RADIANS_TO_ROTATIONS)
            .withFFGains(HAND_kS, HAND_kV, HAND_kA, HAND_kG)
            .withPIDGains(HAND_kP, HAND_kI, HAND_kD, HAND_GRAVITY)
            .withMotionMagic(HAND_MM_CRUISE_VELOCITY, HAND_MM_ACCELERATION, HAND_MM_JERK)
            .withSim(HAND_SIM_MOTOR, HAND_RADIANS_TO_ROTATIONS, HAND_SIM_MOI);

        
        handMotor = new Motor("Hand/HandMotor", config);
        this.handSensor = new DigitalInput(HandConstants.MANIPULATOR_SENSOR_ID);
        setCommand(Command.IDLE);
    }
   
    @Override
    protected void inputPeriodic(){
        handMotor.readInputs();
        hallEffect.readInputs();
        hallDetected = hallEffect.get();
    }

    @Override
    protected void handle(){
        switch(getCommand()){
            case DISABLED:
                handMotor.stop();
                break;
            case IDLE:
                handMotor.setVoltage(0);
                break;

            case HOMING:
                if (firstLoop()) {
                    setSubstate(zeroed ? Homing.SETTLED : Homing.SEEKING);
                }
                switch ((Homing) getSubstate()) {
                    case SEEKING:
                        handMotor.setVoltage(HOMING_VOLTS);
                        if (Robot.isSimulation() || hallDetected) {
                            handMotor.zeroPosition(OPEN_ANGLE_RAD);
                            zeroed = true;
                            setSubstate(Homing.SETTLED);   
                        }
                        break;
                    case SETTLED:
                        setCommand(Command.IDLE);          
                        break;
                }
                break;

            case MANIPULATING:
                if (firstLoop()) {
                    setSubstate(Manipulating.TRAVELING);
                }
                switch ((Manipulating) getSubstate()) {
                    case TRAVELING:
                        handMotor.setMotionMagic(targetAngle_rad);
                        if (atTargetAngle(TOLERANCE_RAD) && !handSensor.get()) {
                            setSubstate(Manipulating.HOLDING);   
                        }
                        break;
                    case HOLDING:
                        handMotor.setMotionMagic(targetAngle_rad);
                        if (!atTargetAngle(TOLERANCE_RAD) || handSensor.get()) {
                            setSubstate(Manipulating.TRAVELING);
                        }
                        break;
                }
                break;

            case MANUAL:
                handMotor.setVoltage(targetVolts);
                break;
        
        }
    }


    @Override
    protected void outputPeriodic(){
        Logger.recordOutput("Hand/Angle_rad", Math.toDegrees(getAngle()));
        Logger.recordOutput("Hand/Target_rad", Math.toDegrees(targetAngle_rad));
        Logger.recordOutput("Hand/HasGamePiece", !handSensor.get());
        Logger.recordOutput("Hand/Zeroed", zeroed);
        Logger.recordOutput("Hand/HallDetected", hallDetected);
        Logger.recordOutput("Hand/Sensor", handSensor.get());
        Logger.recordOutput("Hand/Command", getCommand().toString());
        //Logger.recordOutput("Hand/Substate", getSubstate().toString());
        Logger.recordOutput("Hand/AngleError", Math.toDegrees(targetAngle_rad - getAngle()));
        Logger.recordOutput("Hand/AtTarget", atTargetAngle(TOLERANCE_RAD));
    }

    public void idle() {
        setCommand(Command.IDLE);
    }

    public void home() {
        setCommand(Command.HOMING);
    }

    public void moveToAngle(double angle_rad) {
        angle_rad = MathUtil.clamp(angle_rad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        targetAngle_rad = angle_rad;
        if (getCommand() != Command.MANIPULATING) {
            setCommand(Command.MANIPULATING);
        }
    }


    public void manual(double volts) {
        targetVolts = volts;
        setCommand(Command.MANUAL);
    }

    public double getAngle() {
        return handMotor.getPosition();
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

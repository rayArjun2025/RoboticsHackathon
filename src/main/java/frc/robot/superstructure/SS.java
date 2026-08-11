package frc.robot.superstructure;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.hand.HandConstants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

import frc.robot.subsystems.hand.Hand;

import frc.robot.subsystems.intake.Intake;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

public class SS extends SubsystemBase<SS.Command> {

    public enum Flag {
        HOME,
        DRIVERSTATION_INTAKE_CUBE,
        DRIVERSTATION_INTAKE_CONE,
        L1_CONE,
        L1_CUBE,
        L2_CONE,
        L2_CUBE,
        L3_CONE,
        L3_CUBE,
        MANUAL_UP,
        MANUAL_DOWN
    }

    public enum Command {
        IDLE,
        HOMING,
        STOWING,
        INTAKING,
        SCORING,
        HOLDING,
        MANUAL
    }

    private enum Intaking{
        RAISING, 
        SETTLING, 
        READY_TO_INTAKE,
    }

    private enum Scoring {
        RAISING,
        SETTLING,
        READY_TO_SCORE,
    }

    private enum Holding{
        SETTLING,
        HOLD
    }

    private static final double MANUAL_VOLTS = 2.0;
    
    private static final double SETTLE_TIME_s = 0.2;

    private static SS instance;

    private final EnumSet<Flag> flags = EnumSet.noneOf(Flag.class);

    private final Elevator elevator;
    private final Drive drive;
    private final Hand hand;
    private final Intake intake;

    private double scoreTarget_m = ElevatorConstants.MIN_HEIGHT_m;
    private double targetAngle_deg = ElevatorConstants.ARM_MAX_ANGLE_DEG;
    private double handAngle_rad = HandConstants.OPEN_ANGLE_RAD;


    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private SS() {
        super("Superstructure");
        elevator = Elevator.getInstance();
        drive = Drive.getInstance();
        hand = Hand.getInstance();
        intake = Intake.getInstance();
        setCommand(Command.IDLE);
    }


    public void enable(Flag flag) {
        flags.add(flag);
    }

    public void disable(Flag flag) {
        flags.remove(flag);
    }

    public void set(Flag flag, boolean active) {
        if (active) {
            flags.add(flag);
        } else {
            flags.remove(flag);
        }
    }

    public void toggle(Flag flag) {
        set(flag, !has(flag));
    }

    public boolean has(Flag flag) {
        return flags.contains(flag);
    }

    @Override
    protected void inputPeriodic() {
        
    }

    @Override
    protected void handle() {
        if (has(Flag.HOME)) {
            setCommand(Command.HOMING);
        } else if (has(Flag.MANUAL_UP) || has(Flag.MANUAL_DOWN)) {
            setCommand(Command.MANUAL);
        } else if (has(Flag.L1_CONE) || has(Flag.L1_CUBE)) {
            scoreTarget_m = ElevatorConstants.L1_ELEVATOR_HEIGHT;
            targetAngle_deg = ElevatorConstants.L1_ANGLE_DEG;
            handAngle_rad = has(Flag.L1_CONE) ? HandConstants.CONE_HOLD_ANGLE_RAD : HandConstants.CUBE_HOLD_ANGLE_RAD;
            setCommand(Command.SCORING);
        } else if (has(Flag.L2_CONE) || has(Flag.L2_CUBE)) {
            scoreTarget_m = ElevatorConstants.L2_HEIGHT;
            targetAngle_deg = ElevatorConstants.L2_ANGLE_DEG;
            handAngle_rad = has(Flag.L2_CONE) ? HandConstants.CONE_HOLD_ANGLE_RAD : HandConstants.CUBE_HOLD_ANGLE_RAD;
            setCommand(Command.SCORING);
        } else if(has(Flag.L3_CONE) || has(Flag.L3_CUBE)){
           scoreTarget_m = ElevatorConstants.L3_ELEV_HEIGHT_M;
           targetAngle_deg = ElevatorConstants.L3_ANGLE_DEG;
           handAngle_rad = has(Flag.L3_CONE) ? HandConstants.CONE_HOLD_ANGLE_RAD : HandConstants.CUBE_HOLD_ANGLE_RAD;
           setCommand(Command.SCORING);
        }
        else if(has(Flag.DRIVERSTATION_INTAKE_CONE) || has(Flag.DRIVERSTATION_INTAKE_CUBE)){
            scoreTarget_m = ElevatorConstants.DRIVER_STATION_EXTENSION_M;
            targetAngle_deg = ElevatorConstants.DRIVER_STATION_ANGLE_DEG;
            handAngle_rad = has(Flag.DRIVERSTATION_INTAKE_CONE) ? HandConstants.CONE_GRAB_ANGLE_RAD : HandConstants.CUBE_GRAB_ANGLE_RAD;
            setCommand(Command.INTAKING);
        }
        else {
            setCommand(Command.STOWING);
        }

        switch (getCommand()) {
            case HOMING:
                elevator.home();
                intake.home();
                hand.home();
                break;
            case MANUAL:
                elevator.manual(has(Flag.MANUAL_UP) ? MANUAL_VOLTS : -MANUAL_VOLTS);
                intake.manual(has(Flag.MANUAL_UP) ? MANUAL_VOLTS : -MANUAL_VOLTS);
                hand.manual(has(Flag.MANUAL_UP) ? MANUAL_VOLTS : -MANUAL_VOLTS);
                break;
            case SCORING:
                handleScoring();
                break;
            case INTAKING:
                handleIntaking();
                break;
            case HOLDING:
                elevator.home();
                hand.moveToAngle(handAngle_rad);
                break;
            case STOWING:
                elevator.trackToHeight(ElevatorConstants.MIN_HEIGHT_m);
                elevator.tracktoAngle(ElevatorConstants.ARM_HOMING_DEG);
                hand.moveToAngle(HandConstants.OPEN_ANGLE_RAD);
                break;
            case IDLE:
                break;
        }
    }

    private void handleScoring() {
        if (firstLoop()) {
            setSubstate(Scoring.RAISING);
        }

        switch ((Scoring) getSubstate()) {
            case RAISING:
                elevator.trackToHeight(scoreTarget_m);
                elevator.tracktoAngle(targetAngle_deg);
                hand.moveToAngle(handAngle_rad);
                if (elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) && elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD) && hand.atTargetAngle(HandConstants.TOLERANCE_RAD)) {
                    setSubstate(Scoring.SETTLING);   
                }
                break;

            case SETTLING:
                elevator.trackToHeight(scoreTarget_m);
                elevator.tracktoAngle(targetAngle_deg);
                hand.moveToAngle(handAngle_rad);
                if (!elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) || !elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD) || !hand.atTargetAngle(HandConstants.TOLERANCE_RAD)) {
                    setSubstate(Scoring.RAISING);    
                } else if (substateElapsed(SETTLE_TIME_s)) {
                    setSubstate(Scoring.READY_TO_SCORE);    
                }
                break;

            case READY_TO_SCORE:
                elevator.trackToHeight(scoreTarget_m);
                elevator.tracktoAngle(targetAngle_deg);
                hand.moveToAngle(HandConstants.OPEN_ANGLE_RAD);
                if(elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) && elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD)){
                    setCommand(Command.STOWING);
                }
                else if (!elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) || !elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD) || !hand.atTargetAngle(HandConstants.TOLERANCE_RAD)) {
                    setSubstate(Scoring.RAISING);   
                }
                break;
            
        }
    }

    private void handleIntaking(){
        if(firstLoop()){
            setSubstate(Intaking.RAISING);
        }

        switch((Intaking) getSubstate()){
            case RAISING:
                elevator.trackToHeight(scoreTarget_m);
                elevator.tracktoAngle(targetAngle_deg);
                hand.moveToAngle(handAngle_rad);
                if (elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) && elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD) && hand.atTargetAngle(HandConstants.TOLERANCE_RAD)) {
                    setSubstate(Intaking.SETTLING);   
                }
                break;

            case SETTLING:
                elevator.trackToHeight(scoreTarget_m);
                elevator.tracktoAngle(targetAngle_deg);
                hand.moveToAngle(handAngle_rad);
                if (!elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) || !elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD) || !hand.atTargetAngle(HandConstants.TOLERANCE_RAD)) {
                    setSubstate(Intaking.RAISING);    
                } else if (substateElapsed(SETTLE_TIME_s)) {
                    setSubstate(Intaking.READY_TO_INTAKE);    
                }
                break;

            case READY_TO_INTAKE:
                elevator.trackToHeight(scoreTarget_m);
                elevator.tracktoAngle(targetAngle_deg);
                hand.moveToAngle(handAngle_rad);
                if(elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) && elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD) && hand.atTargetAngle(HandConstants.TOLERANCE_RAD)){
                    if(handAngle_rad == HandConstants.CONE_GRAB_ANGLE_RAD)
                        handAngle_rad = HandConstants.CONE_HOLD_ANGLE_RAD;
                    else
                        handAngle_rad = HandConstants.CUBE_HOLD_ANGLE_RAD;
                    setCommand(Command.HOLDING);
                }
                else if (!elevator.atTargetHeight(ElevatorConstants.TOLERANCE_m) || !elevator.atTargetAngle(ElevatorConstants.ARM_TOLERANCE_RAD) || !hand.atTargetAngle(HandConstants.TOLERANCE_RAD)) {
                    setSubstate(Intaking.RAISING);   
                }
                break;
        }
    }

    

    @Override
    protected void outputPeriodic() {
        String[] active = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", active);
        Logger.recordOutput("Superstructure/Command", getCommand());
    }
}

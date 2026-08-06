package frc.robot.subsystems.hand;
import static frc.robot.subsystems.hand.HandConstants.*;
import frc.robot.subsystems.SubsystemBase;

public class Hand extends SubsystemBase<Hand.Command> {
    public enum Command{
        DISABLED,
        IDLE,
        MANUAL,
        MANIPULATING
    }

    private enum Manipulating{
        BLOCKED,
        GRABBING
    }
    
    private static Hand instance;
    
    private Hand(){
        super("Hand");
    }
   
    @Override
    protected void inputPeriodic(){

    }

    @Override
    protected void handle(){

    }


    @Override
    protected void outputPeriodic(){

    }


}

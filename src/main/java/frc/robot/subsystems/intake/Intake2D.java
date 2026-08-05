package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Intake2D {
  LoggedMechanism2d mech;
  LoggedMechanismRoot2d root;

  public LoggedMechanismLigament2d robotIntake;

  Intake intake;

  public static Intake2D instance;

  String name;

  public Intake2D(String name, Color8Bit color) {
    this.name = name;
    mech = new LoggedMechanism2d(4, 4);
    root = mech.getRoot("Root", 2, 0.5);
    robotIntake = root.append(new LoggedMechanismLigament2d("arm", 2, 90, 10, color));
  }

  public void setAngle(double angle_rad) {
    robotIntake.setAngle(angle_rad);
  }

  public void periodic() {
    SmartDashboard.putData(name, mech);
    Logger.recordOutput(name, mech);    
  }
}

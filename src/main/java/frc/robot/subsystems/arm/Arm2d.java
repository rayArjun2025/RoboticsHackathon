package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;


public class Arm2d {
  LoggedMechanism2d mech;
  LoggedMechanismRoot2d root;

  public LoggedMechanismLigament2d robotArm;

  Arm arm;

  public static Arm2d instance;

  String name;

  public Arm2d(String name, Color8Bit color) {
    this.name = name;
    mech = new LoggedMechanism2d(4, 4);
    root = mech.getRoot("Root", 2, 0.5);
    robotArm = root.append(new LoggedMechanismLigament2d("arm", 0.5, 90, 10, color));
  }

  public void setAngle(double angle_rad) {
    robotArm.setAngle(angle_rad);
  }

  public void periodic() {
    SmartDashboard.putData(name, mech);
    Logger.recordOutput(name, mech);    
  }
}

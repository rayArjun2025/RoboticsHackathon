package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Elevator2d {

  LoggedMechanism2d mech;
  LoggedMechanismRoot2d root;

  public LoggedMechanismLigament2d elev;

  Elevator elevator;

  public static Elevator2d instance;

  String name;

  public Elevator2d(String name, Color8Bit color) {
    this.name = name;
    mech = new LoggedMechanism2d(13, 20);
    root = mech.getRoot("Root", 2, 0.5);
    elev = root.append(new LoggedMechanismLigament2d("elevator", 33, 0, 15, color));
  }

  public void setHeight(double height) {
    elev.setLength(height);
  }

  public void setAngle(double angle_deg) {
    elev.setAngle(angle_deg);
  }

  public void periodic() {
    SmartDashboard.putData(name, mech);
    Logger.recordOutput(name, mech);    
  }
}
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedElevator extends SubsystemBase {
  private ElevatorSim model =
      new ElevatorSim(
          DCMotor.getNEO(2), // number of motors powering the elevator
          1.0, // gear reduction from motors to elevator
          Units.lbsToKilograms(15), // weight of the elevator's carriage that moves up/down, in kg
          Units.inchesToMeters(
              1), // diameter of the drum that winds the chain/rope to move the elevator, in
          // meters
          Units.inchesToMeters(0), // minimum height of the elevator, in meters
          Units.inchesToMeters(100) // maximum height of the elevator, in meters
          );


  // These 3 are for drawing an elevator visualization on Glass
  // Don't modify!
  Mechanism2d mechanismDrawing = new Mechanism2d(3, 3);
  MechanismRoot2d drawingRoot = mechanismDrawing.getRoot("root", 2, 0);
  MechanismLigament2d elevatorDrawing =
      drawingRoot.append(new MechanismLigament2d("Elevator", 0, 90));

  public SimulatedElevator() {}

  @Override
  public void periodic() {
    SmartDashboard.putData("Mechanism2d", mechanismDrawing);
  }
}

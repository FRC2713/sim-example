package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedElevator extends SubsystemBase {
  private final ElevatorSim model =
      new ElevatorSim(
          DCMotor.getNEO(2), // number of motors powering the elevator
          1.0, // gear reduction from motors to elevator
          Units.lbsToKilograms(15), // weight of the elevator's carriage that moves up/down, in kg
          Units.inchesToMeters(
              1), // diameter of the drum that winds the chain/rope to move the elevator, in
          // meters
          0, // minimum height of the elevator, in meters
          3 // maximum height of the elevator, in meters
          );
  private double targetHeightMeters = 0;

  // These 3 are for drawing an elevator visualization on Glass
  // Don't modify!
  Mechanism2d mechanismDrawing = new Mechanism2d(1, 3);
  MechanismRoot2d drawingRoot = mechanismDrawing.getRoot("root", 1, 0);
  MechanismLigament2d elevatorDrawing =
      drawingRoot.append(new MechanismLigament2d("Elevator", 0, 90));

  public SimulatedElevator() {}

  @Override
  public void periodic() {
    model.update(0.02);

    double ffOutput = 0;
    double pidOutput = 0;

    RoboRioSim.setVInVoltage(
        BatterySim.calculateLoadedBatteryVoltage(12, 0.015, model.getCurrentDrawAmps()));
    elevatorDrawing.setLength(model.getPositionMeters());
    SmartDashboard.putData("Mechanism2d", mechanismDrawing);
    SmartDashboard.putNumber("Elevator/Position", model.getPositionMeters());
    SmartDashboard.putNumber("Elevator/Velocity", model.getVelocityMetersPerSecond());
    SmartDashboard.putNumber("Elevator/Current Draw", model.getCurrentDrawAmps());
    SmartDashboard.putNumber("Elevator/Setpoint", targetHeightMeters);
    SmartDashboard.putNumber("Elevator/FF Output", ffOutput);
    SmartDashboard.putNumber("Elevator/PID Output", pidOutput);
    SmartDashboard.putNumber("Elevator/Total output", ffOutput + pidOutput);
    SmartDashboard.putNumber("Elevator/Error", targetHeightMeters - model.getPositionMeters());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }
}

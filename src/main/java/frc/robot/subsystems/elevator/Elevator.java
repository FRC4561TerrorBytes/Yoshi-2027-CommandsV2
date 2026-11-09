// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.command2.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert elevatorDisconnectedAlert;

  public Elevator(ElevatorIO io) {
    this.io = io;
    elevatorDisconnectedAlert = new Alert("Elevator", "Elevator motor disconnected", Alert.Level.HIGH);
  }

  public void setElevatorSetpoint(double elevatorSetpoint) {
    io.setElevatorSetpoint(elevatorSetpoint);
  }

  public double elevatorSetpoint() {
    return inputs.elevatorSetpoint;
  }

  public double getElevatorPositionInches() {
    return getPositionFromAngles(inputs.cancoder1Deg, inputs.cancoder2Deg);
  }

  public static double getPositionFromAngles(double deg1, double deg2) {
    deg1 = ((deg1 % 360.0) + 360.0) % 360.0;
    deg2 = ((deg2 % 360.0) + 360.0) % 360.0;

    int r1 = (int) ((deg1 / 360.0) * Constants.ELEVATORCANCODER1_TEETH);
    int r2 = (int) ((deg2 / 360.0) * Constants.ELEVATORCANCODER2_TEETH);

    if (r1 >= Constants.ELEVATORCANCODER1_TEETH) r1 = 0;
    if (r2 >= Constants.ELEVATORCANCODER2_TEETH) r2 = 0;

    int val1 = r1 * Constants.ELEVATORCANCODER1_KEY;
    int val2 = r2 * Constants.ELEVATORCANCODER2_KEY;

    int totalSteps = (val1 + val2) % Constants.ELEVATOR_MAX_STEPS;
    double inchesPerStep = Constants.ELEVATOR_MAX_RANGE / Constants.ELEVATOR_MAX_STEPS;

    return totalSteps * inchesPerStep;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Elevator/Io", inputs);

    elevatorDisconnectedAlert.set(!inputs.elevatorMotor1Connected || !inputs.elevatorMotor2Connected);

    Logger.recordOutput("Elevator/AbsolutePositionInches", getElevatorPositionInches());

    double absoluteInches = getPositionFromAngles(inputs.cancoder1Deg, inputs.cancoder2Deg);
    io.updateAbsolutePosition(absoluteInches);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static org.wpilib.units.Units.Rotations;

import java.util.logging.Level;

import org.opencv.utils.Converters;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.Alert;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
  private TalonFX elevatorMotor1 = new TalonFX(Constants.ELEVATORMOTOR1_ID, new CANBus(CANPort.CAN_S0));
  private TalonFX elevatorMotor2 = new TalonFX(Constants.ELEVATORMOTOR2_ID, new CANBus(CANPort.CAN_S0));
  private CANcoder elevatorEncoder1 = new CANcoder(Constants.ELEVATORCANCODER1_ID, new CANBus(CANPort.CAN_S0));
  private CANcoder elevatorEncoder2 = new CANcoder(Constants.ELEVATORCANCODER2_ID, new CANBus(CANPort.CAN_S0));

  private final StatusSignal<Current> elevator1StatorCurrent;
  private final StatusSignal<Current> elevator1SupplyCurrent;
  private final StatusSignal<AngularVelocity> elevator1Speed;
  private final StatusSignal<Voltage> elevator1Voltage;
  private final StatusSignal<Temperature> elevator1Temp;
  private final StatusSignal<Angle> cancoder1Position;
  private final StatusSignal<Angle> cancoder2Position;

  private final StatusSignal<Current> elevator2StatorCurrent;
  private final StatusSignal<Current> elevator2SupplyCurrent;
  private final StatusSignal<AngularVelocity> elevator2Speed;
  private final StatusSignal<Voltage> elevator2Voltage;
  private final StatusSignal<Temperature> elevator2Temp;

  private double elevatorSetpoint = 0.0;
  private final MotionMagicVoltage m_request_elevator = new MotionMagicVoltage(0);
  
  private final Alert elevator1Alert = new Alert("Elevator", "Elevator Motor 1 Disconnected.", Alert.Level.HIGH);
  private final Alert elevator1EncoderAlert = new Alert("Elevator", "Elevator Encoder 1 Disconnected.", Alert.Level.HIGH);
  private final Alert elevator2Alert = new Alert("Elevator", "Elevator Motor 2 Disconnected.", Alert.Level.HIGH);
  private final Alert elevator2EncoderAlert = new Alert("Elevator", "Elevator Encoder 2 Disconnected.", Alert.Level.HIGH);

  public ElevatorIOReal() {
    var ElevatorPIDConfig = new Slot0Configs();
    ElevatorPIDConfig.GravityType = GravityTypeValue.Elevator_Static;
    ElevatorPIDConfig.kV = 0;
    ElevatorPIDConfig.kA = 0;
    ElevatorPIDConfig.kP = 75; 
    ElevatorPIDConfig.kI = 0;
    ElevatorPIDConfig.kD = 0;

    var cancoderConfig1 = new CANcoderConfiguration();
    cancoderConfig1.MagnetSensor.withMagnetOffset(0);
    cancoderConfig1.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> elevatorEncoder1.getConfigurator().apply(cancoderConfig1, 0.25));

    var cancoderConfig2 = new CANcoderConfiguration();
    cancoderConfig2.MagnetSensor.withMagnetOffset(0);
    cancoderConfig2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> elevatorEncoder2.getConfigurator().apply(cancoderConfig2, 0.25));

    var elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.Slot0 = ElevatorPIDConfig;
    
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = elevatorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.050;
    // elevatorConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    // elevatorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    elevatorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = Constants.ELEVATOR_STATOR_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ELEVATOR_SUPPLY_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ELEVATOR_ROTATIONS_PER_INCH * Constants.ELEVATOR_MAX_RANGE /*tolerance ->*/ -10;
    
    tryUntilOk(5, () -> elevatorMotor1.getConfigurator().apply(elevatorConfig, 0.25));
    tryUntilOk(5, () -> elevatorMotor2.getConfigurator().apply(elevatorConfig, 0.25));

    cancoder1Position = elevatorEncoder1.getPosition();
    cancoder2Position = elevatorEncoder2.getPosition();

    elevator1StatorCurrent = elevatorMotor1.getStatorCurrent();
    elevator1SupplyCurrent = elevatorMotor1.getSupplyCurrent();
    elevator1Speed = elevatorMotor1.getVelocity();
    elevator1Voltage = elevatorMotor1.getMotorVoltage();
    elevator1Temp = elevatorMotor1.getDeviceTemp();

    elevator2StatorCurrent = elevatorMotor2.getStatorCurrent();
    elevator2SupplyCurrent = elevatorMotor2.getSupplyCurrent();
    elevator2Speed = elevatorMotor2.getVelocity();
    elevator2Voltage = elevatorMotor2.getMotorVoltage();
    elevator2Temp = elevatorMotor2.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        cancoder1Position,
        cancoder2Position,
        elevator1StatorCurrent,
        elevator1SupplyCurrent,
        elevator1Speed,
        elevator1Voltage,
        elevator1Temp,
        elevator2StatorCurrent,
        elevator2SupplyCurrent,
        elevator2Speed,
        elevator2Voltage,
        elevator2Temp
    );

    ParentDevice.optimizeBusUtilizationForAll(elevatorMotor1, elevatorMotor2);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        cancoder1Position, cancoder2Position,
        elevator1StatorCurrent, elevator1SupplyCurrent, elevator1Speed, elevator1Voltage, elevator1Temp,
        elevator2StatorCurrent, elevator2SupplyCurrent, elevator2Speed, elevator2Voltage, elevator2Temp
    );

    inputs.cancoder1Deg = Units.Degrees.convertFrom(cancoder1Position.getValueAsDouble(), Rotations);
    inputs.cancoder2Deg = Units.Degrees.convertFrom(cancoder2Position.getValueAsDouble(), Rotations);

    inputs.elevatorSetpoint = elevatorSetpoint;
    inputs.elevatorMotor1Connected = elevator1StatorCurrent.getStatus().isOK();
    inputs.elevatorMotor2Connected = elevator2StatorCurrent.getStatus().isOK();
    inputs.elevatorMotorConnected = inputs.elevatorMotor1Connected && inputs.elevatorMotor2Connected;

    elevator1Alert.set(!inputs.elevatorMotor1Connected);
    elevator2Alert.set(!inputs.elevatorMotor2Connected);
    elevator1EncoderAlert.set(!cancoder1Position.getStatus().isOK());
    elevator2EncoderAlert.set(!cancoder2Position.getStatus().isOK());
  }

  @Override
  public void updateAbsolutePosition(double absoluteInches) {

    double physicalRotations = absoluteInches * Constants.ELEVATOR_ROTATIONS_PER_INCH;

    elevatorMotor1.setPosition(physicalRotations);
    elevatorMotor2.setPosition(physicalRotations);
  }

  @Override
  public void setElevatorSetpoint(double positionInches) {
    this.elevatorSetpoint = positionInches;

    double targetRotations = positionInches * Constants.ELEVATOR_ROTATIONS_PER_INCH;
    
    elevatorMotor1.setControl(m_request_elevator.withPosition(targetRotations));
    elevatorMotor2.setControl(m_request_elevator.withPosition(targetRotations));
  }
}
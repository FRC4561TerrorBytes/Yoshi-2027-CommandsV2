package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPose = 0.0;
        public double elevatorSetpoint = 0.0;
        public double elevatorStatorCurrent = 0.0;
        public double elevatorSupplyCurrent = 0.0;
        public double elevatorVoltage = 0.0;
        public double elevatorMotorTemp = 0.0;
        public boolean elevatorMotor1Connected = false;
        public boolean elevatorEncoder18tConnected = false;
        public boolean elevatorMotor2Connected = false;
        public boolean elevatorEncoder2Connected = false;
        public boolean elevatorMotorConnected = false;
        public double cancoder1Deg = 0.0;
        public double cancoder2Deg = 0.0;
        public double elevatorSpeed = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorSetpoint(double position) {}

    public default void updateAbsolutePosition(double absoluteInches) {}
    
} 
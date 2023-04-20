// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  /** Creates a new ElevatorTest. */

  private final TalonFX elevatorMotorOne;
  private final TalonFX elevatorMotorTwo;
  // final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */

  private static final double k_openLoopRampRate = 0.1;
  private static final int k_currentLimit = Constants.Elevator.currentLimit; // Current limit for intake falcon 500

  private double m_encoder = 0;
  private double m_goalPosition;

  public ElevatorSub() {

    // initialize motors
    // the right motor will spin clockwise and the left motor will go counter
    // clockwise
    elevatorMotorOne = new TalonFX(Constants.Elevator.motorOneId);
    elevatorMotorTwo = new TalonFX(Constants.Elevator.motorTwoId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitEnable = true;
    config.forwardSoftLimitThreshold = Constants.Elevator.maxExtension;
    config.reverseSoftLimitThreshold = Constants.Elevator.maxExtension;
    config.voltageCompSaturation = 12.0;
    config.openloopRamp = k_openLoopRampRate;
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, k_currentLimit, 0, 0);

    elevatorMotorOne.configAllSettings(config);
    elevatorMotorOne.enableVoltageCompensation(true);
    elevatorMotorOne.setNeutralMode(NeutralMode.Brake);
    elevatorMotorOne.setInverted(TalonFXInvertType.CounterClockwise);
    elevatorMotorOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elevatorMotorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    // elevatorMotorOne.setSelectedSensorPosition(0); // zero the encoder

    elevatorMotorTwo.configAllSettings(config);
    elevatorMotorTwo.enableVoltageCompensation(true);
    elevatorMotorTwo.setNeutralMode(NeutralMode.Brake);
    elevatorMotorTwo.setInverted(TalonFXInvertType.CounterClockwise);
    elevatorMotorTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    /* Config the sensor used for Primary PID and sensor direction */
    elevatorMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Elevator.kPIDLoopIdx,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Elevator.kPIDLoopIdx,
        Constants.Elevator.kTimeoutMs);

    elevatorMotorOne.setSensorPhase(Constants.Elevator.kSensorPhase);
    elevatorMotorTwo.setSensorPhase(Constants.Elevator.kSensorPhase);

    elevatorMotorOne.configAllowableClosedloopError(0, Constants.Elevator.kElevatorAllowableError,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorTwo.configAllowableClosedloopError(0, Constants.Elevator.kElevatorAllowableError,
        Constants.Elevator.kTimeoutMs);
    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    // elevatorMotorOne.config_kF(Constants.Elevator.kPIDLoopIdx,
    // Constants.Elevator. kGains.kF, Constants.Elevator.kTimeoutMs);
    elevatorMotorOne.config_kP(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKP,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorOne.config_kI(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKI,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorOne.config_kD(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKD,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorOne.config_kF(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKF,
        Constants.Elevator.kTimeoutMs);

    elevatorMotorTwo.config_kP(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKP,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorTwo.config_kI(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKI,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorTwo.config_kD(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKD,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorTwo.config_kF(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKF,
        Constants.Elevator.kTimeoutMs);

    //elevatorMotorOne.setSelectedSensorPosition(m_encoder);
    //elevatorMotorTwo.setSelectedSensorPosition();
    m_encoder = elevatorMotorOne.getSelectedSensorPosition(); // * 1.0 / 360.0 * 2.0 * Math.PI * 1.5;
    elevatorMotorTwo.follow(elevatorMotorOne);
  }

  public void setPosition(double goalPosition) {

    
    m_goalPosition = goalPosition;
  }

  public void joystickPosition(double joystickPosition) {
    m_goalPosition = m_goalPosition + joystickPosition;
  }

  public double ElevatorPosition() {
    return elevatorMotorOne.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_encoder = elevatorMotorOne.getSelectedSensorPosition();// * (1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    if (m_goalPosition > Constants.Elevator.maxExtension) {
      m_goalPosition = Constants.Elevator.maxExtension;
    }
    else if (m_goalPosition < Constants.Elevator.minExtension){
      m_goalPosition= Constants.Elevator.minExtension;
    }
    SmartDashboard.putNumber("Elevator Position", m_encoder);
    SmartDashboard.putNumber("Elevator Goal Position", m_goalPosition);
   
    // elevatorMotorOne.set(ControlMode.Position,
    // m_controller.calculate(m_encoder));
    elevatorMotorOne.set(TalonFXControlMode.Position, m_goalPosition);//, DemandType.ArbitraryFeedForward,        Constants.Elevator.elevatorKF);
    // elevatorMotorTwo.set(TalonFXControlMode.Position, m_goalPosition);
        SmartDashboard.putNumber("Elevator MotorOne Percentage", elevatorMotorOne.getMotorOutputPercent());
       SmartDashboard.putNumber("Elevator MotorTwo Percentage", elevatorMotorTwo.getMotorOutputPercent());
    // DemandType.ArbitraryFeedForward,
    // Constants.Elevator.kF);

  }

  public boolean atSetpoint() {//this might keep autonomous from finishing!!!!!!
    if (ElevatorPosition() > m_goalPosition - Constants.Elevator.kElevatorDeadband
        || ElevatorPosition() <m_goalPosition +Constants.Elevator.kElevatorDeadband) {
      return true;
    } else {
      return false;
    }

  }
}

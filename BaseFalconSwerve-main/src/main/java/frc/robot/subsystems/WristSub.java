// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class WristSub extends SubsystemBase {
    /**
     * Creates a new ElevatorTest.
     */
    //private WPI_CANCoder m_CANCoder;
    private final TalonFX wristMotor;

    private static WristSub INSTANCE;


    // final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */

    private static final double k_openLoopRampRate = 0.1;
    private static final int k_currentLimit = Constants.Wrist.currentLimit; // Current limit for intake falcon 500

    private double m_encoder = 0;
    private double m_goalPosition;

    public static WristSub getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new WristSub();
        }
        return INSTANCE;
    }

    public WristSub() {

        // initialize motors
        // the right motor will spin clockwise and the left motor will go counter
        // clockwise
        wristMotor = new TalonFX(Constants.Wrist.wristMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.openloopRamp = k_openLoopRampRate;
        config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, k_currentLimit, 0, 0);

        wristMotor.configAllSettings(config);
        wristMotor.enableVoltageCompensation(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(TalonFXInvertType.CounterClockwise);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.Wrist.kPIDLoopIdx,
                Constants.Wrist.kTimeoutMs);
        wristMotor.setSensorPhase(Constants.Wrist.kSensorPhase);

        wristMotor.configAllowableClosedloopError(0, Constants.Wrist.kWristDeadband, Constants.Wrist.kTimeoutMs);
        /* Config Position Closed Loop gains in slot0, kF stays zero when in position mode. */
        wristMotor.config_kP(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKP, Constants.Wrist.kTimeoutMs);
        wristMotor.config_kI(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKI, Constants.Wrist.kTimeoutMs);
        wristMotor.config_kD(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKD, Constants.Wrist.kTimeoutMs);
        wristMotor.config_kF(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKF, Constants.Wrist.kTimeoutMs);

        wristMotor.setSelectedSensorPosition(0);
        m_encoder = wristMotor.getSelectedSensorPosition(); // * 1.0 / 360.0 * 2.0 * Math.PI * 1.5;

    }

    public void setPosition(double goalPosition) {
        m_goalPosition = goalPosition;
    }

    public void joystickPosition(double joystickPosition) {
        m_goalPosition = m_goalPosition + joystickPosition;
    }

    public double getWristPosition() {
        return m_encoder;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (RobotBase.isReal()) {
            m_encoder = wristMotor.getSelectedSensorPosition();
        } else {
            m_encoder = m_goalPosition; // motor immediately gets to its desired position if not real
        }

        // setting limits - if over or under, go to limit
        if (m_goalPosition > Constants.Wrist.maxExtension) {
            m_goalPosition = Constants.Wrist.maxExtension;
        } else if (m_goalPosition < Constants.Wrist.minExtension) {
            m_goalPosition = Constants.Wrist.minExtension;
        }

        // set motor
        wristMotor.set(TalonFXControlMode.Position, m_goalPosition);

        // update shuffleboard
        SmartDashboard.putNumber("Wrist Position", m_encoder);
        SmartDashboard.putNumber("Wrist Goal Position", m_goalPosition);
        SmartDashboard.putNumber("Wrist Motor Output", wristMotor.getMotorOutputPercent());
    }

    public boolean atSetpoint() {
        return getWristPosition() <= m_goalPosition + Constants.Wrist.kWristAllowableRange
                && getWristPosition() >= m_goalPosition - Constants.Wrist.kWristAllowableRange;
    }
}

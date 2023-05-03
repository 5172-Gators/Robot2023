package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The intake subsysatem will be used to set up the motors and encoders for the
 * intake.
 */
public class IntakeSub extends SubsystemBase {

    // Constants

    private static final double k_openLoopRampRate = 0.1;
    // private static final int k_currentLimit = 30; // Current limit for intake falcon 500

    // Components
    private TalonFX m_intakeMotor;
    private TalonFX m_intakeMotorFollower;

    // private final CANSparkMax intakeMotor;
    //  private final double m_intakeEncoder;

    /* Game Piece Currently In Robot */

    /**
     * Constructor for intake subsystem.
     */

    private static IntakeSub INSTANCE;

    public static IntakeSub getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSub();
        }
        return INSTANCE;
    }


    private IntakeSub() {
        m_intakeMotor = new TalonFX(Constants.Intake.mainMotorId);
        m_intakeMotorFollower = new TalonFX(Constants.Intake.followerMotorId);
        // Configure all settings on Talons
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.openloopRamp = k_openLoopRampRate;
        config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, Constants.Intake.currentLimit, 0, 0);
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, Constants.Intake.currentLimit, 0, 0);


        m_intakeMotor.configAllSettings(config);
        m_intakeMotor.enableVoltageCompensation(true);
        m_intakeMotor.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor.setInverted(TalonFXInvertType.Clockwise);
        m_intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        m_intakeMotorFollower.configAllSettings(config);
        m_intakeMotorFollower.enableVoltageCompensation(true);
        m_intakeMotorFollower.setNeutralMode(NeutralMode.Brake);
        m_intakeMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);
        m_intakeMotorFollower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        m_intakeMotorFollower.follow(m_intakeMotor);
    }
    /*
     * public void setGamePiece(GamePiece piece) {
     * gamePiece = piece;
     * }
     *
     * public GamePiece getGamePiece() {
     * return gamePiece;
     * }
     */

    public void setMotor(double speed) {
        m_intakeMotor.set(TalonFXControlMode.PercentOutput, speed); // setVoltage(speed);
    }

    public double getPDMCurrent() {
        return m_intakeMotor.getStatorCurrent();
    }

    public double getVelocity() {
        return m_intakeMotor.getSelectedSensorVelocity();
    }

    public void stopIntake() {
        m_intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        // returns in amps
        // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel);
        double intakeCurrent = m_intakeMotor.getStatorCurrent();
        SmartDashboard.putNumber("Intake Current", intakeCurrent);
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
        // SmartDashboard.putNumber("Gamepiece", getGamePiece().getDirection());

    }


    public void resetIntakeEncoder() {
        m_intakeMotor.setSelectedSensorPosition(0);
        // m_intakeEncoder= m_intakeMotor.getSelectedSensorPosition();

    }

}
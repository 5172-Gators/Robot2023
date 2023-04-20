// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class LED extends SubsystemBase {
    // Components
    private final CANifier m_canifier;

    // Pre-tuned colors
    public static final Color k_notReady = Color.kOrange;
    public static final Color k_targetInSight = Color.kCyan;
    public static final Color k_ready = Color.kBlue;

    // LED states
    private enum LEDState {
        OFF, FLASHING, STEADY
    }

    // State variables
    private LEDState m_state;
    private Color m_currentColor;
    private Color m_flashColor;
    private static final int k_flashHalfPeriodCycles = 5; // cycles = 100 ms
    private int m_flashCounter = 0;

    /** Creates a new LED. */
    public LED() {
        // Initialize canifier
        m_canifier = new CANifier(RobotMap.CANIFIER_CAN);

        // Configure status frames to reduce can bus congestion
        m_canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.LONG_CAN_TIMEOUT_MS);
        m_canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 255, Constants.LONG_CAN_TIMEOUT_MS);
        m_canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 255, Constants.LONG_CAN_TIMEOUT_MS);
        m_canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 255, Constants.LONG_CAN_TIMEOUT_MS);
        m_canifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 255, Constants.LONG_CAN_TIMEOUT_MS);
        m_canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 255, Constants.LONG_CAN_TIMEOUT_MS);

        // Configure 

        // Initially turn LEDs off
        m_currentColor = new Color(0.1, 0.1, 0.1);
        off();
    }
 
    public void setOnSteady(Color color) {
        m_state = LEDState.STEADY;
        setColor(color);
    }

    public void setOnFlashing(Color color) {
        if (m_state != LEDState.FLASHING) {
            m_state = LEDState.FLASHING;
            m_flashCounter = 0;
            setColor(color);
        }
        m_flashColor = color;
    }

    public void off() {
        m_state = LEDState.OFF;
        setColor(Color.kBlack);
    }

    public Color getColor() {
        return m_currentColor;
    }

    @Override
    public void periodic() {
        if (m_state == LEDState.FLASHING) {
            // toggle LED on/off if time is up
            if (++m_flashCounter == k_flashHalfPeriodCycles) {
                // If color is on, set to off. if off, set to on.
                Color color = (!getColor().equals(Color.kBlack)) ? Color.kBlack : m_flashColor;
                setColor(color);
                m_flashCounter = 0;
            }
        }
    }

    private void setColor(Color color) {
        // If new color is different than previous color, update it
        if (!color.equals(m_currentColor)) {
            m_canifier.setLEDOutput(color.red, CANifier.LEDChannel.LEDChannelB);
            m_canifier.setLEDOutput(color.green, CANifier.LEDChannel.LEDChannelA);
            m_canifier.setLEDOutput(color.blue, CANifier.LEDChannel.LEDChannelC);
            m_currentColor = color;
        }
    }
}

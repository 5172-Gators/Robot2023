// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ResetDriveOdometry extends CommandBase {
    /** Creates a new TeleopElevatorTest. */
    private Swerve s_Swerve;
    private Pose2d pose2d;

    public ResetDriveOdometry(Pose2d pose2d, Swerve s_Swerve) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.s_Swerve = s_Swerve;
        this.pose2d = pose2d;
        addRequirements(s_Swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        s_Swerve.resetOdometry(pose2d);
        System.out.println("resetting drive odometry...");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Swerve.resetOdometry(pose2d);
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("drive odometry reset.");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}




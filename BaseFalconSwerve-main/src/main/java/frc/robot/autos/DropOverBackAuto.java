package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.SetAllPositions;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Intake.IntakeOn;
import frc.robot.commands.Intake.intakeStop;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.WristSub;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import frc.robot.subsystems.ShoulderSub;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropOverBackAuto extends SequentialCommandGroup {
    private final IntakeSub s_Intake = IntakeSub.getInstance();
    private final ElevatorSub s_Elevator =ElevatorSub.getInstance();//new ElevatorSub();
    private final WristSub s_Wrist = WristSub.getInstance();
    private final ShoulderSub s_Shoulder = ShoulderSub.getInstance();


    public DropOverBackAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory OverTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0), new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4.1, 0, new Rotation2d(0)),
                config);

                




                
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        CommandBase swerveControllerCommand = new AutoBalance(s_Swerve);

        CommandBase backOut = 
        new SwerveControllerCommand(
                OverTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
        
            // wait to make sure there's no code running in the background
            new WaitCommand(0.5),

            // set position
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.OUTAKEAUTO, () -> GamePiece.CUBE),

            //outtake cube
            new IntakeOn(s_Intake,false),

            // finish outtaking before stowing + stopping
            new WaitCommand(1),
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () -> GamePiece.CUBE),
            new intakeStop(s_Intake),

            // wait before backing out
            new WaitCommand(0.5),

            // start backing out over charge station
            backOut,

            // wait before starting again
            new WaitCommand(5),
            
            // autobalance
            swerveControllerCommand


        );
    }
}
package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.SetAllPositions;
import frc.robot.commands.Drive.DriveThenAutoBalance;
import frc.robot.commands.Intake.IntakeOn;
import frc.robot.commands.Intake.intakeStop;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.WristSub;
import frc.robot.subsystems.ShoulderSub;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;

//import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedPlaceCubeAutoBalance extends SequentialCommandGroup {
    private final IntakeSub s_Intake = IntakeSub.getInstance();
    private final ElevatorSub s_Elevator = ElevatorSub.getInstance();//new ElevatorSub();
    private final WristSub s_Wrist = WristSub.getInstance();
    private final ShoulderSub s_Shoulder = ShoulderSub.getInstance();

    public TimedPlaceCubeAutoBalance(Swerve s_Swerve){
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        CommandBase swerveControllerCommand = new DriveThenAutoBalance(s_Swerve);

        addCommands(
        
            // wait 0.5 seconds to make sure no code is running in the background
            new WaitCommand(0.5),

            // set position and start outaking
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.OUTAKEAUTO, () -> GamePiece.CUBE),
            new IntakeOn(s_Intake,false),

            // wait for cube to outake
            new WaitCommand(0.3),
            
            // set position to stowed and stop the intake
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () -> GamePiece.CUBE),
            new intakeStop(s_Intake),
    
            // autobalance
            swerveControllerCommand
        );
    }
}
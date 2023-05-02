package frc.robot;

import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;

/* Shuffleboard */
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/* Autos */
import frc.robot.autos.*;
import frc.robot.commands.SetAllPositions;

/* Commands */
import frc.robot.commands.Drive.TeleopSwerve;
//import frc.robot.commands.Elevator.ElevatorSetPositionHigh;
import frc.robot.commands.Elevator.TeleopElevator;
import frc.robot.commands.Wrist.TeleopWrist;
//import frc.robot.commands.Wrist.WristSetPosition;
import frc.robot.commands.Shoulder.TeleopShoulder;

/* Subsystems */
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static GamePiece gamePiece = GamePiece.CONE;

    /* Auto Selector */
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;

    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    /* Controllers */
    private final static Joystick translateStick = new Joystick(0);
    private final static Joystick rotateStick = new Joystick(1);
    private final static Joystick operatorStick = new Joystick(2);

    /* Drive Controls */

    private final int j_translationAxis = Joystick.AxisType.kY.value;
    private final int j_strafeAxis = Joystick.AxisType.kX.value;
    private final int j_rotationAxis = Joystick.AxisType.kX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(rotateStick, 2);
    private final JoystickButton robotCentric = new JoystickButton(operatorStick, 1);

    // Right Stick Buttons
    private static final JoystickButton selectGamepieceTrigger = new JoystickButton(rotateStick, 1);
    private static final JoystickButton resetGyroButton = new JoystickButton(rotateStick, 2);

    // Operator Controls
    private static final int elevatorAxis = Joystick.AxisType.kY.value;
    private static final int wristAxis = Joystick.AxisType.kX.value;

    //private static final JoystickButton stopIntake = new JoystickButton(operatorStick, 1);
    private static final JoystickButton stopIntake = new JoystickButton(operatorStick, 2);
    private static final JoystickButton coneInCubeOutButton = new JoystickButton(operatorStick, 3);
    private static final JoystickButton cubeInConeOutButton = new JoystickButton(operatorStick, 4);
    private static final JoystickButton pickHumanPlayerButton = new JoystickButton(operatorStick, 5);
    private static final JoystickButton pickTippedConeButton = new JoystickButton(operatorStick, 7);
    //private static final JoystickButton midCubeScoreButton = new JoystickButton(operatorStick, 7);
    private static final JoystickButton pickCubeButton = new JoystickButton(operatorStick, 8);
    private static final JoystickButton stowIntakeButton = new JoystickButton(operatorStick, 9);
    private static final JoystickButton placeHighButton = new JoystickButton(operatorStick, 10);
    private static final JoystickButton placeMidButton = new JoystickButton(operatorStick, 6);
    private static final JoystickButton pickDoubleSub = new JoystickButton(operatorStick, 16);
    //private static final JoystickButton placeLowButton = new JoystickButton(operatorStick, 12);

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final IntakeSub s_Intake = IntakeSub.getInstance();
    private final WristSub s_Wrist = WristSub.getInstance();
    private final ElevatorSub s_Elevator = ElevatorSub.getInstance();
    private final ShoulderSub s_Shoulder = ShoulderSub.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -translateStick.getRawAxis(j_translationAxis),
                        () -> -translateStick.getRawAxis(j_strafeAxis),
                        () -> -rotateStick.getRawAxis(j_rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        s_Elevator.setDefaultCommand(
            new TeleopElevator(
                s_Elevator,
                () -> -operatorStick.getY()));

        s_Shoulder.setDefaultCommand(
            new TeleopShoulder(
                s_Shoulder,
                () -> operatorStick.getX() * 1000));

        s_Wrist.setDefaultCommand(new TeleopWrist(
                s_Wrist,
                () -> (Math.abs(operatorStick.getTwist()) > 0.2) ? operatorStick.getTwist() * 1000 : 0));

        // Configure the button bindings
         configureButtonBindings();

        autoChooser.setDefaultOption("Do Nothing", null);
        autoChooser.addOption("Place Cube Low + Auto Balance", new TimedPlaceCubeAutoBalance(s_Swerve));
        autoChooser.addOption("High Cube + Exit Community + Auto Balance", new TrajectoryPlaceCubeExitCommunityAutoBalance(s_Swerve));
        // autoChooser.addOption("Middle Auto", new middleAuto(s_Swerve, s_Elevator, s_Shoulder, s_Wrist, s_Intake));
        // autoChooser.addOption("Left Or Right Auto", new sideAuto(s_Swerve, s_Elevator, s_Shoulder, s_Wrist, s_Intake));

        SmartDashboard.putData(autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it t o a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        // Translate Stick

        // intakeTrigger.onTrue(new InstantCommand(() -> s_Intake.setMotor(1))); // button 1
        // outtakeButton.onTrue(new InstantCommand(() -> s_Intake.setMotor(-1))); // button 2
        // selectConeButton.onTrue(new InstantCommand(()->
        // setGamePiece(GamePiece.CONE))); // button 3
        // selectCubeButton.onTrue(new InstantCommand(()->
        // setGamePiece(GamePiece.CUBE))); // button 4

        // Rotate Stick

        // selectGamepieceTrigger.onTrue(new InstantCommand(() ->
        // setGamePiece(GamePiece.CONE)));
        zeroGyro.onTrue(new InstantCommand(() ->
                s_Swerve.resetOdometry(
                        DriverStation.getAlliance() == DriverStation.Alliance.Blue ? new Pose2d(): new Pose2d(0,0, Rotation2d.fromDegrees(180))
                ))
        ); // button 2

        /* Operator Buttons */

        //stopIntake.onTrue(new Intake);// button 2

       //// coneInCubeOutButton.whileTrue(new IntakeOn(s_Intake)); // button 3

        //cubeInConeOutButton.whileTrue(new InstantCommand(() -> s_Intake.setMotor(0.75)));// button 4

        stopIntake.onTrue(new InstantCommand(() -> s_Intake.setMotor(0)));// button 2

        coneInCubeOutButton.whileTrue(new InstantCommand(() -> s_Intake.setMotor(-0.60))); // button 3

        cubeInConeOutButton.whileTrue(new InstantCommand(() -> s_Intake.setMotor(0.60)));// button 4





        pickHumanPlayerButton.onTrue(new SequentialCommandGroup( // button 5
        new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder,
        Position.HUMANPLAYERINTAKE, () -> GamePiece.CONE)));

        // pickStandingConeButton.onTrue(new SequentialCommandGroup( // button 6
        // new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder,
        // Position.STANDINGCONEINTAKE, () -> GamePiece.CONE)));

        pickTippedConeButton.onTrue(new SequentialCommandGroup( // button 7
        new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder,
        Position.TIPPEDCONEINTAKE, () -> GamePiece.CONE)));

        placeMidButton.onTrue(new SequentialCommandGroup( // button 6
                new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
                new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder,
                        Position.CUBEMID, () -> GamePiece.CUBE)));

        pickCubeButton.onTrue(new SequentialCommandGroup( // button 8
        new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.CUBEINTAKE, ()
        -> GamePiece.CUBE)));


        stowIntakeButton.onTrue(new SequentialCommandGroup( // button 9
        new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.HUMANPLAYERINTAKE, ()
        -> GamePiece.CUBE)).
        andThen(new WaitCommand(0.5)).
        andThen(new SequentialCommandGroup(
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () ->
        GamePiece.CONE))));

        //stowIntakeButton.onTrue(new SequentialCommandGroup( // button 9
        //new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () ->
        //GamePiece.CONE)));

        placeHighButton.onTrue(new SequentialCommandGroup( // button 10
        new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.CONEHIGH, () ->
        GamePiece.CUBE)));
       // andThen(new WaitCommand(1.00)).
        //andThen(new SequentialCommandGroup(
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.CONEHIGH, () ->
        // GamePiece.CONE))));
        //trying to smoothen transistion 
        

        placeMidButton.onTrue(new SequentialCommandGroup( // button 11
        new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.CONEMID, () ->
        GamePiece.CUBE)));


        
        pickDoubleSub.onTrue(new SequentialCommandGroup( // button 16
        new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
        new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.DOUBLESUBSTATION, () ->
        GamePiece.CONE)));



        // placeLowButton.onTrue(new SequentialCommandGroup( // button 12
        // new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.LOW, () ->
        // GamePiece.CUBE)));

        // not sure what place low is calling, so it is commented out

    }

    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece piece) {
        gamePiece = piece;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
       //TrajectoryPlaceCubeExitCommunityAutoBalance auto = new TrajectoryPlaceCubeExitCommunityAutoBalance(s_Swerve);
    //    TimedPlaceCubeAutoBalance auto  = new TimedPlaceCubeAutoBalance(s_Swerve);
       // s_Swerve.resetOdometry(auto.getInitialTrajectoryPose());
        // return auto;
    }
}

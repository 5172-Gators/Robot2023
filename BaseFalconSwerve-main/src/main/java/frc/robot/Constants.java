package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int LONG_CAN_TIMEOUT_MS = 250;

 

    public static final class Swerve {
        public static final int pigeonID = 14;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.5); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelBase = Units.inchesToMeters(18.5); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(244.42);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(328.53);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(221.66);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(255.84);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Intake {
        public static final int motorId = 53;

        public static final int pdpChannel = 2; // update number later

        public static final double stoppedRPMThreshold = .01;

        public static final double coneIntakeSpeed = .75;
        public static final double cubeIntakeSpeed = -.75;

        public static final double coneOuttakeSpeed = 4;
        public static final double coneShootSpeed = 12;
        public static final double cubeOuttakeSpeed = 7;

        public static final int currentLimit = 60;

        public enum EjectSpeed {
            FAST(12),
            NORMAL(7);

            public final double speed;

            EjectSpeed(double speed) {
                this.speed = speed;
            }
        }
    }

    public static final class Elevator {
        public static final int motorOneId = 41;
        public static final int motorTwoId = 42;
        public static final int currentLimit = 60;
       // public static final double maxMotorVoltage = 1;

        public static final int kRisingSlotIdx = 0;
        public static final int kFallingSlotIdx = 1;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;

        public static final double elevatorKP = .4;
        public static final double fallingElevatorKP = .2;
        public static final double elevatorKI = 0.0;
        public static final double elevatorKD = .5;
        //public static final double kF = 0.02;
        public static final double elevatorKF = 0;
		public static final double minExtension = 0;
        
        public static final double maxExtension = 58000.0;

        public static boolean kSensorPhase = true;
    	public static boolean kMotorInvert = true;
        public static final double kElevatorAllowableError = 50.0; // used for allowable error configuration
        public static final double kElevatorDeadband = 1000.0; // used to check if elevator at position
        public static final double elevatorMaxOutput = .2;
        public static final double elevatorDownOutput = -.2;
        public static final double elevatorHoldOutput = 0.02;
        //static final Gains kGains = new Gains(0.05, 0.0000, 6.0, 0.0, 0, .5);
    }

    public static final class Wrist {
        public static final int wristMotorID = 22;
        public static final int currentLimit = 30;
        public static final double maxMotorVoltage = 12;
        public static final int EncoderID = 16;

        /* limits in motor ticks */
        public static final double lowerLimit = 387;
        public static final double upperLimit = 448;

        /* CANCoder limits */
        public static final double CANCoderLowerLimit = 74.268;
        public static final double CANCoderUpperLimit = 282.393;

        public enum PIDFFmode {
            WEIGHTED(
                    Wrist.weightedP,
                    Wrist.weightedI,
                    Wrist.weightedD,
                    Wrist.weightedS,
                    Wrist.weightedV,
                    Wrist.weightedA,
                    Wrist.weightedG),
            UNWEIGHTED(
                    Wrist.unweightedP,
                    Wrist.unweightedI,
                    Wrist.unweightedD,
                    Wrist.unweightedS,
                    Wrist.unweightedV,
                    Wrist.unweightedA,
                    Wrist.unweightedG);

            public final double kP;
            public final double kI;
            public final double kD;
            public final double kS;
            public final double kV;
            public final double kA;
            public final double kG;

            private PIDFFmode(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
                this.kS = kS;
                this.kV = kV;
                this.kA = kA;
                this.kG = kG;
            }

        }

        public static double weightedP = 2.8;
        public static double weightedI = 0.0;
        public static double weightedD = 0.2;

        public static double weightedS = 0.4361;
        public static double weightedV = 0.79036;
        public static double weightedA = 0.0;
        public static double weightedG = 0.86416;

        public static double unweightedP = 2.2;
        public static double unweightedI = 0.0;
        public static double unweightedD = 0.2;

        public static double unweightedS = 0.11237;
        public static double unweightedV = 0.56387;
        public static double unweightedA = 0.041488;
        public static double unweightedG = 0.76416;
        

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;

        public static final double wristKP = .15;
        public static final double wristKI = 0.0;
        public static final double wristKD = 0.0;
        public static final double wristKF = 0;


        public static final double motorGearRatio = 1 / 15.0;
        public static final double absoluteEncoderOffset = 5.412927;
        public static final double kWristDeadband = 50;
        public static final int kWristCancoderID = 0;
        public static final boolean kSensorPhase = true;
        public static final double minAngle = -70000; // ticks
        public static final double maxAngle = 0; // ticks

    }

    public enum Position {

        HIGH(21051, Elevator.maxExtension, -7328),
        CONEHIGH(-45735, 57894, -17061),
        CUBEHIGH(-20000, Elevator.maxExtension, -15000),
        MID(0, 0, 0),
        LOW(-56672, Elevator.maxExtension, -17555),
        CONEMID(-58632, 56913,-16840),
        CUBEMID(-11496, 45900, -562),
        CUBEINTAKE(-2897, 2900, 60),
        //STANDINGCONEINTAKE(5.106, 14.380, 0),
        //TIPPEDCONEINTAKE(5.572, 1.333, 0),
        HUMANPLAYERINTAKE(31970, 59, -17733),
        STOWED(500, 100, 1000);
        

        private double wristPos;
        private double elevatorPos;
        private double shoulderPos;

        Position(double wrist, double elev, double shld) {
            this.wristPos = wrist;
            this.elevatorPos = elev;
            this.shoulderPos = shld;
        }

        public double getWrist() {
            return wristPos;
        }

        public double getElev() {
            return elevatorPos;
        }
        public double getShoulder (){
            return shoulderPos;
        }

    }
    public enum GamePiece {
        CUBE(1),
        CONE(-1);

        private double direction;

        GamePiece(double value) {
            direction = value;
        }

        public double getDirection() {
            return direction;
        }
    }

    public static final class Shoulder{
        public static final int ShoulderMotorID = 52;
        // public static final int currentLimit = 20;
        // public static final double maxMotorVoltage = 12;
        public static final int EncoderID = 15;

        /* limits in motor ticks (abs value) */
        public static final double upperLimit = -725; 
        public static final double lowerLimit = -15800; 

        /* CANCoder limits */
        public static final double CANCoderUpperLimit = 285.117;
        public static final double CANCoderLowerLimit = 310.0;
       
        public enum PIDFFmode {
            WEIGHTED(
                    Wrist.weightedP,
                    Wrist.weightedI,
                    Wrist.weightedD,
                    Wrist.weightedS,
                    Wrist.weightedV,
                    Wrist.weightedA,
                    Wrist.weightedG),
            UNWEIGHTED(
                    Wrist.unweightedP,
                    Wrist.unweightedI,
                    Wrist.unweightedD,
                    Wrist.unweightedS,
                    Wrist.unweightedV,
                    Wrist.unweightedA,
                    Wrist.unweightedG);

            public final double kP;
            public final double kI;
            public final double kD;
            public final double kS;
            public final double kV;
            public final double kA;
            public final double kG;

            private PIDFFmode(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
                this.kS = kS;
                this.kV = kV;
                this.kA = kA;
                this.kG = kG;
            }

        

        }

        // public static double weightedP = 2.0;
        // public static double weightedI = 0.0;
        // public static double weightedD = 0.0;

        // public static double weightedS = 0.4361;
        // public static double weightedV = 0.79036;
        // public static double weightedA = 0.0;
        // public static double weightedG = 0.86416;

        // public static double unweightedP = 0.0;
        // public static double unweightedI = 0.0;
        // public static double unweightedD = 0.0;

        // public static double unweightedS = 0.11237;
        // public static double unweightedV = 0.56387;
        // public static double unweightedA = 0.041488;
        // public static double unweightedG = 0.76416;
        public static final int currentLimit = 20;
        public static final double maxMotorVoltage = 10;
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;

        public static final double shoulderKP = .7;
        public static final double shoulderKI = 0.0;
        public static final double shoulderKD = 0.0;
        public static final double shoulderkF = 0.0;

        public static boolean kSensorPhase = true;
        public static double kAllowableError=500;

        public static final double motorGearRatio = 1 / 60.0;
        public static final double absoluteEncoderOffset = 5.412927;
        public static final double ShoulderKF = 0;
        public static final double kShoulderAllowableError = 50;

        public static final double kShoulderDeadband = 1000;
        public static final double maxAngle = 0;
        public static final double minAngle = -24305;

        
    }
}

    
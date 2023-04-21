package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
//import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Wrist.PIDFFmode;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.WristSub;

public class IntakeOn extends CommandBase {
    private IntakeSub s_Intake;
    private boolean intaking;

    public IntakeOn(IntakeSub s_Intake, boolean intaking) {
        this.s_Intake = s_Intake;
        this.intaking=intaking;
        addRequirements(s_Intake);
    }

    @Override
    public void execute() {
        s_Intake.setMotor(intaking ? 0.40 : -0.40);
        
    }

    public void end(boolean Interrupted){
    }

    @Override
    public boolean isFinished() {

      return true;
    }
    
}
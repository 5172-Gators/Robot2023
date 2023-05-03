package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;

public class IntakeOn extends CommandBase {
    private IntakeSub s_Intake;
    private boolean intaking;

    public IntakeOn(IntakeSub s_Intake, boolean intaking) {
        this.s_Intake = s_Intake;
        this.intaking=intaking;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setMotor(intaking ? Constants.Intake.kIntakeOnSpeed : Constants.Intake.kIntakeOutSpeed);
    }

    @Override
    public void execute() {
        s_Intake.setMotor(intaking ? Constants.Intake.kIntakeOnSpeed : Constants.Intake.kIntakeOutSpeed);
    }

    public void end(boolean Interrupted){
    }

    @Override
    public boolean isFinished() {

      return true;
    }
    
}
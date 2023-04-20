// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.ShoulderSub;
public class ShoulderSetPosition extends CommandBase {
  /* Creates a new shoulder */
  
  private ShoulderSub s_Shoulder;

  public ShoulderSetPosition(ShoulderSub s_Shoulder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shoulder = s_Shoulder;

    addRequirements(s_Shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
      
      s_Shoulder.setPosition(10); 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(s_Shoulder.ShoulderPosition() == 10)//< 300 + 100  && s_Shoulder.ShoulderPosition() > 300 - 100)
    {
      return true;
    } 
    else
    {
      return false; 
    }
    
}
}

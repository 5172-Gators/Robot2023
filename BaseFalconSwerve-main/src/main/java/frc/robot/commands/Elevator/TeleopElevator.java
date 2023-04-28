// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import java.lang.Math;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSub;
public class TeleopElevator extends CommandBase {
  /** Creates a new TeleopElevatorTest. */
  private ElevatorSub s_Elevator;
  private DoubleSupplier s_GoalPosition;

  public TeleopElevator(ElevatorSub ElevatorSub, DoubleSupplier GoalPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Elevator = ElevatorSub;
    this.s_GoalPosition = GoalPosition;

    addRequirements(ElevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
      
      s_Elevator.joystickPosition(Math.pow(s_GoalPosition.getAsDouble(),3)*1000); //TWP: was 1000. Trying to fix jumpy motion
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}




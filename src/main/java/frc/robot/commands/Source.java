// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;


public class Source extends Command {

  PivotSubsystem m_pivot;
  IntakeSubsystem m_intake;

  double sourcePosition = 37;  

  public Source(PivotSubsystem m_pivot, IntakeSubsystem m_intake) 
  {
    this.m_pivot = m_pivot;
    this.m_intake = m_intake;

    addRequirements(m_pivot, m_intake); 
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
   m_pivot.setPosition(sourcePosition);
   //m_intake.feedIn();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

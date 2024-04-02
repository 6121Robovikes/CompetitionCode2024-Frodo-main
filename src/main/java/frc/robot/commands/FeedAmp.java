// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;


public class FeedAmp extends Command {

  IntakeSubsystem m_intake;
  PivotSubsystem m_pivot;   
  double speed;
  double ampPosition = 41;

  public FeedAmp(IntakeSubsystem m_intake, PivotSubsystem m_pivot) {
   
    this.m_intake = m_intake;
    this.m_pivot = m_pivot;
    addRequirements(m_intake, m_pivot); 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intake.ampOut();
    m_pivot.setPosition(ampPosition); 
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

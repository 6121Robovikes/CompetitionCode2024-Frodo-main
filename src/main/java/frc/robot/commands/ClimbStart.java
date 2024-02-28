// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftClimberSubsystem;
import frc.robot.subsystems.RightClimberSubsystem;

public class ClimbStart extends Command {
    LeftClimberSubsystem m_leftClimb;
  RightClimberSubsystem m_rightClimb;

    double rightExtendPosition = 2; //determine this position 
    double leftExtendPosition = 10;
    double retractPosition = 0.1; //Just above retracted to prevent tension on the climber during normal operation
  /** Creates a new ClimbStart. */
  public ClimbStart(LeftClimberSubsystem m_leftClimb, RightClimberSubsystem m_rightClimb) {
    
    this.m_leftClimb = m_leftClimb;
    this.m_rightClimb = m_rightClimb;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(m_leftClimb, m_rightClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leftClimb.climbExtend(retractPosition);
    m_rightClimb.climbExtend(retractPosition);
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

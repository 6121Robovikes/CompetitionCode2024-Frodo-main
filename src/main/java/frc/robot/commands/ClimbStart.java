// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbStart extends Command {
    ClimberSubsystem m_climb;


    double extendPosition = 5;
    double retractPosition = 0.1; //Just above retracted to prevent tension on the climber during normal operation
  /** Creates a new ClimbStart. */
  public ClimbStart(ClimberSubsystem m_climb) {
    
    this.m_climb = m_climb;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.climbExtend(extendPosition);

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ToggleSolenoidCommand extends Command {
  /** Creates a new ToggleSolenoidCommand. */
  SwerveSubsystem m_swerve;
  public ToggleSolenoidCommand(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.toggleSolenoid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

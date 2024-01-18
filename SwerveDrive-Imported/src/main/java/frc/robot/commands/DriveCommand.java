// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vector;

public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  Joystick driverJoystick;
  SwerveSubsystem m_swerve;
  double strafeMagnatude;
  double strafeTargetDegree;
  double rotationMagnatude;

  public DriveCommand(SwerveSubsystem swerve, Joystick stick) {
    m_swerve = swerve;
    driverJoystick = stick;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    strafeMagnatude = driverJoystick.getMagnitude();
    strafeTargetDegree = driverJoystick.getDirectionDegrees();
    //double rotationMagnatude = -driverJoystick.getRawAxis(4);
    double x = driverJoystick.getRawAxis(4);
    double y = -driverJoystick.getRawAxis(5);
    //System.out.println(y);
    // double magnatude = Math.abs(Math.sqrt(Math.pow(x,2) + Math.pow(y,2)));
    Vector v = new Vector(x,y);
    double degreeOffset = m_swerve.getDegreeOffset() * -1;
    if (degreeOffset < 0){
      degreeOffset += 360;
    }
  
    rotationMagnatude = m_swerve.driveToDegree(degreeOffset,v.getDegree());
    if (strafeTargetDegree < 0) {
      strafeTargetDegree += 360;
    }

    strafeTargetDegree = 360 - strafeTargetDegree;
    // strafeTargetDegree = 0;
    // rotationMagnatude = 0;
    // strafeMagnatude = 0.1;
    m_swerve.drive(strafeMagnatude, strafeTargetDegree, rotationMagnatude);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

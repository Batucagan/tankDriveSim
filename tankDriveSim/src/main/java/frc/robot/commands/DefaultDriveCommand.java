// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

  DriveSubsystem m_driveSubsystem;

  DoubleSupplier m_speed,m_rotation;
  boolean m_allowTurningPlace;
  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(DoubleSupplier speed, DoubleSupplier rotation, boolean allowTurningPlace, DriveSubsystem drive) {
    m_speed = speed;
    m_rotation = rotation;
    m_driveSubsystem = drive;
    m_allowTurningPlace = allowTurningPlace;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(m_speed, m_rotation, m_allowTurningPlace);
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

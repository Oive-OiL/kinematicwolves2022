// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterToSpeed extends CommandBase {
  private final ShooterSubsystem m_shooterSubsystem;
  private double m_speedRPM;
  /** Creates a new SetShooterToSpeed. */
  public SetShooterToSpeed(ShooterSubsystem shooterSubsystem, double speedRPM) {
    m_shooterSubsystem = shooterSubsystem;
    m_speedRPM = speedRPM;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterMotorSpeed(m_speedRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooterMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

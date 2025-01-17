// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterOpenLoop extends CommandBase {

  private final ShooterSubsystem m_shooter;
  double m_commandedOutputFraction;

  /** Creates a new RunShooterOpenLoop. */
  public RunShooterOpenLoop(ShooterSubsystem shooter, double commandedOutputFraction) {
    this.m_shooter = shooter;
    this.m_commandedOutputFraction = commandedOutputFraction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.runShooterMotor(m_commandedOutputFraction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.runShooterMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

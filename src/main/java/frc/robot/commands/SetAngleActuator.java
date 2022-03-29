// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class SetAngleActuator extends CommandBase {

  private final ClimberSubsystem setAngleActuator;
  double commandedOutputFraction;

  /** Creates a new Climber2. */
  public SetAngleActuator(ClimberSubsystem climberSubsystem) {
    this.setAngleActuator = climberSubsystem;
    addRequirements(setAngleActuator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (setAngleActuator.servoAtPosition(Constants.LOWER_SERVO_POS_LIMIT)){
      setAngleActuator.servoAtPosition(0.4);
    }
    else{
      setAngleActuator.servoAtPosition(Constants.LOWER_SERVO_POS_LIMIT);
    }
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Motors;

public class GoAtVelocity extends CommandBase {
  /** Creates a new GoAtVelocity. */
  private final Motors motors;
  private double desiredRPM;

  public GoAtVelocity(Motors motors, double desiredRPM) {
    this.motors = motors;
    this.desiredRPM = desiredRPM;
    addRequirements(this.motors);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (desiredRPM == 0) {
      motors.setCurrentState(Motors.MotorState.DISABLED);
    } else {
      motors.setCurrentState(Motors.MotorState.VELOCITY);
      motors.setRPM(desiredRPM);
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
    return true;
  }
}

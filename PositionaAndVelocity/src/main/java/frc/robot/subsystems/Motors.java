// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

public class Motors extends SubsystemBase {

  public static enum MotorState {
    SETPOINT, DISABLED
  }

  private MotorState currentState;

  private double setpointMeters;

  /** Creates a new Motors. */
  public Motors() {
    Hardware.Motors.right = new WPI_TalonSRX(1);
    Hardware.Motors.left = new WPI_TalonSRX(2);

    Hardware.Motors.right.configFactoryDefault();
    Hardware.Motors.left.configFactoryDefault();

    Hardware.Motors.right.setInverted(true);
    Hardware.Motors.left.setInverted(false);

    Hardware.Motors.right.config_kP(0, Constants.Motors.kP);
    Hardware.Motors.right.config_kI(0, Constants.Motors.kI);
    Hardware.Motors.right.config_kD(0, Constants.Motors.kD);

    Hardware.Motors.left.config_kP(0, Constants.Motors.kP);
    Hardware.Motors.left.config_kI(0, Constants.Motors.kI);
    Hardware.Motors.left.config_kD(0, Constants.Motors.kD);

    setCurrentState(MotorState.DISABLED);
    setSetpointMeters(0);
  }

  public void setCurrentState(MotorState newState){
    currentState = newState;
  }

  public void setSetpointMeters(double newSetpointMeters){
    setpointMeters = newSetpointMeters;
  }

  public int metersToTicks(double distanceMeters){
    double wheelCircumference = (2 * Math.PI * Constants.Motors.wheelRadiusMeters);
    double wheelRotations = distanceMeters / wheelCircumference;

    double motorRotations = wheelRotations * Constants.Motors.wheelGearRatio;
    int ticks = (int)(motorRotations * Constants.Motors.motorTicksPerRevolution);
    return ticks;
  }

  public void moveToSetpoint(){
    int ticks = metersToTicks(setpointMeters);
    Hardware.Motors.right.set(ControlMode.Position, ticks);
    Hardware.Motors.left.set(ControlMode.Position, ticks);

    // counter gravity
    // Hardware.Motors.right.set(ControlMode.Position, ticks, DemandType.ArbitraryFeedForward, Constants.arbitraryFeedForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    switch(currentState){
      case SETPOINT:
        moveToSetpoint();
        break;
      case DISABLED:
        Hardware.Motors.right.set(ControlMode.PercentOutput, 0);
        Hardware.Motors.left.set(ControlMode.PercentOutput, 0);
        break;
    }
    log();

    // if(!Robot.isReal()){
    //   simulationPeriodic();
    // }
  }

  public void simulationPeriodic(){
    // for later
  }

  public void log(){
    SmartDashboard.putNumber("Motor Current Position (Ticks)", Hardware.Motors.left.getSelectedSensorPosition());
    SmartDashboard.putNumber("Setpoint (Meters)", setpointMeters);
    SmartDashboard.putNumber("Error (ticks)", metersToTicks(setpointMeters) - Hardware.Motors.left.getSelectedSensorPosition());
  }
}

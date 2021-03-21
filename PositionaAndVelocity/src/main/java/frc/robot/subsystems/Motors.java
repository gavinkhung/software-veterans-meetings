// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

public class Motors extends SubsystemBase {

  public static enum MotorState {
    SETPOINT, DISABLED, VELOCITY
  }

  private MotorState currentState;

  private DifferentialDrivetrainSim drivetrainSim;

  // set the encoder of the talon
  private TalonSRXSimCollection leftMotorControllerSim;
  private TalonSRXSimCollection rightMotorControllerSim;

  // meters
  private double setpointMeters;

  // rotations per minute
  private double desiredRPM;

  /** Creates a new Motors. */
  public Motors() {
    drivetrainSim = new DifferentialDrivetrainSim(Hardware.Motors.DRIVE_MOTOR, //Drive Motor ( Falcon500(2) )
                                                   Constants.Motors.gearRatio, //Gear Ratio ( 10:1 )
                                                   Constants.Motors.rotationalInertia, //Rotational Inertia ( 7.469 )
                                                   Constants.Motors.robotMass, //Mass of Robot in kg
                                                   Constants.Motors.wheelRadius, //Wheel Radius in Meters
                                                   Constants.Motors.trackWidth, //Track Width in Meters
                                                   null //Standard Deviations ( leave null for now )
                                                   );

    Hardware.Motors.right = new WPI_TalonSRX(1);
    Hardware.Motors.left = new WPI_TalonSRX(2);

    leftMotorControllerSim = Hardware.Motors.left.getSimCollection();
    rightMotorControllerSim = Hardware.Motors.right.getSimCollection();


    // Hardware.Motors.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    Hardware.Motors.right.configFactoryDefault();
    Hardware.Motors.left.configFactoryDefault();

    // Hardware.Motors.right.setInverted(true);
    // Hardware.Motors.left.setInverted(false);

    Hardware.Motors.right.config_kP(0, Constants.Motors.kP);
    Hardware.Motors.right.config_kI(0, Constants.Motors.kI);
    Hardware.Motors.right.config_kD(0, Constants.Motors.kD);
    Hardware.Motors.right.config_kF(0, Constants.Motors.kF);

    Hardware.Motors.left.config_kP(0, Constants.Motors.kP);
    Hardware.Motors.left.config_kI(0, Constants.Motors.kI);
    Hardware.Motors.left.config_kD(0, Constants.Motors.kD);
    Hardware.Motors.left.config_kF(0, Constants.Motors.kF);

    setCurrentState(MotorState.DISABLED);
    setSetpointMeters(0);
    setRPM(0);
  }

  public void setCurrentState(MotorState newState){
    currentState = newState;
  }

  public void setSetpointMeters(double newSetpointMeters){
    setpointMeters = newSetpointMeters;
  }

  public void setRPM(double newRPM){
    desiredRPM = newRPM;
  }

  // converts a number in meters to ticks
  public int metersToTicks(double distanceMeters){
    double wheelCircumference = (2 * Math.PI * Constants.Motors.wheelRadiusMeters);
    double wheelRotations = distanceMeters / wheelCircumference;

    double motorRotations = wheelRotations * Constants.Motors.wheelGearRatio;
    int ticks = (int)(motorRotations * Constants.Motors.motorTicksPerRevolution);
    return ticks;
  }

  // tells the motors to go to a set number of ticks
  public void moveToSetpoint(){
    int ticks = metersToTicks(setpointMeters);
    // ControlMode.Position accepts the number of ticks to move as the parameter
    Hardware.Motors.right.set(ControlMode.Position, ticks);
    Hardware.Motors.left.set(ControlMode.Position, ticks);
    System.out.println("ticks: "+ticks);
    drivetrainSim.setInputs(Hardware.Motors.left.getMotorOutputVoltage(), Hardware.Motors.right.getMotorOutputVoltage());

    // counter gravity
    // Hardware.Motors.right.set(ControlMode.Position, ticks, DemandType.ArbitraryFeedForward, Constants.arbitraryFeedForward);
  }

  // converts a number in RPM to ticks per 100 ms
  public int rpmToTicksPer100MS(double rpm){
    int ticksPer100MS = (int)(rpm / 600 * Constants.Motors.ticksPerRevolution * Constants.Motors.wheelGearRatio);
    return ticksPer100MS;
  }

  // tells the motors to go at a specific rpm
  public void goToVelocity(){
    // ticks per 100 ms
    double ticksPer100MS = rpmToTicksPer100MS(desiredRPM);

    Hardware.Motors.right.set(ControlMode.Velocity, ticksPer100MS);
    Hardware.Motors.left.set(ControlMode.Velocity, ticksPer100MS);

    drivetrainSim.setInputs(Hardware.Motors.left.getMotorOutputVoltage(), Hardware.Motors.right.getMotorOutputVoltage());

    System.out.println("ticksPer100MS: "+ticksPer100MS);
  }

  public void temp(){
    Hardware.Motors.right.set(ControlMode.PercentOutput, 0.5);
    Hardware.Motors.left.set(ControlMode.PercentOutput, 0.5);

    drivetrainSim.setInputs(Hardware.Motors.left.getMotorOutputVoltage(), Hardware.Motors.right.getMotorOutputVoltage());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    switch(currentState){
      case SETPOINT:
        Hardware.Motors.right.config_kF(0, 0);
        Hardware.Motors.left.config_kF(0, 0);
        moveToSetpoint();
        break;
      case VELOCITY:
        Hardware.Motors.right.config_kF(0, Constants.Motors.kF);
        Hardware.Motors.left.config_kF(0, Constants.Motors.kF);
        // by setting the kF, output = kF * ticksPer100MS + PID
        goToVelocity();
        break;
      case DISABLED:
        Hardware.Motors.right.set(ControlMode.PercentOutput, 0);
        Hardware.Motors.left.set(ControlMode.PercentOutput, 0);
        drivetrainSim.setInputs(0, 0);
        break;
    }
    log();
  }

  public void simulationPeriodic(){
    super.simulationPeriodic();

    drivetrainSim.update(.02); // Simulates 0.02s of time. 0.02s is also how often the roborio/ these methods runs

    // ticks
    leftMotorControllerSim.setQuadratureRawPosition(metersToTicks(drivetrainSim.getLeftPositionMeters()));
    rightMotorControllerSim.setQuadratureRawPosition(metersToTicks(drivetrainSim.getLeftPositionMeters()));

    leftMotorControllerSim.setQuadratureVelocity(MPSToTicksPer100MS(drivetrainSim.getLeftVelocityMetersPerSecond()));
    rightMotorControllerSim.setQuadratureVelocity(MPSToTicksPer100MS(drivetrainSim.getLeftVelocityMetersPerSecond()));

    // ticks per 100ms

    //Does some funky math to determine the current voltage of the battery based on the current pull of the drivetrain motors
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrainSim.getCurrentDrawAmps()));
  }

  public void log(){

    SmartDashboard.putString("MotorState", currentState.name());
    SmartDashboard.putNumber("Motor Current Position (Ticks)", Hardware.Motors.left.getSelectedSensorPosition());
    SmartDashboard.putNumber("Desired Position (Ticks)", metersToTicks(setpointMeters));

    SmartDashboard.putNumber("Motor Current RPM", getWheelRPM());
    SmartDashboard.putNumber("Motor Current TicksPer100MS", rpmToTicksPer100MS(getWheelRPM()));
    SmartDashboard.putNumber("Desired RPM", desiredRPM);
  }

  public int MPSToTicksPer100MS(double metersPerSecond){
    double wheelCircumference = (2 * Math.PI * Constants.Motors.wheelRadiusMeters);
    int ticksPer100MS = (int)(metersPerSecond / 10 / wheelCircumference * Constants.Motors.ticksPerRevolution / Constants.Motors.wheelGearRatio);
    return ticksPer100MS;
  }

  public double getWheelRPM() {
    // return Hardware.Motors.left.getSelectedSensorVelocity(); returns the value in ticks per 100ms
    return Hardware.Motors.left.getSelectedSensorVelocity() * 600 / Constants.Motors.ticksPerRevolution / Constants.Motors.wheelGearRatio;
  }
}

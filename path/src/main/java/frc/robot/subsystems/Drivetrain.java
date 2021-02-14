// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  // encoder and gypo readings go to the odometry class
  // - odometry class calculates the robot's current location
  // the odometry class gives the location to the ramsete controller
  // the trajectory is also passsed to the ramsete controller
  // - trajectory is like a list of points

  // ramsete controller compares the desired location and current location
  // - outputs linear and angular velocity we need to travel

  // the velocities are passed into the kinematics class which converts it to right and left velocities
  // - the output velocities are passed into the pid controllers which will correct errors
  // - robot characterization outputs a voltage to the motors from velocity
  
  // there is actually an equation to get voltage from speed

  // we will set the voltage of the motors using the value from pid controller as feedback 
  // and use the value from robot characterization as feedforward

  // encoders meausre the distance traveled
  // gypo can measure the angle traveled

  // sparkmax velocities will be in rpm
  CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANSparkMax leftSlave = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

  // returns angle of robot in degrees
  // positive for clockwise
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  // all units are standard units

  // converts angular and linear velocity to left and right velocities
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));

  // accepts gyro angle as an instance of gyro angle
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  // computes feedforward
  // use robot characterization gets voltage to go at a certain velocity 
  // this is an estimation
  // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.2, 0.1);

  // use pidcontroller to correct the error 
  PIDController leftPIDController = new PIDController(0.1, 0.1, 0.1); 
  PIDController rightPIDController = new PIDController(0.1, 0.1, 0.1); 

  // Pose2d stores our current position
  Pose2d pose;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    rightMaster.setInverted(true);
    leftMaster.setInverted(false);
  }

  public void setOutput(double leftVoltage, double rightVoltage){
    leftMaster.set(leftVoltage/12);
    rightMaster.set(rightVoltage/12);
  }

  public Pose2d getPose(){
    return pose;
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    // we need to convert motor rpm to wheel rpm by dividing by gear ratio
    // wheel rpm to meters per min by multiplying by circumference
    // m per min to m/s divide by 60

    // getVelco
    double gearRatio = 7.29;
    double wheelRadius = 5.0;
    double circumference = 2 * Math.PI * wheelRadius;
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getEncoder().getVelocity() / gearRatio * wheelRadius / 60,
      rightMaster.getEncoder().getVelocity() / gearRatio * wheelRadius / 60
    );

  }

  // return gyro angle
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  // get speeds from encoders
  // we need to convert rpm of motor to m/s

  public double getLeftDistance(){
    // multiply by 2 pi r
    double circumference = 2 * Math.PI * 2;
    return leftMaster.getEncoder().getPosition() * circumference;
  }

  public double getRightDistance(){
    // multiply by 2 pi r
    double circumference = 2 * Math.PI * 2;
    return rightMaster.getEncoder().getPosition() * circumference;
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update our position every 20 ms
    odometry.update(getHeading(), getLeftDistance(), getRightDistance());
    pose = odometry.getPoseMeters();
  }
}

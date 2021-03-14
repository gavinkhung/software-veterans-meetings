// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;

/** Add your docs here. */
public class Hardware {
    public static class Motors {
        // required to create an instance of the drivetrain sim
        public static DCMotor DRIVE_MOTOR = DCMotor.getFalcon500(2);

        public static WPI_TalonSRX right;
        public static WPI_TalonSRX left;
    }
}

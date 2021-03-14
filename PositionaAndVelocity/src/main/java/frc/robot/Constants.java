// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Motors {

        // all necessary to create an instance of the robot simulation
        public static double robotMass = 68.03;  // 150 pounds to Kg
        public static double wheelRadius = 0.0762; // in meters
        public static double trackWidth = 0.508; // in meters
        public static double gearRatio = 10;
        public static double rotationalInertia = 7.469;
        public static double ticksPerRevolution = 4096;

        // pid on talon
        public static final double kP = 0.2;
        public static final double kI = 0.1;
        public static final double kD = 0.1;

        // used ControlMode.Position to go from meters to ticks
        public static final double wheelRadiusMeters = 0.2;
        public static final double wheelGearRatio = 1.0;
        public static final double motorTicksPerRevolution = 64;

        // setpoints
        public static final double setpointMiddleMeters = 0.5;
        public static final double setpointTopMeters = 2;
    }

}

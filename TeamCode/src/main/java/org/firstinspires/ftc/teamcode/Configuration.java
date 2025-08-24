/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.automaducks.pond.commands.FollowTrajectoryCommandParams;
import com.automaducks.pond.kinematics.*;
import com.automaducks.pond.subsystems.HolonomicControllerParams;

/**
 * FTC Dashboard configuration directory.
 * <p>
 * Connect to the robot wifi and open <a href="http://192.168.43.1:8080/dash">Dashboard</a> to change values.
 */
public class Configuration {

    /**
     * Chassis configurations.
     */
    @Config
    public static class Chassis {

        // Motors
        public static MotorParams MotorFrontLeft    = new MotorParams("front_left", MotorParams.Mode.RUN_WITHOUT_ENCODER, false);
        public static MotorParams MotorFrontRight   = new MotorParams("front_right", MotorParams.Mode.RUN_WITHOUT_ENCODER, true);
        public static MotorParams MotorRearLeft     = new MotorParams("rear_left", MotorParams.Mode.RUN_WITHOUT_ENCODER, false);
        public static MotorParams MotorRearRight    = new MotorParams("rear_right", MotorParams.Mode.RUN_WITHOUT_ENCODER, true);

        // Dead wheels parameters
        public static DeadWheelsKinematicsParams DeadWheels = new DeadWheelsKinematicsParams(
            63,
            196,
            2 * Math.PI * 18.688524,
            8192);                          // https://www.revrobotics.com/rev-11-1271/
    }

    /**
     * Arm configuration
     */
    @Config
    public static class Arm {

        public static AngularActuatorParams PivotActuation = new AngularActuatorParams(
            StandardGearRatios.GOBILDA_223_RPM,
            5.0);

        public static MotorParams MotorArm = new MotorParams("slide_pivot", MotorParams.Mode.RUN_TO_POSITION, true);
    }

    /**
     * Autonomous configuration
     */
    @Config
    public static class Autonomous {

        // Road Runner
        public static double maximum_translational_speed_mm_sec = 700;
        public static double maximum_angular_speed_rad_sec = Math.PI / 2;
        public static double maximum_acceleration_mm_sec2 = 700;
        public static double maximum_deceleration_mm_sec2 = 500;

        public static FollowTrajectoryCommandParams TrajectoryParams = new FollowTrajectoryCommandParams(
            10,
            Math.toRadians(10),
            100,
            Math.toRadians(100)
        );

        public static HolonomicControllerParams HolonomicConfig = new HolonomicControllerParams(
            0.01, 0.01, 2.0,
            0.001,0.001, 0.0);
    }
}
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

        // These are the master values for chassis PID, Autonomous Controller, etc. They should reflect the
        // measures values and should not be changed in order to accommodate to these individual subsystems
        public static double MeasureMaximumTranslationalVelocityCmPerSec = 200.0;
        public static double MeasuredMaximumAngularVelocityRadPerSec = Math.PI;

        // Motors
        public static MotorParams MotorFrontLeft    = new MotorParams("front_left", MotorParams.Mode.RUN_WITHOUT_ENCODER, false);
        public static MotorParams MotorFrontRight   = new MotorParams("front_right", MotorParams.Mode.RUN_WITHOUT_ENCODER, true);
        public static MotorParams MotorRearLeft     = new MotorParams("rear_left", MotorParams.Mode.RUN_WITHOUT_ENCODER, false);
        public static MotorParams MotorRearRight    = new MotorParams("rear_right", MotorParams.Mode.RUN_WITHOUT_ENCODER, true);

        // Dead wheels parameters
        public static DeadWheelsKinematicsParams DeadWheels = new DeadWheelsKinematicsParams(
            -7.5,
            16.905,
            Math.PI * 3.2,
            2000);
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
        public static double MaxTranslationSpeedCmPerSec = 180;
        public static double MaxAngularSpeedRadPerSec = Math.PI / 2;
        public static double MaxAccelerationCmPerSec2 = 150;
        public static double MaxDecelerationCmPerSec2 = 100;


        public static FollowTrajectoryCommandParams TrajectoryParams = new FollowTrajectoryCommandParams(
            10,
            Math.toRadians(10),
            100,
            Math.toRadians(100)
        );

        public static HolonomicControllerParams HolonomicConfig = new HolonomicControllerParams(
            Chassis.MeasureMaximumTranslationalVelocityCmPerSec,
            Chassis.MeasuredMaximumAngularVelocityRadPerSec,
            0.08, 0.08, 0.5,
            0.02, 0.02, 0.1,
            0.8,1.4, 0.5);
    }
}

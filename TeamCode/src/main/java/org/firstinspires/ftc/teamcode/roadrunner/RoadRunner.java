/**
 * Copyright (c) 2024 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.*;
import com.automaducks.pond.common.Pose2D;
import com.automaducks.pond.kinematics.ITimedTrajectory;

import org.firstinspires.ftc.teamcode.Configuration;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

/**
 * Road runner helper class.
 */
public class RoadRunner {

    /**
     * Creates a road runner trajectory builder.
     * @param initialPose initial pose.
     * @return a new road runner trajectory builder.
     */
    @NonNull
    public static ITimedTrajectory createTrajectory(
        Pose2D initialPose,
        Function<TrajectoryBuilder, List<Trajectory>> builder) {

        // Configure Road Runner
        TrajectoryBuilderParams trajectoryBuilderParams = new TrajectoryBuilderParams(
            1e-6,
            new ProfileParams(5, 0.1, 1e-2));

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(Configuration.Autonomous.maximum_translational_speed_mm_sec),
            new AngularVelConstraint(Configuration.Autonomous.maximum_angular_speed_rad_sec)
        ));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(
            -Configuration.Autonomous.maximum_deceleration_mm_sec2,
            Configuration.Autonomous.maximum_acceleration_mm_sec2);

        // Create a trajectory builder
        TrajectoryBuilder builderInstance = new TrajectoryBuilder(
            trajectoryBuilderParams,
            fromPondToRoadRunnerPose(initialPose),
            0.0,
            baseVelConstraint,
            baseAccelConstraint);

        // Execute the lambda
        List<Trajectory> trajectoryList = builder.apply(builderInstance);

        // Adapt the trajectory
        return new TimedTrajectory(trajectoryList);
    }

    /**
     * Converts from a Pond Pose2D to a RoadRunner Pose2d
     * @param pose2D Pond Pose2D
     * @return RoadRunner Pose2d
     */
    public static Pose2d fromPondToRoadRunnerPose(Pose2D pose2D) {
        return new Pose2d(pose2D.x, pose2D.y, pose2D.w);
    }

    /**
     * Converts from a RoadRunner Pose2d to a Pond Pose2D
     * @param pose2d RoadRunner Pose2d
     * @return Pond Pose2D
     */
    public static Pose2D fromRoadRunnerToPondPose(Pose2d pose2d) {
        return new Pose2D(pose2d.position.x, pose2d.position.y, pose2d.heading.log());
    }

    /**
     * Converts from a RoadRunner Pose2d to a Pond Pose2D
     * @param velocity2dDual RoadRunner Pose2d
     * @return Pond Pose2D
     */
    public static Pose2D fromRoadRunnerToPondPose(PoseVelocity2dDual<Time> velocity2dDual) {
        return new Pose2D(
            velocity2dDual.linearVel.x.value(),
            velocity2dDual.linearVel.y.value(),
            velocity2dDual.angVel.value());
    }
}
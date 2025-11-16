/**
 * Copyright (c) 2024 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.automaducks.pond.common.Duration;
import com.automaducks.pond.common.PoseAndVelocity;
import com.automaducks.pond.kinematics.ITimedTrajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Provides an implementation of pond ITimedTrajectory on top of a Road Runner trajectory.
 */
public class TimedTrajectory implements ITimedTrajectory {

    private final List<TimeTrajectory> trajectoryList = new ArrayList<>();
    private final Duration totalDuration;
    private double trajectoryTimeBaseSeconds = 0;

    /**
     * Constructs a new instance of the RoadRunnerTrajectoryAdapter class.
     * @param trajectoryList road runner trajectory list.
     */
    public TimedTrajectory(List<Trajectory> trajectoryList) {

        // Stores the road runner timed trajectories.
        for (Trajectory t : trajectoryList) {
            this.trajectoryList.add(new TimeTrajectory(t));
        }

        // Calculates the total duration.
        double totalDuration = 0;
        for (TimeTrajectory t : this.trajectoryList){
            totalDuration += t.duration;
        }
        this.totalDuration = Duration.ofSeconds(totalDuration);
    }

    @Override
    public PoseAndVelocity getTargetPose(Duration elapsedTime) {

        double trajectoryTimeOffsetSeconds = elapsedTime.toSeconds() - this.trajectoryTimeBaseSeconds;

        // Remove  completed trajectories
        TimeTrajectory currentTrajectory = trajectoryList.get(0);
        while (trajectoryTimeOffsetSeconds > currentTrajectory.duration && trajectoryList.size() > 1) {

            // Add time passed
            this.trajectoryTimeBaseSeconds += currentTrajectory.duration;
            trajectoryTimeOffsetSeconds = elapsedTime.toSeconds() - this.trajectoryTimeBaseSeconds;

            // Move to next trajectory
            trajectoryList.remove(0);
            currentTrajectory = trajectoryList.get(0);
        }

        Pose2dDual<Time> targetPose = currentTrajectory.get(trajectoryTimeOffsetSeconds);

        return new PoseAndVelocity(
            RoadRunner.fromRoadRunnerToPondPose(targetPose.value()),
            RoadRunner.fromRoadRunnerToPondPose(targetPose.velocity())
        );
    }

    @Override
    public Duration getTotalDuration() {
        return this.totalDuration;
    }
}

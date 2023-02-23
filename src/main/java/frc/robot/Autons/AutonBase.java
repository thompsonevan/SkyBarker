package frc.robot.Autons;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;

public abstract class AutonBase {
    public State desState;
    public Rotation2d targetTheta;
    public Timer timer;
    public Pose2d initalPose;
    public Pose2d endingPose;
    public Arm.ArmPos armPos;
    public boolean intakeOn;
    public boolean pickUpObject;
    public boolean driving;
    public boolean autoBalance;

    public abstract void runAuto();
    public abstract void reset();

    public Trajectory createTrajectory(Pose2d startingPose, Pose2d endPose){
        double headingAngle = Math.toDegrees(Math.atan2(endPose.getY()-startingPose.getY(), 
                                             endPose.getX()-startingPose.getX()));

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startingPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
            List.of(),
            new Pose2d(endPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
            new TrajectoryConfig(7, 2.5));
    }

    public State getState(double time, Trajectory traj, Rotation2d heading){
        State curState = traj.sample(time);
            
        return new State(time,
            curState.velocityMetersPerSecond,
            curState.accelerationMetersPerSecondSq,
            new Pose2d(curState.poseMeters.getX(), curState.poseMeters.getY(), heading),
            1000);
    }
}

package frc.robot.Autons;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;

import javax.sound.midi.Track;
import javax.swing.TransferHandler;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;

public class Auton67 extends AutonBase{
    enum AutoState {
        step1,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    PathPlannerTrajectory trajectory;

    public Auton67(){
        autoState = AutoState.step1;

        timer.reset();
        timer.start();

        trajectory = PathPlanner.loadPath("67", new PathConstraints(3, 2));

        desState = new State();

        initalState = trajectory.getInitialState();
    }

    public void reset(){
        autoState = AutoState.step1;

        desState = new State();

        initalState = trajectory.getInitialState();

        timer.reset();
        timer.start();
    }

    PathPlannerState state;

    public void runAuto(){
        state = (PathPlannerState) trajectory.sample(timer.get());

        Pose2d newPose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation);

        desState = new State(timer.get(), state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter);

        SmartDashboard.putNumber("Des X", desState.poseMeters.getX());
        SmartDashboard.putNumber("Des Y", desState.poseMeters.getY());
        SmartDashboard.putNumber("Des Theta", desState.poseMeters.getRotation().getDegrees());
    }
}

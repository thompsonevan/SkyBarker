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
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;

public class Auton1 extends AutonBase{
    enum AutoState {
        step1,
        step2,
        step3,
        step4,
        end
    }
    
    public AutoState autoState;

    public Timer timer = new Timer();

    PathPlannerTrajectory trajectory;

    public Auton1(){
        autoState = AutoState.step1;

        timer.reset();
        timer.start();

        trajectory = PathPlanner.loadPath("simple", new PathConstraints(1,1));

        desState = new State();

        // Drivetrain.setPose(trajectory.getInitialHolonomicPose());
        // Pigeon.zeroSensor(trajectory.getInitialPose().getRotation().getDegrees());
    }

    // public void reset(){
    //     Drivetrain.setPose(trajectory.getInitialHolonomicPose());
    //     Pigeon.zeroSensor(trajectory.getInitialHolonomicPose().getRotation().getDegrees());
    // }

    public void runAuto(){
        PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());

        Pose2d newPose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation);

        desState = new State(timer.get(), state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter);

        trajectory.getMarkers();

        SmartDashboard.putNumber("Des X", desState.poseMeters.getX());
        SmartDashboard.putNumber("Des Y", desState.poseMeters.getY());
        SmartDashboard.putNumber("Des Theta", desState.poseMeters.getRotation().getDegrees());
    }
}

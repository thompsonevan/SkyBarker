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
        end
    } 

    public AutoState autoState;

    public Timer timer = new Timer();

    PathPlannerTrajectory trajectory;
    PathPlannerTrajectory trajectory1;

    
PathPlannerTrajectory[] steps;
    public Auton1(){
        autoState = AutoState.step1;

        timer.reset();
        timer.start();

        trajectory = PathPlanner.loadPath("path1", new PathConstraints(1,1));
        trajectory1 = PathPlanner.loadPath("path2", new PathConstraints(1,1));

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
        // switch(autoState){
        //     case step1:
        //         state = (PathPlannerState) trajectory.sample(timer.get());

        //         if((Drivetrain.getPose().getX() - trajectory.getEndState().poseMeters.getX()) < .1
        //         && (Drivetrain.getPose().getY() - trajectory.getEndState().poseMeters.getY()) < .1){
        //             autoState = AutoState.step2;
        //         Drivetrain.setPose(trajectory1.getInitialState());
        //         }
        
        //     break;
        //     case step2:
        //         state = (PathPlannerState) trajectory1.sample(timer.get());

        //         if((Drivetrain.getPose().getX() - trajectory1.getEndState().poseMeters.getX()) < .1
        //         && (Drivetrain.getPose().getY() - trajectory1.getEndState().poseMeters.getY()) < .1){
        //             autoState = AutoState.end;
        //         }
        //     break;
        //     case end:
        //         Drivetrain.stopMotors();
        //     break;
        // }

        switch(autoState){
            case step1:
                state = (PathPlannerState) trajectory.sample(timer.get());

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    autoState = AutoState.step2;
                    Drivetrain.setPose(trajectory1.getInitialState().poseMeters, trajectory1.getInitialState().holonomicRotation);
                    timer.reset();
                }
        
            break;
            case step2:
                state = (PathPlannerState) trajectory1.sample(timer.get());

                if(timer.get() > trajectory1.getTotalTimeSeconds()){
                    autoState = AutoState.end;
                }
            break;
            case end:
                Drivetrain.stopMotors();
            break;
        }

        Pose2d newPose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation);

        desState = new State(timer.get(), state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter);

        SmartDashboard.putNumber("Des X", desState.poseMeters.getX());
        SmartDashboard.putNumber("Des Y", desState.poseMeters.getY());
        SmartDashboard.putNumber("Des Theta", desState.poseMeters.getRotation().getDegrees());
    }
}

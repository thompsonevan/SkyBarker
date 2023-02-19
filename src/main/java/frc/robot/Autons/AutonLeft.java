package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;

import java.util.List;

import javax.sound.midi.Track;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class AutonLeft extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject,
        pickUpObject,
        driveBack,
        secondPlace,
        chargingStation,
        end
    } 

    public AutoState autoState;

    public Timer timer = new Timer();

    Trajectory trajectory;
    Trajectory trajectory1;

    public AutonLeft(){
        reset();
    }

    public void reset(){
        autoState = AutoState.firstPlace;

        desState = new PathPlannerState();

        initalPose = new Pose2d(new Translation2d(1.75,4.45), Rotation2d.fromDegrees(-90));
        endingPose = new Pose2d(new Translation2d(6.62,4.63), Rotation2d.fromDegrees(0));

        double headingAngle = Math.toDegrees(Math.atan2(endingPose.getY()-initalPose.getY(), 
                                                        endingPose.getX()-initalPose.getX()));

        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(initalPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
            List.of(),
            new Pose2d(endingPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
            new TrajectoryConfig(7, 2.5));

        Drivetrain.setPose(initalPose, initalPose.getRotation());

        timer.reset();
        timer.start();
    }

    State state;
    Pose2d newPose;

    public void runAuto(){
        if(autoState == AutoState.firstPlace){
            driving = true;
            desState = trajectory.sample(timer.get());
            
    
            desState = new State(timer.get(),
                desState.velocityMetersPerSecond,
                desState.accelerationMetersPerSecondSq,
                new Pose2d(desState.poseMeters.getX(), desState.poseMeters.getY(), Rotation2d.fromDegrees(0)),
                1000);

            if(timer.get() > trajectory.getTotalTimeSeconds() + 5){
                initalPose = new Pose2d(new Translation2d(6.62,4.63), Pigeon.getRotation2d());
                endingPose = new Pose2d(new Translation2d(1.75,4.45), Rotation2d.fromDegrees(-90));
        
                double headingAngle = Math.toDegrees(Math.atan2(endingPose.getY()-initalPose.getY(), 
                                                                endingPose.getX()-initalPose.getX()));
        
                trajectory1 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(initalPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
                    List.of(),
                    new Pose2d(endingPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
                    new TrajectoryConfig(7, 2.5));
        
                Drivetrain.setPose(initalPose, initalPose.getRotation());

                autoState = AutoState.driveBack;
                timer.reset();
            } 
        } else {
            driving = true;
            desState = trajectory1.sample(timer.get());
            
            desState = new State(timer.get(),
                desState.velocityMetersPerSecond,
                desState.accelerationMetersPerSecondSq,
                new Pose2d(desState.poseMeters.getX(), desState.poseMeters.getY(), Rotation2d.fromDegrees(-90)),
                1000);
        }

    }
}

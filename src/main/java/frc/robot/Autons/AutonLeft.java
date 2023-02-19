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

        trajectory = createTrajectory(initalPose, endingPose);

        Drivetrain.setPose(initalPose, initalPose.getRotation());

        timer.reset();
        timer.start();
    }

    public void runAuto(){
        if(autoState == AutoState.firstPlace){
            driving = true;
            desState = getState(timer.get(), trajectory, endingPose.getRotation());

            if(timer.get() > trajectory.getTotalTimeSeconds() + 5){
                initalPose = new Pose2d(new Translation2d(6.62,4.63), Pigeon.getRotation2d());
                endingPose = new Pose2d(new Translation2d(1.75,4.45), Rotation2d.fromDegrees(-90));
        
                trajectory1 = createTrajectory(initalPose, endingPose);
        
                Drivetrain.setPose(initalPose, initalPose.getRotation());

                autoState = AutoState.driveBack;
                timer.reset();
            } 
        } else {
            driving = true;
            desState = getState(timer.get(), trajectory, endingPose.getRotation());
        }
    }
}

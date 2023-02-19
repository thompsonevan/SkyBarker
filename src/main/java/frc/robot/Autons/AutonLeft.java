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

    PathPlannerTrajectory trajectory;
    PathPlannerTrajectory trajectory1;

    public AutonLeft(){
        reset();
    }

    public void reset(){
        autoState = AutoState.firstPlace;

        desState = new PathPlannerState();

        trajectory = PathPlanner.loadPath("path", new PathConstraints(3,2));

        // trajectory = PathPlanner.generatePath(
        //     new PathConstraints(4, 2), 
        //     new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        //     new PathPoint(new Translation2d(0,-3), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90)));

        // trajectory = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0,0,ROt), null, initalPose, null)
        
        // initalPose = new Pose2d(0,0, Rotation2d.fromDegrees(0));

        initalPose = trajectory.getInitialPose();

        // Drivetrain.setPose(initalPose, initalPose.getRotation());

        Drivetrain.setPose(initalPose, trajectory.getInitialHolonomicPose().getRotation());

        // Drivetrain.setPose(initalPose, trajectory.getInitialPose().getRotation());

        SmartDashboard.putNumber("Inital Holomonic Pose", trajectory.getInitialHolonomicPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Inital Non Holomonic Pose", trajectory.getInitialPose().getRotation().getDegrees());

        timer.reset();
        timer.start();
    }

    State state;
    Pose2d newPose;

    public void runAuto(){
        driving = true;
        PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
        
        desState = new State(timer.get(), 
            state.velocityMetersPerSecond,
            state.accelerationMetersPerSecondSq,
            new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation),
            1000);
    
        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //     , null, null, null, null, null, null)
    }
}

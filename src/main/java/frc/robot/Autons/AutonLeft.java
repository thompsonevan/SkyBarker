package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class AutonLeft extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject,
        pickUpObject,
        driveBack,
        secondPlace,
        end
    } 

    public AutoState autoState;

    public Timer timer = new Timer();

    PathPlannerTrajectory trajectory;
    PathPlannerTrajectory trajectory1;

    public AutonLeft(){
        autoState = AutoState.firstPlace;

        timer.reset();
        timer.start();

        trajectory = PathPlanner.loadPath("path1", new PathConstraints(1,1));
        trajectory1 = PathPlanner.loadPath("path2", new PathConstraints(1,1));

        desState = new State();

        initalState = trajectory.getInitialState();
    }

    public void reset(){
        autoState = AutoState.firstPlace;

        desState = new State();

        initalState = trajectory.getInitialState();

        timer.reset();
        timer.start();
    }

    PathPlannerState state;

    Pose2d newPose;

    public void runAuto(){
        switch(autoState){
            case firstPlace:
                armPos = ArmPos.topNode;

                if(timer.get() > 5){
                    armPos = ArmPos.packagePos;
                    autoState = AutoState.driveToObject;
                    Drivetrain.stopMotors();
                    Drivetrain.setPose(trajectory.getInitialState().poseMeters, trajectory.getInitialState().holonomicRotation);
                    timer.reset();
                }
            break;
            case driveToObject:
                state = (PathPlannerState) trajectory.sample(timer.get());

                newPose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation);
                desState = new State(timer.get(), state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter);

                if((Drivetrain.getPose().getX() - trajectory.getEndState().poseMeters.getX()) < .1
                    && (Drivetrain.getPose().getY() - trajectory.getEndState().poseMeters.getY()) < .1){
                    autoState = AutoState.pickUpObject;
                    Drivetrain.stopMotors();
                    timer.reset();
                }
            break;
            case pickUpObject:
                intakeOn = true;
                pickUpObject = true;

                if(timer.get() > 5){
                    intakeOn = false;
                    pickUpObject = false;
                    autoState = AutoState.driveBack;
                    Drivetrain.stopMotors();
                    Drivetrain.setPose(trajectory1.getInitialState().poseMeters, trajectory1.getInitialState().holonomicRotation);
                    timer.reset();
                }
            break;
            case driveBack:
                state = (PathPlannerState) trajectory1.sample(timer.get());

                newPose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation);
                desState = new State(timer.get(), state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter);

                if((Drivetrain.getPose().getX() - trajectory1.getEndState().poseMeters.getX()) < .1
                    && (Drivetrain.getPose().getY() - trajectory1.getEndState().poseMeters.getY()) < .1){
                    autoState = AutoState.secondPlace;
                    Drivetrain.stopMotors();
                    timer.reset();
                }
            break;
            case secondPlace:
                armPos = ArmPos.middleNode;

                if(timer.get() > 5){
                    autoState = AutoState.end;
                    Drivetrain.stopMotors();
                    timer.reset();
                }
            break;
            case end:
                armPos = ArmPos.packagePos;
                Drivetrain.stopMotors();
            break;
        }

        SmartDashboard.putNumber("Des X", desState.poseMeters.getX());
        SmartDashboard.putNumber("Des Y", desState.poseMeters.getY());
        SmartDashboard.putNumber("Des Theta", desState.poseMeters.getRotation().getDegrees());
    }
}

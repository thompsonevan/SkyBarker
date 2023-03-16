package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class BlueAutoLeft3 extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        autoPickup1,
        driveToScore1,
        lineupScore1,
        score2,
        driveToObject2,
        autoPickup2,
        driveToScore2,
        lineupScore2,
        score3,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(4.5,.45), Rotation2d.fromDegrees(0)), //4.82, .5
                                new Pose2d(new Translation2d(0,.5), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(5.2,0), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(0,.9), Rotation2d.fromDegrees(-95)));

    Trajectory trajectory;

    public BlueAutoLeft3(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(-90);

        point = 0;
        
        timer.reset();
        timer.start();

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        switch(autoState){
            case firstPlace:
                driving = false;
                if(timer.get() > 3){
                    gripperSpeed = 0;

                    trajectory = createTrajectory(path.get(point), path.get(point+1),
                    Rotation2d.fromDegrees(42.5), Rotation2d.fromDegrees(-12.5),
                    5,3);
            
                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject1;
                }
            break;
            case driveToObject1:
                driving = true;

                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    timer.reset();

                    autoState = AutoState.autoPickup1;
                }
            break;
            case autoPickup1:
                driving = false;

                if(timer.get() > .25){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(-10 + 180), Rotation2d.fromDegrees(10 + 180),
                    5,3);

                    point++;

                    timer.reset();

                    autoState = AutoState.driveToScore1;
                }
            break;
            case driveToScore1:
                driving = true;

                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    timer.reset();

                    autoState = AutoState.lineupScore1;
                }
            break;
            case lineupScore1:
            break;
            case score2:
                driving = false;

                if(timer.get() > 3){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(25), Rotation2d.fromDegrees(-25),
                    5,3);

                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
            break;
            case autoPickup2:
            break;
            case driveToScore2:
            break;
            case lineupScore2:
            break;
            case score3:
            break;
            case end:
            break;
        }
    }
}
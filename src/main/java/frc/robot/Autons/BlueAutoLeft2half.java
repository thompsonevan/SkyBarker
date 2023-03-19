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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

public class BlueAutoLeft2half extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        driveToScore1,
        score2,
        driveToObject2,
        driveToBalance,
        balance,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(4.75,.7), Rotation2d.fromDegrees(0)), //4.82, .5
                                new Pose2d(new Translation2d(0.1,.5), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(4.5,.8), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(5.05,-.5), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(.9,-1.25), Rotation2d.fromDegrees(-90)));

    Trajectory trajectory;
    Trajectory trajectory1;

    public BlueAutoLeft2half(){
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
                if(timer.get() > .05 && timer.get() < 3){
                    if(timer.get() < 2.5){
                        gripperSpeed = -.4;
                    }else{
                        gripperSpeed = .4;
                    }
                    armPos = ArmPos.topNodeCone;
                } else {
                    gripperSpeed = -.4;
                    armPos = ArmPos.intake;
                }
                if(timer.get() > 3) {
                    gripperSpeed = 0;

                    trajectory = createTrajectory(path.get(point), path.get(point+1),
                    Rotation2d.fromDegrees(42.5), Rotation2d.fromDegrees(-12.5),
                    7,5);
            
                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject1;
                }
            break;
            case driveToObject1:
                driving = true;
                armPos = ArmPos.intake;
                if(timer.get() > .5){
                    intakeOn = true;
                }
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    trajectory = createTrajectory(path.get(point), path.get(point+1),
                    Rotation2d.fromDegrees(-12 + 180), Rotation2d.fromDegrees(12 + 180),
                    7,5);
            
                    point++;

                    timer.reset();

                    autoState = AutoState.driveToScore1;
                }
            break;
            case driveToScore1:
                driving = true;
                intakeOn = false;

                if(timer.get()>.75){
                    armPos = ArmPos.packagePos;
                } else if (timer.get() > 1.25){
                    armPos = ArmPos.topNodeCone;
                }
                gripperSpeed = -.4;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();//Rotation2d.fromDegrees((path.get(point).getRotation().getDegrees() / (trajectory.getTotalTimeSeconds() - .2)) * timer.get());

                if(timer.get() > trajectory.getTotalTimeSeconds()){                    
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score2;
                }
            break;
            case score2:
                driving = false;
                if(timer.get() < 2.75){
                    if(timer.get() < 2.25){
                        gripperSpeed = -.4;
                    }else{
                        gripperSpeed = .2;
                    }
                    armPos = ArmPos.topNodeCube;
                } else {
                    gripperSpeed = 0;
                    armPos = ArmPos.intake;

                    trajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(path.get(point).getTranslation(), Rotation2d.fromDegrees(5)), 
                        List.of(path.get(point+1).getTranslation()),
                        new Pose2d(path.get(point+2).getTranslation(), Rotation2d.fromDegrees(-90)), 
                        new TrajectoryConfig(7, 5));
            
                    point += 2;

                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
                driving = true;

                armPos = ArmPos.intake;
                if(timer.get() > .75){
                    intakeOn = true;
                }
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){      
                    trajectory = createTrajectory(path.get(point), path.get(point+1),
                    Rotation2d.fromDegrees(90 + 180), Rotation2d.fromDegrees(0 + 180),
                    2,1.5);
            
                    point++;

                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.driveToBalance;
                }
            break;
            case driveToBalance:
                driving = true;
                intakeOn = false;

                if(timer.get()>.75){
                    armPos = ArmPos.packagePos;
                }
                gripperSpeed = -.4;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();
            break;
            case balance:
            break;
            case end:
            break;
        }

    }
}
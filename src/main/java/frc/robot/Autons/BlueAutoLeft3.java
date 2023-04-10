package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;
import frc.robot.subsystems.Arm.IntakePos;
import frc.robot.subsystems.Arm.IntakeSpeed;

import java.util.List;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class BlueAutoLeft3 extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        driveToObject2,
        score2,
        driveToObject3,
        driveToObject4,
        score3,
        end,
        pause1,
        pause2
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(5,.75), Rotation2d.fromDegrees(-10)), //4.82, .5
                                new Pose2d(new Translation2d(0.1,.35), Rotation2d.fromDegrees(-180)),
                                new Pose2d(new Translation2d(4.85, .5), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(5.05, -.75), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(4.25, .25), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(-.3, -.1), Rotation2d.fromDegrees(-90)));

    Trajectory trajectory;

    double armTime;

    public BlueAutoLeft3(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(-90);

        point = 0;
        
        timer.reset();
        timer.start();

        armTime = 0;

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        switch(autoState){
            case firstPlace:
                driving = false;
                if(!Arm.getAchivedPostion() || timer.get() < .75){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCone;
                    armTime = timer.get();
                } else {
                    if(Math.abs(armTime - timer.get()) < .35){
                        gripperSpeed = .75;
                    } else {
                        armPos = ArmPos.packagePos;
                        gripperSpeed = 0;

                        trajectory = createTrajectory(path.get(point), path.get(point+1),
                        Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(0),
                        4,2.25);
                
                        point++;

                        timer.reset();

                        autoState = AutoState.driveToObject1;
                    }
                }
            break;
            case driveToObject1:
                driving = true;
                if(timer.get() > 1){
                    armPos = ArmPos.Zero;

                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                } else {
                    armPos = ArmPos.packagePos;
                }
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .1 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .1){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(-12 + 180), Rotation2d.fromDegrees(12 + 180),
                    4,2.25);

                    point++;

                    timer.reset();

                    autoState = AutoState.pause1;
                }
            break;
            case pause1:
                if(timer.get() > .1){
                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
                driving = true;

                intakePos = IntakePos.pack;
                intakeSpeed = IntakeSpeed.none;

                gripperSpeed = 0;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .075 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .075){                    
                    timer.reset();

                    intakeSpeed = IntakeSpeed.out;
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score2;
                }
            break;
            case score2:
                driving = false;
                
                if(timer.get() < .2){
                    intakeSpeed = IntakeSpeed.out;
                } else {
                    trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(path.get(point).getTranslation(), Rotation2d.fromDegrees(15)), 
                    List.of(path.get(point+1).getTranslation()),
                    new Pose2d(path.get(point+2).getTranslation(), Rotation2d.fromDegrees(-90)), 
                    new TrajectoryConfig(4, 1.75));

                    intakeSpeed = IntakeSpeed.none;

                    point += 2;

                    timer.reset();

                    autoState = AutoState.driveToObject3;
                }
            break;
            case driveToObject3:
                driving = true;
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                armPos = ArmPos.intake;

                if(timer.get() > 1.5){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                }

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .05){       
                    trajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(path.get(point).getTranslation(), Rotation2d.fromDegrees(-45+180)), 
                        List.of(path.get(point+1).getTranslation()),
                        new Pose2d(path.get(point+2).getTranslation(), Rotation2d.fromDegrees(10 + 180)), 
                        new TrajectoryConfig(4, 1.75));
                    
                    point += 2;

                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.pause2;
                }
            break;
            case pause2:
            if(timer.get() > .1){
                timer.reset();

                autoState = AutoState.driveToObject4;
            }
            break;
            case driveToObject4:
                driving = true;

                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();
                
                intakePos = IntakePos.cubeHandoff;

                if(timer.get() > 2){
                    armPos = ArmPos.yeetCube;
                } else if(timer.get() > 1.4){
                    armPos = ArmPos.packagePos;
                }

                gripperSpeed = -.75;

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .05){       
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score3;
                }
            break;
            case score3:
                driving = false;
                if(timer.get() < .75){
                    gripperSpeed = -.4;
                    armPos = ArmPos.yeetCube;
                } else {
                    gripperSpeed = 1;
                }
            break;
            case end:
                driving = false;
                intakePos = IntakePos.pack;
                intakeSpeed = IntakeSpeed.none;
            break;
        }

        HotLogger.Log("AutoState", autoState.toString());
        SmartDashboard.putString("AutoState", autoState.toString());
        SmartDashboard.putBoolean("Arm Achieved Position", Arm.getAchivedPostion());
    }
}
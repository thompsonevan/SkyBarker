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

public class BlueAutoLeftBalance extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        pause,
        driveToObject2,
        score2,
        driveToBal,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                               // new Pose2d(new Translation2d(1.75,.75), Rotation2d.fromDegrees(-45)),
                                new Pose2d(new Translation2d(5.3,.75), Rotation2d.fromDegrees(-10)), //4.82, .5
                                new Pose2d(new Translation2d(-.025,.1), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(.5, -.5), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(2.7, -1), Rotation2d.fromDegrees(-90)));

    Trajectory trajectory;

    double armTime;

    public BlueAutoLeftBalance(){
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
                if(timer.get() < .25){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCone;
                    armTime = timer.get();
                } else if(!Arm.getAchivedPostion()){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCone;
                    armTime = timer.get();
                } else {
                    if(Math.abs(armTime - timer.get()) < .35){
                        gripperSpeed = 1;
                    } else {
                        armPos = ArmPos.packagePos;
                        gripperSpeed = 0;

                        trajectory = createTrajectory(path.get(point), path.get(point+1),
                        Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(0),
                        4,2.5);
                
                        // trajectory = TrajectoryGenerator.generateTrajectory(
                        //     new Pose2d(path.get(point).getTranslation(), Rotation2d.fromDegrees(40)), 
                        //     List.of(path.get(point+1).getTranslation()),
                        //     new Pose2d(path.get(point+2).getTranslation(), Rotation2d.fromDegrees(0)), 
                        //     new TrajectoryConfig(4, 2.5));
    
                        //     // trajectory = createTrajectory(path.get(point), path.get(point+1));
        
                        // point += 2;

                        point++;

                        timer.reset();

                        autoState = AutoState.driveToObject1;
                    }
                }
            break;
            case driveToObject1:
                driving = true;
                armPos = ArmPos.intake;
                if(timer.get() > .65){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                }
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .1 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .1){
                    timer.reset();

                    autoState = AutoState.pause;
                }
            break;
            case pause:
                driving = false;

                intakePos = IntakePos.cubeHandoff;
                intakeSpeed = IntakeSpeed.cubeHandoff;

                armPos = ArmPos.intake;
                gripperSpeed = -.4;
                if(timer.get() > .25){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(-12 + 180), Rotation2d.fromDegrees(12 + 180),
                    4,2.5);

                    

                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
                driving = true;
                intakeOn = false;
                
                intakePos = IntakePos.cubeHandoff;
                intakeSpeed = IntakeSpeed.cubeHandoff;

                if(timer.get() > 2){
                    armPos = ArmPos.topNodeCube;
                    intakePos = IntakePos.pack;
                } else if(timer.get() > 1.25){
                    armPos = ArmPos.packagePos;
                }

                gripperSpeed = -.75;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .075 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .075){                    
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score2;
                }
            break;
            case score2:
                driving = false;
                if(timer.get() < .75){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCube;
                } else {
                    if(timer.get() < 1.25){
                        gripperSpeed = .5;
                    } else if(timer.get() < 1.75){
                        armPos = ArmPos.packagePos;
                        gripperSpeed = 0;
                    } else {
                        trajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(path.get(point).getTranslation(), Rotation2d.fromDegrees(-45)), 
                        List.of(path.get(point+1).getTranslation()),
                        new Pose2d(path.get(point+2).getTranslation(), Rotation2d.fromDegrees(0)), 
                        new TrajectoryConfig(2, 1));

                        // trajectory = createTrajectory(path.get(point), path.get(point+1));
    
                        point += 2;
                        // point++;

                        timer.reset();

                        autoState = AutoState.driveToBal;
                    }
                }
            break;
            case driveToBal:
                driving = true;
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .05){                    
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.end;
                }
            break;
            case end:
                driving = false;
                xMode = true;
            break;
        }

        SmartDashboard.putString("AutoState", autoState.toString());
        SmartDashboard.putBoolean("Arm Achieved Position", Arm.getAchivedPostion());
    }
}
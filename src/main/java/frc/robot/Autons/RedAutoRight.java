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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class RedAutoRight extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        pause,
        driveToObject2,
        score2,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(90)),
                                new Pose2d(new Translation2d(5.3,-.35), Rotation2d.fromDegrees(10)), //4.82, .5
                                new Pose2d(new Translation2d(-.1,-.225), Rotation2d.fromDegrees(90)));

    Trajectory trajectory;

    double armTime;

    public RedAutoRight(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(90);

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
                if(!Arm.getAchivedPostion()){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCone;
                    armTime = timer.get();
                } else {
                    if(Math.abs(armTime - timer.get()) < .25){
                        gripperSpeed = .75;
                    } else {
                        armPos = ArmPos.packagePos;
                        gripperSpeed = 0;

                        trajectory = createTrajectory(path.get(point), path.get(point+1),
                        Rotation2d.fromDegrees(-42.5), Rotation2d.fromDegrees(12.5),
                        5,3);
                
                        point++;

                        timer.reset();

                        autoState = AutoState.driveToObject1;
                    }
                }
            break;
            case driveToObject1:
                driving = true;
                armPos = ArmPos.intake;
                if(timer.get() > .6){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                }
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .05){
                    timer.reset();

                    autoState = AutoState.pause;
                }
            break;
            case pause:
                driving = false;

                intakePos = IntakePos.pack;
                intakeSpeed = IntakeSpeed.onCube;

                armPos = ArmPos.intake;
                gripperSpeed = -.4;
                if(timer.get() > .25){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(12 + 180), Rotation2d.fromDegrees(-12 + 180),
                    5,3);

                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
                driving = true;
                intakeOn = false;
                
                intakeSpeed = IntakeSpeed.none;

                if(timer.get() > 1){
                    armPos = ArmPos.topNodeCube;
                } else if (timer.get()>.75){
                    armPos = ArmPos.packagePos;
                }
                gripperSpeed = -.4;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .05){                    
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score2;
                }
            break;
            case score2:
                driving = false;
                if(!Arm.getAchivedPostion()){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCube;
                    armTime = timer.get();
                } else {
                    if(Math.abs(armTime - timer.get()) < .25){
                        gripperSpeed = .5;
                    } else {
                        armPos = ArmPos.packagePos;
                        gripperSpeed = 0;

                        timer.reset();

                        autoState = AutoState.end;
                    }
                }
            case end:
                driving = false;
            break;
        }
    }
}
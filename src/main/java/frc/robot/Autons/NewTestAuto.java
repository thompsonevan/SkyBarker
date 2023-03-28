package frc.robot.Autons;

import edu.wpi.first.wpilibj.Filesystem;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class NewTestAuto extends AutonBase{
    enum AutoState {
        score1,
        driveToObject,
        driveToScore,
        score2,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    Trajectory driveToObject;
    Trajectory driveToScore;
    
    double armTime;

    public NewTestAuto(){
        driveToObject = importTraj("paths/output/blueleft1.wpilib.json");
        driveToScore = importTraj("paths/output/blueleft2.wpilib.json");

        initalPose = new Pose2d(driveToObject.getInitialPose().getTranslation(), Rotation2d.fromDegrees(-90));
        // initalPose = new Pose2d(0,0,new Rotation2d());

        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(initalAngle);
        
        timer.reset();
        timer.start();

        armTime = 0;

        autoState = AutoState.score1;
    }

    public void runAuto(){
        switch(autoState){
            case score1:
                driving = false;

                armPos = ArmPos.packagePos;
                intakePos = IntakePos.none;
                intakeSpeed = IntakeSpeed.none;

                if(timer.get() > 2){
                    timer.reset();

                    autoState = AutoState.driveToObject;
                }
            break;
            case driveToObject:
                driving = true;

                desState = driveToObject.sample(timer.get());
                targetTheta = Rotation2d.fromDegrees(-90);

                if(Math.abs(Drivetrain.getPose().getX() - driveToObject.getStates().get(driveToObject.getStates().size()-1).poseMeters.getX()) < .075 &&
                Math.abs(Drivetrain.getPose().getY() - driveToObject.getStates().get(driveToObject.getStates().size()-1).poseMeters.getY()) < .075){
                }
            break;
            case driveToScore:
            break;
            case score2:
            break;
            case end:
            break;
        }
    }
}
package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.AutonBase;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class AutonCommader extends RobotCommander{

    AutonBase auton;

    public void initAuton(AutonBase passedAuton){
        auton = passedAuton;
    }

    public void runAuto(){
        auton.runAuto();
    }

    @Override
    public double getForwardCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getStrafeCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getTurnCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getResetIMU() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getArmPosition1(){
        return false;
    }

    @Override
    public boolean getArmPosition2(){
        return false;
    }

    public State getDesiredState(){
        return auton.desState;
    }

    public Rotation2d getTargetTheta(){
        return auton.targetTheta;
    }
}

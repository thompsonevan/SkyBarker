// package frc.robot.Autons;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;
// import java.util.concurrent.atomic.AtomicBoolean;

// import com.revrobotics.SparkMaxRelativeEncoder;

// import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.math.trajectory.Trajectory.State;
// import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.AutonCommader;
// import frc.robot.Constants;
// import frc.robot.sensors.Pigeon;
// import frc.robot.subsystems.Drivetrain;

// public class AutonLeft extends AutonCommader {
//     enum AutoState {
//         prepareToShootInitialBall,
//         shootInitialBall,
//         driveToBall,
//         pickUpBall,
//         autoComplete, 
//         returnToStart, 
//         finalShot
//     }

//     private AutoState autoState; 

//     private boolean autonInProgress;

//     private Timer timer;

//     private State lastDesiredState;

//     private boolean driveRequested;

//     private State startState;

//     public AutonLeft() {
//         autoState = AutoState.prepareToShootInitialBall;
//         timer = new Timer();    
//     }

//     @Override
//     public double getForwardCommand() {
//         // TODO Auto-generated method stub
//         return 0;
//     }

//     @Override
//     public double getStrafeCommand() {
//         // TODO Auto-generated method stub
//         return 0;
//     }

//     @Override
//     public double getTurnCommand() {
//         // TODO Auto-generated method stub
//         return 0;
//     }

//     @Override
//     public boolean getResetIMU() {
//         // TODO Auto-generated method stub
//         return false;
//     }

//     @Override
//     public Pose2d getInitialPose() {
//         return lastDesiredState.poseMeters;
//     }




//     @Override
//     public void initializeAuton() {
//         lastDesiredState = new State(0.0, 0.0, 0.0, new Pose2d(0, 0,new Rotation2d(Math.toRadians(315))), 
//         1000);

//         startState = lastDesiredState;
//     }


//     public boolean getAutonInProgress() {
//         return autonInProgress;
//     }


//     public void updateCommand(Pigeon pigeon, Drivetrain drivetrain) {
//         if (autoState == AutoState.prepareToShootInitialBall) {
//             autonInProgress = true;
//             pigeon.initializeAuton(this);
//             autoState = AutoState.shootInitialBall;
//             timer.reset();
//             timer.start();
//         }
//         if (autoState == AutoState.shootInitialBall) {
//             autonInProgress = true;
//             driveRequested = false;            
//             if(timer.get() > .5) { // .5
//                 timer.reset();
//                 timer.start();
//                 drivetrain.initializeAuton(this);
//                 autoState = AutoState.driveToBall;        
//             }
//         }

//         if (autoState == AutoState.driveToBall){
//             autonInProgress = true;
//             driveRequested = true;
//             desiredState = new State(timer.get(), .6/2*timer.get(), .6/2, new Pose2d(lastDesiredState.poseMeters.getX()-2/2*timer.get(),
//                                                                     lastDesiredState.poseMeters.getY()+1.1/2*timer.get(),
//                                                                     lastDesiredState.poseMeters.getRotation()), 
//                                                                     1000);
            
//             setTargetTheta(Rotation2d.fromDegrees(275));
            
//             SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
//             SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
//             SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

//             if (timer.get() > 2) {
//                 lastDesiredState = desiredState;
//                 autoState = AutoState.returnToStart;
//                 timer.reset();
//                 timer.start();
//             }                  
//         }
//         if (autoState == AutoState.returnToStart) {
//             autonInProgress = true;
//             driveRequested = true;
//             deployRightIntake = false;
//             deployLeftIntake = false;
//             hoodPosition = Shot.AUTO2;
//             autoAim = false;

//             SmartDashboard.putNumber("LastTargetX", lastDesiredState.poseMeters.getTranslation().getX());
//             SmartDashboard.putNumber("LastTargetY", lastDesiredState.poseMeters.getTranslation().getY());
//             SmartDashboard.putNumber("LastTargetTheta", lastDesiredState.poseMeters.getRotation().getDegrees());

//             desiredState = new State(timer.get(), .075/1.5*timer.get(),  .075/1.5, new Pose2d(lastDesiredState.poseMeters.getX() + (startState.poseMeters.getX() - lastDesiredState.poseMeters.getX())/1.5*timer.get(),
//                                                                     lastDesiredState.poseMeters.getY() + (startState.poseMeters.getY() - lastDesiredState.poseMeters.getY())/1.5*timer.get(),
//                                                                     lastDesiredState.poseMeters.getRotation()), 
//                                                                     1000);
//             setTargetTheta(Rotation2d.fromDegrees(315));

//             SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
//             SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
//             SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
//             if (timer.get() > 1.5) {
//                 autoState = AutoState.finalShot;
//                 lastDesiredState = desiredState;
//                 timer.reset();
//                 timer.start();
//             }              
//         }
//         if(autoState == AutoState.finalShot){
//             autonInProgress = true;
//             driveRequested = false;
//             autoAim = false;
//             shoot = true;
            
//             if(timer.get() > 3) {
//                 timer.reset();
//                 timer.start();
//                 autoState = AutoState.autoComplete;        
//             }
//         }
//         if (autoState == AutoState.autoComplete) {
//             deployRightIntake = false;
//             deployLeftIntake = false;
//             autoAim = false;
//             autonInProgress = false;
//             driveRequested = false;
//             hoodPosition = Shot.NEUTRAL;
//         }
       
//         SmartDashboard.putString("AutoState", ""+autoState);

//         SmartDashboard.putNumber("Start Pose X", startState.poseMeters.getX());
//         SmartDashboard.putNumber("Start Pose Y", startState.poseMeters.getY());
        
//     }




//     @Override
//     public boolean getOverrideShooterMotor() {
//         // TODO Auto-generated method stub
//         return false;
//     }




//     @Override
//     public boolean getOverrideBallivatorMotor() {
//         // TODO Auto-generated method stub
//         return false;
//     }




//     @Override
//     public boolean getOverrideIntakmotor() {
//         // TODO Auto-generated method stub
//         return false;
//     }




//     @Override
//     public boolean getDriveRequested() {
//         // TODO Auto-generated method stub
//         return driveRequested;
//     }

//     @Override
//     public double getRightIntakeCommand() {
//         // TODO Auto-generated method stub
//         return 0;
//     }




//     @Override
//     public double getLeftIntakeCommand() {
//         // TODO Auto-generated method stub
//         return 0;
//     }




//     @Override
//     public boolean getAutoAimSetTarget() {
//         // TODO Auto-generated method stub
//         return false;
//     }




//     @Override
//     public boolean getAutoIntakeDeploy() {
//         // TODO Auto-generated method stub
//         return false;
//     }




//     @Override
//     public boolean getYButtonHeld() {
//         // TODO Auto-generated method stub
//         return false;
//     }
// }

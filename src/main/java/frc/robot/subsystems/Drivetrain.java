package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants;
import frc.robot.sensors.Camera2;
import frc.robot.sensors.Pigeon;

public class Drivetrain {
    
    private HolonomicDriveController holonomicController;
    private SwerveDrivePoseEstimator poseEstimator;
    
    
    private TalonFX frontLeftDrive;
    private TalonFX frontRightDrive;
    private TalonFX backLeftDrive;
    private TalonFX backRightDrive;
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private double frontLeftPos;
    private double frontRightPos;
    private double backLeftPos;
    private double backRightPos;
    private SwerveModulePosition[] positions;
    private SwerveModuleState[] states;

    private Pigeon pigeon;


    private SwerveDriveOdometry odometry;
    private ChassisSpeeds currentChassisSpeeds;

    
    public ChassisSpeeds getCurrentChassisSpeeds() {
        return currentChassisSpeeds;
    }

    public Drivetrain(Pigeon pigeon, double initialAngle) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        this.pigeon = pigeon;

        if(Constants.realBot){
            frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                    tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                    .withSize(2, 4)
                                    .withPosition(0, 0),
                                    Mk4iSwerveModuleHelper.GearRatio.L2,
                                    Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                    Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                    Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                    Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

            frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(2, 0),
                                            Mk4iSwerveModuleHelper.GearRatio.L2,
                                            Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                            Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

            backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(4, 0),
                            Mk4iSwerveModuleHelper.GearRatio.L2,
                            Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                            Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                            Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                            Constants.BACK_LEFT_MODULE_STEER_OFFSET);

            backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(6, 0),
                            Mk4iSwerveModuleHelper.GearRatio.L2,
                            Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                            Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                            Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                            Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

            frontLeftDrive = frontLeftModule.getDriveMotor();
            frontRightDrive = frontRightModule.getDriveMotor();
            backLeftDrive = backLeftModule.getDriveMotor();
            backRightDrive = backRightModule.getDriveMotor();

            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        } else {
            frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

            frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(2, 0),
                                            Mk4SwerveModuleHelper.GearRatio.L2,
                                            Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                            Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

            backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(4, 0),
                                            Mk4SwerveModuleHelper.GearRatio.L2,
                                            Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                            Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                                            Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                                            Constants.BACK_LEFT_MODULE_STEER_OFFSET);

            backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(6, 0),
                                            Mk4SwerveModuleHelper.GearRatio.L2,
                                            Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                            Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                            Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                            Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

            frontLeftDrive = frontLeftModule.getDriveMotor();
            frontRightDrive = frontRightModule.getDriveMotor();
            backLeftDrive = backLeftModule.getDriveMotor();
            backRightDrive = backRightModule.getDriveMotor();

            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        
            positions = new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    
            positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
            positions[0].distanceMeters = frontLeftPos;
    
            positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
            positions[1].distanceMeters = frontRightPos;
    
            positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
            positions[2].distanceMeters = backLeftPos;
    
            positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
            positions[3].distanceMeters = backRightPos;
    
            states = Constants.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds());
    
            ProfiledPIDController thetaController = new ProfiledPIDController(1,.25, 0,
                                                    new TrapezoidProfile.Constraints(6.28, 3.14));
    
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
            holonomicController = new HolonomicDriveController(
                new PIDController(3.5, .6, .025),  //x Long side of field
                new PIDController(3.5, .6, .025), //y Short side of field
                thetaController); // (2Pk,PI) constrains to 1 2pi/sec
    
            Rotation2d initialRotaion = Rotation2d.fromDegrees(initialAngle);

            poseEstimator = new SwerveDrivePoseEstimator(
                Constants.KINEMATICS,
                initialRotaion,
                positions,
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
            );

            poseEstimator.resetPosition(initialRotaion, positions, new Pose2d(0, 0, initialRotaion));

            odometry = new SwerveDriveOdometry(Constants.KINEMATICS, initialRotaion, positions);

            odometry.resetPosition(initialRotaion, positions, new Pose2d(0, 0, initialRotaion));
        }
    }

    public void resetSteer(){
        frontLeftModule.zeroMotorPos();
        frontRightModule.zeroMotorPos();
        backLeftModule.zeroMotorPos();
        backRightModule.zeroMotorPos();
    }

    public void updatePose(Camera2 camera) {
        if(Constants.realBot){
            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        } else {
            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        }

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        if (camera.aprilTagsDetected()) {
            poseEstimator.addVisionMeasurement(camera.getBotPose(), Timer.getFPGATimestamp());
        }

        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), pigeon.getRotation2dWrapped(), positions);

        odometry.update(pigeon.getRotation2dWrapped(), positions);

        SmartDashboard.putNumber("Estimated Theta", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Estimated X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Estimated Y", poseEstimator.getEstimatedPosition().getY());

        
        SmartDashboard.putNumber("odometry Theta", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("odometry Y", odometry.getPoseMeters().getY());
    }

    private double modifyAxis(double returnAxis, double otherAxis) {
        boolean deadband = 0.13 > Math.sqrt(Math.pow(returnAxis, 2) + Math.pow(otherAxis, 2));

        if (deadband) {
            return 0;
        } else {
            return Math.abs(returnAxis) * returnAxis;
        }
    }

    private double deadband(double value, double deadband, double maxRange){
        if(Math.abs(value) < deadband){
            return 0;
        } else if (value < 0) {
            return  ((value + deadband)/(1.0 - deadband)) * maxRange;
        } else {
            return  ((value - deadband)/(1.0 - deadband)) * maxRange;
        }
    }

    public void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        this.currentChassisSpeeds = chassisSpeeds;
        states = Constants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
                        states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
                        states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
                        states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
                        states[3].angle.getRadians());
    }

    public void driveToPoint(State desiredState, Rotation2d targetRotation) {
        ChassisSpeeds chassisSpeeds = holonomicController.calculate(poseEstimator.getEstimatedPosition(), 
                                                                    desiredState,
                                                                    targetRotation);

                                                                    System.out.print("X: ");
                                                                    System.out.print(desiredState.poseMeters.getX());
                                                                    System.out.print(", Y: ");
                                                                    System.out.print(desiredState.poseMeters.getY());
                                                                    System.out.print(", Theta: ");

        SmartDashboard.putNumber("Commanded Theta", targetRotation.getDegrees());
        SmartDashboard.putNumber("Commanded X", desiredState.poseMeters.getX());
        SmartDashboard.putNumber("Commanded Y", desiredState.poseMeters.getY());

        setSwerveModuleStates(chassisSpeeds);
    }

    public void driveWithController(XboxController driver) {
            
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -modifyAxis(driver.getLeftY(),driver.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                -modifyAxis(driver.getLeftX(),driver.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                deadband(Math.abs(driver.getRightX()) * driver.getRightX(), 0.13, 0.4) * (Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                pigeon.getRotation2dWrapped());

            setSwerveModuleStates(chassisSpeeds);
    }
}

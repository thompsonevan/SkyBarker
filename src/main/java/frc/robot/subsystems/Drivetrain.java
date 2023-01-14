package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.math.Nat;
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
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.AutonCommader;
import frc.robot.Constants;
import frc.robot.RobotCommander;
import frc.robot.TeleopCommander;
import frc.robot.sensors.Pigeon;

import static frc.robot.Constants.*;

import static frc.robot.sensors.Pigeon.*;

public class Drivetrain{
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final TalonFX frontLeftDrive;
    private final TalonFX frontRightDrive;
    private final TalonFX backLeftDrive;
    private final TalonFX backRightDrive;

    private final TalonFX frontLeftSteer;
    private final TalonFX frontRightSteer;
    private final TalonFX backLeftSteer;
    private final TalonFX backRightSteer;

    private final AbsoluteEncoder frontLeftEncoder;
    private final AbsoluteEncoder frontRightEncoder;
    private final AbsoluteEncoder backLeftEncoder;
    private final AbsoluteEncoder backRightEncoder;

    static SwerveModulePosition[] positions;

    public static SwerveDrivePoseEstimator poseEstimator;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SwerveModuleState[] states;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private HolonomicDriveController holonomicController;
    private static SwerveDriveOdometry odometry;

    double frontLeftPos;
    double frontRightPos;
    double backLeftPos;
    double backRightPos;


    // private SwerveDrivePoseEstimator poseExstimator;

    public Drivetrain(){
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(2, 0),
                        Mk4SwerveModuleHelper.GearRatio.L2,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(4, 0),
                        Mk4SwerveModuleHelper.GearRatio.L2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(6, 0),
                        Mk4SwerveModuleHelper.GearRatio.L2,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET);

        frontLeftDrive = frontLeftModule.getDriveMotor();
        frontRightDrive = frontRightModule.getDriveMotor();
        backLeftDrive = backLeftModule.getDriveMotor();
        backRightDrive = backRightModule.getDriveMotor();

        frontLeftSteer = frontLeftModule.getSteerMotor();
        frontRightSteer = frontRightModule.getSteerMotor();
        backLeftSteer = backLeftModule.getSteerMotor();
        backRightSteer = backRightModule.getSteerMotor();

        frontLeftEncoder = frontLeftModule.getEncoder();
        frontRightEncoder = frontRightModule.getEncoder();
        backLeftEncoder = backLeftModule.getEncoder();
        backRightEncoder = backRightModule.getEncoder();

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

        odometry = new SwerveDriveOdometry(KINEMATICS, Pigeon.getRotation2d(), positions);

        states = kinematics.toSwerveModuleStates(new ChassisSpeeds());

        ProfiledPIDController thetaController = new ProfiledPIDController(1.7, .3, 0,
                                                new TrapezoidProfile.Constraints(6.28, 3.14));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicController = new HolonomicDriveController(
            new PIDController(1.8, .5, 0),  //x Long side of field
            new PIDController(2.35, .5, 0), //y Short side of field
            thetaController); // (2Pk,PI) constrains to 1 2pi/sec

        poseEstimator = new SwerveDrivePoseEstimator(
            KINEMATICS,
            Pigeon.getRotation2d(),
            positions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
    }

    public void zero(){
        frontLeftDrive.setSelectedSensorPosition(0);
        frontRightDrive.setSelectedSensorPosition(0);
        backLeftDrive.setSelectedSensorPosition(0);
        backRightDrive.setSelectedSensorPosition(0);

        frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();

        positions[0].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        poseEstimator.resetPosition(Pigeon.getRotation2d(),
                                    positions,
                                    new Pose2d());

        odometry.resetPosition(Pigeon.getRotation2d(), positions, new Pose2d());
    }

    private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[3].angle.getRadians());
    }

    public void setPose(Pose2d pose){
        frontLeftDrive.setSelectedSensorPosition(0);
        frontRightDrive.setSelectedSensorPosition(0);
        backLeftDrive.setSelectedSensorPosition(0);
        backRightDrive.setSelectedSensorPosition(0);

        frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();

        positions[0].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        odometry.resetPosition(Pigeon.getRotation2d(), positions, pose);
    }

    public void teleAction(TeleopCommander commander){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            commander.getForwardCommand(),
            commander.getStrafeCommand(),
            commander.getTurnCommand(),
            Pigeon.getRotation2d());

        setSwerveModuleStates(chassisSpeeds);

        SmartDashboard.putNumber("X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Theta", Rotation2d.fromDegrees(Pigeon.getAngle()).getDegrees());

    }

    public void disabled(){
        SmartDashboard.putNumber("Encoder Driven", ((backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction()) * (Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()));
        SmartDashboard.putNumber("Moudle Angle", backLeftEncoder.getAbsoluteAngle());
    }

    public static Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public void autonAction(AutonCommader autonCommader){
        frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        poseEstimator.update(Pigeon.getRotation2d(), positions);

        odometry.update(Pigeon.getRotation2d(), positions);

        setSwerveModuleStates(holonomicController.calculate(poseEstimator.getEstimatedPosition(), 
                                                            autonCommader.getDesiredState(),
                                                            autonCommader.getDesiredState().poseMeters.getRotation()));

        SmartDashboard.putNumber("Theta", Rotation2d.fromDegrees(Pigeon.getAngle()).getDegrees());
        SmartDashboard.putNumber("X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometry.getPoseMeters().getY());
    }
}

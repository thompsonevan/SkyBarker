package frc.robot;

import java.util.List;

import org.hotutilites.hotlogger.HotLogger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.BlueAutoLeft;
import frc.robot.Autons.BlueAutoLeft3;
import frc.robot.Autons.BlueAutoLeftBalance;
import frc.robot.Autons.BlueAutoMid1Bal;
import frc.robot.Autons.CableAuto;
import frc.robot.Autons.NewTestAuto;
import frc.robot.Autons.OhCrap;
import frc.robot.Autons.RedAutoMid1Bal;
import frc.robot.Autons.RedAutoRight;
import frc.robot.Autons.RedAutoRight3;
import frc.robot.Autons.RedAutoRightBalance;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;

public class Robot extends TimedRobot {
    private TeleopCommander teleopCommander;
    private Drivetrain drivetrain;
    private Hopper hopper;
    private Arm arm;
    private Pigeon pigeon;
    private Camera camera;
    private AutonCommader autonCommader;
    private Gripper gripper;
    private BlueAutoLeft blueAutoLeft;
    private OhCrap ohCrap;
    private RedAutoRight redAutoRight;
    private BlueAutoMid1Bal blueAutoMid1Bal;
    private BlueAutoLeft3 blueAutoLeft3;
    private RedAutoRightBalance redAutoRightBalance;
    private RedAutoRight3 redAutoRight3;
    private BlueAutoLeftBalance blueAutoLeftBalance;
    private CableAuto cableAuto;
    private NewTestAuto newTestAuto;

    LED leds;

    private String autonSelection = "Red Mid 1";

    private Alliance alliance;

    private String autoSelected = "Blue Left";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed",
        "TargetX", "TargetY", "TargetTheta", "Pose Theta", "poseX", "poseY",
        "Shoulder Absolute Pos", "Shoulder Motor Pos", "Extension Pos", "Elbow Absolute Pos", "Elbow Motor Pos",
        "Shoulder Desired Pos", "Extension Desired Pos", "Elbow Desired Pos","determineArmZoneHandOff","DesiredIntakeAngle",
        "Commanded Extension Position", "AutoState", "Driving");

        m_chooser.setDefaultOption("Blue Left", "Blue Left");
        m_chooser.addOption("Red Right", "Red Right");
        m_chooser.addOption("Blue Mid 1", "Blue Mid 1");
        m_chooser.addOption("Red Mid 1 (Intake Towards Right)", "Red Mid 1");
        m_chooser.addOption("Blue Left 3", "Blue Left 3");
        m_chooser.addOption("Red Right 3", "Red Right 3");
        m_chooser.addOption("Red Right Balance", "Red Right Balance");
        m_chooser.addOption("Blue Left Balance", "Blue Left Balance");
        m_chooser.addOption("Cable", "Cable");
        m_chooser.addOption("Test", "Test");


        Shuffleboard.getTab("Competition")
        .add("Auto Selector", m_chooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2, 2);

        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        camera = new Camera();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        arm = new Arm();
        gripper = new Gripper(Constants.GRIPPER);
        hopper = new Hopper();
        blueAutoLeft = new BlueAutoLeft();
        ohCrap = new OhCrap();
        redAutoRight = new RedAutoRight();
        blueAutoMid1Bal = new BlueAutoMid1Bal();
        blueAutoLeft3 = new BlueAutoLeft3();
        leds = new LED();
        redAutoRightBalance = new RedAutoRightBalance();
        redAutoRight3 = new RedAutoRight3();
        blueAutoLeftBalance = new BlueAutoLeftBalance();
        cableAuto = new CableAuto();
        newTestAuto = new NewTestAuto();

        camera.disabled();
    }

    @Override
    public void robotPeriodic() {
        arm.updatePose();
        camera.logData();
        pigeon.logData();
        hopper.logData();
        arm.logdata();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("FPGA Time", Timer.getFPGATimestamp());
        drivetrain.updatePose();
        SmartDashboard.putString("Auton Selected", autoSelected);
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Robot Mode", "Disabled");

        camera.disabled();
        // arm.coastMode();

        leds.disabledAction();
    }

    @Override
    public void disabledPeriodic() {
        arm.armPercentOutZero();
        // arm.coastMode();
    }

    @Override
    public void autonomousInit() {
        autonSelection = m_chooser.getSelected();

        SmartDashboard.putString("Robot Mode", "Autonomous");

        SmartDashboard.getString("Auton Selection", autonSelection);

        alliance = DriverStation.getAlliance();

        if(autonSelection == "Blue Left"){
            autonCommader.initAuton(blueAutoLeft);
        } else if(autonSelection == "Red Right"){
            autonCommader.initAuton(redAutoRight);
        }else if(autonSelection == "Blue Mid 1"){
            autonCommader.initAuton(blueAutoMid1Bal);
        }else if(autonSelection == "Red Mid 1"){
            alliance = Alliance.Blue;
            autonCommader.initAuton(blueAutoMid1Bal);
        } else if(autonSelection == "Blue Left 3"){
            autonCommader.initAuton(blueAutoLeft3);
        } else if (autonSelection == "Red Right Balance"){
            autonCommader.initAuton(redAutoRightBalance);
        } else if(autonSelection == "Red Right 3"){
            autonCommader.initAuton(redAutoRight3);
        } else if(autonSelection == "Blue Left Balance"){
            autonCommader.initAuton(blueAutoLeftBalance);
        } else if(autonSelection == "Cable"){
            alliance = Alliance.Blue;
            autonCommader.initAuton(cableAuto);
        } else if(autonSelection == "Test"){
            autonCommader.initAuton(newTestAuto);
        }else {
            autonCommader.initAuton(ohCrap);
        }

        autonCommader.allaince = alliance;

        Pigeon.zeroSensor(autonCommader.auton.initalPose.getRotation().getDegrees());
        drivetrain.zero(autonCommader.auton.initalPose.getRotation().getDegrees(), autonCommader.auton.initalPose);
        autonCommader.auton.reset();

        leds.autonInit();

        // camera.enabled();

        arm.initilizeOffsets();
    }

    @Override
    public void autonomousPeriodic() {
        autonCommader.runAuto();
        pigeon.enabledAction(teleopCommander);
        drivetrain.autonAction(autonCommader);
        arm.action(autonCommader);
        hopper.HopperPeriodic(autonCommader);
        gripper.action(autonCommader);

        leds.autonAction();
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Robot Mode", "Teleop");

        teleopCommander.allaince = alliance;

        drivetrain.zero();

        // camera.enabled();

        arm.brakeMode();
        arm.initilizeOffsets();
    }

    @Override
    
    public void teleopPeriodic() {
        pigeon.enabledAction(teleopCommander);
        drivetrain.teleAction(teleopCommander);
        arm.action(teleopCommander);
        arm.brakeMode();
        gripper.action(teleopCommander);
        hopper.HopperPeriodic(teleopCommander);

        leds.teleopAction(teleopCommander);
    }
}

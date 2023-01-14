package frc.robot.Autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

public abstract class AutonBase {
    public State desState;
    public Rotation2d targetTheta;
    public Timer timer;

    public abstract void runAuto();
}

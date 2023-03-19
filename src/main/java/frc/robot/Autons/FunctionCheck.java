package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;

public class FunctionCheck extends AutonBase{

    enum TestState{
        intakeOut,
        intakeIn,
        pack1,
        high,
        cube,
        cone,
        bumpUp,
        bumpDown,
        release,
        mid,
        low,
        pack2,
        humanPlayerReady,
        humanPlayerPickup,
        pack3,
        swerveSpin,
        end
    }

    TestState curState;

    Timer timer = new Timer();

    public FunctionCheck(){
        reset();
    }

    @Override
    public void reset() {
        curState = TestState.intakeOut;

        timer.start();
        timer.reset();
    }

    @Override
    public void runAuto() {
        switch(curState){
            case intakeOut:
            break;
            case intakeIn:
            break;
            case pack1:
            break;
            case high:
            break;
            case cube:
            break;
            case cone:
            break;
            case bumpUp:
            break;
            case bumpDown:
            break;
            case release:
            break;
            case mid:
            break;
            case low:
            break;
            case pack2:
            break;
            case humanPlayerReady:
            break;
            case humanPlayerPickup:
            break;
            case pack3:
            break;
            case swerveSpin:
            break;
            case end:
            break;
        }
    }
}

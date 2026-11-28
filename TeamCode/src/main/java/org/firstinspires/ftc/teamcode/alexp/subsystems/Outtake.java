package org.firstinspires.ftc.teamcode.alexp.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.config.OuttakeConfig;

public class Outtake {
    public DcMotor outtakeMotor;
    private ElapsedTime buttonTimer;
    public enum OuttakeState {
        SHOOTING, STOPPED
    }
    public Outtake.OuttakeState outtakeState = Outtake.OuttakeState.STOPPED;
    private double outtakeSpeed = OuttakeConfig.OUTTAKE_SPEED;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        outtakeMotor = hwMap.get(DcMotor.class, HardwareMapConfig.OUTTAKE_MOTOR_ID);
        outtakeMotor.setDirection(OuttakeConfig.OUTTAKE_MOTOR_DIRECTION);
        stop();
        buttonTimer = new ElapsedTime();
        buttonTimer.reset();
    }

    public void drive(boolean shoot, boolean speed_up, boolean speed_down) {
        if (buttonTimer.milliseconds() > 200) {
            if (shoot) {
                spit();
                buttonTimer.reset();
            }
            if (speed_up) {
                set_speed(get_speed()+0.05);
                buttonTimer.reset();
            }
            else if (speed_down) {
                set_speed(get_speed()-0.05);
                buttonTimer.reset();
            }
        }
    }
    public void spit() {
        if (outtakeState == OuttakeState.SHOOTING) {
            stop();
        } else {
            outtakeMotor.setPower(-outtakeSpeed);
            outtakeState = Outtake.OuttakeState.SHOOTING;
        }
    }

    public void stop() {
        outtakeMotor.setPower(0);
        outtakeState = Outtake.OuttakeState.STOPPED;
    }

    public double get_speed() {
        return outtakeSpeed;
    }

    public void set_speed(double newSpeed) {
        newSpeed = Range.clip(newSpeed, -1.0, 1.0);
        outtakeSpeed = newSpeed;
    }
}

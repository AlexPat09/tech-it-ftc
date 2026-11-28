package org.firstinspires.ftc.teamcode.alexp.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.config.IntakeConfig;


public class Intake {
    public DcMotor intakeMotor;
    private ElapsedTime buttonTimer;

    public enum IntakeState {
        COLLECTING, DROPPING, STOPPED
    }
    public IntakeState intakeState = IntakeState.STOPPED;
    private double intakeSpeed = IntakeConfig.INTAKE_SPEED;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        intakeMotor = hwMap.get(DcMotor.class, HardwareMapConfig.INTAKE_MOTOR_ID);
        intakeMotor.setDirection(IntakeConfig.INTAKE_MOTOR_DIRECTION);
        stop();
        buttonTimer = new ElapsedTime();
        buttonTimer.reset();
    }

    public void swallow() {
        if (intakeState == IntakeState.COLLECTING) {
            stop();
        } else {
            intakeMotor.setPower(intakeSpeed);
            intakeState = IntakeState.COLLECTING;
        }
    }

    public void spit() {
        if (intakeState == IntakeState.DROPPING) {
            stop();
        } else {
            intakeMotor.setPower(-intakeSpeed);
            intakeState = IntakeState.DROPPING;
        }
    }

    public void stop() {
        intakeMotor.setPower(0);
        intakeState = IntakeState.STOPPED;
    }

    public void drive(boolean swallow, boolean spit) {
        if (buttonTimer.milliseconds() > 200) {
            if (swallow) {
                swallow();
                buttonTimer.reset();
            } else if (spit) {
                spit();
                buttonTimer.reset();
            }
        }
    }

    public double get_speed() {
        return intakeSpeed;
    }

    public void set_speed(double newSpeed) {
        newSpeed = Range.clip(newSpeed, -1.0, 1.0);
        intakeSpeed = newSpeed;
    }
}

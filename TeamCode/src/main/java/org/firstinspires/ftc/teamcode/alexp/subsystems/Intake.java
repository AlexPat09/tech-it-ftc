package org.firstinspires.ftc.teamcode.alexp.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.config.IntakeConfig;


public class Intake {
    public DcMotor intakeMotor;
    public enum IntakeState {
        COLLECTING, DROPPING, STOPPED
    }

    public IntakeState intakeState = IntakeState.STOPPED;
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        intakeMotor = hwMap.get(DcMotor.class, HardwareMapConfig.INTAKE_MOTOR_ID);
        intakeMotor.setDirection(IntakeConfig.INTAKE_MOTOR_DIRECTION);
        stop();
    }

    public void swallow() {
        intakeMotor.setPower(IntakeConfig.INTAKE_SPEED);
        intakeState = IntakeState.COLLECTING;
    }

    public void spit() {
        intakeMotor.setPower(-IntakeConfig.INTAKE_SPEED);
        intakeState = IntakeState.DROPPING;
    }

    public void stop() {
        intakeMotor.setPower(0);
        intakeState = IntakeState.STOPPED;
    }
}

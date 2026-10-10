package org.firstinspires.ftc.teamcode.alexp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.subsystems.TechItDrive;

@Config
@TeleOp(name = "FIRST TECH IT TEST", group = "TECH IT")
public class TechItTeleOp extends OpMode {
    private TechItDrive drive;
    private ElapsedTime buttonTimer;
    private DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, HardwareMapConfig.INTAKE_MOTOR_ID);
        buttonTimer = new ElapsedTime();
        drive = new TechItDrive();
        drive.init(hardwareMap, telemetry, 0.6);
    }

    @Override
    public void loop() {
        if (buttonTimer.milliseconds() > 200) {
            if (gamepad1.dpad_up) {
                drive.set_max_speed(drive.get_max_speed()+0.05);
                buttonTimer.reset();
            }
            else if (gamepad1.dpad_down) {
                drive.set_max_speed(drive.get_max_speed()-0.05);
                buttonTimer.reset();
            }
        }

        drive.drive(-gamepad1.left_stick_y,gamepad1.right_stick_x);
        telemetry.update();
    }
}
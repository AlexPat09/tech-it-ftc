package org.firstinspires.ftc.teamcode.alexp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.alexp.subsystems.MecanumDrive;

@TeleOp(name = "MechanumTeleOp", group = "TECH IT")
public class MecanumTeleOp extends OpMode {
    MecanumDrive mecanumDrive;
    ElapsedTime buttonTimer;

    @Override
    public void init() {
        buttonTimer = new ElapsedTime();
        mecanumDrive = new MecanumDrive();
        mecanumDrive.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (buttonTimer.milliseconds() > 200) {
            if (gamepad1.dpad_up) {
                mecanumDrive.set_max_speed(mecanumDrive.get_max_speed()+0.01);
                buttonTimer.reset();
            }
            else if (gamepad1.dpad_down) {
                mecanumDrive.set_max_speed(mecanumDrive.get_max_speed()-0.01);
                buttonTimer.reset();
            }
        }
        mecanumDrive.driveFieldCentric(-gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.update();
    }
}

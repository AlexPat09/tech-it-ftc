package org.firstinspires.ftc.teamcode.alexp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.subsystems.Intake;
import org.firstinspires.ftc.teamcode.alexp.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "PedroPathingTeleOp")
public class PedroPathingTeleOp extends OpMode {
    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, HardwareMapConfig.INTAKE_MOTOR_ID);
        outtakeMotor = hardwareMap.get(DcMotor.class, HardwareMapConfig.OUTTAKE_MOTOR_ID);
        intake = new Intake();
        outtake = new Outtake();
        intake.init(hardwareMap,telemetry);
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,0));
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );
        intake.drive(gamepad1.left_bumper,gamepad1.right_bumper);
        outtake.drive(gamepad1.aWasPressed(), gamepad1.dpad_up, gamepad1.dpad_down);

        ManualDrive.driveOrHold(follower, powers);
        follower.update();
        Pose robotPose = follower.pose();

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}

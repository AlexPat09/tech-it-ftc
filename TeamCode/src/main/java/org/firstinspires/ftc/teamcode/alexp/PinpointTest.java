package org.firstinspires.ftc.teamcode.alexp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Test Code", group = "Test")
public class PinpointTest extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(11.5, -2.5, DistanceUnit.CM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.setPosition(new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES,0));

        telemetry.addLine("Pinpoint Initialized. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Must call update() once per loop to refresh position and velocity data
            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();
            telemetry.addData("X Position (mm)", pose.getX(DistanceUnit.MM));
            telemetry.addData("Y Position (mm)", pose.getY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.update();
        }
    }
}

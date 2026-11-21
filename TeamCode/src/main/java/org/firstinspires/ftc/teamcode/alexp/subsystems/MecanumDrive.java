package org.firstinspires.ftc.teamcode.alexp.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.config.MecanumDriveConfig;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;
    private double maxSpeed = 1.0;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(11.5, -2.5, DistanceUnit.CM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
        telemetry.addData("Pinpoint", "INITIALIZED");

        frontLeftMotor = hwMap.get(DcMotor.class, HardwareMapConfig.FRONT_LEFT_DRIVE_ID);
        frontRightMotor = hwMap.get(DcMotor.class, HardwareMapConfig.FRONT_RIGHT_DRIVE_ID);
        backLeftMotor = hwMap.get(DcMotor.class, HardwareMapConfig.BACK_LEFT_DRIVE_ID);
        backRightMotor = hwMap.get(DcMotor.class, HardwareMapConfig.BACK_RIGHT_DRIVE_ID);

        frontLeftMotor.setDirection(MecanumDriveConfig.FRONT_LEFT_DRIVE_DIRECTION);
        backLeftMotor.setDirection(MecanumDriveConfig.BACK_LEFT_DRIVE_DIRECTION);
        frontRightMotor.setDirection(MecanumDriveConfig.FRONT_RIGHT_DRIVE_DIRECTION);
        backRightMotor.setDirection(MecanumDriveConfig.BACK_RIGHT_DRIVE_DIRECTION);

        frontLeftMotor.setZeroPowerBehavior(MecanumDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        frontRightMotor.setZeroPowerBehavior(MecanumDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        backLeftMotor.setZeroPowerBehavior(MecanumDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        backRightMotor.setZeroPowerBehavior(MecanumDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        telemetry.addData("Drive Train", "INITIALIZED");
    }

    public void updateOdometry() {
        pinpoint.update();
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = -forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = -forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldCentric(double forward, double strafe, double rotate) {
        updateOdometry();

        updateOdometry();

        double botHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

        double newForward = forward * Math.cos(-botHeading) - strafe * Math.sin(-botHeading);
        double newStrafe  = forward * Math.sin(-botHeading) + strafe * Math.cos(-botHeading);

        this.drive(newForward, newStrafe, rotate);
    }

    public void logTelemetry() {
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.addData("X Position (cm)", pose.getX(DistanceUnit.CM));
        telemetry.addData("Y Position (cm)", pose.getY(DistanceUnit.CM));
        telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public void resetHeading() {
        Pose2D current = pinpoint.getPosition();
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.CM,
                current.getX(DistanceUnit.CM),
                current.getY(DistanceUnit.CM),
                AngleUnit.DEGREES,
                0
        ));
    }

    public double get_max_speed() {
        return maxSpeed;
    }

    public void set_max_speed(double newSpeed) {
        newSpeed = Range.clip(newSpeed, -1.0, 1.0);
        maxSpeed = newSpeed;
    }
}

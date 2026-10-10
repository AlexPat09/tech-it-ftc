package org.firstinspires.ftc.teamcode.alexp.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.config.MecanumDriveConfig;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    private Telemetry telemetry;
    private double maxSpeed = 1.0;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
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

        //if (ROBOT_IMU_TYPE == MecanumDriveConfig.ImuType.BHI260)
        //    imu = hwMap.get(BHI260IMU.class, HardwareMapConfig.IMU_ID);
        //else
        //    imu = hwMap.get(IMU.class, HardwareMapConfig.IMU_ID);

        imu = hwMap.get(IMU.class, HardwareMapConfig.IMU_ID);

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        MecanumDriveConfig.CONTROL_HUB_LOGO_DIRECTION,
                        MecanumDriveConfig.CONTROL_HUB_USB_PORT_DIRECTION);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        this.telemetry = telemetry;
        telemetry.addData("Drive Train", "INITIALIZED");
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
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
        double theta = Math.atan2(forward,strafe);
        double r = Math.hypot(forward,strafe);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    public double get_max_speed() {
        return maxSpeed;
    }

    public void set_max_speed(double newSpeed) {
        newSpeed = Range.clip(newSpeed, -1.0, 1.0);
        maxSpeed = newSpeed;
    }
}

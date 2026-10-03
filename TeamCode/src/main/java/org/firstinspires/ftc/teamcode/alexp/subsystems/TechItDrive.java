package org.firstinspires.ftc.teamcode.alexp.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.alexp.config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.alexp.config.MecanumDriveConfig;
import org.firstinspires.ftc.teamcode.alexp.config.TechItDriveConfig;

public class TechItDrive {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    private Telemetry telemetry;
    private double maxSpeed;

    public void init(HardwareMap hwMap, Telemetry telemetry, double maxSpeed) {
        this.maxSpeed = maxSpeed;
        //frontLeftMotor = hwMap.get(DcMotor.class, HardwareMapConfig.FRONT_LEFT_DRIVE_ID);
        //frontRightMotor = hwMap.get(DcMotor.class, HardwareMapConfig.FRONT_RIGHT_DRIVE_ID);
        backLeftMotor = hwMap.get(DcMotor.class, HardwareMapConfig.BACK_LEFT_DRIVE_ID);
        backRightMotor = hwMap.get(DcMotor.class, HardwareMapConfig.BACK_RIGHT_DRIVE_ID);

        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //frontLeftMotor.setZeroPowerBehavior(TechItDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        //frontRightMotor.setZeroPowerBehavior(TechItDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        backLeftMotor.setZeroPowerBehavior(TechItDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        backRightMotor.setZeroPowerBehavior(TechItDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);

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

    public void drive(double forward, double turn) {
        double rotationalSensitivity = 1;
        //double frontLeftPower = Range.clip(forward + (rotationalSensitivity*turn), -1.0, 1.0);
        double backLeftPower = Range.clip(forward + (rotationalSensitivity*turn), -1.0, 1.0);
        //double frontRightPower = Range.clip(forward - (rotationalSensitivity*turn), -1.0, 1.0);
        double backRightPower = Range.clip(forward - (rotationalSensitivity*turn), -1.0, 1.0);

        //frontLeftMotor.setPower(maxSpeed * (frontLeftPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower));
        //frontRightMotor.setPower(maxSpeed * (frontRightPower));
        backRightMotor.setPower(maxSpeed * (backRightPower));
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", maxSpeed*backLeftPower, maxSpeed*backRightPower);
    }

    public double get_max_speed() {
        return maxSpeed;
    }

    public void set_max_speed(double newSpeed) {
        newSpeed = Range.clip(newSpeed, -1.0, 1.0);
        maxSpeed = newSpeed;
    }
}

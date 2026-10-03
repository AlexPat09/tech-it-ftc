package org.firstinspires.ftc.teamcode.alexp.config;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@Config
public class MecanumDriveConfig {
    public enum ImuType {
        BHI260, BNO055
    }

    // Non-Tunable constants
    public final static ImuType ROBOT_IMU_TYPE = ImuType.BNO055;

    public final static DcMotorSimple.Direction FRONT_LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public final static DcMotorSimple.Direction BACK_LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public final static DcMotorSimple.Direction FRONT_RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public final static DcMotorSimple.Direction BACK_RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public final static DcMotor.ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

    public final static RevHubOrientationOnRobot.LogoFacingDirection CONTROL_HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public final static RevHubOrientationOnRobot.UsbFacingDirection CONTROL_HUB_USB_PORT_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
}
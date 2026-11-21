package org.firstinspires.ftc.teamcode.alexp.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config

public class IntakeConfig {
    public final static DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double INTAKE_SPEED = 1.0;
}

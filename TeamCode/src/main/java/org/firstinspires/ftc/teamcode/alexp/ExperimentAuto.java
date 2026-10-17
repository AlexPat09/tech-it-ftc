package org.firstinspires.ftc.teamcode.alexp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.alexp.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.alexp.config.TechItDriveConfig;
import org.firstinspires.ftc.teamcode.alexp.subsystems.Intake;
import org.firstinspires.ftc.teamcode.alexp.subsystems.TechItDrive;

@Autonomous(name="BasicAutoEncoderCm", group="Linear Opmode")
public class ExperimentAuto extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Intake intake;

    static final double COUNTS_PER_MOTOR_REV  = 537.7 ;    // GoBILDA 5202 Series (19.2:1)
    static final double DRIVE_GEAR_REDUCTION  = 1.0 ;     // No external gearing (1:1)
    static final double WHEEL_DIAMETER_CM     = 9.6 ;      // 96mm wheels = 9.6 cm
    static final double COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * Math.PI);

    @Override
    public void runOpMode() {

        // Initialize hardware
        leftDrive  = hardwareMap.get(DcMotor.class, "bld");
        rightDrive = hardwareMap.get(DcMotor.class, "brd");

        leftDrive.setDirection(TechItDriveConfig.BACK_LEFT_DRIVE_DIRECTION);
        rightDrive.setDirection(TechItDriveConfig.BACK_RIGHT_DRIVE_DIRECTION);

        leftDrive.setZeroPowerBehavior(TechItDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);
        rightDrive.setZeroPowerBehavior(TechItDriveConfig.MOTOR_ZERO_POWER_BEHAVIOR);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake = new Intake();
        intake.init(hardwareMap,telemetry);


        telemetry.addData("Status", "Initialized. Encoders Ready.");
        telemetry.update();

        waitForStart();

        encoderDrive(0.1, -10.0, -10.0);
        intake.swallow();
        encoderDrive(0.7, 100.0, 100.0);
        sleep(2000);
        encoderDrive(0.7, -110.0, -110.0);
        intake.stop();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void encoderDrive(double speed, double leftCm, double rightCm) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Calculate new target positions based on metric constants
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftCm * COUNTS_PER_CM);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightCm * COUNTS_PER_CM);

            // Set targets to motors
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn on RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start motion
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // Wait until both motors reach their targets
            while (opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {
                // Display target and current positions to the driver station
                telemetry.addData("Target Ticks",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Current Ticks", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION and return to normal mode
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
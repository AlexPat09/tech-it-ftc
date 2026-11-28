package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fld");
        c.frontRightName.set("frd");
        c.backLeftName.set("bld");
        c.backRightName.set("brd");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.390289366714598);
        c.yPodOffset.set(1.3411694624292572);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1768302107536072);
                Controller secondaryTranslationalForward = Controller.proportional(0.06533403788320513);
                Controller primaryTranslationalLateral = Controller.proportional(0.22633478344760627);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0836246547070933);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01734583765207453));
                c.brake.set(Controller.proportionalFeedforward(0.01474396200426335));

                c.headingFeedback.set(Controller.proportional(3.2000715792934624));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.044069723431652164, 0.0033889346206432067));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0708926149931788, 0.04726910136447942));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010568932644004062, 0.0014259527821413665));

                c.maxAchievableForwardVelocity.set(56.77981617163422);
                c.maxAchievableStrafeVelocity.set(48.89994637649067);
                c.naturalForwardDeceleration.set(30.337921261912296);
                c.naturalStrafeDeceleration.set(53.689914311398105);
            }
    );
}
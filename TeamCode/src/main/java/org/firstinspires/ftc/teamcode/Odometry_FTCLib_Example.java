package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This sample shows how to use dead wheels encoders.
 * The external encoders we are using are REV through-bore.
 */

@Autonomous(name="FTCLib Odometry Example", group="Linear Opmode")
//@Disabled

public class Odometry_FTCLib_Example extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 14.25;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = -3.25;
    public static final double WHEEL_DIAMETER = 1.5;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx leftEncoder, rightEncoder,centerEncoder;
    private HolonomicOdometry odometry;


    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new MotorEx(hardwareMap, "left_encoder");
        rightEncoder = new MotorEx(hardwareMap, "right_encoder");
        centerEncoder = new MotorEx(hardwareMap, "center_encoder");

        leftEncoder.encoder.setDirection(Motor.Direction.REVERSE);

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftEncoder.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerEncoder.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                centerEncoder::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        // read the current position from the position tracker
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            // control loop
            odometry.updatePose(); // update the position

            // Show the elapsed game time and wheel distance.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Distance", "%4.2f",leftEncoder.getDistance());
            telemetry.addData("Right Distance", "%4.2f",rightEncoder.getDistance());
            telemetry.addData("Center Distance", "%4.2f",centerEncoder.getDistance());
            telemetry.addData("2D Pose", odometry.getPose());
            telemetry.update();
        }

    }
}

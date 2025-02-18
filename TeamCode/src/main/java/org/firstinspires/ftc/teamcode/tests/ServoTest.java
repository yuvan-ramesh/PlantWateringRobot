package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Servo Test", group = "Teleop")
public class ServoTest extends LinearOpMode {
    private Servo servo;
    private Rev2mDistanceSensor laserSensor;

    private static final double MAX = 0.0; // Maximum servo position
    private static final double MIN = 0.7; // Minimum servo position
    private static final double SERVO_INCREMENT = 0.01; // Amount by which the servo position is incremented
    private static final double STOP_DISTANCE_CM = 35.0; // Distance at which the servo stops turning
    private static final double CELL_SIZE = 4.0;

    @Override
    public void runOpMode() {
        // Initialize hardware components
        servo       = hardwareMap.get(Servo.class, "servo");
        laserSensor = hardwareMap.get(Rev2mDistanceSensor.class, "laserSensor");

        // Set the initial servo position
        double servoPosition = MIN;
        servo.setPosition(servoPosition);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            double distance = laserSensor.getDistance(DistanceUnit.CM);

            // If no object is detected within STOP_DISTANCE_CM, continue turning the servo
            if (distance > STOP_DISTANCE_CM) {
                servoPosition -= SERVO_INCREMENT;

                // Limit the servo position to the range [MIN, MAX]
                if (servoPosition < MAX) {
                    servoPosition = MAX;
                }
                servo.setPosition(servoPosition);
            } else {
                double angle = ((0.7-servo.getPosition()) * 180)/0.7;
                int x = (int)(distance * Math.cos(Math.toRadians(angle)) / CELL_SIZE);
                int y = (int)(distance * Math.sin(Math.toRadians(angle)) / CELL_SIZE);

                // Stop the servo when an object is detected
                telemetry.addData("Object detected, stopping servo.", "");
                telemetry.addData("x ", x);
                telemetry.addData("y ", y);
                telemetry.addData("angle", angle);
            }



            // Display telemetry data
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("Laser Range", String.format("%.01f cm", distance));
            telemetry.update();
        }
    }
}

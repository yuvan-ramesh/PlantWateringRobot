package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Odometry.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot;


import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "Obstacle Mapping Auto", group = "Autonomous")
public class ObstacleMapping extends LinearOpMode {
    private SimplifiedOdometryRobot robot;
    private Rev2mDistanceSensor laserSensor;
    private Servo servo;
    private GoBildaPinpointDriver odo;

    private static final int GRID_SIZE = 20;
    private static final double CELL_SIZE = 10.0;
    private static int[][] grid = new int[GRID_SIZE][GRID_SIZE];

    @Override
    public void runOpMode() {
//        DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
//        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
//        DcMotor leftRear = hardwareMap.get(DcMotor.class, "backLeft");
//        DcMotor rightRear = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize hardware components


        robot = new SimplifiedOdometryRobot(this);
        robot.initialize(true);

        laserSensor = hardwareMap.get(Rev2mDistanceSensor.class, "laserSensor");
        servo = hardwareMap.get(Servo.class, "servo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");


        waitForStart();

        for (int step = 0; step < 10; step++) {
            scanSurroundings();
            moveToSafeLocation();
        }

        printGrid();
    }

    // Scans surroundings using the laser sensor mounted on a servo
    private void scanSurroundings() {
        double robotX = odo.getEncoderX();
        double robotY = odo.getEncoderY();
        double robotHeading = odo.getHeading();

        for (int angle = 0; angle <= 180; angle += 10) {
            servo.setPosition(angle / 180.0);
            sleep(200);

            double distance = laserSensor.getDistance(DistanceUnit.CM);

            if (distance > 0 && distance < 200) {
                int[] obstaclePos = convertPolarToCartesian(distance, Math.toRadians(angle + robotHeading));
                markObstacle(obstaclePos[0], obstaclePos[1]);
            }
        }
    }

    // Converts polar coordinates to Cartesian
    private int[] convertPolarToCartesian(double distance, double angleRad) {
        int x = (int) (odo.getEncoderX() + (distance * Math.cos(angleRad)) / CELL_SIZE);
        int y = (int) (odo.getEncoderY() + (distance * Math.sin(angleRad)) / CELL_SIZE);
        return new int[]{x, y};
    }

    // Marks the detected obstacle on the grid
    private void markObstacle(int x, int y) {
        if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
            grid[x][y] = 1;
            telemetry.addData("Obstacle detected at", "(" + x + ", " + y + ")");
            telemetry.update();
        }
    }

    // Move to the closest non-obstacle location
    private void moveToSafeLocation() {
        int robotGridX = (int) (odo.getEncoderX() / CELL_SIZE);
        int robotGridY = (int) (odo.getEncoderY() / CELL_SIZE);

        int[] newPosition = findClosestOpenSpace(robotGridX, robotGridY);
        if (newPosition != null) {
            robot.moveRobot(newPosition[0] * CELL_SIZE,newPosition[1] * CELL_SIZE, odo.getHeading());
            // moveTo( );
            sleep(2000);
            telemetry.addData("Moving to", "(" + newPosition[0] + ", " + newPosition[1] + ")");
            telemetry.update();
        } else {
            telemetry.addData("No safe space found!", "Robot remains in place.");
            telemetry.update();
        }
    }

    // Finds the nearest open space using Breadth-First Search (BFS)
    private int[] findClosestOpenSpace(int startX, int startY) {
        boolean[][] visited = new boolean[GRID_SIZE][GRID_SIZE];
        Queue<int[]> queue = new LinkedList<>();
        queue.add(new int[]{startX, startY});

        int[][] directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}}; // Right, Left, Down, Up

        while (!queue.isEmpty()) {
            int[] current = queue.poll();
            int x = current[0];
            int y = current[1];

            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && !visited[x][y]) {
                visited[x][y] = true;

                if (grid[x][y] == 0) { // If space is open, return its position
                    return new int[]{x, y};
                }

                // Add adjacent cells to the queue
                for (int[] dir : directions) {
                    int newX = x + dir[0];
                    int newY = y + dir[1];
                    queue.add(new int[]{newX, newY});
                }
            }
        }
        return null; // No open space found
    }

    // Prints the 2D grid map
    private void printGrid() {
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                telemetry.addData("", grid[x][y] == 1 ? " X " : " . ");
            }
            telemetry.update();
        }
    }
}


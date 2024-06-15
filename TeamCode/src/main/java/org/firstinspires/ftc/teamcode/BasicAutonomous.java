package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Basic Autonomous", group="Autonomous")
public class BasicAutonomous extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run your autonomous code here
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
        sleep(1000);  // Drive forward for 1 second

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}

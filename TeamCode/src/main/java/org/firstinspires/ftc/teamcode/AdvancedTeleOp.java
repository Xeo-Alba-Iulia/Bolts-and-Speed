package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Advanced TeleOp", group="TeleOp")
public class AdvancedTeleOp extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor turnMotor;
    private Servo clawMotor;

    private int enter = 0;
    private int targetBreak = 0;

    private int previousEncoderPosition;
    private long previousTime;

//    private boolean StrngCalEnter = false;

    @Override
    public void init() {
        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        turnMotor = hardwareMap.get(DcMotor.class, "turn_drive");
        clawMotor = hardwareMap.get(Servo.class, "claw_drive");

        turnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        previousEncoderPosition = turnMotor.getCurrentPosition();
        previousTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
//        if (!StrngCalEnter){
//            turnMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            turnMotor.setPower(0.5);
//            telemetry.addData("Calibrating...", 1);
//            telemetry.update();
//            try {
//                sleep(2000);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//            telemetry.addData("Current position", turnMotor.getCurrentPosition());
//            telemetry.update();
//            turnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            turnMotor.setTargetPosition(turnMotor.getCurrentPosition() - 700);  //(Math.abs(targetTurn) + 50) * coeff);
//            turnMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turnMotor.setPower(1.0);
//
//            try {
//                sleep(1500);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//
//            turnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            StrngCalEnter = true;
//        }
//        else {
            double lt = gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;
            double joy_axis = gamepad1.left_stick_x;
            boolean claw = gamepad1.circle;
            boolean hand_break = gamepad1.square;

            // Call the movement logic function
            MovementOutput output = movementLogic(lt, rt, joy_axis);

            if (!output.isValid) {
                telemetry.addData("Error", "Invalid inputs");
            } else if (!hand_break) {
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // Set motor power
                leftMotor.setPower(output.leftMotor);
                rightMotor.setPower(output.rightMotor);

                int targetTurn = output.turnAngle;
                //int currentTurn = turnMotor.getCurrentPosition();
                //int coeff = targetTurn / Math.max(Math.abs(targetTurn), 1);
                turnMotor.setTargetPosition(targetTurn);  //(Math.abs(targetTurn) + 50) * coeff);
                turnMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turnMotor.setPower(1.0);

                enter = 0;

                // Telemetry data for debugging
                telemetry.addData("Left Motor Power", leftMotor.getPower());
                telemetry.addData("Right Motor Power", rightMotor.getPower());
                telemetry.addData("Turn Motor Power", turnMotor.getPower());
                telemetry.addData("Turn Motor Position", output.turnAngle);
                telemetry.addData("Real turn angle position n:", turnMotor.getCurrentPosition());
                double turnMotorSpeed = getMotorSpeed(rightMotor);
                telemetry.addData("Right Motor Speed", turnMotorSpeed);
            } else {
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftMotor.setPower(0);

                telemetry.addData("Left Motor Power", leftMotor.getPower());
                telemetry.addData("Right Motor Power", rightMotor.getPower());
                telemetry.addData("Turn Motor Position", output.turnAngle);
                telemetry.addData("Real turn angle position n:", turnMotor.getCurrentPosition());
                double turnMotorSpeed = getMotorSpeed(rightMotor);
                telemetry.addData("Right Motor Speed", turnMotorSpeed);

                if (enter > 0) {
                    rightMotor.setTargetPosition(targetBreak);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (Math.abs(rightMotor.getCurrentPosition()) - 50 > Math.abs(targetBreak)) {
                        rightMotor.setPower(0.69);
                    } else {
                        rightMotor.setPower(0.15);
                    }
                } else {
                    targetBreak = rightMotor.getCurrentPosition();
                }

                enter++;
            }
            telemetry.update();
            if (claw) {
                clawMotor.setPosition(0.35);
            } else {
                clawMotor.setPosition(1.0);
            }
//        }
    }

    private double getMotorSpeed(DcMotor motor) {
        int currentEncoderPosition = motor.getCurrentPosition();
        long currentTime = System.currentTimeMillis();

        int positionDifference = currentEncoderPosition - previousEncoderPosition;
        long timeDifference = currentTime - previousTime;

        // Update previous values
        previousEncoderPosition = currentEncoderPosition;
        previousTime = currentTime;

        return positionDifference / (double) timeDifference;
    }

    private MovementOutput movementLogic(double lt, double rt, double joy_axis) {
        MovementOutput output = new MovementOutput();

        double max_angle;
        double max_pow;

        if (gamepad1.cross) {
            max_angle = 450;
        }
        else {
            max_angle = 300;
        }

        if (gamepad1.triangle) {
            max_pow = -0.4;
        }
        else {
            max_pow = -1.0;
        }

        if (!(lt >= -1 && lt <= 1 && rt >= -1 && rt <= 1 && joy_axis >= -1 && joy_axis <= 1)) {
            output.isValid = false;
            return output;
        }

        output.isValid = true;
        double speed = rt - lt;
        output.turnAngle = (int) (joy_axis * -max_angle);
        double l_motor =  speed * max_pow;
        double r_motor =  speed * max_pow;

        output.leftMotor = Math.max(-1, Math.min(l_motor, 1));
        output.rightMotor = Math.max(-1, Math.min(r_motor, 1));

        return output;
    }

    private static class MovementOutput {
        boolean isValid;
        int turnAngle;
        double leftMotor;
        double rightMotor;
    }
}

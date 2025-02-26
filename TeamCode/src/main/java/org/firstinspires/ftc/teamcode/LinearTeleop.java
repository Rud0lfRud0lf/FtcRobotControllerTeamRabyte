package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class LinearTeleop extends DriveBase {
    @Override
    public void runOpMode() throws InterruptedException {
        db();
        //zmienne
        //mechanum

        //pivot
        Servo sparkMini;
        DigitalChannel encoderA;
        DigitalChannel encoderB;
        int currentPosition = 0;
//        int targetPosition = 4;
//        final double kP = 0.05; // Proportional gain, adjust as needed


        //pivot mniejszy
        double pivot_SERVO_SPEED = 0.5;
        final double SMALL_PIVOT_RANGE_MIN = -2.0;
        final double SMALL_PIVOT_RANGE_MAX = 0.1;


        //intake
        double SERVO_SPEED = 0.5;


        //deklaracje

        //pivot
        sparkMini = hardwareMap.servo.get("pivotMotor");
        encoderA = hardwareMap.digitalChannel.get("encoder_a");
        encoderB = hardwareMap.digitalChannel.get("encoder_b");
        encoderA.setMode(DigitalChannel.Mode.INPUT);
        encoderB.setMode(DigitalChannel.Mode.INPUT);


        //pivot mniejszy
        Servo smallPivotServo;
        smallPivotServo = hardwareMap.get(Servo.class, "smallPivotServo");
        smallPivotServo.scaleRange(SMALL_PIVOT_RANGE_MIN, SMALL_PIVOT_RANGE_MAX);

        //intake
        CRServo intakeServoOne;
        CRServo intakeServoTwo;

        intakeServoOne = hardwareMap.get(CRServo.class, "intakeServoOne");
        intakeServoTwo = hardwareMap.get(CRServo.class, "intakeServoTwo");
        intakeServoOne.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServoTwo.setDirection(DcMotorSimple.Direction.REVERSE);


        //enkodery
        //        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        //        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //        int degrees = 9;
        //        //koniec enkoderow
        ////        //silniki
        //        DcMotor motorOneDb;
        //        motorOneDb = hardwareMap.get(DcMotor.class, "motor_db_One");
        //        motorOneDb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //        motorOneDb.setDirection(DcMotorSimple.Direction.FORWARD);
        //        motorOneDb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        waitForStart();
        //
        //        DcMotorEx motorTwoDb;
        //        motorTwoDb = hardwareMap.get(DcMotorEx.class, "motor_db_Two");
        //        //koniec silnikow
        //
        //        //serva
        //        Servo servoOne;
        //        servoOne= hardwareMap.get(Servo.class, "servo_One");
        //        servoOne.scaleRange(0.2,0.8);
        //
        //        CRServo servoTwo;
        //        servoTwo = hardwareMap.get(CRServo.class, "servo_Two");
        //
        //        //koniec

        //pivot

//        //gamepad
//        boolean isDown = true;
//        boolean lastCycle = false, currCycle = false;


        //gamepad ned
        while (opModeIsActive()) {


            //gamepads
            //zawieszenie
            if (gamepad1.dpad_up)
            {
                //targetPosition = 10;
            }
            if(gamepad1.dpad_down)
            {
                //targetPosition=-10;
            }
            //if (gamepad1.right_bumper && gear < 1) {
            //    gear += 0.05;
            //}
            //if (gamepad1.left_bumper && gear > 0) {
            //    gear -= 0.05;
            //}
            if (gamepad2.square) {
                smallPivotServo.setPosition(1);

            } else if (gamepad2.circle) {
                smallPivotServo.setPosition(0.0);
            }
            if (gamepad2.dpad_right) { //dodac funkcjonalnosc - raz wciesniete kreci sie albo jak czujnik zwroci 1 albo jak operator wcisnie guzik
                if (!gamepad2.triangle) {
                    intakeServoOne.setPower(SERVO_SPEED);
                    intakeServoTwo.setPower(SERVO_SPEED);
                } else {
                    intakeServoOne.setPower(0);
                    intakeServoTwo.setPower(0);
                    break;

                }
            }

            if (gamepad2.dpad_left) { //dodac funkcjonalnosc - raz wciesniete kreci sie albo jak czujnik zwroci 1 albo jak operator wcisnie guzik
                {
                    if (!gamepad2.triangle) {
                        intakeServoOne.setPower(-SERVO_SPEED);
                        intakeServoTwo.setPower(-SERVO_SPEED);
                    } else {
                        intakeServoOne.setPower(0);
                        intakeServoTwo.setPower(0);
                        break;

                    }
                }
            }
//            if(gamepad2.circle) { //dodac funkcjonalnosc - raz wciesniete kreci sie albo jak czujnik zwroci 1 albo jak operator wcisnie guzik
//                while (!SENSOR)
//                {
//                }
//            }

            //pivot kod
            // Simple encoder reading
            updateEncoderPosition(encoderA, encoderB, currentPosition);
            // adjustPivotPower(targetPosition, currentPosition, sparkMini, kP);
            //sparkMini.setPower(0.5);

            // Display pivot position
            //    telemetry.addData("Pivot Position", currentPosition);
            telemetry.addData("Pivot Power", sparkMini.getPosition());
        }
    }

    public void updateEncoderPosition(DigitalChannel encoderA, DigitalChannel encoderB, int currentPosition)
    {
        if (encoderA.getState() != encoderB.getState()) {
            currentPosition++;
        } else {
            currentPosition--;
        }
    }

    public void adjustPivotPower(int targetPosition, int currentPosition, Servo sparkMini, double kP) {
        int error = targetPosition - currentPosition;
        double power = error * kP;
        power = Math.max(-1, Math.min(1, power)); // Limit power to [-1, 1]
        sparkMini.setPosition(0);
    }

    public void setTargetPosition(int position, int targetPosition) {
        targetPosition = position;
    }
}
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

public class LinearTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //zmienne
        //mechanum
        double gear=1;

        //pivot
        double TARGET_ANGLE = 45.0; // Set your desired angle here
        final double COUNTS_PER_DEGREE = 10.0; // Adjust based on your setup
        final double TOLERANCE = 1.0; // Tighter tolerance for holding position
        final double MAX_POWER = 0.7;
        final double MIN_POWER = 0.05; // Reduced minimum power for holding
        final double PIVOT_P = 0.02; // Proportional constant

        //suwnica
        final double COUNTS_PER_CM = 100.0; // Adjust based on your crane's gear ratio
        final double MAX_POWER_CR = 0.7;
        final double MIN_POWER_CR = 0.1;
        final double CRANE_P = 0.02; // Proportional constant
        double TARGET_CM =10.0;

        //pivot mniejszy
        double pivot_SERVO_SPEED = 1.0;
        final double SMALL_PIVOT_RANGE_MIN = 0.2;
        final double SMALL_PIVOT_RANGE_MAX = 0.7;


        //intake
        double SERVO_SPEED = 0.5;
        boolean SENSOR =false;



        //deklaracje
        //mechanum
        double drive, turn, strafe;
        double fLeftPower, fRightPower, bLeftPower, bRightPower;
        // drive left stick y
        //turn right joystick x
        //strafe left jostick x
        DcMotor frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class,"backRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class,"backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //pivot
        CRServo pivotMotor = hardwareMap.get(CRServo.class, "pivotMotor");
        DigitalChannel encoderMotor = hardwareMap.get(DigitalChannel.class, "encoderMotor");

        //suwnica
        CRServo craneMotor;
        DigitalChannel encoderMotor_cr;


        craneMotor = hardwareMap.get(CRServo.class, "craneMotor");
        encoderMotor_cr = hardwareMap.get(DigitalChannel.class, "cr_encoderMotor");

        //pivot mniejszy
        //            Servo smallPivotServo;
        //
        //            smallPivotServo = hardwareMap.get(Servo.class, "smallPivotServo");
        //            smallPivotServo.scaleRange(SMALL_PIVOT_RANGE_MIN, SMALL_PIVOT_RANGE_MAX);

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
        boolean isDown = true;
        boolean lastCycle = false, currCycle = false;



        //gamepad ned
        while(opModeIsActive())
        {
            //gamepads
            if(gamepad1.right_bumper && gear<1)
            {
                gear+=0.05;
            }

            if(gamepad1.left_bumper && gear>0)
            {
                gear-=0.05;
            }
            if(gamepad2.a)
            {
                TARGET_ANGLE = 40; //pivot
                TARGET_CM = 30;
                //smallPivotServo.setPosition(0.8);

            }
            else if (gamepad2.b)
            {
                TARGET_ANGLE = 70; //pivot
                TARGET_CM = 60;
                //smallPivotServo.setPosition(0.2);
            }
            if(gamepad2.square) { //dodac funkcjonalnosc - raz wciesniete kreci sie albo jak czujnik zwroci 1 albo jak operator wcisnie guzik
                while (!SENSOR)
                {
                    if(!gamepad2.cross)
                    {
                        intakeServoOne.setPower(SERVO_SPEED);
                        intakeServoTwo.setPower(SERVO_SPEED);
                    }
                    else
                    {
                        intakeServoOne.setPower(0);
                        intakeServoTwo.setPower(0);
                        break;

                    }
                }
            } if(gamepad2.circle) { //dodac funkcjonalnosc - raz wciesniete kreci sie albo jak czujnik zwroci 1 albo jak operator wcisnie guzik
                while (!SENSOR)
                {
                    if(!gamepad2.cross)
                    {
                        intakeServoOne.setPower(-SERVO_SPEED);
                        intakeServoTwo.setPower(-SERVO_SPEED);
                    }
                    else
                    {
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





            //mechanum kod
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            fLeftPower = drive + turn + strafe;
            fRightPower = drive - turn - strafe ;
            bLeftPower = drive + turn - strafe ;
            bRightPower = drive - turn + strafe ;
            telemetry.addData(String.valueOf(fLeftPower), "dupa0");
            telemetry.addData(String.valueOf(fRightPower), "dupa1");
            telemetry.addData(String.valueOf(bLeftPower), "dupa2");
            telemetry.addData(String.valueOf(bRightPower), "dupa3");
            telemetry.update();
            double[] appliedPowers = scalePowers(fLeftPower, fRightPower, bLeftPower, bRightPower);

            frontLeft.setPower(appliedPowers[0]*gear);
            frontRight.setPower(appliedPowers[1]*gear);
            backLeft.setPower(appliedPowers[2]*gear);
            backRight.setPower(appliedPowers[3]*gear);



            //pivot kod
            //                int currentPosition = encoderMotor.getCurrentPosition();
            //                int targetPosition = (int)(TARGET_ANGLE * COUNTS_PER_DEGREE);
            //                double error = targetPosition - currentPosition;
            //                double power = error * PIVOT_P;
            //                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
            //                pivotMotor.setPower(set_pivot(error, COUNTS_PER_DEGREE, TOLERANCE, MIN_POWER, power )); //chyba muszę to cały czas wykonywać, zmieniając tylko TARGET_ANGLE


            //                //suwnica kod
            //
            //                int targetPosition_cr = (int)(TARGET_CM*COUNTS_PER_CM);
            //                int currentPosition_cr = encoderMotor.getCurrentPosition();
            //                double error_cr = targetPosition_cr - currentPosition_cr;
            //
            //                double power_cr = error_cr * CRANE_P;
            ///                power_cr = Math.max(-MAX_POWER_CR, Math.min(MAX_POWER_CR, power_cr));
            //
            //                if (Math.abs(error) > COUNTS_PER_CM * 0.1 && Math.abs(power_cr) < MIN_POWER_CR) {
            //                    power_cr = Math.copySign(MIN_POWER_CR, power_cr);
            //                }
            //
            //                craneMotor.setPower(power_cr);

            //pivot_mniejszy



            //intake


//            //enkdoery
//            arm.setTargetPosition(degrees);
//            arm.setTargetPositionTolerance(3); //zeby nie chybotało;
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setPower(1);
//            //ogl enkodery trzeeba przeliczyc przez przekladnie jesli uzywamy tych na silniku do pivota
//            //koniec enkoderow
//

//            //gamepad
//            //komputer szybki, zeby nie zczytywal guzika caly czas (efektywnie tylko zmiane stanu wykrywa w porownaniu do poprzedniego wykonania.
//            lastCycle = currCycle;
//            currCycle = gamepad1.a;
//
//            if(currCycle && !lastCycle)
//            {
//                isDown=!isDown;
//                if(isDown)
//                {
//                    servoOne.setPosition(0);
//                }
//                else
//                {
//                    servoOne.setPosition(1);
//                }
//            }
//
//            //gamepad end
//
//            //silniki
//            motorOneDb.setPower(1);
//            motorTwoDb.setVelocity(150);
//            //koniec silnikow
//
//            //serva
//                servoTwo.setPower(0.5);
//            //koniec serv
        }

    }


    public double set_pivot(double error, double COUNTS_PER_DEGREE, double TOLERANCE, double MIN_POWER, double power)
    {

        while (opModeIsActive()) {


            if (Math.abs(error) > TOLERANCE * COUNTS_PER_DEGREE && Math.abs(power) < MIN_POWER) {
                power = Math.copySign(MIN_POWER, power);
            }
        }
        return power;
    }
    public double[] scalePowers(double fLeftPower, double fRightPower, double bLeftPower, double bRightPower)
    {
        double max = Math.max(Math.abs(fLeftPower), Math.max(Math.abs(fRightPower), Math.max(Math.abs(bLeftPower), Math.abs(bRightPower))));
        if(max >1)
        {
            fLeftPower /= max;
            fRightPower /=max;
            bLeftPower /= max;
            bRightPower /= max;
        }
        double [] motorPowers = new double[]{fLeftPower, fRightPower, bLeftPower, bRightPower};
        return motorPowers;
    }
}

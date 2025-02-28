package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class DriveBase extends LinearOpMode {
    public void db(){
        double gear = 1;
        double drive, turn, strafe;
        double fLeftPower, fRightPower, bLeftPower, bRightPower;
        // drive left stick y
        //turn right joystick x
        //strafe left jostick x
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            double controlerSensivity = 0.05;
            if(Math.abs(gamepad1.left_stick_y) > controlerSensivity){
                drive = gamepad1.left_stick_y * -1;
            }
            else{
                drive = 0;
            }
            if(Math.abs(gamepad1.right_stick_x) > controlerSensivity){
                turn = gamepad1.right_stick_x * -1;
            }
            else{
                turn = 0;
            }
            if(Math.abs(gamepad1.left_stick_x) > controlerSensivity){
                strafe = gamepad1.left_stick_x;
            }
            else{
                strafe = 0;
            }

            fLeftPower = drive + turn + strafe;
            fRightPower = drive - turn - strafe;
            bLeftPower = drive + turn - strafe;
            bRightPower = drive - turn + strafe;

            telemetry.addData(String.valueOf(fLeftPower), "dupa0");
            telemetry.addData(String.valueOf(fRightPower), "dupa1");
            telemetry.addData(String.valueOf(bLeftPower), "dupa2");
            telemetry.addData(String.valueOf(bRightPower), "dupa3");
            telemetry.update();

            double[] appliedPowers = scalePowers(fLeftPower, fRightPower, bLeftPower, bRightPower);

            frontLeft.setPower(appliedPowers[0] * gear);
            frontRight.setPower(appliedPowers[1] * gear);
            backLeft.setPower(appliedPowers[2] * gear);
            backRight.setPower(appliedPowers[3] * gear);
        }
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

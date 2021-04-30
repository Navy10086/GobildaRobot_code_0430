/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="单人模式")
//@Disabled
public class TeleOp_OneDriver extends OpMode
{
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private DcMotorEx arm = null;
    private Servo shootTrigger = null;
    private Servo rubbleCatch = null;
    private boolean isIntake = false;
    private boolean isShoot = false;
    private boolean isSlowly = false;
    private boolean isCatch = false;
    private boolean isLoose = false;
    private ElapsedTime keyDelay = new ElapsedTime();
    private ElapsedTime catchDelay = new ElapsedTime();
    private ElapsedTime looseDelay = new ElapsedTime();

    private final double INTAKE_POWER = 1.0;
    private final double TRIGGER_ON = 0.62;
    private final double TRIGGER_OFF = 0.35;
    private final double CATCH_OPEN = 0.75;
    private final double CATCH_CLOSE = 0.5;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class,"leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class,"rightRear");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        arm = hardwareMap.get(DcMotorEx.class,"arm");
        shootTrigger = hardwareMap.get(Servo.class,"shootTrigger");
        rubbleCatch = hardwareMap.get(Servo.class,"rubbleCatch");

        //
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        keyDelay.reset();
    }

    @Override
    public void loop() {
        //drive base
        double leftPower;
        double rightPower;
        double drive = gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        if(gamepad1.y && keyDelay.seconds() > 0.5)
        {
            isSlowly = !isSlowly;
            keyDelay.reset();
        }
        if(isSlowly)
        {
            drive *= 0.7;
            turn *= 0.65;
        }
        drive = Math.signum(drive) * drive * drive;
        turn = Math.signum(turn) * turn * turn;
        leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;

        leftFront.setPower(leftPower);
        leftRear.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightRear.setPower(rightPower);
        //intake
        if(gamepad1.left_bumper && keyDelay.seconds() > 0.3)
        {
            isIntake = !isIntake;
            keyDelay.reset();
        }
        if(gamepad1.right_bumper)
        {
            intake.setPower(-INTAKE_POWER);
        }
        else
        {
            if(isIntake)
            {
                intake.setPower(INTAKE_POWER);
            }
            else
            {
                intake.setMotorDisable();
            }
        }


        //shooter
        if(gamepad1.left_trigger > 0.5 && keyDelay.seconds() > 0.5)
        {
            isShoot = !isShoot;
            keyDelay.reset();
        }
        if(isShoot)
        {
            shooter.setPower(1.0);
        }
        else {
            shooter.setMotorDisable();
        }
        //launch trigger
        if(gamepad1.right_trigger > 0.5)
        {
            shootTrigger.setPosition(TRIGGER_ON);
        }
        else
        {
            shootTrigger.setPosition(TRIGGER_OFF);
        }
        //arm
        if(gamepad1.x && keyDelay.seconds() > 0.3)
        {
            keyDelay.reset();
            looseDelay.reset();
            isLoose = !isLoose;
        }
        if(gamepad1.b && keyDelay.seconds() > 0.3)
        {
            keyDelay.reset();
            catchDelay.reset();
            isCatch = !isCatch;
        }
        if(isLoose)
        {
            arm.setTargetPosition(450);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.4);
            if(looseDelay.seconds() > 1.0)
            {
                rubbleCatch.setPosition(CATCH_OPEN);
            }
        }
        else
        {
            if(isCatch)
            {
                rubbleCatch.setPosition(CATCH_OPEN);
                arm.setTargetPosition(800);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                arm.setPower(0.4);
            }
            else
            {
                rubbleCatch.setPosition(CATCH_CLOSE);
                if(catchDelay.seconds() > 0.8)
                {
                    arm.setTargetPosition(100);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.4);
                }
            }
        }


    }

    @Override
    public void stop() {
    }

}

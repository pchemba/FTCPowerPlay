package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.PowerPlayBot;

@Disabled
@TeleOp(name = "WheelTest", group = "Test")
public class TestWheelEncoders extends LinearOpMode {

    private static final double COUNTS_PER_SCISSOR_INCH = 227.2;
    private static final double COUNTS_PER_SIDE_INCH = 50;
    private static final double COUNTS_PER_DEGREE = 8.5;
    private static final double MOTOR_SPEED = 0.8;


    // Creating a  robot object
    PowerPlayBot bot = new PowerPlayBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        bot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive())
        {

            //*Get the position of the motor(s) being tested and print*//
            //Wheel
           // telemetry.addData("FR", bot.getChassisAssembly().getFrontRightWheelCurrentPosition());
           // telemetry.addData("FL", bot.getChassisAssembly().getFrontLeftWheelCurrentPosition());
            telemetry.addData("BR", bot.getChassisAssembly().getBackRightWheelCurrentPosition());
            telemetry.addData("BL", bot.getChassisAssembly().getBackLeftWheelCurrentPosition());

            telemetry.update();

            if(gamepad1.y)
            {
           //     bot.getChassisAssembly().robotHardware.frontRightWheel.setPower(1);
            }
            else if(gamepad1.a)
            {
                bot.getChassisAssembly().robotHardware.backRightWheel.setPower(1);
            }
            else if (gamepad1.dpad_up)
            {
           //     bot.getChassisAssembly().robotHardware.frontLeftWheel.setPower(1);
            }
            else if(gamepad1.dpad_down)
            {
                bot.getChassisAssembly().robotHardware.backLeftWheel.setPower(1);
            }
            else
            {
                bot.getChassisAssembly().stopMoving();
            }
        }
    }
}

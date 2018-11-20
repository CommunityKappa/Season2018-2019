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

package org.firstinspires.ftc.team9351;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Aristoteles", group="Iterative Opmode")
@Disabled
public class Ari2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareAri robot           = new HardwareAri();

    @Override
    public void runOpMode() {
        double leftPower;               //power para calcular
        double rightPower;
        double centrePower;


        double drive = -gamepad1.left_stick_y;   //se calcula el power para las llantas
        double turn = gamepad1.left_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        centrePower = gamepad1.right_stick_x;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello negrote");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            // Combine drive and turn for blended motion.
            if (gamepad1.right_trigger > 0) {
                leftPower = leftPower * 0.75;
                rightPower = rightPower * 0.75;
                centrePower = centrePower * 0.75;
            } else if (gamepad1.left_trigger > 0) {
                leftPower = leftPower * 0.5 + leftPower * 0.5 * (1 - gamepad1.left_trigger);
                rightPower = rightPower * 0.5 + rightPower * 0.5 * (1 - gamepad1.left_trigger);
                centrePower = centrePower * 0.5 + centrePower * 0.5 * (1 - gamepad1.left_trigger);
            }
            //send calculated values to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            robot.centreDrive.setPower(centrePower);

            if (gamepad1.a) {
                robot.recogedor.setPower(0.7);
            } else {
                robot.recogedor.setPower(0);
            }

            if (gamepad1.dpad_up) {
                robot.lift.setPower(1);
            } else if (gamepad1.dpad_down) {
                robot.lift.setPower(-1);
            } else {
                robot.lift.setPower(0);
            }
           //start b
            if (gamepad2.a){
                robot.recogedor.setPower(-1);
            }else{
                robot.recogedor.setPower(0);
            }

            if (gamepad2.left_bumper){
                robot.slider.setPower(0.8);
            } else if (gamepad2.right_bumper){
                robot.slider.setPower(-0.8);
            }else{
                robot.slider.setPower(0);
            }

            if (gamepad2.dpad_up){
                robot.arti.setPower(1);
            }else if (gamepad2.dpad_down){
                robot.arti.setPower(-1);
            }else{
                robot.arti.setPower(0);
            }
            // Send telemetry message to signify robot running;

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}

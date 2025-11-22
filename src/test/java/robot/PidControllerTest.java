package robot;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class PidControllerTest {

    // @Test
    public void test_pid() {
        // SimpleMotorFeedforward
        PIDController m_drivePIDController = new PIDController(0.6, 0, 0);
        double measurement = 0.6;
        for (int i = 0; i < 10; i++) {
            double setpoint = 1.0;
            double output = m_drivePIDController.calculate(measurement, setpoint);
            System.out.printf("Measurement: %.2f, Setpoint: %.2f, Output: %.2f%n", measurement, setpoint, output);
        }

        for (int i = 0; i < 100; i++) {
            double setpoint = 1.0;
            double output = m_drivePIDController.calculate(measurement, setpoint);
            System.out.printf("Measurement: %.2f, Setpoint: %.2f, Output: %.2f%n", measurement, setpoint, output);
            measurement = Math.min(1.0, output + measurement);
        }
        m_drivePIDController.close();
    }

    @Test
    public void test_feedforward() {
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 1, 0.01);
        var v = ff.calculateWithVelocities(0, 0.15);
        System.out.println("FF: " + v);
    }

}

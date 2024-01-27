package frc.robot.subsystems;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class FireControlTest {
   private FireControl testingControl;
    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);

    }
    
    public Pose2d getPoseOne() {
        return new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180));
        
    }
    public Pose2d getPoseTwo() {
        return new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180+45));

    }
    public void initializeTestOne() {
        testingControl = new FireControl(this::getPoseOne);
        testingControl.periodic();
    }
    public void initializeTestTwo() {
        testingControl = new FireControl(this::getPoseTwo);
        testingControl.periodic();
    }
    @Test
    void testOneGetAngle() {
        initializeTestOne();
        assertEquals(1.486912877, testingControl.getAngle(), 0.001, "wrong angle");        
    }
    @Test
    void testTwoGetAngle() {
        initializeTestTwo();
        assertEquals(1.493098916, testingControl.getAngle(), 0.001, "wrong angle");
        
    }

    @Test
    void testOneGetVelocity() {
        initializeTestOne();
        assertEquals(6.59354785, testingControl.getVelocity(), 0.001, "wrong velocity");

    }
     @Test
    void testTwoGetVelocity() {
        initializeTestTwo();
        assertEquals(6.63808213, testingControl.getVelocity(), 0.001, "wrong Velocity");

    }
}

package frc.robot.subsystems;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.function.Supplier;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class FireControlTest {
   private FireControl testingControl;
   private PoseSupplier poseSupplier;

    public class PoseSupplier {
        private LinkedList<Pose2d> poseList = new LinkedList<Pose2d>();
        private Iterator<Pose2d> poseIterator;

        public PoseSupplier() {
            poseList.add(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180)));
            poseList.add(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180+45)));

            reset();
        }

        public Pose2d getPose() {
            return poseIterator.next();
        }

        public void reset() {
            poseIterator = poseList.iterator();
        }
    }

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        poseSupplier = new PoseSupplier();
        Supplier<Pose2d> poses = poseSupplier::getPose;
        testingControl = new FireControl(poses);
    }

    @Test
    void testGetAngle() {
        testingControl.periodic();
        assertEquals(1.486912877, testingControl.getAngle().getRadians(), 0.001, "wrong angle");
        
        testingControl.periodic();
        assertEquals(1.493098916, testingControl.getAngle().getRadians(), 0.001, "wrong angle");
        
    }

    @Test
    void testGetVelocity() {
        testingControl.periodic();
        assertEquals(6.59354785, testingControl.getVelocity(), 0.006, "wrong velocity");
        testingControl.periodic();
        assertEquals(6.63808213, testingControl.getVelocity(), 0.006, "wrong Velocity");
    }
    @Test
    void testGetDesiredAngle() {
        testingControl.periodic();
        assertEquals(Rotation2d.fromRadians(2.743578338).getRadians(), testingControl.getDesiredRobotAngle().getRadians(), 0.001, "wrong target angle");
    }
}

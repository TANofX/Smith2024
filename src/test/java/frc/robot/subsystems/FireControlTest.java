package frc.robot.subsystems;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Optional;
import java.util.function.Supplier;

import javax.xml.transform.TransformerFactoryConfigurationError;

//import javax.swing.text.html.Option;

//import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FireControlTest {
   private static final double DELTA = 0.1;
   private FireControl testingControl;
   private PoseSupplier poseSupplier;

    public static class AllianceSupplier {
        private static Alliance currentAlliance = null;

        public static Optional<Alliance> getAlliance() {
            return Optional.ofNullable(currentAlliance);
        }

        public static void makeRed() {
            currentAlliance = Alliance.Red;
        }

        public static void makeBlue() {
            currentAlliance = Alliance.Blue;
        }
    }

    public class PoseSupplier {
        private LinkedList<Pose2d> poseList = new LinkedList<Pose2d>();
        private Iterator<Pose2d> poseIterator;

        public PoseSupplier() {
          //  poseList.add(new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(200), Rotation2d.fromDegrees(0)));
          //  poseList.add(new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(200), Rotation2d.fromDegrees(-45)));

            poseList.add(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180)));
            poseList.add(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180+45)));
            
            poseList.add(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180)));
            poseList.add(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(200), Rotation2d.fromDegrees(180+45)));

            poseList.add(new Pose2d(1, 5.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(0.7071, 6.2571, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(0, 6.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(-0.7071, 6.2571, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(-1, 5.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(-0.7071, 4.8428, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(0, 4.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(0.7071, 4.8429, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(9.211, 5.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(8.9181, 6.2571, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(8.211, 6.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(7.5039, 6.2571, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(7.211, 5.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(7.5039, 4.8429, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(8.211, 4.55, Rotation2d.fromDegrees(0)));
            poseList.add(new Pose2d(8.9181, 4.8429, Rotation2d.fromDegrees(0)));
           

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
        testingControl = new FireControl(poses, AllianceSupplier::getAlliance);
    }

    @Test
    void testGetAngle() {
        testingControl.periodic();
        assertEquals(1.486912877, testingControl.getAngle().getRadians(), DELTA, "wrong angle");
        
        testingControl.periodic();
        assertEquals(1.493098916, testingControl.getAngle().getRadians(), DELTA, "wrong angle");
        
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
        AllianceSupplier.makeBlue();
        testingControl.periodic();
        assertEquals(Rotation2d.fromRadians(2.743578338).getRadians(), testingControl.getDesiredRobotAngle().getRadians(), DELTA, "wrong target angle");
        testingControl.periodic();
        assertEquals(Rotation2d.fromDegrees(108.5713854).getRadians(), testingControl.getDesiredRobotAngle().getRadians(), DELTA, "wrong target angle");
        
        AllianceSupplier.makeRed();
        testingControl.periodic();
        assertEquals(Rotation2d.fromDegrees(3.7902427).getRadians(), testingControl.getDesiredRobotAngle().getRadians(), DELTA, "wrong target angle");
        testingControl.periodic();
        assertEquals(Rotation2d.fromDegrees(-40.31464526).getRadians(), testingControl.getDesiredRobotAngle().getRadians(), DELTA, "wrong target angle");

    }

    @Test
    void testAllBlue() {
        testingControl.periodic();
        testingControl.periodic();
        testingControl.periodic();
        testingControl.periodic();
        AllianceSupplier.makeBlue();
        testingControl.periodic();
        assertEquals(0, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(45, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(90, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(135, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(180, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-135, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-90, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-45, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        assertEquals(68.500027, testingControl.getAngle().getDegrees(), DELTA, "Wrong Elevation");
        testingControl.periodic();
        assertEquals(0, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(9.061, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(4.53339, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.7961, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(6.9437, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.1261, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(5.3831, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(7.3871, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(0, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(7.061, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-5.3831, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(7.3871, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        assertEquals(25.292834, testingControl.getAngle().getDegrees(), DELTA, "Wrong Elevation");
        testingControl.periodic();
        assertEquals(-6.9437, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.1261, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-4.53339, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.7961, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
    }

    @Test
    void testAllRed() {
        testingControl.periodic();
        testingControl.periodic();
        testingControl.periodic();
        testingControl.periodic();
        AllianceSupplier.makeRed();
        testingControl.periodic();
        assertEquals(180, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(7.061, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(174.6169, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(7.3871, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(173.0563, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.1261, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(175.46661, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.7961, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(180, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(9.061, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-175.46661, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.7961, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-173.0563, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(8.1261, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-174.6169, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(7.3871, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        assertEquals(25.292834, testingControl.getAngle().getDegrees(), DELTA, "Wrong Elevation");
        testingControl.periodic();
        assertEquals(0, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(45, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(90, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(135, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(180, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-135, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-90, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        testingControl.periodic();
        assertEquals(-45, testingControl.getDesiredRobotAngle().getDegrees(), DELTA, "Angle Wrong");
        assertEquals(0.85, testingControl.getDistanceToTarget(), DELTA, "Wrong Distance");
        assertEquals(68.500027, testingControl.getAngle().getDegrees(), DELTA, "Wrong Elevation");
    }
}

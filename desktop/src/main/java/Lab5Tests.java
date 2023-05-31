import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import mixedreality.lab.exercise3.Camera;
import mixedreality.lab.exercise5.StereoScene;
import org.junit.Assert;
import org.junit.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class Lab5Tests {

    private StereoScene testStereoScene = new StereoScene();

    @Test
    public void testGetDerivativeErrorFunction() {
        Vector3f test1 = new Vector3f(1, 1, 1);
        Vector3f expRes1 = new Vector3f((float)12.390 , (float)-84.015, (float)-543.762);
        Vector3f accRes1 = testStereoScene.getDerivativeErrorFunction(test1);

        Vector3f test2 = new Vector3f(-5, 8, 100);
        Vector3f expRes2 = new Vector3f((float)171.875, (float)124.023, (float)7.324);
        Vector3f accRes2 = testStereoScene.getDerivativeErrorFunction(test2);

        Vector3f test3 = new Vector3f(5, 2, 0);
        Vector3f expRes3 = new Vector3f((float)42.480, (float)-322.51, (float)337.891);
        Vector3f accRes3 = testStereoScene.getDerivativeErrorFunction(test3);

        assertEquals(expRes1.x, accRes1.x, 0.001);
        assertEquals(expRes1.y, accRes1.y, 0.001);
        assertEquals(expRes1.z, accRes1.z, 0.001);

        assertEquals(expRes2.x, accRes2.x, 0.001);
        assertEquals(expRes2.y, accRes2.y, 0.001);
        assertEquals(expRes2.z, accRes2.z, 0.001);

        assertEquals(expRes3.x, accRes3.x, 0.001);
        assertEquals(expRes3.y, accRes3.y, 0.001);
        assertEquals(expRes3.z, accRes3.z, 0.001);
    }

    @Test
    public void testTransformWithDerivateIntoPixelCoordinates() {
        Camera testCam = new Camera(new Vector3f(3f, 2, 0), new Vector3f(0.5f, 0.5f, 0.5f), new Vector3f(0, 1, 0),
                degrees2Radiens(90), 1f, 1920, 1080);
        Vector2f[] accResult = testStereoScene.transformWithDerivateIntoPixelCoordinates(new Vector3f(0,0,0), 1,1,1, testCam);
        Vector2f[] expResult = new Vector2f[]{new Vector2f(1,1), new Vector2f(1,1), new Vector2f(1,1)};

        for (int i=0; i < accResult.length; i++) {
            assertTrue(vector2fEqual(accResult[i], expResult[i]));
        }
    }

    private float degrees2Radiens(float angleDegrees) {
        return angleDegrees / 180.0f * FastMath.PI;
    }

    private boolean vector2fEqual(Vector2f vec1, Vector2f vec2) {
        return vec1.x == vec2.x && vec1.y == vec2.y;
    }



}

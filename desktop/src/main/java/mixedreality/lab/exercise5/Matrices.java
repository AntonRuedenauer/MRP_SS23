package mixedreality.lab.exercise5;

import Jama.Matrix;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Vector3f;
import com.jme3.math.Vector4f;
import mixedreality.lab.exercise5.StereoScene;

public class Matrices {

    private StereoScene stereoCam = new StereoScene();

    /**
     * ViewMatrix for left camera
     */
    public Matrix4f createViewMatrixCamA() {
        // Set up the view matrix and transform point
        Matrix4f viewMatrix = getViewMatrix(stereoCam.leftCamera.getEye(), stereoCam.leftCamera.getRef(), stereoCam.leftCamera.getUp());
        return viewMatrix;
    }
    /**
     * ViewMatrix for right camera
     */
    public Matrix4f createViewMatrixCamB() {
        // Set up the view matrix and transform point
        Matrix4f viewMatrix = getViewMatrix(stereoCam.rightCamera.getEye(), stereoCam.rightCamera.getRef(), stereoCam.rightCamera.getUp());
        return viewMatrix;
    }

    /**
     * Projection matrix. Takes parameter if left or right cam
     */
    public Matrix4f createProjectionMatrix(boolean rightCamera) {
        // Set up the projection matrix and transform point
        Matrix4f projMatrix = getProjectionMatrix(rightCamera);
        //Vector4f pbild = projMatrix.mult(vec);
        //pbild.divide(pbild.z);
        return projMatrix;
    }

    /**
     * Screenmap matrix. Takes parameter if left or right cam
     */
    public Matrix4f createScreenMappingMatrix(boolean rightCamera) {
        // Set up the screenMapping matrix and transform point
        Matrix4f screenMapMatrix = getScreenMapMatrix(rightCamera);
        //Vector4f ppixel = screenMapMatrix.mult(vec);
        return screenMapMatrix;
    }

    private Matrix4f getViewMatrix(Vector3f eye, Vector3f ref, Vector3f up) {
        Vector3f tmp = ref.subtract(eye);
        Vector3f z = tmp.divide(tmp.length());
        Vector3f x = up.cross(z);
        Vector3f y = z.cross(x);
        Matrix4f viewMatrix = new Matrix4f(x.x, y.x, z.x, eye.x,
                x.y, y.y, z.y, eye.y,
                x.z, y.z, z.z, eye.z,
                0, 0, 0, 1);
        return viewMatrix;
    }

    private Matrix4f getProjectionMatrix(boolean right) {
        float z0 = 0.0f;
        if (right) {
            z0 = stereoCam.rightCamera.getZ0();
        }
        else {
            z0 = stereoCam.leftCamera.getZ0();
        }
        Matrix4f projectionMatrix = new Matrix4f(1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 1 / z0, 0);
        return projectionMatrix;
    }

    private Matrix4f getScreenMapMatrix(boolean rightCamera) {
        float f = 0.0f;
        if (rightCamera) {
            f = stereoCam.rightCamera.getWidth() / 2 * FastMath.tan(stereoCam.rightCamera.getFovX() / 2);
            Matrix4f screenMapMatrix = new Matrix4f(f, 0, 0, stereoCam.rightCamera.getWidth() / 2,
                    0, f, 0, stereoCam.rightCamera.getHeight() / 2,
                    0, 0, 0, 0,
                    0, 0, 0, 0);
            return screenMapMatrix;
        }
        else {
            f = stereoCam.leftCamera.getWidth() / 2 * FastMath.tan(stereoCam.leftCamera.getFovX() / 2);
            Matrix4f screenMapMatrix = new Matrix4f(f, 0, 0, stereoCam.leftCamera.getWidth() / 2,
                    0, f, 0, stereoCam.leftCamera.getHeight() / 2,
                    0, 0, 0, 0,
                    0, 0, 0, 0);
            return screenMapMatrix;
        }
    }
}

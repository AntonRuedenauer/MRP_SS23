package mixedreality.lab.exercise3;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Vector3f;
import com.jme3.math.Vector4f;
import mixedreality.base.mesh.TriangleMesh;

public class MatrixTransformer {

    public Vector3f transformOnePoint(Vector3f vector, Camera camera) {
        Vector3f returnVector = createModelMatrix(vector);
        returnVector = createViewMatrix(camera, returnVector);
        returnVector = createProjectionMatrix(returnVector);
        returnVector = createScreenMappingMatrix(returnVector, camera);
        return returnVector;
    }

    public Vector3f createModelMatrix (Vector3f vector3f) {
        // Set up the model matrix and transform points
        Matrix4f matrix4f = new Matrix4f().IDENTITY;
        return matrix4f.mult(vector3f);
    }

    public Vector3f createViewMatrix(Camera camera, Vector3f vector3f) {
        // Set up the view matrix and transform points
        Matrix4f viewMatrix = getViewMatrix(camera.getEye(), camera.getRef(), camera.getUp());
        return viewMatrix.mult(vector3f);
    }

    public Vector3f createProjectionMatrix(Vector3f vector3f) {
        // Set up the projection matrix and transform points
        Matrix4f projMatrix = getProjectionMatrix();
        Vector3f pbild1 = projMatrix.mult(vector3f);
        pbild1.divide(pbild1.z);
        return pbild1;
    }

    public Vector3f createScreenMappingMatrix(Vector3f vector3f, Camera camera) {
        // Set up the screenMapping matrix and transform points
        Matrix4f screenMapMatrix = getScreenMapMatrix(camera);
        return screenMapMatrix.mult(vector3f);
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

    private Matrix4f getProjectionMatrix() {
        int z0 = 1;
        Matrix4f projectionMatrix = new Matrix4f(1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 1/z0, 0);
        return projectionMatrix;
    }

    private Matrix4f getScreenMapMatrix(Camera camera) {
        float f = camera.getWidth() / 2 * FastMath.tan(camera.getFovX() / 2);
        Matrix4f screenMapMatrix = new Matrix4f(f, 0, 0, camera.getWidth()/2,
                0, f, 0, camera.getHeight()/2,
                0, 0, 0, 0,
                0, 0, 0, 0);
        return screenMapMatrix;
    }
}

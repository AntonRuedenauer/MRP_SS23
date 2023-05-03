package mixedreality.lab.exercise3;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Vector3f;
import mixedreality.base.mesh.TriangleMesh;

public class MatrixTransformer {

    public Vector3f[] createModelMatrix (TriangleMesh triangleMesh, int i) {
        // Set up the model matrix and transform points
        Matrix4f matrix4f = new Matrix4f().IDENTITY;
        Vector3f pwelt1 = matrix4f.mult(triangleMesh.getVertex(triangleMesh.getTriangle(i).getA()).getPosition());
        Vector3f pwelt2 = matrix4f.mult(triangleMesh.getVertex(triangleMesh.getTriangle(i).getB()).getPosition());
        Vector3f pwelt3 = matrix4f.mult(triangleMesh.getVertex(triangleMesh.getTriangle(i).getC()).getPosition());
        return new Vector3f[]{pwelt1, pwelt2, pwelt3};
    }

    public Vector3f[] createViewMatrix(Camera camera, Vector3f[] arr) {
        // Set up the view matrix and transform points
        Matrix4f viewMatrix = getViewMatrix(camera.getEye(), camera.getRef(), camera.getUp());
        Vector3f pcam1 = viewMatrix.mult(arr[0]);
        Vector3f pcam2 = viewMatrix.mult(arr[1]);
        Vector3f pcam3 = viewMatrix.mult(arr[2]);
        return new Vector3f[]{pcam1, pcam2, pcam3};
    }

    public Vector3f[] createProjectionMatrix(Vector3f[] arr) {
        // Set up the projection matrix and transform points
        Matrix4f projMatrix = getProjectionMatrix();
        Vector3f pbild1 = projMatrix.mult(arr[0]);
        Vector3f pbild2 = projMatrix.mult(arr[1]);
        Vector3f pbild3 = projMatrix.mult(arr[2]);
        pbild1.divide(pbild1.z);
        pbild2.divide(pbild2.z);
        pbild3.divide(pbild3.z);
        return new Vector3f[]{pbild1, pbild2, pbild3};
    }

    public Vector3f[] createScreenMappingMatrix(Vector3f[] arr, Camera camera) {
        // Set up the screenMapping matrix and transform points
        Matrix4f screenMapMatrix = getScreenMapMatrix(camera);
        Vector3f ppixel1 = screenMapMatrix.mult(arr[0]);
        Vector3f ppixel2 = screenMapMatrix.mult(arr[1]);
        Vector3f ppixel3 = screenMapMatrix.mult(arr[2]);
        return new Vector3f[]{ppixel1, ppixel2, ppixel3};
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

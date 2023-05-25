/**
 * Diese Datei ist Teil des Vorgabeframeworks für die Veranstaltung "Mixed Reality"
 * <p>
 * Prof. Dr. Philipp Jenke, Hochschule für Angewandte Wissenschaften Hamburg.
 */

package mixedreality.lab.exercise5;

import com.jme3.asset.AssetManager;
import com.jme3.light.PointLight;
import com.jme3.material.Material;
import com.jme3.math.*;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.shape.Sphere;
import mixedreality.base.mesh.TriangleMesh;
import mixedreality.base.mesh.TriangleMeshTools;
import mixedreality.base.mesh.Vertex;
import mixedreality.lab.exercise3.Camera;
import ui.AbstractCameraController;
import ui.Scene3D;

import mixedreality.lab.exercise5.Matrices;

import javax.swing.*;

/**
 * Base 3D scene for exercise 5.
 */
public class StereoScene extends Scene3D {

  /**
   * The asset manager is used to read content (e.g. triangle meshes or texture)
   * from file to jMonkey.
   */
  private AssetManager assetManager;

  /**
   * This is the root node of the scene graph with all the scene content.
   */
  private Node rootNode;

  /**
   * These objects represent the left and the right camera
   */
  protected Camera leftCamera;
  protected Camera rightCamera;

  /**
   * Pixel coordinates on the camera screens of the point to be computed
   */
  protected Vector2f leftScreenCoords, rightScreenCoords;

  /**
   * Matrices for camera projection
   */
  private Matrices matrix = new Matrices();

  public StereoScene() {
    assetManager = null;
    rootNode = null;

    leftCamera = new Camera(new Vector3f(3f, 2, 0), new Vector3f(0.5f, 0.5f, 0.5f), new Vector3f(0, 1, 0),
            degrees2Radiens(90), 1f, 1920, 1080);
    rightCamera = new Camera(new Vector3f(-2f, 1f, -2f), new Vector3f(0.5f, 0.5f, 0.5f), new Vector3f(0, 1, 0),
            degrees2Radiens(90), 1f, 1920, 1080);

    // Vorgabe
    leftScreenCoords = new Vector2f(1554, 666);
    rightScreenCoords = new Vector2f(821, 676);
  }

  /**
   * Compute the world coordinates of a given screen pixel.
   */
  private Vector3f toWorldCoordinateSystem(Vector2f screenCoords, Camera cam) {
    float dx = FastMath.tan(cam.getFovX() / 2.0f) * cam.getZ0();
    float dy = FastMath.tan(cam.getFovY() / 2.0f) * cam.getZ0();
    Matrix4f camMatrix = cam.makeCameraMatrix();
    Vector4f screenOrigin = camMatrix.mult(new Vector4f(-dx, -dy, cam.getZ0(), 1));
    float pixelSizeX = dx * 2 / cam.getWidth();
    float pixelSizeY = dy * 2 / cam.getHeight();
    Vector4f r4 = screenOrigin.add(
            camMatrix.mult(Vector4f.UNIT_X).mult(pixelSizeX * screenCoords.x)).add(
            camMatrix.mult(Vector4f.UNIT_Y).mult(pixelSizeY * screenCoords.y));
    return new Vector3f(r4.x, r4.y, r4.z);
  }

  @Override
  public void setupLights(Node rootNode, ViewPort viewPort) {
    // Lights
    PointLight light = new PointLight();
    light.setColor(new ColorRGBA(1f, 1f, 1f, 1));
    light.setPosition(new Vector3f(0, 0, 0));
    rootNode.addLight(light);

    PointLight light2 = new PointLight();
    light2.setColor(new ColorRGBA(0.5f, 0.5f, 0.5f, 1));
    light2.setPosition(new Vector3f(0, 5, 0));
    rootNode.addLight(light2);
  }

  @Override
  public void init(AssetManager assetManager, Node rootNode, AbstractCameraController cameraController) {
    this.assetManager = assetManager;
    this.rootNode = rootNode;
    cameraController.setup(new Vector3f(-3, 3, -3),
            new Vector3f(0, 0, 0), new Vector3f(0, 1, 0));

    // Cameras
    visualizeCamera(leftCamera, ColorRGBA.Pink);
    visualizeCamera(rightCamera, ColorRGBA.Cyan);

    // Coordinate system
    addLine(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0), ColorRGBA.Red);
    addLine(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0), ColorRGBA.Green);
    addLine(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1), ColorRGBA.Blue);

    // p in left and right screen
    Vector3f lsc3D = toWorldCoordinateSystem(leftScreenCoords, leftCamera);
    Vector3f rsc3D = toWorldCoordinateSystem(rightScreenCoords, rightCamera);
    addPoint(lsc3D, ColorRGBA.Yellow);
    addPoint(rsc3D, ColorRGBA.Yellow);
  }

  /**
   * Add geometry for the camera into the scene.
   */
  private void visualizeCamera(Camera cam, ColorRGBA color) {
    TriangleMesh mesh = new TriangleMesh();

    // Compute the camera corner points
    float dx = FastMath.tan(cam.getFovX() / 2.0f) * cam.getZ0();
    float dy = FastMath.tan(cam.getFovY() / 2.0f) * cam.getZ0();
    Matrix4f camMatrix = cam.makeCameraMatrix();
    Vector4f a = camMatrix.mult(new Vector4f(-dx, -dy, cam.getZ0(), 1));
    Vector4f b = camMatrix.mult(new Vector4f(dx, -dy, cam.getZ0(), 1));
    Vector4f c = camMatrix.mult(new Vector4f(dx, dy, cam.getZ0(), 1));
    Vector4f d = camMatrix.mult(new Vector4f(-dx, dy, cam.getZ0(), 1));
    mesh.addVertex(new Vertex(new Vector3f(a.x, a.y, a.z)));
    mesh.addVertex(new Vertex(new Vector3f(b.x, b.y, b.z)));
    mesh.addVertex(new Vertex(new Vector3f(c.x, c.y, c.z)));
    mesh.addVertex(new Vertex(new Vector3f(d.x, d.y, d.z)));
    mesh.addVertex(new Vertex(cam.getEye()));
    mesh.addTriangle(0, 1, 2);
    mesh.addTriangle(0, 2, 3);
    mesh.addTriangle(1, 0, 4);
    mesh.addTriangle(2, 1, 4);
    mesh.addTriangle(3, 2, 4);
    mesh.addTriangle(0, 3, 4);
    mesh.setColor(color);
    mesh.computeTriangleNormals();
    Geometry geo = TriangleMeshTools.createJMonkeyMesh(assetManager, mesh);
    geo.setShadowMode(RenderQueue.ShadowMode.Cast);
    rootNode.attachChild(geo);

    // Camera coordinate system
    Vector4f x4 = camMatrix.mult(Vector4f.UNIT_X);
    Vector4f y4 = camMatrix.mult(Vector4f.UNIT_Y);
    Vector4f z4 = camMatrix.mult(Vector4f.UNIT_Z);
    Vector3f x3 = new Vector3f(x4.x, x4.y, x4.z);
    Vector3f y3 = new Vector3f(y4.x, y4.y, y4.z);
    Vector3f z3 = new Vector3f(z4.x, z4.y, z4.z);
    addLine(cam.getEye(), cam.getEye().add(x3), ColorRGBA.Red);
    addLine(cam.getEye(), cam.getEye().add(y3), ColorRGBA.Green);
    addLine(cam.getEye(), cam.getEye().add(z3), ColorRGBA.Blue);
  }

  /**
   * Add a line from start to end to the scene.
   */
  protected void addLine(Vector3f start, Vector3f end, ColorRGBA color) {
    Mesh lineMesh = new Mesh();
    lineMesh.setMode(Mesh.Mode.Lines);
    lineMesh.setBuffer(VertexBuffer.Type.Position, 3, new float[]{start.x, start.y, start.z,
            end.x, end.y, end.z});
    lineMesh.setBuffer(VertexBuffer.Type.Index, 2, new short[]{0, 1});
    lineMesh.updateBound();
    lineMesh.updateCounts();
    Geometry lineGeometry = new Geometry("line", lineMesh);
    Material lineMaterial = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
    lineMaterial.setColor("Color", color);
    lineGeometry.setMaterial(lineMaterial);
    rootNode.attachChild(lineGeometry);
  }

  /**
   * Add a point visualized as a sphere in the scene.
   */
  protected void addPoint(Vector3f p, ColorRGBA color) {
    Sphere sphere = new Sphere(10, 10, 0.05f);
    Material mat = new Material(assetManager,
            "Common/MatDefs/Light/Lighting.j3md");
    mat.setColor("Diffuse", color);
    mat.setBoolean("UseVertexColor", false);
    Geometry sphereGeometry = new Geometry("sphere", sphere);
    sphereGeometry.setLocalTranslation(p);
    sphereGeometry.setMaterial(mat);
    rootNode.attachChild(sphereGeometry);
  }

  @Override
  public void update(float time) {
  }

  @Override
  public void render() {
  }

  @Override
  public String getTitle() {
    return "Mixed Reality";
  }

  /**
   * Helper method to convert degrees to radiens.
   */
  private float degrees2Radiens(float angleDegrees) {
    return angleDegrees / 180.0f * FastMath.PI;
  }

  /**
   * Error function
   */
  private double getDerivativeErrorFunction() {
    Vector4f initP = new Vector4f(0, 0, 0, 1);
    double stepSize = 1*10^-7;
    Vector4f leftScreenCoordidnates = new Vector4f(leftScreenCoords.x, leftScreenCoords.y, 0, 0);
    Vector4f rightScreenCoordidnates = new Vector4f(rightScreenCoords.x, rightScreenCoords.y, 0, 0);

    //
    Matrix4f cameraProjetctionMatrix = matrix.createViewMatrixCamA().mult(matrix.createProjectionMatrix(true)).mult(matrix.createScreenMappingMatrix(true));

    Vector4f forward_diff_x = cameraProjetctionMatrix.mult(new Vector4f((float) (initP.x + stepSize), initP.y, initP.z, initP.w)).subtract(cameraProjetctionMatrix.mult(initP));
  }

  private double gradientProcess() {
    return 0;
  }

}

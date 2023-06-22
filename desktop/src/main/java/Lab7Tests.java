import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import com.jme3.math.Vector3f;
import mixedreality.base.mesh.TriangleMesh;
import java.util.Optional;
import mixedreality.lab.exercise7.Index8Bit;
import mixedreality.lab.exercise7.MarchingCubes;

public class Lab7Tests {

    @Test
    public void testGetMesh() {
        // Arrange
        Index8Bit index = new Index8Bit();
        short value = 42;
        index.set(0, value);
        float[] values = {0.2f, 0.8f, 0.6f, 0.4f, 0.9f, 0.3f, 0.7f, 0.1f};
        float isovalue = 0.5f;
        MarchingCubes testObj = new MarchingCubes();

        Optional<TriangleMesh> result = testObj.getMesh(index, values, isovalue);
    }
}

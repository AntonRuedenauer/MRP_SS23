import com.jme3.math.Matrix3f;
import com.jme3.math.Vector2f;
import mixedreality.lab.exercise4.EdgeCollapse;
import mixedreality.lab.exercise4.Polygon;
import mixedreality.lab.exercise4.PolygonEdge;
import mixedreality.lab.exercise4.QuadricErrorMetricsSimplification2D;
import org.junit.Before;
import org.junit.Test;

import javax.swing.*;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class Lab4Tests {

    PolygonGenerator generator = new PolygonGenerator(6, 100, 100, 50);
    private QuadricErrorMetricsSimplification2D testObj;
    private Polygon polygon = new Polygon();
    private JComboBox<String> cbPoly;

    @Before
    public void init() {
        cbPoly = new JComboBox<>();
        cbPoly.addItem("polygons/hamburg.polygon");
        polygon.readFromFile((String) cbPoly.getSelectedItem());
        testObj = new QuadricErrorMetricsSimplification2D(polygon);
    }

    @Test
    public void testcomputeEdgeCollapseResult() {
        PolygonEdge input = polygon.getEdge(0);
        EdgeCollapse output = new EdgeCollapse(5.6269993365276605E-5,
                new Matrix3f(
                        (float) 2.8735104, (float) 0.5017073, (float) 2.8560898,
                        (float) 0.5017073, (float) 1.1264896, (float) 0.445045,
                        (float) 2.8560898, (float) 0.445045, (float) 2.841599), new Vector2f((float) -1.002949, (float) 0.05161336));
        EdgeCollapse result = testObj.computeEdgeCollapseResult(input);
        assertEquals(output.error, result.error);
        assertEquals(output.Q, result.Q);
        assertEquals(output.newPos, result.newPos);
    }
}

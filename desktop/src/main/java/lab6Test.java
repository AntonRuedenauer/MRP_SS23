import mixedreality.lab.exercise6.LSystemScene2D;
import org.junit.Test;

import static junit.framework.TestCase.assertEquals;

public class lab6Test {

    LSystemScene2D testObj = new LSystemScene2D(200, 200);

    @Test
    public void testReplaceOneRuleset() {
        assertEquals("F+F--F+F", testObj.replaceOneRuleset("F"));
        assertEquals("F+F--F+FF+F--F+F", testObj.replaceOneRuleset("FF"));
        assertEquals("F+F--F+F-F+F--F+F", testObj.replaceOneRuleset("F-F"));
        assertEquals("F+F--F+F+-+-", testObj.replaceOneRuleset("F+-+-"));
    }
}

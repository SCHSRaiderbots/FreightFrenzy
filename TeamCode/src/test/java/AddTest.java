// package org.firstinspires.ftc.teamcode;

// TODO: package above is screwy
// the package name must follow the directory structure....

// I got JUnit 4 to work...
//   Added to build.gradle (TeamCode):
//      testImplementation 'junit:junit:4.12'
import static org.junit.Assert.assertEquals;

import org.junit.Test;

// but not JUnit Jupiter...
// import static org.junit.jupiter.api.Assertions.assertEquals;

// import org.junit.jupiter.api.Test;

// for more involved tests, see
// https://ftc9929.com/2020/08/14/writing-ftc-robot-code-without-a-robot/

public class AddTest {

    @Test
    public void testAdd() {
        assertEquals(3, 2+1);
    }

    @Test
    public void testSub() {
        assertEquals(2, 3-1);
    }
}

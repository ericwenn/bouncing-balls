
/**
 * The physics model.
 * <p>
 * This class is where you should implement your bouncing balls model.
 * <p>
 * The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.
 *
 * @author Simon Robillard
 */
class Model {

    private double areaWidth, areaHeight;

    private static final double GRAVITY = -9.81;

    Ball[] balls;

    Model(double width, double height) {
        areaWidth = width;
        areaHeight = height;

        // Initialize the model with a few balls
        balls = new Ball[10];
        balls[0] = new Ball(3, 2, 0, 0, .2);
        balls[1] = new Ball(1.8, 1.2, .2, .2, 0.2);
        balls[2] = new Ball(1.8, 2.2, .5, 2.2, 0.2);
        balls[3] = new Ball(2.8, 3.2, .3, 1.2, 0.2);
        balls[4] = new Ball(1.8, 2.5, .7, 3.2, 0.2);
        balls[5] = new Ball(0.8, 1.2, .8, 1.2, 0.2);
        balls[6] = new Ball(0.3, 1.2, .1, 2, 0.2);
        balls[7] = new Ball(1.2, 0.2, 1.2, 3.2, 0.2);
        balls[8] = new Ball(2.8, .5, .2, 0.2, 0.2);
        balls[9] = new Ball(3.2, 1.2, .2, .2, 0.2);
    }

    void step(double deltaT) {


        double totalEnergy = 0;

        for (int i = 0; i < balls.length; i++) {

            Ball b = balls[i];

            // Check collision with borders
            // If collision, change direction and reset position to account for too big deltaT.
            if (b.x < b.radius || b.x > areaWidth - b.radius) {
                if (b.vx > 0) {
                    b.x = areaWidth - b.radius;
                } else {
                    b.x = b.radius;
                }
                b.vx *= -0.95;
            }
            if (b.y < b.radius || b.y > areaHeight - b.radius) {
                if (b.vy > 0) {
                    b.y = areaHeight - b.radius;
                } else {
                    b.y = b.radius;
                }
                b.vy *= -.95;
            }


            // Check collision with other balls
            // Only check each pair once
            for (int j = i + 1; j < balls.length; j++) {

                Ball ob = balls[j];


                if (doesCollide(b, ob)) {

                    // Separate balls until they are *just* colliding.
                    // Again this is because the deltaT might be too big.
                    int stepsReversed = reverse(b, ob);


                    Vector b_velocity = new Vector(b.vx, b.vy);
                    Vector ob_velocity = new Vector(ob.vx, ob.vy);

                    // Get vector between the two balls centres, this is the collision vector
                    Vector collisionVector = new Vector(ob.x - b.x, ob.y - b.y);


                    // Calculate both balls velocity parallel to the collision vector
                    Polar b_collisionVelocityBefore = project(b_velocity, collisionVector);
                    Polar ob_collisionVelocityBefore = project(ob_velocity, collisionVector);


                    // The velocity parallel to the collision will be recalculated, so we remove this part for now
                    Vector b_tmpVelocity = Vector.subtract(b_velocity, polarToRect(b_collisionVelocityBefore));
                    Vector ob_tmpVelocity = Vector.subtract(ob_velocity, polarToRect(ob_collisionVelocityBefore));


                    // Calculate the balls new velocity parallel to the collsion vector
                    VelocityPair vp = collide(b_collisionVelocityBefore.r, b.mass, ob_collisionVelocityBefore.r, ob.mass);

                    Polar b_collisionVelocityAfter = new Polar(vp.v1, b_collisionVelocityBefore.ang);
                    Polar ob_collisionVelocityAfter = new Polar(vp.v2, ob_collisionVelocityBefore.ang);


                    // Add the velocity prallell to the collision to the original velocity
                    b_velocity = Vector.add(b_tmpVelocity, polarToRect(b_collisionVelocityAfter));
                    ob_velocity = Vector.add(ob_tmpVelocity, polarToRect(ob_collisionVelocityAfter));


                    // Change the balls velocity
                    b.vx = b_velocity.x;
                    b.vy = b_velocity.y;

                    ob.vx = ob_velocity.x;
                    ob.vy = ob_velocity.y;

                    unreverse(b, ob, stepsReversed);

                }


            }

            b.vy = b.vy + GRAVITY * deltaT;

            // compute new position according to the speed of the ball
            b.x += deltaT * b.vx;
            b.y += deltaT * b.vy;
        }
    }

    private static VelocityPair collide(double u1, double m1, double u2, double m2) {
        double I = u1 * m1 + u2 * m2;

        double R = -(u2 - u1);

        double v1 = (I - m2 * R) / (m2 + m1);
        double v2 = R + v1;

        return new VelocityPair(v1, v2);
    }


    private static int reverse(Ball b1, Ball b2) {

        double delta = -.001;

        int steps = 0;
        while (doesCollide(b1, b2)) {
            b1.x += delta * b1.vx;
            b1.y += delta * b1.vy;

            b2.x += delta * b2.vx;
            b2.y += delta * b2.vy;
            steps++;
        }
        return steps;
    }

    private static void unreverse(Ball b1, Ball b2, int steps) {
        double delta = 0.001;
        for( int i = 0; i<steps; i++) {
            b1.x += delta * b1.vx;
            b1.y += delta * b1.vy;

            b2.x += delta * b2.vx;
            b2.y += delta * b2.vy;
        }
    }



    private static boolean doesCollide(Ball b1, Ball b2) {

        double distance = Math.sqrt(Math.pow(b1.x - b2.x, 2) + Math.pow(b1.y - b2.y, 2));
        return distance < b1.radius + b2.radius;

    }


    private static Polar project(Vector v, Vector projectOn) {
        return project(rectToPolar(v), rectToPolar(projectOn));
    }


    private static Polar project(Polar v, Polar projectOn) {
        double angBetween = Math.abs(projectOn.ang - v.ang);
        double cos = Math.cos(Math.toRadians(angBetween));
        double r = v.r * cos;
        return new Polar(r, projectOn.ang);
    }


    /**
     * Simple inner class describing balls.
     */
    class Ball {

        Ball(double x, double y, double vx, double vy, double r) {
            this.x = x; // Center of ball
            this.y = y; // Center of ball
            this.vx = vx;
            this.vy = vy;
            this.radius = r;
            this.mass = Math.pow(r, 2) * 10;
        }

        /**
         * Position, speed, and radius of the ball. You may wish to add other attributes.
         */
        double x, y, vx, vy, radius, mass;

        @Override
        public String toString() {
            return "Ball{" +
                    "x=" + x +
                    ", y=" + y +
                    ", vx=" + vx +
                    ", vy=" + vy +
                    '}';
        }
    }


    static class Vector {

        double x;
        double y;

        public Vector(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public String toString() {
            return "Vector{" +
                    "x=" + x +
                    ", y=" + y +
                    '}';
        }


        public static Vector subtract(Vector v1, Vector v2) {
            return new Vector(v1.x - v2.x, v1.y - v2.y);
        }

        public static Vector add(Vector v1, Vector v2) {
            return new Vector(v1.x + v2.x, v1.y + v2.y);

        }


    }


    static class VelocityPair {

        double v1;
        double v2;

        public VelocityPair(double v1, double v2) {
            this.v1 = v1;
            this.v2 = v2;
        }
    }

    static class Polar {
        double r;
        double ang;

        Polar(double r, double ang) {
            this.r = r;
            this.ang = ang;
        }

        @Override
        public String toString() {
            return "Polar{" +
                    "r=" + r +
                    ", ang=" + ang +
                    '}';
        }
    }


    private static Polar rectToPolar(Vector v) {

        int maybeAddition = v.x < 0 ? 180 : 0;
        double r = Math.sqrt(Math.pow(v.x, 2) + Math.pow(v.y, 2));

        double ang = Math.toDegrees(Math.atan(v.y / v.x)) + maybeAddition;

        ang = ang < 0 ? ang + 360 : ang;
        return new Polar(r, ang);

    }

    private static Vector polarToRect(Polar p) {
        double x = p.r * Math.cos(Math.toRadians(p.ang));
        double y = p.r * Math.sin(Math.toRadians(p.ang));

        return new Vector(x, y);
    }


    public static void main(String[] args) {

        testProjection();
        testPolarToRect();
        testRectToPolar();
        testEnergyPreserved();

    }


    private static void testPolarToRect() {

        Polar[] polars = new Polar[]{
                new Polar(1, 0),
                new Polar(1, 90),
                new Polar(1, 180),
                new Polar(1, 270)
        };

        Vector[] vectors = new Vector[]{
                new Vector(1, 0),
                new Vector(0, 1),
                new Vector(-1, 0),
                new Vector(0, -1),
        };

        for (int i = 0; i < polars.length; i++) {

            Vector calculatedVector = polarToRect(polars[i]);
            Vector correctVector = vectors[i];
            if (Math.abs(calculatedVector.x - correctVector.x) > 1e-10 || Math.abs(calculatedVector.x - correctVector.x) > 1e-10) {
                System.out.println("polarToRect does not work. Expected " + correctVector + ", but got " + calculatedVector);
                return;
            }

        }

        System.out.println("PolarToRect works");

    }


    private static void testRectToPolar() {

        Polar[] polars = new Polar[]{
                new Polar(1, 0),
                new Polar(1, 90),
                new Polar(1, 180),
                new Polar(1, 270)
        };

        Vector[] vectors = new Vector[]{
                new Vector(1, 0),
                new Vector(0, 1),
                new Vector(-1, 0),
                new Vector(0, -1),
        };

        for (int i = 0; i < vectors.length; i++) {

            Polar calculatedPolar = rectToPolar(vectors[i]);
            Polar correctPolar = polars[i];

            if (Math.abs(calculatedPolar.r - correctPolar.r) > 1e-10 || Math.abs(calculatedPolar.ang - correctPolar.ang) > 1e-10) {
                System.out.println("rectToPolar does not work. Expected " + correctPolar + ", but got " + calculatedPolar);
                return;
            }

        }

        System.out.println("RectToPolar works");

    }

    private static void testProjection() {

        Vector v = new Vector(20, 20);
        Vector projectionVector = new Vector(1, 0);

        Polar projection = project(v, projectionVector);

        if (projection.r - 20 < 1e-5 && projection.ang == 0) {
            System.out.println("Projection works");
        } else {
            System.out.println("Projections does not work. r=" + projection.r + " (expected 20) ang=" + projection.ang + " (expected 0)");
        }


    }


    private static void testEnergyPreserved() {


        double u1 = 2.0;
        double m1 = 2.0;

        double u2 = -1.0;
        double m2 = 1.5;

        VelocityPair vp = collide(u1, m1, u2, m2);

        double v1 = vp.v1;
        double v2 = vp.v2;

        double delta = (m1 * Math.pow(u1, 2) + m2 * Math.pow(u2, 2)) / 2 - (m1 * Math.pow(v1, 2) + m2 * Math.pow(v2, 2)) / 2;

        if (Math.abs(delta) < 1e-10) {
            System.out.println("Energy preserved");
        } else {
            System.out.println("Energy not preserved, delta=" + delta);
        }
    }


}

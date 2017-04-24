
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

    Ball[] balls;

    Model(double width, double height) {
        areaWidth = width;
        areaHeight = height;

        // Initialize the model with a few balls
        balls = new Ball[2];
        balls[0] = new Ball(3, 2, -1.5, -1.2, .2);
        balls[1] = new Ball(1.8, 1.2, .2, .2, 0.2);
    }

    void step(double deltaT) {

        for (int i = 0; i < balls.length; i++) {
            Ball b = balls[i];
            // detect collision with the border

            if (b.x < b.radius || b.x > areaWidth - b.radius) {
                if( b.vx > 0) {
                    b.x = areaWidth - b.radius;
                } else {
                    b.x = b.radius;
                }
                b.vx *= -1; // change direction of ball
            }

            if (b.y < b.radius || b.y > areaHeight - b.radius) {
                if( b.vy > 0) {
                    b.y = areaHeight - b.radius;
                } else {
                    b.y = b.radius;
                }
                b.vy *= -1;
            }


            for (int j = i + 1; j < balls.length; j++) {
                Ball ob = balls[j];
                if (doesCollide(b, ob)) {

                    separate(b, ob, true);



                    Polar cvv = rectToPolar(new Vector(ob.x - b.x, ob.y - b.y));


                    Polar v1 = veloRelaVect(new Vector(b.vx, b.vy), cvv);

                    Polar v2 = veloRelaVect(new Vector(ob.vx, ob.vy), cvv);

                    VelocityPair vp = newVelo(v1.r, b.mass, v2.r, ob.mass);

                    Polar vv1 = new Polar(vp.v1, cvv.ang);
                    Polar vv2 = new Polar(vp.v2, cvv.ang);



                    Vector rel1 = polarToRect(v1);
                    Vector relafter1 = polarToRect(vv1);
                    Vector rel2 = polarToRect(v2);
                    Vector relafter2 = polarToRect(vv2);


                    Vector newVelo1 = new Vector(b.vx - rel1.x + relafter1.x, b.vy - rel1.y + relafter1.y);
                    Vector newVelo2 = new Vector(ob.vx - rel2.x + relafter2.x, ob.vy - rel2.y + relafter2.y);


                    b.vx = newVelo1.x;
                    b.vy = newVelo1.y;

                    ob.vx = newVelo2.x;
                    ob.vy = newVelo2.y;

                    //separate(b, ob, false);
                }
            }


            // compute new position according to the speed of the ball
            b.x += deltaT * b.vx;
            b.y += deltaT * b.vy;
        }
    }


    private static void separate(Ball b1, Ball b2, boolean reverse) {

        double delta = .001;

        if (reverse) {
            delta *= -1;
        }

        while (doesCollide(b1, b2)) {
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


    private static Polar veloRelaVect(Vector v, Polar collisionVector) {

        Polar vPolar = rectToPolar(v);
        double angBetween = Math.abs(collisionVector.ang - vPolar.ang);
        double cos = Math.cos(Math.toRadians(angBetween));
        double r = vPolar.r * cos;
        return new Polar(r, collisionVector.ang);
    }

    private static VelocityPair newVelo(double u1, double m1, double u2, double m2) {


        double I = u1 * m1 + u2 * m2;

        double R = -(u2 - u1);

        double v1 = (I - m2 * R) / (m2 + m1);
        double v2 = R + v1;

        return new VelocityPair(v1, v2);

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
            this.mass = Math.pow(r,2)*10;
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

        double ang = Math.toDegrees(Math.atan(v.y / v.x));
        return new Polar(r, ang + maybeAddition);

    }

    private static Vector polarToRect(Polar p) {
        double x = p.r * Math.cos(Math.toRadians(p.ang));
        double y = p.r * Math.sin(Math.toRadians(p.ang));

        return new Vector(x, y);
    }


    public static void main(String[] args) {

        testVeloRelaVect();
/*
        Vector v1 = new Vector(1, 0);
        Vector v2 = new Vector(-1, 0);
        Vector v3 = new Vector(1, 1);
        Vector v4 = new Vector(-1, 1);
        Vector v5 = new Vector(2, 1);
        Vector v6 = new Vector(1, 3);


        Polar p1 = rectToPolar(v1);
        System.out.println(p1);

        Polar p2 = rectToPolar(v2);
        System.out.println(p2);

        Polar p3 = rectToPolar(v3);
        System.out.println(p3);

        Polar p4 = rectToPolar(v4);
        System.out.println(p4);

        Polar p5 = rectToPolar(v5);
        System.out.println(p5);

        Polar p6 = rectToPolar(v6);
        System.out.println(p6);
        */

    }

    private static void testVeloRelaVect() {

        Polar p = new Polar(100, 90);


        Vector v1 = new Vector(1, 1);
        Vector v2 = new Vector(1, 0);
        Vector v3 = new Vector(0, 1);
        Vector v4 = new Vector(-1, 1);

        System.out.println(veloRelaVect(v1,p));
        System.out.println(veloRelaVect(v2,p));
        System.out.println(veloRelaVect(v3,p));
        System.out.println(veloRelaVect(v4,p));
    }
}

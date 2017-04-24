
/**
 * The physics model.
 * 
 * This class is where you should implement your bouncing balls model.
 * 
 * The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.
 * 
 * @author Simon Robillard
 *
 */
class Model {

	private double areaWidth, areaHeight;
	
	Ball [] balls;

	Model(double width, double height) {
		areaWidth = width;
		areaHeight = height;
		
		// Initialize the model with a few balls
		balls = new Ball[2];
		balls[0] = new Ball(width / 3, height * 0.9, 1.2, 1.6, 0.2);
		balls[1] = new Ball(2 * width / 3, height * 0.7, -0.6, 0.6, 0.3);
	}

	void step(double deltaT) {
		// TODO this method implements one step of simulation with a step deltaT
		for (Ball b : balls) {
			// detect collision with the border
			if (b.x < b.radius || b.x > areaWidth - b.radius) {
				b.vx *= -1; // change direction of ball
			}

			if (b.y < b.radius || b.y > areaHeight - b.radius) {
				b.vy *= -1;
			}


			for( Ball otherB : balls) {
				if( b != otherB) {
					if(doesCollide(b, otherB)) {
						b.vx *= -2;
						b.vy *= -2;
					}

				}
			}

			// compute new position according to the speed of the ball
			b.x += deltaT * b.vx;
			b.y += deltaT * b.vy;
		}
	}



	static boolean doesCollide(Ball b1, Ball b2) {
		double distance = Math.sqrt(Math.pow(b1.x - b2.x, 2) + Math.pow(b1.y - b2.y, 2));
		return distance < b1.radius + b2.radius;
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
		}

		/**
		 * Position, speed, and radius of the ball. You may wish to add other attributes.
		 */
		double x, y, vx, vy, radius;
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

	static class Polar {
		double r;
		double ang;

		public Polar(double r, double ang) {
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



	static Polar rectToPolar( Vector v) {

		double r = Math.sqrt( Math.pow(v.x, 2) + Math.pow(v.y, 2));
		double ang = Math.toDegrees(Math.atan( v.y/v.x));

		return new Polar( r, ang );

	}

	static Vector polarToRect( Polar p ) {
		double x = p.r*Math.cos(Math.toRadians(p.ang));
		double y = p.r*Math.sin(Math.toRadians(p.ang));

		return new Vector(x,y);
	}



	public static void main( String[] args ) {
		Vector v = new Vector(1,-5);
		Polar p = rectToPolar(v);
		Vector v1 = polarToRect(p);
		System.out.println( p);
		System.out.println( v1);
	}
}

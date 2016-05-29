import 'package:physics2d/physics2d.dart';

//TODO: Just make this a function?

class Edge2D {
  Vector2D position;
  Vector2D direction;

  Edge2D(Vector2D start, Vector2D end) {
    this.position = start;
    this.direction = end - start;
  }

  /*
  p + t r = q + u s

  segment from p to p+r (so r is not normalized? depends on what the scalar value is (0 to 1?).)

  point p on a
  point q on b
  r direction of a
  s direction of b
  t run param for a
  u run param for b


  solving for t and u:
  t = (q − p) × s / (r × s)
  u = (q − p) × r / (r × s)

  There are 4 cases
  If r × s = 0 and (q − p) × r = 0, then the two lines are collinear.
  If r × s = 0 and (q − p) × r ≠ 0, then the two lines are parallel and non-intersecting.
  If r × s ≠ 0 and 0 ≤ t ≤ 1 and 0 ≤ u ≤ 1, the two line segments meet at the point p + t r = q + u s.
  Otherwise, the two line segments are not parallel but do not intersec
  */
  bool intersects(Edge2D other) {
    Vector2D diff = (other.position - this.position);
    double rXs = this.direction.crossProduct(other.direction);

    double t = diff.crossProduct(other.direction)/rXs;
    double diffXr = diff.crossProduct(this.direction);
    double u = diffXr/rXs;

    if (rXs == 0 && diffXr == 0) { //COLLINEAR
      print("collinear");
      return true;
    } else
    if (rXs == 0 && diffXr != 0) { //PARALLEL (NON-INTERSECTING)
      return false;
    } else
    if (rXs != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1) {
      //Intersecting and point can be determined.
      return true;
    } else { //NOT PARALLEL BUT NOT INTERSECTING
      return false;
    }
  }

}

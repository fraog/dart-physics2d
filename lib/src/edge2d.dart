import 'package:physics2d/physics2d.dart';

/// A 2d edge has a start position and a direction (which denotes the end).
/// It is no different than a line segment.
class Edge2D {

  Vector2D position;
  Vector2D direction;

  Edge2D(Vector2D start, Vector2D end) {
    this.position = start;
    this.direction = end - start;
  }

  /// Returns true if these two edges intersect.
  bool intersects(Edge2D other) {
    Vector2D diff = (other.position - this.position);
    double rXs = this.direction.crossProduct(other.direction);

    double t = diff.crossProduct(other.direction)/rXs;
    double diffXr = diff.crossProduct(this.direction);
    double u = diffXr/rXs;

    if (rXs == 0 && diffXr == 0) {
      //COLLINEAR
      //TODO: Determine if they are overlapping.
      return true;
    } else
    if (rXs == 0 && diffXr != 0) {
      //Parallel (not intersecting)
      return false;
    } else
    if (rXs != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1) {
      //Intersecting and point can be determined.
      return true;
    } else {
      //Not parallel but not intersecting.
      return false;
    }
  }

}

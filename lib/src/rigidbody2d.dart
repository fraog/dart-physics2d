import 'package:physics2d/physics2d.dart';

class RigidBody2D extends Rect2D {

  Vector2D velocity;

  //These are both in radians.
  double rotation = 0.0;
  double angularVelocity = 0.0;
  num maxr = 0;
  double mass = 1.0;

  //TEMP:
  Vector2D impactPoint = null;
  Vector2D impactPoint2 = null;
  List<Vector2D> overlaps = null;

  RigidBody2D(double x, double y, num w, num h) : super(x, y, w, h) {
    this.velocity = new Vector2D(0.0, 0.0);
    this.maxr = (w > h) ? w/2 : h/2;
  }

  List<Vector2D> get normalizedAxes {
    List<Vector2D> axes = [ new Vector2D(1.0, 0.0), new Vector2D(0.0, 1.0) ];
    axes[0].rotateBy(this.rotation);
    axes[1].rotateBy(this.rotation);
    return axes;
  }

  List<Vector2D> getRelativeCorners(List<Vector2D> normalAxes) {
    if (normalAxes == null) {
      normalAxes = this.normalizedAxes;
    }

    List<Vector2D> axesf = [normalAxes[0]*hw, normalAxes[1]*hh];
    List<Vector2D> points = new List<Vector2D>(4);
    points[3] = axesf[0]+axesf[1]; //BR
    points[0] = points[3].inverted; //TL
    points[1] = axesf[0]-axesf[1]; //TR
    points[2] = points[1].inverted; //BL

    //TL, TR, BL, BR
    return points;
  }

  List<Vector2D> getAbsoluteCorners(List<Vector2D> normalAxes) {
    List<Vector2D> points = this.getRelativeCorners(normalAxes);
    for (Vector2D v in points) v.add(this);
    return points;
  }

}

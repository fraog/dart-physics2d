import 'package:physics2d/physics2d.dart';

/// A 2d rigid body. 
class RigidBody2D extends Rect2D {

  Vector2D velocity;

  //These are both in radians.
  double rotation = 0.0;
  double angularVelocity = 0.0;
  //num maxr = 0;
  double mass = 1.0;
  double momentOfInertia = 0.0;

  Vector2D corner = null;
  int bufferCount = 0;

  RigidBody2D(double x, double y, num w, num h) : super(x, y, w, h) {
    this.velocity = new Vector2D(0.0, 0.0);
    //this.maxr = (w > h) ? w/2 : h/2;
    this.mass = 1.0;
    this.momentOfInertia = this.mass*(w*w + h*h)/12;
    this.corner = new Vector2D(w/2, h/2);
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

  List<Edge2D> getAbsoluteEdges(List<Vector2D> points) {
    if (points == null) points = this.getAbsoluteCorners(null);
    List<Edge2D> edges = new List<Edge2D>(4);
    edges[0] = new Edge2D(points[0], points[1]); //TOP -TL TO TR
    edges[1] = new Edge2D(points[2], points[3]); //BOT - BL TO BR
    edges[2] = new Edge2D(points[0], points[2]); //LEFT - TL TO BL
    edges[3] = new Edge2D(points[1], points[3]); //RIGHT - TR TO BR
    return edges;
  }

  void applyImpulse(Vector2D force) {
    this.velocity = this.velocity + force/this.mass;
  }

  //TODO: this changes the rotational velocity as well?
  // var wa2 = wa1 + (rap.crossProduct(jn)/Ia)/3.0;
  // or is that impulse at point only?
}

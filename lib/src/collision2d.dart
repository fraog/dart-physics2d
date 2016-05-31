import 'package:physics2d/physics2d.dart';

/// A 2d collision object consists of an impact point and an impact normal (representing the edge that
/// was impacted).
class Collision2D {

  Vector2D impactPoint;
  Vector2D impactNormal;

  Collision2D(this.impactPoint, this.impactNormal);
}

import 'dart:math';

class Vector2D {

  double x;
  double y;

  Vector2D(this.x, this.y);

  String toString() => this.x.toString()+", "+this.y.toString();

  void add(Vector2D other) {
    this.x += other.x;
    this.y += other.y;
  }

  void subtract(Vector2D other) {
    this.x -= other.x;
    this.y -= other.y;
  }

  void multiply(num scalar) {
    this.x *= scalar;
    this.y *= scalar;
  }

  //operator ==(Vector2D other) => (this.x == other.x && this.y == other.y);
  operator +(Vector2D other) => new Vector2D(this.x+other.x, this.y+other.y);
  operator -(Vector2D other) => new Vector2D(this.x-other.x, this.y-other.y);
  operator *(num other) => new Vector2D(this.x*other, this.y*other);
  operator /(num other) => new Vector2D(this.x/other, this.y/other);

  void normalize() {
    num m = this.magnitude;
    this.x /= m;
    this.y /= m;
  }

  Vector2D get normalized {
    num m = this.magnitude;
    return new Vector2D(this.x/m, this.y/m);
  }

  num get magnitude => sqrt( (x*x) + (y*y) );
  num get magnitudeSqr => (x*x) + (y*y);

  void positive() {
    if (this.x < 0) {
      this.x = 0-this.x;
    }
    if (this.y < 0) {
      this.y = 0-this.y;
    }
  }

  /**
   * U x V = Ux*Vy-Uy*Vx
   */
  double crossProduct(Vector2D other) => this.x*other.y-this.y*other.x;
  double dotProduct(Vector2D other) => this.x*other.x + this.y*other.y;

  Vector2D get rightNormal => new Vector2D(-this.y, this.x);
  Vector2D get leftNormal => new Vector2D(this.y, -this.x);

  /* Projected onto b. */
  Vector2D project(Vector2D b) {
    num dp = this.dotProduct(b);
    num sqrs = dp/(b.x*b.x + b.y*b.y);
    return new Vector2D(sqrs * b.x, sqrs * b.y);
  }

  /* Projected onto unit vector b. */
  Vector2D projectUnit(Vector2D b) {
      num dp = this.dotProduct(b);
      return new Vector2D(dp*b.x, dp*b.y);
  }


  double scalarProjection(Vector2D b) {
    Vector2D proj = this.project(b);
    return proj.dotProduct(b);
  }



  void rotateBy(double rads) {
    double s = sin(rads);
    double c = cos(rads);
    double nx = this.x * c - this.y * s;
    double ny = this.x * s + this.y * c;
    this.x = nx;
    this.y = ny;
  }

  Vector2D rotatedBy(double rads) {
    double s = sin(rads);
    double c = cos(rads);
    return new Vector2D(this.x * c - this.y * s, this.x * s + this.y * c);
  }

  void invert() {
    this.x = -this.x;
    this.y = -this.y;
  }

  Vector2D get inverted => new Vector2D(-this.x, -this.y);
}

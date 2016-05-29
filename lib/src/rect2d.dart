import 'package:physics2d/physics2d.dart';

class Rect2D extends Vector2D {

  num w;
  num h;

  num hw;
  num hh;

  Rect2D(num x, num y, num w, num h) : super(x, y) {
    this.w = w;
    this.h = h;
    this.hw = w/2;
    this.hh = h/2;
  }

  num get top => this.y-this.hh;
  num get bot => this.y+this.hh;
  num get left => this.x-this.hw;
  num get right => this.x+this.hw;








}

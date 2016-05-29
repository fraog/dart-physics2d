// Copyright (c) 2016, <your name>. All rights reserved. Use of this source code
// is governed by a BSD-style license that can be found in the LICENSE file.

import 'package:physics2d/physics2d.dart';
import 'package:test/test.dart';

void main() {
  Vector2D vector = new Vector2D(0.0, 10.0);
  Rect2D rect = new Rect2D(0.0, 0.0, 5.0, 5.0);
  RigidBody2D body = new RigidBody2D(0.0, 0.0, 5.0, 5.0);
  Edge2D edge = new Edge2D(vector, new Vector2D(5.0, 5.0));
}

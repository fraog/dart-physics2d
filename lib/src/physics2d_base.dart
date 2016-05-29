import 'dart:async';
import 'dart:collection';
import 'package:physics2d/physics2d.dart';

class Physics {

  static HashMap<String, RigidBody2D> Cache = new HashMap<String, RigidBody2D>();
  static double TimeScale = 1.0;

  static Duration updateInterval = new Duration(milliseconds: 25);
  static Timer timer;

  static void start() {
    if (timer != null) return;
    print("starting physics loop.");
    timer = new Timer.periodic(updateInterval, Physics.update);
    //TODO: startTime
  }

  static void update(Timer t) {
    Queue<RigidBody2D> moved = new Queue<RigidBody2D>();

    for (RigidBody2D body in Physics.Cache.values) {
      if (body.velocity.x != 0 || body.velocity.y != 0 || body.angularVelocity != 0) {
        body.add(body.velocity*TimeScale);
        body.rotation += body.angularVelocity;
        if (body.rotation > 6.283) body.rotation -= 6.283;
        moved.add(body);
      }
    }

    while (moved.length > 0) {
      Physics.ResolveCollisions(moved.removeFirst());
    }
  }

  static bool CollisionBroadPhase(RigidBody2D body, RigidBody2D other) {
    /*
     * Are our bounding spheres overlapping?
     * This would almost always be true for a long skinny bounding box.
     */
    Vector2D diff = body - other;
    diff.positive();
    num maxdist = body.maxr+other.maxr;
    return (diff.x < maxdist && diff.y < maxdist);
  }

  static void ProjectVertices(Vector2D axis, List<Vector2D> points, List<double> minmax) {
    minmax[0] = axis.dotProduct(points[0]);
    minmax[1] = minmax[0];
    for (int i=1; i<points.length; i++) {
      double p = axis.dotProduct(points[i]);
      if (p < minmax[0]) minmax[0] = p;
      else if (p > minmax[1]) minmax[1] = p;
    }
  }

  /**
   * Separating Axis Theorem:
   * Given two convex shapes, if we can find an axis in which the projection of the two shapes does not overlap,
   * then the shapes dont overlap. (all axes of the shapes must overlap for collision).
   *
   * There are two axes per AABB (horiz, vert)
   * When projecting B onto an edge of A we project the relative position of a
   *
   * This will return the MTV or Minimum Translation Vector (for resolving collision) or null if no collision.
   */
  static List<Vector2D> CollisionNarrowPhase(RigidBody2D body, RigidBody2D other) {

    /**
     * Get axes of body and other.
     * TODO: I might have to make relative.
     */
    List<Vector2D> axes = body.normalizedAxes;
    List<Vector2D> axes2 = other.normalizedAxes;

    /* Get the points of body and other using their axes. */
    List<Vector2D> points1 = body.getAbsoluteCorners(axes);
    List<Vector2D> points2 = other.getAbsoluteCorners(axes2);

    /*
     * For each axis on body and other, project points from both body and other and check for overlap of
     * their minimum and maximum.
     */
    axes.addAll(axes2);

    List<double> range1 = [ 0.0, 0.0 ];
    List<double> range2 = [ 0.0, 0.0 ];

    double overlap = -1.0;
    //Vector2D minAxis = null;

    List<Vector2D> overlaps = new List<Vector2D>();

    for (Vector2D axis in axes) {



      //Get the min and max of all the dot products of axis and each point.
      Physics.ProjectVertices(axis, points1, range1);
      Physics.ProjectVertices(axis, points2, range2);

      //Check if they overlap
      if (range1[1] < range2[0] || range1[0] > range2[1]) {
        return null;
      } else {
        //Get the overlap for 4 cases - left hanging, right hanging, containment, contained
        num o = 0;
        if (range1[0] < range2[0] && range1[1] > range2[0]) {
          o = range1[1]-range2[0];
        } else if (range2[0] < range1[0] && range2[1] > range1[0]) {
          o = range2[1]-range1[0];
        } else {
          //TODO: containment cases, how to resolve.
          //o = range1[1]-range1[0];
          print("contain");
        }

        overlaps.add(axis * o);

        /*
        if (o == 0) continue;

        //Update min overlap/axis.
        if (overlap < 0 || o < overlap) {
          overlap = o;
          minAxis = axis;
        }
        */
      }
    }

    /* If they were all overlapping, they must be colliding. */
    return overlaps;
  }

  //TEMP:
  static List<Edge2D> getAbsoluteEdges(RigidBody2D body, List<Vector2D> normalAxes) {
    List<Vector2D> points = body.getAbsoluteCorners(normalAxes);
    List<Edge2D> edges = new List<Edge2D>(4);
    edges[0] = new Edge2D(points[0], points[1]); //TOP -TL TO TR
    edges[1] = new Edge2D(points[2], points[3]); //BOT - BL TO BR
    edges[2] = new Edge2D(points[0], points[2]); //LEFT - TL TO BL
    edges[3] = new Edge2D(points[1], points[3]); //RIGHT - TR TO BR
    return edges;
  }

  static void ResolveCollisions(RigidBody2D body) {
    //DEBUG: skip the big one
    if (body.angularVelocity > 0.0 || body.rotation > 0.0) return;

    /*
     * We will definitely not be going through the physics cache to do this.
     * generate a collision graph before resolution.
     */
     for (RigidBody2D other in Physics.Cache.values) {
       if (body == other) continue;
       if (CollisionBroadPhase(body, other)) {

         var overlaps = CollisionNarrowPhase(body, other);
         if (overlaps != null) {

           /*
            * TODO: Resolve the collision, apply force based on impact. moment of inertia yadda
            * mark pair as resolved.
            */

            //TEMP
            //TOP BOT LEFT RIGHT
            List<Edge2D> edges1 = getAbsoluteEdges(body, body.normalizedAxes);
            List<Edge2D> edges2 = getAbsoluteEdges(other, other.normalizedAxes);

            //WHat if i just used an assoc array so i would know which edge by (1, 2, 3, 4) - top, bot, left, right
            HashSet<int> inters = new HashSet<int>();
            HashSet<int> ointers = new HashSet<int>();

            for (int i=0; i < edges1.length; i++) {
              for (int o=0; o < edges2.length; o++) {
                if (edges1[i].intersects(edges2[o])) {
                  inters.add(i);
                  ointers.add(o);
                  break;
                }
              }
            }

            //FIXME: get min overlap again.
            Vector2D minOverlap = null;
            int minIndex = -1;
            for (int i=2; i<overlaps.length; i++) {
              var o = overlaps[i];
              if (minOverlap == null || o.magnitude < minOverlap.magnitude) {
                minOverlap = o;
                minIndex = i;
              }
            }
            body.impactPoint2 = minOverlap;

            //Cases:
            //FIXME: THIS IS ONLY USING AXES
            body.impactPoint = new Vector2D(0.0, 0.0);
            if (inters.contains(0)) {
              print("my top");
              //body.impactPoint.add((edges1[0].direction/2).inverted);
              //VERT AXIS HALF HEIGHT.
              body.impactPoint.add((edges1[2].direction/2).inverted);
            }
            if (inters.contains(1)) {
              print("my bot");
              body.impactPoint.add(edges1[2].direction/2);
            }
            print(body.impactPoint.toString());

            if (inters.contains(2)) {
              print("my left");
              body.impactPoint.add((edges1[0].direction/2).inverted);
            }
            if (inters.contains(3)) {
              print("my right");
              body.impactPoint.add(edges1[0].direction/2);
            }
            print(body.impactPoint.toString());

            //My corner overlaps with B's edge.
            if (inters.length == 2 && ointers.length == 1) {
              print("my corner collided with an edge.");
              if (ointers.contains(0)) {
                print("his top");
              }
              if (ointers.contains(1)) {
                print("his bot");
              }
              if (ointers.contains(2)) {
                print("his left");
              }
              if (ointers.contains(3)) {
                print("his right");
              }

              //My bottom or my right needs invert.
              if (inters.contains(1)) {
                minOverlap.y = -minOverlap.y;
              }
              if (inters.contains(3)) {
                minOverlap.x = -minOverlap.x;
              }

              if (minIndex == 0 || minIndex == 3) {
                print("his horizontal");
                body.add(minOverlap);
              } else if (minIndex == 1 || minIndex == 2) {
                print("his vertical");
                body.add(minOverlap);
              }


            } else
            //Both of our corners overlap
            if (inters.length == 2 && ointers.length == 2) {
              print("my corner collided with another corner.");
              print("overlap index "+minIndex.toString());
              print(minOverlap.toString());



              if (inters.contains(0)) {
                //make pos
                if (minOverlap.y < 0) minOverlap.y = -minOverlap.y;
              } else
              if (inters.contains(1)) {
                //Make negative
                if (minOverlap.y > 0) minOverlap.y = -minOverlap.y;
              }
              if (inters.contains(2)) { //eft
                //make positive
                if (minOverlap.x < 0) minOverlap.x = -minOverlap.x;
              } else
              if (inters.contains(3)) {
                //make negative
                if (minOverlap.x > 0) minOverlap.x = -minOverlap.x;
              }

              body.add(minOverlap);
            }
            //TODO: Our edges overlap (3 intersections).



            body.velocity.x = 0.0;
            body.velocity.y = 0.0;
            body.angularVelocity = 0.0;
            other.velocity.x = 0.0;
            other.velocity.y = 0.0;
            other.angularVelocity = 0.0;
            break;
          }
       }
     }
  }

  /**
   * RESPONSE:
   * Formula for velocity of an arbitrary point on a rotating and translating rigid body.
   *
   *  VelpA = VelA + AngleA X Rap
   *  X - cross product.
   *  Rap = point from center of mass A to impact P
   *  AngleA regarded as 3d vector, crossproduct:
   *  ω × r = (0, 0, ω) × (rx, ry, 0) = (−ω ry, ω rx, 0)
   *
   *  Post collision velocities:
   *  Vab1 - the pre collision relative velocity
   *  Vab1 = VelpA - VelpB
   *  Vab2 =
   *
   *  dot product:
   *  n is the normal (perpendicular) of the edge on B that A is impacting
   *  A's relative normal velocity = vab1 · n
   *
   *  va2 = va1 + j n / ma
   *  vb2 = vb1 − j n / mb
   *  ωa2 = ωa1 + (rap × j n) / Ia
   *
   *
   *   j =  	−(1 + e) vab1 · n
   *   --------------------------------
        1⁄ma + 1⁄mb + (rap × n)2 ⁄ Ia + (rbp × n)2 ⁄ Ib
   */


}

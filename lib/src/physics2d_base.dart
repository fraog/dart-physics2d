import 'dart:async';
import 'dart:collection';
import 'package:physics2d/physics2d.dart';

/// Physics is a singleton and should not be constructed.
class Physics {

  static HashMap<String, RigidBody2D> Cache = new HashMap<String, RigidBody2D>();
  static double TimeScale = 0.3; //TODO: Calculate this (desired 60 fps?)

  static Duration updateInterval = new Duration(milliseconds: 5);
  static Timer timer;

  static double Elasticity = 1.0;

  //DEBUG:
  static HashSet<RigidBody2D> disabled = new HashSet<RigidBody2D>();
  static HashSet<RigidBody2D> resolved;
  static bool PAUSED = false;

/// Starts the physics simulation.
  static void start() {
    if (timer != null) return;
    print("starting physics loop.");
    timer = new Timer.periodic(updateInterval, Physics.update);
    //TODO: startTime
  }

/// Process one step of the simulation.
  static void update(Timer t) {
    if (PAUSED) return;

    Queue<RigidBody2D> moved = new Queue<RigidBody2D>();

    for (RigidBody2D body in Physics.Cache.values) {
      if (body.velocity.x != 0 || body.velocity.y != 0 || body.angularVelocity != 0) {
        body.add(body.velocity*TimeScale);
        body.rotation += body.angularVelocity*TimeScale;
        if (body.rotation > 6.283) body.rotation -= 6.283;
        moved.add(body);
      }
    }

    resolved = new HashSet<RigidBody2D>();
    while (moved.length > 0) {
      try {
        Physics.ResolveCollisions(moved.removeFirst());
      } catch (e, stackTrace) {
        PAUSED = true;
        print(stackTrace.toString());
        break;
      }
    }
  }

/// Determines if a collision is possible between two rigid bodies using their radii.
  static bool CollisionBroadPhase(RigidBody2D body, RigidBody2D other) {
    Vector2D diff = other - body;
    num maxdist = (body.corner+other.corner).magnitudeSqr;
    return (diff.magnitudeSqr <= maxdist);
  }

/// Projects points onto an axis using 2d dot product and returns the min and max dotproduct.
  static void ProjectVertices(Vector2D axis, List<Vector2D> points, List<double> minmax) {
    minmax[0] = axis.dotProduct(points[0]);
    minmax[1] = minmax[0];
    for (int i=1; i<points.length; i++) {
      double p = axis.dotProduct(points[i]);
      if (p < minmax[0]) minmax[0] = p;
      else if (p > minmax[1]) minmax[1] = p;
    }
  }

/// Uses SAT (Separating Axis Theorem) to determine if two objects are colliding.
/// If they are, an impact point and impact normal are generated and returned as a
/// Collision2D object. Otherwise returns null.
///
/// Separating Axis Theorem: Given two convex shapes, if we can find an axis in
/// which the projection of the two shapes does not overlap, then the shapes dont overlap. (all axes of the shapes must overlap for collision).
  static Collision2D CollisionNarrowPhase(RigidBody2D body, RigidBody2D other) {

    //List<Vector2D> axes = body.normalizedAxes;
    //List<Vector2D> axes2 = other.normalizedAxes;

    //TEMP: Relative axes
    List<Vector2D> axes = [ new Vector2D(1.0, 0.0), new Vector2D(0.0, 1.0) ];
    List<Vector2D> axes2 = [ new Vector2D(1.0, 0.0), new Vector2D(0.0, 1.0) ];

    double rdiff = other.rotation-body.rotation;

    //double rdiff = -body.rotation;
    axes2[0].rotateBy(rdiff);
    axes2[1].rotateBy(rdiff);

    /* Get the points of body and other using their axes. */
    //List<Vector2D> points1 = body.getAbsoluteCorners(axes);
    //List<Vector2D> points2 = other.getAbsoluteCorners(axes2);

    //TEMP: Relative points
    List<Vector2D> points1 = body.getRelativeCorners(axes);
    List<Vector2D> points2 = other.getRelativeCorners(axes2);

    Vector2D diff = other - body;
    diff.rotateBy(-body.rotation);
    for (Vector2D p in points2) {
      p.add(diff);
    }


    /*
     * For each axis on body and other, project points from both body and other and check for overlap of
     * their minimum and maximum.
     */
    axes.addAll(axes2);

    //These are the projection min/max of the shape
    List<double> range1 = [ 0.0, 0.0 ];
    List<double> range2 = [ 0.0, 0.0 ];

    //Min overlap and axis on other's proejctions.
    double overlap = -1.0;
    Vector2D minOverlap = null;

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
          throw new Exception("Error: Overlap containment case not handled.");
        }

        //Update OTHER min overlap/axis.
        if (overlap < 0 || o < overlap) {
          overlap = o;
          minOverlap = axis * overlap;
        }
      }
    }

    //If we got here, we have a collision, so we'll need a collision object.
    Collision2D c = new Collision2D(null, null);

    //The impact normal is actually represented by the axis with the min overlap.
    c.impactNormal = minOverlap;


    //Now, we need to determine the impact point and MTV (minimal translation vector).
    List<Edge2D> edges1 = body.getAbsoluteEdges(points1);
    List<Edge2D> edges2 = other.getAbsoluteEdges(points2);

    //Find intersections on a and b.
    //If I just use indices i know which edge by (1, 2, 3, 4) - top, bot, left, right
    //FIXME: bad
    List<int> inters = new List<int>(4);
    List<int> ointers = new List<int>(4);
    for (int i=0; i<4; i++) {
      inters[i] = -1;
      ointers[i] = -1;
    }

    int intersLen = 0;
    //int ointersLen = 0;
    for (int i=0; i < edges1.length; i++) {
      for (int o=0; o < edges2.length; o++) {
        if (edges1[i].intersects(edges2[o])) {
          if (inters[i] == -1) intersLen++;
          inters[i] = o;
          //if (ointers[o] == -1) ointersLen++;
          ointers[o] = i;
        }
      }
    }

    //Correct the overlap vector based on intersections.
    //TODO: We could probably combien cases 1 and 2.
    if (intersLen == 0) {
      print("Error: None of my edges are colliding from the start!");
      throw new Exception("Error: None of my edges are colliding from the start!");
    } else if (intersLen == 1) {
      //print("Error: Only one of my edges is colliding from the start!");
      if ((inters[0] >= 0 && c.impactNormal.y < 0) ||
          (inters[1] >= 0 && c.impactNormal.y > 0))  c.impactNormal.y = -c.impactNormal.y;
      if ((inters[2] >= 0 && c.impactNormal.x < 0) ||
          (inters[3] >= 0 && c.impactNormal.x > 0))  c.impactNormal.x = -c.impactNormal.x;

      //Note: We can't correct this case initially.

    } else if (intersLen == 2) {
      //print("Initially, two of my edges are colliding.");
      if ((inters[0] >= 0 && c.impactNormal.y < 0) ||
          (inters[1] >= 0 && c.impactNormal.y > 0))  c.impactNormal.y = -c.impactNormal.y;
      if ((inters[2] >= 0 && c.impactNormal.x < 0) ||
          (inters[3] >= 0 && c.impactNormal.x > 0))  c.impactNormal.x = -c.impactNormal.x;

      //Exit early if we have non-adjacent inters. (0, 1 or 2, 3)
      if (intersLen != 2 || !( (inters[0] >= 0 && inters[1] >= 0)  || (inters[2] >= 0 && inters[3] >= 0) )) {

        //Correct collision
        c.impactNormal.multiply(0.99); //TODO: do this when creating the overlap.
        body.add(c.impactNormal);
        for (Vector2D p in points1) p.add(c.impactNormal);

        //Update my intersections:
        for (int i=0; i<4; i++) {
          if (inters[i] >= 0) {
            if (!edges1[i].intersects(edges2[inters[i]])) {
              inters[i] = -1;
              intersLen--;
            } else {
              //make sure ointers wasnt cleared.
              ointers[inters[i]] = i;
            }
          }
        }
      }
    } else if (intersLen == 3) {
      print("Error: Three of my edges are colliding from the start!");
    }

    //Generate an impact point:
    if (intersLen == 0) {
      print("Error: None of my edges are colliding after correction.");
      throw new Exception("Error: None of my edges are colliding after correction.");
    } else if (intersLen == 1) {
      //print("The impact point is on the other object's corner.");
      c.impactPoint = new Vector2D(diff.x, diff.y);
      if (ointers[0] >= 0) c.impactPoint.add((edges2[2].direction/2).inverted);
      if (ointers[1] >= 0) c.impactPoint.add(edges2[2].direction/2);
      if (ointers[2] >= 0) c.impactPoint.add((edges2[0].direction/2).inverted);
      if (ointers[3] >= 0) c.impactPoint.add(edges2[0].direction/2);
    } else if (intersLen == 2) {

      Edge2D otherEdge = null;
      double dist = 0.0;

      if (inters[0] >= 0 && inters[1] >= 0) {
        otherEdge = edges2[inters[0]];
        dist = body.hw;
      } else if (inters[2] >= 0 && inters[3] >= 0) {
        otherEdge = edges2[inters[2]];
        dist = body.hh;
      }

      if (otherEdge != null) {
        c.impactPoint = otherEdge.position + (otherEdge.direction/2);
        c.impactPoint.subtract(other);
        c.impactPoint.normalize();
        c.impactNormal = c.impactPoint;
        c.impactPoint = c.impactPoint.inverted;
        c.impactPoint.multiply(dist);
      } else {
        //print("The impact point is on one of my corners.");.
        c.impactPoint = new Vector2D(0.0, 0.0);
        if (inters[0] >= 0) c.impactPoint.add((edges1[2].direction/2).inverted);
        if (inters[1] >= 0) c.impactPoint.add(edges1[2].direction/2);
        if (inters[2] >= 0) c.impactPoint.add((edges1[0].direction/2).inverted);
        if (inters[3] >= 0) c.impactPoint.add(edges1[0].direction/2);
      }
    } else if (intersLen == 3) {
      print("Error: Three of my edges colliding afterwards.");
      throw new Exception("Error: Three of my edges are colliding: "+ointers.toString());
    }

    /* If they were all overlapping, they must be colliding. */
    return c;
  }

/// Detects, resolves and then responds to collisions on a rigidbody.
  static void ResolveCollisions(RigidBody2D body) {

    if (body.bufferCount > 0) {
      body.bufferCount--;
      return;
    }

    /*
     * We will definitely not be going through the physics cache to do this.
     * generate a collision graph before resolution.
     */
    for (RigidBody2D other in Physics.Cache.values) {

       if (body == other) continue;
       if (resolved.contains(other)) continue; //WE have already resolved from his point of view.

       if (CollisionBroadPhase(body, other)) {

         Collision2D c = CollisionNarrowPhase(body, other);
         if (c != null) {

            /**
            * RESPONSE:
            * Formula for velocity of an arbitrary point on a rotating and translating rigid body.
            *
            *  VelpA = VelA + AngleA X Rap
            *  Vap = Va1 + wa1 X Rap
            *  X - cross product.
            *  Rap = point from center of mass A to impact P
            *  AngleA regarded as 3d vector, crossproduct:
            *  ω × r = (0, 0, ω) × (rx, ry, 0) = (−ω ry, ω rx, 0)
            *
            *  Post collision velocities:
            *  Vab1 - the pre collision relative velocity
            *  Vab1 = VelpA - VelpB
            *
            *  dot product:
            *  n is the normal (perpendicular) of the edge on B that A is impacting
            *  A's relative normal velocity = vab1 · n
            *
            *  va2 = va1 + j n / ma
            *  vb2 = vb1 − j n / mb
            *  ωa2 = ωa1 + (rap × j n) / Ia
            *  ωb2 = ωb1 + (rbp × j n) / Ib
            *
            *   j =  	−(1 + e) vab1 · n
            *   --------------------------------
            1⁄ma + 1⁄mb + (rap × n)2 ⁄ Ia + (rbp × n)2 ⁄ Ib

            Impulse = velocity + (impulse vector/mass)
            */

            var n = c.impactNormal;
            n.normalize(); //FIXME: Shouldn't need to do this?

            //Impact point is relative to body.
            var rap = c.impactPoint;
            var rbp = other - body + c.impactPoint;

            var wa1 = body.angularVelocity;
            var wb1 = other.angularVelocity;

            var wa1Xrap = new Vector2D(rap.y * -wa1, rap.x * wa1);
            var vap = body.velocity + wa1Xrap;

            var wb1Xrbp = new Vector2D(rbp.y * -wb1, rbp.x * wb1);
            var vbp = other.velocity - wb1Xrbp;

            //Pre collision relative velocity.
            var vab1 = vap - vbp;

            var rapXn = rap.crossProduct(n);
            var rbpXn = rbp.crossProduct(n);

            var rapXns = rapXn*rapXn;
            var rbpXns = rbpXn*rbpXn;

            var ef = 0-(1+Elasticity);
            var j = (ef * vab1.dotProduct(n))/ (1/body.mass + 1/other.mass + rapXns/body.momentOfInertia + rbpXns/other.momentOfInertia);
            //j should always be positive.

            var jn = n * j;
            var va2 = body.velocity + jn/body.mass;
            var vb2 = other.velocity - jn/other.mass;

            var wa2 = wa1 + (rap.crossProduct(jn)/body.momentOfInertia)/3.0;
            var wb2 = wb1 - (rbp.crossProduct(jn)/other.momentOfInertia)/3.0;

            body.velocity = va2;
            body.angularVelocity = wa2;

            other.velocity = vb2;
            other.angularVelocity = wb2;

            body.bufferCount = 5;
            resolved.add(body);
          }
       }
     }
  }
}

package artisynth.istar.lloyd;

import java.awt.Color;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import artisynth.core.workspace.*;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.femmodels.*;
import artisynth.core.materials.*;
import artisynth.core.probes.*;
import artisynth.core.driver.*;
import artisynth.core.gui.*;

import maspack.util.*;
import maspack.widgets.*;
import maspack.matrix.*;
import maspack.geometry.*;
import maspack.properties.*;
import maspack.collision.*;
import maspack.render.*;
import maspack.render.Renderer.*;
import maspack.render.Renderer;
import maspack.properties.*;

/**
 * A class containing information about the clipping plane for a
 * mandible segment. This includes:
 *
 * <ul>
 *  <li> a PolygonalMesh used to represent the plane
 *  <li> a MeshBody that contains the mesh for rendering purposes
 *  <li> the coordinate transform TPS from the plane to the segment frame
 *  <li> the main intersection polygon between the plane and the segment mesh
 * </ul>
 */
public class CutPlaneMesh extends RenderableComponentBase {

   RigidTransform3d myTPS; // transform from the plane to the segment frame
   PolygonalMesh myMesh;   // mesh representing the plane; used for clipping
   FixedMeshBody myBody;   // mesh body containing the mesh
   Polygon2d mySegPoly; // intersection contour of the plane and segment mesh

   /**
    * Create a new cut plane mesh.
    *
    * @param nrm normal of the cut plane (world coordinates)
    * @param pnt origin of the cut plane (world coordinates)
    * @param width width of the plane in each direction
    * @param noffset offset along the normal from the true plane position.
    * May be needed to ensure the plane intersects properly with the
    * segment mesh, if the mesh is already clipped. Otherwise can
    * be set to zero.
    * @param TSW transform from segment to world coordinates
    */
   public CutPlaneMesh (
      Vector3d nrm, Point3d pnt, 
      double width, double noffset, RigidTransform3d TSW) {
      // create original mesh in the x/y plane
      myMesh = MeshFactory.createRectangle (
         width, width, /*texture=*/false);
      // use TSW to transform nrm and pnt to local coordinates
      Vector3d nrmLoc = new Vector3d();
      nrmLoc.inverseTransform (TSW, nrm);
      Point3d pntLoc = new Point3d();
      pntLoc.inverseTransform (TSW, pnt);
      // add offset along the normal, to ensure we can intersect
      // with the segment mesh:
      pntLoc.scaledAdd (noffset, nrmLoc, pntLoc);
      myTPS = new RigidTransform3d();
      myTPS.p.set (pntLoc);
      myTPS.R.setZDirection (nrmLoc);
      myMesh.transform (myTPS);
      // create a fixed mesh body to store and render the mesh.  Note: we could
      // dispense with the mesh body and just render the mesh directly.
      myBody = new FixedMeshBody (myMesh);
      myBody.setPose (TSW);
      // Set render properties for the mesh body so that it renders as
      // transparent CYAN
      RenderProps.setFaceColor (myBody, Color.CYAN);
      RenderProps.setFaceStyle (myBody, FaceStyle.FRONT_AND_BACK);
      RenderProps.setAlpha (myBody, 0.6);
      setRenderProps (createRenderProps());
   }

   /**
    * Sets the pose of this clip plane. This done when pose of the associated
    * segment is changed.
    */
   public void setPose (RigidTransform3d TSW) {
      myBody.setPose (TSW);
   }

   /**
    * Intersect this plane with the segment mesh and store the resulting
    * contour, if any, in mySegPoly.
    */
   public void intersectWithSegment (RigidBody segBody) {
      mySegPoly = intersectMesh (segBody);
   }

   /**
    * Intersect this clip plane with the surface mesh of a rigid body and
    * return the main intersection contour, if any, as a 2D polygon in
    * plane coordinates.
    */
   public Polygon2d intersectMesh (RigidBody body) {
      SurfaceMeshIntersector smi = new SurfaceMeshIntersector();
      // use a SurfaceMeshIntersector to find all intersection
      // contours between the mesh and the plane. These will be
      // computed in world coordinates.
      ArrayList<IntersectionContour> contours =
         smi.findContours (myMesh, body.getSurfaceMesh());
      Polygon2d poly = null;
      if (contours.size() > 0) {
         // Contours found. Determine the transform TPW which transforms points
         // from plane to world coordinates.
         RigidTransform3d TPW = new RigidTransform3d();
         TPW.mul (myBody.getPose(), myTPS);
         Point3d pplane = new Point3d(); // 3d point in plane coordinates
         // contour points in plane coordinates, stored as 2d points:
         ArrayList<Point2d> ipnts = new ArrayList<>();
         double maxArea = 0;
         // Go through all contours, convert them to 2d polygons in plane
         // coordinates, and compute their areas. The polygon with the maximum
         // positive area will be selected as the main contour.
         for (IntersectionContour c : contours) {
            if (c.isClosed()) {
               Point2d lastp2d = null;
               double area = 0;
               ipnts.clear();
               for (Point3d p : c) {
                  // transform from world to plane:
                  pplane.inverseTransform (TPW, p);
                  Point2d p2d = new Point2d (pplane.x, pplane.y);
                  // area is the sum of each lastp2d X p2d
                  if (lastp2d != null) {
                     area += lastp2d.x*p2d.y - lastp2d.y*p2d.x;
                  }
                  ipnts.add (p2d);
                  lastp2d = p2d;
               }
               if (area > 0 && area > maxArea) {
                  maxArea = area;
                  poly = new Polygon2d (ipnts.toArray(new Point2d[0]));
               }
            }
         }
      }
      return poly;
   }
   
   /* --- begin Renderable implementation --- */

   /**
    * Render this clip plane. Right now, this involves only rendering the
    * intersection contour with the segment mesh using line render properties.
    * The mesh itself is rendered separately by its MeshBody, but otherwise 
    * it could also be rendered here.
    */
   public void render (Renderer r, int flags) {
      Polygon2d segPoly = mySegPoly;
      RenderProps props = myRenderProps;
      if (segPoly != null) {
         // Determine the transform TPW which transforms points from plane to
         // world coordinates.
         RigidTransform3d TPW = new RigidTransform3d();
         TPW.mul (myBody.getPose(), myTPS);
         r.setLineWidth (props.getLineWidth());
         r.setColor (props.getLineColor());
         r.beginDraw(DrawMode.LINE_LOOP);
         // Draw the contour using the corresponding world coordinates of each
         // point on the polygon:
         for (Point2d p2d : segPoly.getPoints()) {
            Point3d p = new Point3d();
            p.transform (TPW, new Point3d (p2d.x, p2d.y, 0));
            r.addVertex(p);
         }
         r.endDraw();
      }
   }
}

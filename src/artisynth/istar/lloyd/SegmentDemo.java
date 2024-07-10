package artisynth.istar.lloyd;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.IOException;
import java.util.List;

import javax.swing.JMenuItem;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.RootModel;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.util.InternalErrorException;
import maspack.util.PathFinder;
import maspack.widgets.GuiUtils;

/**
 * A simple demo showing how to reposition a mandible reconstruction segment
 * onto a fibula donor bone.
 */
public class SegmentDemo extends RootModel {

   private static double PLANE_WIDTH = 60.0;
   private static double NRM_OFFSET = -0.2; // distance 

   RigidBody mySegment; // contains mandible segment mesh
   RigidBody myDonor;   // contains donor mesh

   CutPlaneMesh myCutPlaneMesh0; // first segment cut plane 
   CutPlaneMesh myCutPlaneMesh1; // second segment cut plane

   double myTheta = 0; // rotation of the relocated segment the z axis
   boolean mySegmentOnDonor = false; // segment has been relocated to the donor
   // pose of the segment and cut planes when relocated to the donor:
   RigidTransform3d mySegmentDonorPose = new RigidTransform3d();

   // --- begin custom properties for this model --- 

   public static PropertyList myProps =
      new PropertyList (SegmentDemo.class, RootModel.class);

   static {
      myProps.add ("theta", "rotation about donnor", 0, "[-180,180]");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   /**
    * Returns the current rotation of the relocated segment about the donor z
    * axis.
    */
   public double getTheta() {
      return myTheta;
   }

   /**
    * Sets the rotation of the relocated segment about the donor z axis.
    */
   public void setTheta (double theta) {
      myTheta = theta;
      // Set pose of the segment when it is on the donor to
      //
      // TSW = TDW * TR
      //
      // where TR rotates about the donor z axis by theta      
      mySegmentDonorPose.set (myDonor.getPose());
      RigidTransform3d TR = 
         new RigidTransform3d (0, 0, 0, Math.toRadians(myTheta), 0, 0);
      mySegmentDonorPose.mul (TR);
      if (mySegmentOnDonor) {
         setSegmentPose(mySegmentDonorPose);
      }
   }

   // --- end custom properties

   /**
    * Creates a cut plane mesh and adds it to a mechmodel.
    *
    * @param mech MechModel to add the mesh to
    * @param segbody contains the mandible segment mesh
    * @param nrm plane normal (in world coordinates)
    * @param pnt plane origin (in world coordinate)s
    */
   CutPlaneMesh createCutPlaneMesh (
      MechModel mech, RigidBody segbody, Vector3d nrm, Point3d pnt) {

      CutPlaneMesh cpmesh = new CutPlaneMesh (
         nrm, pnt, PLANE_WIDTH, NRM_OFFSET, segbody.getPose());
      cpmesh.intersectWithSegment (segbody);
      // add the cut plane mesh itself to mech as a mesh body
      mech.addMeshBody (cpmesh.myBody);
      // add the cut plane containing object to mech as a renderable
      mech.addRenderable (cpmesh);
      // set some render properties
      RenderProps.setLineWidth (cpmesh, 2);
      RenderProps.setLineColor (cpmesh, Color.RED);
      return cpmesh;
   }

   /**
    * Estimate the surface normal at the mesh location nearest to a prescribed
    * postion.
    *
    * @param pos prescribed position, in world coordinates
    * @param mesh mesh for which surface is to be estimated
    * @return estimated normal, in world coordinates
    */
   Vector3d estimateSurfaceNormal (Point3d pos, PolygonalMesh mesh) {
      // find the face and uv coordinates of the mesh point nearest to pos:
      Vector2d uv = new Vector2d();
      Face face = mesh.nearestFaceToPoint (null, uv, pos);
      // compute the normal as the sum of the face vertex normals, weighted by
      // the barycentric coordinates
      Vector3d nrm = new Vector3d();
      Vector3d vnrm = new Vector3d();
      Vertex3d[] vtxs = face.getVertices();
      if (vtxs.length != 3) {
         throw new InternalErrorException (
            "Body mesh is not triangular");
      }
      vtxs[0].computeWorldNormal (vnrm);
      nrm.scale (1-uv.x-uv.y, vnrm);
      vtxs[1].computeWorldNormal (vnrm);      
      nrm.scaledAdd (uv.x, vnrm);
      vtxs[2].computeWorldNormal (vnrm);      
      nrm.scaledAdd (uv.y, vnrm);
      nrm.normalize();
      return nrm;
   }

   public void build (String[] args) throws IOException {
      MechModel mech = new MechModel ("mech");
      addModel (mech);

      // mesh data is stored in 'geometry' folder relative to this class's
      // source code
      String dataDir = PathFinder.getSourceRelativePath (this, "geometry/");

      // create mandible and donor segment meshes:
      mySegment = 
         RigidBody.createFromMesh (
            "segment", new PolygonalMesh (
               dataDir+"mandibleSegment.obj"), 1000.0, 1.0);
      mech.addRigidBody (mySegment);
      mySegment.setDynamic (false);

      myDonor = 
         RigidBody.createFromMesh (
            "donor", new PolygonalMesh (dataDir+"clippedDonor.obj"), 1000.0, 1.0);
      mech.addRigidBody (myDonor);
      myDonor.setDynamic (false);

      // segPos0 and segPos1 define the RDP line end points on the mandible
      // segment, in world coordinates. We use these to define a new coordinate
      // system for the segment, for which the origin is given by the midpoint
      // between the lines, the z direction is parallel to segPos1 - segPos1,
      // and the x direction is set to be as close as possible to the average
      // of the estimated surface normals at segPos0 and segPos1.

      // Note: in this demo, segPos0 and segPos1 are hardwired, but in general
      // they will be computed from the RDP line.
      Point3d segPos0 = new Point3d (-6.0614191, -33.58753, -17.241963);
      Point3d segPos1 = new Point3d (-30.617977, -3.2813151, -10.863514);
      Point3d segCenter = new Point3d();
      segCenter.add (segPos1, segPos0);
      segCenter.scale (0.5);
      Vector3d zvec = new Vector3d();
      zvec.sub (segPos1, segPos0);
      zvec.normalize();     
      // compute average of the estimate surface normals at segPos0 and segPos1
      Vector3d xvec = new Vector3d();      
      xvec.set (estimateSurfaceNormal (segPos0, mySegment.getSurfaceMesh()));
      xvec.add (estimateSurfaceNormal (segPos1, mySegment.getSurfaceMesh()));
      xvec.normalize();
      // Compute new segment coordinate frame TSW
      RigidTransform3d TSW = new RigidTransform3d();
      TSW.R.setZXDirections (zvec, xvec);
      TSW.p.set (segCenter);
      // change the segment coordinate frame to TSW
      mySegment.getSurfaceMesh().inverseTransform (TSW);
      mySegment.setPose (TSW);

      // create clip plane meshes for the clip planes at each end of the
      // mandible segment. For this example, the normals and origins of these
      // meshes are hardwired.
      myCutPlaneMesh0 = createCutPlaneMesh (
         mech, mySegment,
         new Vector3d (0.93457, -0.355699, 0.007627),
         new Point3d (-4.458348, -28.87495, -15.653029));

      myCutPlaneMesh1 = createCutPlaneMesh (
         mech, mySegment,
         new Vector3d (-0.422427, 0.861509, 0.281706),
         new Point3d (-24.098866, 0.447264, -10.795219));

      // donPos0 and donPos1 define points corresponding to segPos0 and segPos1
      // on the donor mesh. We use these to compute a new donor coordinate
      // frame TDW in the same way we computed TSW above.
      Point3d donPos0 = new Point3d (7.7414564, -0.35307344, 56.376092);
      Point3d donPos1 = new Point3d (3.8945486, -2.5990411, 16.102412);
      Point3d donCenter = new Point3d();
      donCenter.add (donPos1, donPos0);
      donCenter.scale (0.5);
      zvec.sub (donPos1, donPos0);
      zvec.normalize();     
      xvec.set (estimateSurfaceNormal (donPos0, myDonor.getSurfaceMesh()));
      xvec.add (estimateSurfaceNormal (donPos1, myDonor.getSurfaceMesh()));
      xvec.normalize();
      RigidTransform3d TDW = new RigidTransform3d();
      TDW.R.setZXDirections (zvec, xvec);
      TDW.p.set (donCenter);
      // change the donor coordinate frame to TDW
      myDonor.getSurfaceMesh().inverseTransform (TDW);
      myDonor.setPose (TDW);

      // add some markers and axial springs to the model to show the locations
      // of segPos0, segPos1, donPos0, donPos1, and the line segments between
      // them:
      FrameMarker segMkr0 = mech.addFrameMarkerWorld (mySegment, segPos0);
      FrameMarker segMkr1 = mech.addFrameMarkerWorld (mySegment, segPos1);
      AxialSpring segLine = new AxialSpring (0, 0, 0);
      mech.attachAxialSpring (segMkr0, segMkr1, segLine);

      FrameMarker donMkr0 = mech.addFrameMarkerWorld (myDonor, donPos0);
      FrameMarker donMkr1 = mech.addFrameMarkerWorld (myDonor, donPos1);
      AxialSpring donLine = new AxialSpring (0, 0, 0);
      mech.attachAxialSpring (donMkr0, donMkr1, donLine);

      setTheta(0);
      addControlPanel (createControlPanel());

      // set some render properties for points and lines
      RenderProps.setSphericalPoints (mech, 2.0, Color.WHITE);
      RenderProps.setCylindricalLines (segLine, 1.0, Color.RED);
      RenderProps.setCylindricalLines (donLine, 1.0, Color.BLUE);
   }

   /**
    * Create control panel with a widget that allows 'theta' to be adjusted.
    */
   ControlPanel createControlPanel() {
      ControlPanel panel = new ControlPanel();
      panel.addWidget (this, "theta");
      return panel;
   }

   /**
    * Override of {@code getMenuItems} to make custom menu items appear in the
    * Application menu.
    */
   public boolean getMenuItems(List<Object> items) {
      JMenuItem item =
         GuiUtils.createMenuItem (this, "moveToDonor", "moveToDonor");
      items.add (item);
      return true;
   }

   /**
    * Override of {@code actionPerformed} to implement custom menu items.
    */
   public void actionPerformed(ActionEvent event) {
      if (event.getActionCommand().equals ("moveToDonor")) {
         setSegmentPose(mySegmentDonorPose);
         Main.getMain().rerender();
         mySegmentOnDonor = true;
      }
   } 

   /**
    * Set the pose of the segment and its two cut planes.
    */
   void setSegmentPose (RigidTransform3d TSW) {
      mySegment.setPose (TSW);
      myCutPlaneMesh0.setPose (TSW);
      myCutPlaneMesh1.setPose (TSW);
   }
}

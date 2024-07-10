/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.istar.Prisman;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.AbstractAction;
import javax.swing.JButton;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.renderables.EditablePolygonalMeshComp;
import artisynth.core.workspace.RootModel;
import artisynth.demos.test.IntersectionTester;
import artisynth.istar.Prisman.undo.PrepCommand;
import maspack.geometry.MeshFactory;
import maspack.geometry.MeshICP;
import maspack.collision.SurfaceMeshIntersector;
import maspack.collision.SurfaceMeshIntersectorTest;
import maspack.collision.SurfaceMeshIntersector.CSG;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.MeshICP.AlignmentType;
import maspack.geometry.OBB;
import maspack.geometry.io.StlReader;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;
import maspack.spatialmotion.SpatialInertia;
import maspack.util.InternalErrorException;

public class SetUpManualRecon extends RootModel{

   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   RigidBody resectionMeshRigidBody = null;   
   SurfaceMeshIntersector intersector =
      new maspack.collision.SurfaceMeshIntersector ();
   final double PLANE_LENGTH = 90;
   final double BOX_WIDTH = 180;
   double BOX_HEIGHT = 180;
   double BOX_LENGTH = 180;
   RigidBody mandibleCuttingPlane1;
   RigidBody mandibleCuttingPlane2;
   FixedMeshBody cutDonorPlane1Body;
   PolygonalMesh cutDonorPlane1;
   Point3d centerCutDonorPlane1 = new Point3d ();
   Vector3d normalCutDonorPlane1 = new Vector3d ();
   RigidBody fibulaOrig;
   RigidBody donorClipped;
   SurfaceMeshIntersector.CSG csg =
      SurfaceMeshIntersector.CSG.DIFFERENCE10;
   SurfaceMeshIntersector.CSG csg2 =
      SurfaceMeshIntersector.CSG.DIFFERENCE01;
   ControlPanel myControlPanel2;
   IntersectionTester myTester, myTester2;
   IntersectionTester activeTester;
   MechModel myMechModel;
   PolygonalMesh myClippedDonor;
   FixedMeshBody myClippedDonorBody;
   List<PolygonalMesh> myfibulaReconSegmentMesh = new ArrayList<PolygonalMesh>();
   List<FixedMeshBody> myfibulaReconSegmentMeshBody = new ArrayList<FixedMeshBody>();
   PolygonalMesh myplane2Mesh;

   public void setUpManualReconstruction (MechModel mechModel, PolygonalMesh mandibleMesh, PolygonalMesh resectionMesh, 
      PolygonalMesh clippedDonorMesh, RigidBody clippedDonorMeshBody, PolygonalMesh plane1Mesh, PolygonalMesh ClippedDonor,
      List<PolygonalMesh> fibulaReconSegmentMesh, List<FixedMeshBody> fibulaReconSegmentMeshBody, PolygonalMesh plane2Mesh){

      // TODO: create constructor for these variables
      myMechModel = mechModel;
      myClippedDonor = ClippedDonor;
      myfibulaReconSegmentMesh = fibulaReconSegmentMesh;
      myfibulaReconSegmentMeshBody = fibulaReconSegmentMeshBody;
      myplane2Mesh = plane2Mesh;
      
      //difference10 code for mandible cutting planes and fibula
      
      EditablePolygonalMeshComp myEditMesh0;
      EditablePolygonalMeshComp myEditMesh1;

    
      // create planes to cut donor
      //mechModel.removeMeshBody (cutDonorPlane1Body);

         // MeshPlane is created at (0,0,0)
      cutDonorPlane1 =  MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
      maspack.matrix.RigidTransform3d transformCutDonorPlane =
            new maspack.matrix.RigidTransform3d ();
      Vector3d mandibleCentroid = new Vector3d ();
      mandibleMesh.computeCentroid (mandibleCentroid);
      transformCutDonorPlane.setTranslation (mandibleCentroid);
      cutDonorPlane1.transform (transformCutDonorPlane);

      cutDonorPlane1Body = new FixedMeshBody ("cutDonorPlane1Body", cutDonorPlane1);
      cutDonorPlane1.translateToCentroid ();
      
      // save normal and center of cutDonorPlane1
      RigidTransform3d pose = cutDonorPlane1Body.getPose ();
      pose.R.getColumn (2, normalCutDonorPlane1);
      centerCutDonorPlane1.x = pose.p.x;
      centerCutDonorPlane1.y = pose.p.y;
      centerCutDonorPlane1.z = pose.p.z;
      
      mechModel.addMeshBody (cutDonorPlane1Body);
      
      
      // set pose for donor mesh
      Vector3d maxillaResectCentroid = new Vector3d ();
      resectionMesh.computeCentroid (maxillaResectCentroid);

      RigidTransform3d transformDonor = new maspack.matrix.RigidTransform3d ();
      transformDonor.setTranslation (maxillaResectCentroid);
      clippedDonorMeshBody.setPose (transformDonor);
      // DonorMesh.inverseTransform(transform);

      mechModel.removeRigidBody (clippedDonorMeshBody);

      mandibleCuttingPlane1 = addBody (mechModel, plane1Mesh, plane1Mesh.getMeshToWorld ());
      fibulaOrig =
         addBody (
            mechModel, clippedDonorMesh, clippedDonorMesh.getMeshToWorld ());
      
      System.out.println (plane1Mesh.getMeshToWorld ());

      // myEditMesh0 = addEditMesh (mechModel, DonorMesh);
      // myEditMesh1 = addEditMesh (mechModel, resectionMesh);

      myTester =
         new IntersectionTester (mandibleCuttingPlane1.getMesh (), fibulaOrig.getMesh (), 0);


      if (csg != null) {
         myTester.setCSGOperation (csg);
      }

      RenderProps.setSphericalPoints (myTester, 0.02, Color.CYAN);
      RenderProps.setFaceColor (myTester, new Color (0.8f, 0.8f, 1f));
      addController (myTester);
      
      activeTester = myTester;


      JButton AJLClipDonor = new JButton ("AJL Clip Donor");
      AJLClipDonor.addActionListener (new AJLDonorClipButtonClicked ());
      JButton CutDonor = new JButton ("Cut Donor");
      CutDonor.addActionListener (new CutDonorButtonClicked ());
      JButton SetUpClippingLastPlane = new JButton ("Set Up Clipping Last Plane");
      SetUpClippingLastPlane.addActionListener (new SetUpLastAJLClipButtonClicked ());

      myControlPanel2 = new ControlPanel ("options", "");
      myControlPanel2.addWidget (myTester, "renderCSGMesh");
      myControlPanel2.addWidget (AJLClipDonor);
      myControlPanel2.addWidget (CutDonor);
      myControlPanel2.addWidget (SetUpClippingLastPlane);

      RenderProps.setLineColor (myTester, Color.BLUE);

      addControlPanel (myControlPanel2);
      
   }
   
   public class SetUpLastAJLClipButtonClicked extends AbstractAction {
      public SetUpLastAJLClipButtonClicked () {
         putValue (NAME, "Just Click It");
      }
      
      @Override
      public void actionPerformed (ActionEvent evt) {
         fibulaOrig =
            addBody (
               myMechModel, myClippedDonor, myClippedDonor.getMeshToWorld ());
         mandibleCuttingPlane2 = addBody (myMechModel, myplane2Mesh, myplane2Mesh.getMeshToWorld ());
         
         myTester2 =
            new IntersectionTester (mandibleCuttingPlane2.getMesh (), fibulaOrig.getMesh (), 0);

         if (csg != null) {
            myTester2.setCSGOperation (csg);
         }
         
         RenderProps.setSphericalPoints (myTester2, 0.02, Color.CYAN);
         RenderProps.setFaceColor (myTester2, new Color (0.8f, 0.8f, 1f));
         addController (myTester2);

         myControlPanel2.addWidget (myTester2, "renderCSGMesh");

         RenderProps.setLineColor (myTester2, Color.BLUE);
         
         activeTester = myTester2;
      }
   }
   public class CutDonorButtonClicked extends AbstractAction {
      EditablePolygonalMeshComp myEditCSGMesh;
      FixedMeshBody donorResectionMeshBody, donorNonResectionMeshBody;
      

      public CutDonorButtonClicked () {
         putValue (NAME, "Just Click It");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         myMechModel.removeMeshBody (donorResectionMeshBody);
         myMechModel.removeMeshBody (donorNonResectionMeshBody);
         
         PolygonalMesh DonorCutBox1 = new PolygonalMesh(MeshFactory.createBox (BOX_WIDTH, BOX_LENGTH, BOX_HEIGHT));
         FixedMeshBody DonorCutBox1Body = new FixedMeshBody ("DonorCutBox1Body", DonorCutBox1);
         
         RigidTransform3d p = cutDonorPlane1Body.getPose ().copy ();
//         centerPlane1 = new Point3d (p.p); 
         p.R.getColumn (2, normalCutDonorPlane1);
         Vector3d t = new Vector3d (normalCutDonorPlane1);
         t.normalize ();
         t.scale (BOX_WIDTH / 2);
         p.addTranslation (t);
         DonorCutBox1Body.setPose (p);
         
         SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
         PolygonalMesh donorResectionMesh = intersector.findIntersection(myClippedDonor, DonorCutBox1);
         PolygonalMesh donorNonResectionMesh = intersector.findDifference01 (myClippedDonor, DonorCutBox1);

         donorNonResectionMeshBody =
            new FixedMeshBody ("Donor Unresect", donorNonResectionMesh);
         donorResectionMeshBody = new FixedMeshBody ("Donor Resect", donorResectionMesh);

         myfibulaReconSegmentMesh.add (donorNonResectionMesh.clone ());
         myfibulaReconSegmentMeshBody.add (new FixedMeshBody ("DonorSegment" + myfibulaReconSegmentMeshBody.size (), donorNonResectionMesh));
         myMechModel.addMeshBody (myfibulaReconSegmentMeshBody.get (myfibulaReconSegmentMeshBody.size () - 1));
//         mechModel.addMeshBody (donorResectionMeshBody);
//         mechModel.addMeshBody (donorNonResectionMeshBody);
         
         RigidBody donorCuttingPlane = addBody (myMechModel, cutDonorPlane1, cutDonorPlane1.getMeshToWorld ());
         donorClipped = addBody (myMechModel, donorResectionMesh, donorResectionMesh.getMeshToWorld ());
         
         myTester =
            new IntersectionTester (donorCuttingPlane.getMesh (), donorClipped.getMesh (), 0);
         
         activeTester = myTester;
         
         myTester.setCSGOperation (csg);
         RenderProps.setSphericalPoints (myTester, 0.02, Color.CYAN);
         RenderProps.setFaceColor (myTester, new Color (0.8f, 0.8f, 1f));
         addController (myTester);
         myControlPanel2.addWidget (myTester, "renderCSGMesh");
         RenderProps.setLineColor (myTester, Color.BLUE);
         
         myMechModel.removeMeshBody (myClippedDonorBody);
      }
   }
   
   public class AJLDonorClipButtonClicked extends AbstractAction {
      EditablePolygonalMeshComp myEditCSGMesh;

      public AJLDonorClipButtonClicked () {
         putValue (NAME, "Just Click It");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         myMechModel.removeMeshBody (myClippedDonorBody);
         
         myClippedDonor = activeTester.getCSGMesh ();
         myClippedDonorBody = new FixedMeshBody ("ClippedDonorBody", myClippedDonor);

         myMechModel.addMeshBody (myClippedDonorBody);
         myMechModel.removeRigidBody (fibulaOrig);
         myMechModel.removeRigidBody (donorClipped);
      }
   }

   void removeEditMesh (MechModel mech, EditablePolygonalMeshComp editMesh) {
      mech.removeRenderable (editMesh);
   }

   RigidBody addBody (
      MechModel mech, PolygonalMesh mesh, RigidTransform3d pose) {
      RigidTransform3d TMW =
         (pose != null ? new RigidTransform3d (pose) : null);
      RigidBody body = RigidBody.createFromMesh (null, mesh, 1000, 1.0);
      body.setDynamic (false);

      if (TMW != null) {
         body.setPose (TMW);
         // XXX hack since body.setPose() may alter pose slightly from TMW
         mesh.setMeshToWorld (TMW);
      }
      RenderProps.setDrawEdges (body, true);
      RenderProps.setEdgeColor (body, Color.WHITE);
      RenderProps.setFaceStyle (body, FaceStyle.NONE);
      mech.addRigidBody (body);
      return body;
   }

   EditablePolygonalMeshComp addEditMesh (
      MechModel mech, PolygonalMesh mesh) {
      EditablePolygonalMeshComp editMesh =
         new EditablePolygonalMeshComp (mesh);

      RenderProps.setVisible (editMesh, false);
      RenderProps.setShading (editMesh, Shading.FLAT);

      mech.addRenderable (editMesh);
      return editMesh;
   }
}

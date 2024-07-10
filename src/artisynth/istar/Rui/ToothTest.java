package artisynth.istar.Rui;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.*;
import java.util.*;
import javax.swing.JMenuItem;
import artisynth.core.workspace.*;
import artisynth.core.mechmodels.*;
import artisynth.core.gui.*;
import artisynth.core.gui.editorManager.EditorUtils;
import maspack.util.*;
import maspack.matrix.*;
import maspack.geometry.*;
import maspack.widgets.GuiUtils;
import maspack.render.*;
import maspack.render.Renderer.*;

public class ToothTest extends RootModel {

   Intersector myIntersector;
   FixedMeshBody myMandible;
   FixedMeshBody myClipmesh;

   double EPS = 1e-14;

   private static Color BONE = new Color (1f, 1f, 0.8f);

   PolygonalMesh createClipMesh () {
      PolygonalMesh mesh = MeshFactory.createPlane (100.0, 70.0, 20, 14);
      // bend the leading y edge up
      for (Vertex3d v : mesh.getVertices()) {
         Point3d p = v.getPosition();
         if (p.y < -25+EPS) {
            p.z = -p.y - 25.0;
            p.y = -25.0;
         }
      }
      mesh.notifyVertexPositionsModified();
      mesh.flip();
      return mesh;
   }


   public void build (String[] args) throws IOException {
      MechModel mech = new MechModel ("mech");
      addModel (mech);

      String mandibleDir = PathFinder.getSourceRelativePath (this, "mandible/proposedT48/");
      String clipmeshDir = PathFinder.getSourceRelativePath (this, "mandible/savedCMesh/");

      PolygonalMesh mandible = new PolygonalMesh (mandibleDir+"004-Mandible.stl");
      myMandible = new FixedMeshBody (
         "mandible", mandible);
      CSGSegManual.transformMeshOrigin(mandible);
//      myMandible.setPose (
//         new RigidTransform3d (0, 0, 0, Math.PI, 0, 0));
//      myClipmesh = new FixedMeshBody ("clipmesh", createClipMesh());
//      myClipmesh.setPose
//         (new RigidTransform3d (0, -22.0, 0, Math.toRadians(175), 0, 0));
      
      myClipmesh = new FixedMeshBody (
         "clipmesh", new PolygonalMesh (clipmeshDir+"004-Clipmesh.stl"));
//      myClipmesh.setPose (
//         new RigidTransform3d (0, 0, 0, Math.PI, 0, 0));

      mech.addMeshBody (myMandible);
      mech.addMeshBody (myClipmesh);

      myIntersector = new Intersector (
         (PolygonalMesh)myMandible.getMesh(),
         (PolygonalMesh)myClipmesh.getMesh(), 0, Color.red);

      mech.addRenderable (myIntersector);
      addControlPanel();
      
      // set render properties
      RenderProps.setFaceColor (myMandible, BONE);
      RenderProps.setFaceStyle (myClipmesh, FaceStyle.NONE);
      RenderProps.setDrawEdges (myClipmesh, true);
      RenderProps.setFaceColor (myIntersector, new Color(0.8f, 0.8f, 1.0f));
      RenderProps.setLineColor (myIntersector, Color.RED);
      RenderProps.setLineWidth (myIntersector, 2);
   }

   private void addControlPanel() {
      ControlPanel panel = new ControlPanel ("options", "");
      if (myMandible != null) {
         panel.addWidget (
            "mandibleVisible", myMandible, "renderProps.visible");
      }
      if (myClipmesh != null) {
         panel.addWidget (
            "clipmeshVisible", myClipmesh, "renderProps.visible");
      }
      if (myIntersector != null) {
         panel.addWidget (myIntersector, "renderDiffMesh");
         //panel.addWidget (myIntersector, "applyDiff");
         panel.addWidget (myIntersector, "contoursOnly");
      }
      addControlPanel (panel);
   }

   public boolean getMenuItems (List<Object> items) {
      JMenuItem item = GuiUtils.createMenuItem (
         this, "Save difference mesh", "");
      if (myIntersector.getDiffMesh() == null) {
         item.setEnabled (false);
      }
      items.add (item);
      return true;
   }

   public void actionPerformed (ActionEvent event) {
      String cmd = event.getActionCommand();
      if (cmd.equals("Save difference mesh")) {
         if (myIntersector.getDiffMesh() != null) {
            EditorUtils.saveMesh (myIntersector.getDiffMesh(), null);
         }
      }
   }   

   public void prerender (RenderList list) {
      super.prerender (list);
   }
}

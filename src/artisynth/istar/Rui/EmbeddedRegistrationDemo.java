package artisynth.istar.Rui;
//Rui: from artisynth_models_registration
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemGeometryTransformer;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.core.femmodels.EmbeddedFem;
import artisynth.models.registration.DynamicRegistrationController;
import artisynth.models.registration.correspondences.ICPMeshCorrespondence;
import artisynth.models.registration.weights.GaussianWeightFunction;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.MeshUtilities;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.widgets.GuiUtils;

public class EmbeddedRegistrationDemo extends RootModel {

   FixedMeshBody mySourceNoTeeth;
   FemGeometryTransformer myFemTransformer;

   // Rui: @ is Java Annotations, means that the method is overriding the parent class. 
   // Rui: The super keyword in java is a reference variable that is used to refer parent class objects.
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      // turn off gravity for registration
      mech.setGravity(0, 0, 0);
           
      String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");
      String targetMeshName = "Mandible006Aligned.stl";   //  target
      String sourceMeshName = "mandible_3kv.stl";   // template with teeth  
      String sourceNoTeethMeshName = "16clipe.stl"; // template without teeth
      Point3d sourceTransform = new Point3d();
      
      // load source mesh (template with teeth)
      System.out.println("Loading mesh " + sourceMeshName + "...");
      PolygonalMesh sourcePolyMesh = GenericModel.loadGeometry(meshDirectory, sourceMeshName);
      //fixMesh(sourcePolyMesh);
      sourceTransform = transformMeshOrigin(sourcePolyMesh);
      
      // load target mesh
      System.out.println("Loading mesh " + targetMeshName + "...");               
      PolygonalMesh target = GenericModel.loadGeometry(meshDirectory, targetMeshName);
      //fixMesh(target);
      transformMeshOrigin(target);
      FixedMeshBody fixedTarget = new FixedMeshBody("target", target);
      mech.addMeshBody(fixedTarget);
      
      // scale source mesh / template
      double s = target.computeVolume() / sourcePolyMesh.computeVolume();
      s = Math.cbrt(s);
      sourcePolyMesh.scale(s);
          
      // create a FEM for source mesh (template with teeth)
      FemModel3d fem = createVoxelizedFem(null, sourcePolyMesh, 15);
      fem.setName("fem");
      fem.setDensity(0.000001); //1000      
      FemMeshComp source = fem.addMesh("source", sourcePolyMesh);  // "embed" the mesh in the FEM
      fem.setMaterial(new LinearMaterial(100, 0.3));     // 15000 // make it more flexible for registration
      myFemTransformer = new FemGeometryTransformer(fem) ; // transformer to do the transforming
      mech.addModel(fem);
            
      // load template without teeth
      System.out.println("Loading mesh " + sourceNoTeethMeshName + "...");
      PolygonalMesh sourceNoTeethPolyMesh = GenericModel.loadGeometry(meshDirectory, sourceNoTeethMeshName);
      sourceNoTeethPolyMesh.translate(sourceTransform);
      sourceNoTeethPolyMesh.scale(s);
      mySourceNoTeeth =
         new FixedMeshBody("sourceNoTeethPolyMesh", sourceNoTeethPolyMesh);
      mech.addMeshBody(mySourceNoTeeth);
     
      //TransformGeometryContext.transform (fixedSourceNoTeeth, femtrans, /*flags=*/0);
  
      
      ICPMeshCorrespondence icp = new ICPMeshCorrespondence();
      icp.setWeightFunction (new GaussianWeightFunction (1, 0.01, false));
      DynamicRegistrationController reg = new DynamicRegistrationController(mech);
      reg.setName ("registration");
      reg.addRegistrationTarget (source, target, 1.0, icp);
      reg.setForceScaling (1e6);
      addController(reg);
         
       /*
      // Use Pressure based registration works
      GravityPressureFunction pressureFunction = new GravityPressureFunction();
      pressureFunction.setTargetRadius (10);
      pressureFunction.setMaxPressure(50);
      
      PressureMeshRegistrationForce pressure = new PressureMeshRegistrationForce();
      pressure.setForceScaling (1e5);      
      pressure.setPressureFunction (pressureFunction);
      
      MeshRegistrationController controller = new MeshRegistrationController(
        source, target, pressure);
      controller.setName("registration");
      addController(controller);
            
        */               
   
      /*
      DynamicRegistrationController controller = new DynamicRegistrationController (mech);
      
      GMMMeshCorrespondence gmm = new GMMMeshCorrespondence ();
      gmm.setNoiseFraction (0.01);
      gmm.setNearestK (10);  // consider 10 nearest points
      
      controller.addRegistrationTarget (source, target, 1.0, gmm);
      controller.setForceScaling (1000);       
      controller.setName("registration");
      addController(controller);
      */
      
      RenderProps.setAlpha(fixedTarget, 0.9);
      RenderProps.setFaceColor(fixedTarget, Color.CYAN);
      RenderProps.setAlpha(source, 1);
      RenderProps.setFaceColor(source, Color.RED);
      RenderProps.setAlpha(mySourceNoTeeth, 0.9);
      RenderProps.setFaceColor(mySourceNoTeeth, Color.YELLOW);
      
      rerender();
      
   }
   

   
   // Rui: move the center of mesh to origin, align
   public static Point3d transformMeshOrigin(PolygonalMesh mesh)
   {
           ArrayList<Vertex3d> verts = mesh.getVertices();
           Point3d com = new Point3d();
           
           //Advanced For Loop
           for (Vertex3d vertex3d : verts) {
                com.add(vertex3d.getPosition());
           }
           com.scale(1.0/verts.size());
           com.negate();
           mesh.translate(com);
           
           return com;
   }
  
   private void fixMesh(PolygonalMesh mesh) {
      
      // merge vertices
       mesh.mergeCloseVertices (1e-10);

      // remove detached pieces
      mesh.removeDisconnectedFaces ();
      mesh.removeDisconnectedVertices ();

      // reduce size
      MeshUtilities.quadricEdgeCollapse (mesh, mesh.numVertices () - 8000);
      
   }
   
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      
      ControlPanel panel = new ControlPanel("controls");
      // "get" the controller we added in build and add its properties to the control panel
      DynamicRegistrationController controller = (DynamicRegistrationController)getControllers().get("registration");
      DynamicRegistrationController.addControls(panel, controller);
      
      MechModel mech = (MechModel)(models().get("mech"));
      FemModel3d fem = (FemModel3d)(mech.models().get("fem"));     
      panel.addWidget("fem material", fem, "material");
      
      //FemModel3d femNoTeeth = (FemModel3d)(mech.models().get("femNoTeeth"));
     // panel.addWidget("femNoTeeth material", femNoTeeth, "materialNoTeeth");
      
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }
      addControlPanel(panel);
     
   }
   
   // creates a voxelized finite element model surrounding a supplied surface mesh
   public static FemModel3d createVoxelFem(
      FemModel3d fem, PolygonalMesh mesh, int minRes, double maxElemWidth) {
      if (fem == null) {
         fem = new FemModel3d();
      }

      OBB obb = new OBB(mesh);
      // fem from OBB
      Vector3d hw = new Vector3d(obb.getHalfWidths());
      double dz = 2 * hw.z / minRes;
      if (dz > maxElemWidth) {
         dz = maxElemWidth;
      }
      // add a little bit
      // hw.add(dz / 8, dz / 8, dz / 8);

      int[] res = new int[3];
      res[0] = (int)(Math.round(2 * hw.x / dz));
      res[1] = (int)(Math.round(2 * hw.y / dz));
      res[2] = (int)(Math.round(2 * hw.z / dz));

      FemFactory.createHexGrid(
         fem, 2 * hw.x, 2 * hw.y, 2 * hw.z, res[0], res[1], res[2]);
      fem.transformGeometry(obb.getTransform());

      double dx, dy;
      dx = 2 * hw.x / res[0];
      dy = 2 * hw.y / res[1];
      dz = 2 * hw.z / res[2];
      double r = 1.05 * Math.sqrt(dx * dx + dy * dy + dz * dz);

      // outside and farther than r
      BVFeatureQuery query = new BVFeatureQuery();

      HashSet<FemNode3d> deleteThese = new HashSet<FemNode3d>();
      for (FemNode3d node : fem.getNodes()) {
         boolean inside =
            query.isInsideOrientedMesh(mesh, node.getPosition(), r);
         if (!inside) {
            deleteThese.add(node);
         }
      }

      // remove elements/nodes
      for (FemNode3d node : deleteThese) {
         // remove element dependencies
         ArrayList<FemElement3d> elems =
            new ArrayList<>(node.getElementDependencies());
         for (FemElement3d elem : elems) {
            fem.removeElement(elem);
         }
         // remove node
         fem.removeNode(node);
      }

      // remove un-necessary nodes
      deleteThese.clear();
      for (FemNode3d node : fem.getNodes()) {
         if (node.getElementDependencies().size() < 1) {
            deleteThese.add(node);
         }
      }
      for (FemNode3d node : deleteThese) {
         fem.removeNode(node);
      }

      return fem;
   }
   
   
   protected FemModel3d createVoxelizedFem(FemModel3d fem, PolygonalMesh mesh, int res) {
      fem = EmbeddedFem.createVoxelizedFem(fem, mesh, RigidTransform3d.IDENTITY, res, 0);
      
      // since mesh is not actually closed, can cause some issues with inside/outside tests, remove stray elements
      ArrayList<FemElement3d> toremove = new ArrayList<> ();
      BVFeatureQuery bvq = new BVFeatureQuery ();
      Point3d centroid = new Point3d();
      Point3d near = new Point3d();
      for (FemElement3d elem : fem.getElements ()) {
         elem.computeCentroid (centroid);
         bvq.nearestFaceToPoint (near, null, mesh, centroid);
         if (near.distance (centroid) > 5) { //0.01
            toremove.add (elem);
         }
      }
      
      for (FemElement3d elem : toremove) {
         fem.removeElement (elem);
      }
      
      ArrayList<FemNode3d> rnodes = new ArrayList<>();
      for (FemNode3d node : fem.getNodes ()) {
         if (node.numAdjacentElements () < 1) {
            rnodes.add (node);
         }
      }
      for (FemNode3d node : rnodes) {
         fem.removeNode (node);
      }
      
      
      return fem;
   }

   public void deformMandible() {
      TransformGeometryContext.transform (
         mySourceNoTeeth, myFemTransformer, /*flags=*/0);
      rerender();
   }

   public boolean getMenuItems(List<Object> items) {
      items.add (GuiUtils.createMenuItem (
                    this, "deform mandible", "deform the toothless mandible"));
      return true;
   }   


   public void actionPerformed (ActionEvent event) {
      if (event.getActionCommand().equals ("deform mandible")) {
         deformMandible();
      }
   } 


}

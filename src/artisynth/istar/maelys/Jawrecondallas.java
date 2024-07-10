package artisynth.istar.maelys;


import java.awt.Color;
import java.io.*;
import java.util.ArrayList;

import javax.swing.JSeparator;

import maspack.geometry.*;
import maspack.properties.*;
import maspack.render.*;
import maspack.render.Renderer.*;
import maspack.util.*;
import maspack.matrix.*;

import artisynth.core.workspace.*;
import artisynth.core.mechmodels.*;
import artisynth.core.mechmodels.MechSystemSolver.*;
import artisynth.core.modelbase.*;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.femmodels.*;
import artisynth.core.femmodels.FemModel.*;
import artisynth.core.materials.*;
import artisynth.core.gui.*;

/**
 * Simple example showing how to bolt together components of an FEM model of a
 * reconstructed jaw.
 */
public class Jawrecondallas extends RootModel {

        
        protected ArrayList<Muscle> myMuscles = new ArrayList<Muscle>();
        protected double myMuscleDamping = 0.00;
        ControlPanel panel = new ControlPanel("options");

        
   // booleans to control model configuration

   // set up collisions between segments. Otherwise, just connect them
   boolean myUseCollisions = true;

   // Approximate the TMJ with a hinge joint on the right mandible segment.
   // Otherwise, just fix the segment.
   boolean myAddTMJ =true;

   // use "screws" to connect the plate to the donor segments. Otherwise, just
   // attach plate nodes.
   boolean myUseScrews = false;

   // render surfaces of donor segments using stress maps.
   boolean myShowDonorStress = true;

   // units for this model are mm, Kg and seconds (mmKS), so quantities given
   // in MKS need to be converted.
   
   double DENSITY_TO_mmKS = 1e-9; // convert density from MKS tp mmKS
   double PRESSURE_TO_mmKS = 1e-3; // convert pressure from MKS tp mmKS

   // material properties for bone and titanium, converted from MKS to mmKS.

   double myBoneDensity = 1900.0 * DENSITY_TO_mmKS;
   double myBoneE = 17*1e9 * PRESSURE_TO_mmKS;
   double myBoneNu = 0.3;

   double myTitaniumDensity = 4420.0 * DENSITY_TO_mmKS;
   double myTitaniumE = 100*1e9 * PRESSURE_TO_mmKS;
   double myTitaniumNu = 0.3;

   // color definitions

   private static Color BONE = new Color (1f, 1f, 0.8f);
   private static Color GOLD = new Color (1f, 0.8f, 0.1f);
   private static Color PALE_BLUE = new Color (0.6f, 0.6f, 1.0f);
   
   // references to model components
   FemModel3d myDonor0;
   FemModel3d myDonor1;
   FemModel3d myPlate;
   RigidBody myMandibleLeft;
   RigidBody myMandibleRight;
   
// geometry folder path, relative to source file for this class
   String myGeoDir = PathFinder.getSourceRelativePath (JawReconNelson.class, "geometry/");
    //change
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public void build (String[] args) {
   
      // create the mech model
      MechModel mech = new MechModel ("mech");
      addModel (mech);
      SetUpModel(mech);
      // create the body representing the left mandible
      myMandibleLeft = createRigidBody (
         mech, "mandibleLeft", "DallasMandibleLeft.obj", myBoneDensity);
      // damping parameters are important for stablity
      myMandibleLeft.setFrameDamping (0.3);
      myMandibleLeft.setRotaryDamping (300);     
      // create the body representing the right mandible
      myMandibleRight = createRigidBody (
         mech, "mandibleRight", "DallasMandibleRight.obj", myBoneDensity);
      myMandibleRight.setFrameDamping (1.0);
      myMandibleRight.setRotaryDamping (2000);   
      myDonor0 = createFemModel (
         mech, "donor0", "DallasDonor00.obj", myBoneDensity, myBoneE, myBoneNu);//change
      myDonor1 = createFemModel (
         mech, "donor1", "DallasDonor01.obj", myBoneDensity, myBoneE, myBoneNu);//change

      if (myAddTMJ) {
         // add a simple hinge joint to mandible right to emulate the TMJ
         HingeJoint tmjR = new HingeJoint (
            myMandibleRight, null, new Point3d(59.961776, 179.77821, -220.13718), Vector3d.X_UNIT);
         tmjR.setShaftLength (20);
         RenderProps.setFaceColor (tmjR, Color.GREEN);
         mech.addBodyConnector (tmjR);
      }
      else {
         // otherwise, just fix mandible right
         myMandibleRight.setDynamic (false);
      }
      
   // set up donor segment interactions
      //setDonorSegmentInteractions (mech);
      
      // attach the plate to the left and right mandible segments. We use
      // explicitly defined nodes to do this, since the plate may be some
      // distance from the segments.
      int[] rightAttachNodes = {
         };
      attachFemToBody (mech, myPlate, myMandibleRight, rightAttachNodes);
      int[] leftAttachNodes = {
         };
      attachFemToBody (mech, myPlate, myMandibleLeft, leftAttachNodes);

      // attach plate to the donor segments
     attachPlateToDonorSegments (mech);
      
      
      // create a control panel for displaying and controlling certain
      // properties.
      panel.addWidget ("totalMaxStress", this, "maxStress");
      addControlPanel (panel);
      if (myShowDonorStress) {
         // set donor FEM models to display stress on their surfaces
         myDonor0.setSurfaceRendering (SurfaceRender.Stress);
         myDonor0.setStressPlotRanging (Ranging.Fixed);
         myDonor0.setStressPlotRange (new DoubleInterval(0, 200));
         myDonor1.setSurfaceRendering (SurfaceRender.Stress);
         myDonor1.setStressPlotRanging (Ranging.Fixed);
         myDonor1.setStressPlotRange (new DoubleInterval(0, 200));
         // allow stress ranges to be controlled in the control panel
         panel.addWidget ("stressRanging0", myDonor0, "stressPlotRanging");
         panel.addWidget ("stressRange0", myDonor0, "stressPlotRange");
         panel.addWidget ("stressRanging1", myDonor1, "stressPlotRanging");
         panel.addWidget ("stressRange1", myDonor1, "stressPlotRange");
      }
      setDefaultViewOrientation (AxisAlignedRotation.X_Z);
      panel.addWidget (new JSeparator());
   
      setupRenderProps();
      assembleMuscles();
      setUpMusclesLeft(mech);
      setUpMusclesRight(mech);
      setUpMusclesLow(mech);
      
   }
   
   
   public void attachMuscle (MechModel mech, String name, RigidBody mandible, Point3d insertion, Point3d origin ) {
      

      FrameMarker instertion_marker = new FrameMarker(mandible, insertion);
      mech.addFrameMarker(instertion_marker);
      RenderProps.setPointColor(instertion_marker, Color.GREEN);
      
      Particle p1 = new Particle(1 , origin);
      p1.setDynamic(false);
      mech.addParticle(p1);
      RenderProps.setSphericalPoints (p1, 1, Color.PINK);
      
      
      if (mandible == myMandibleLeft) {
              instertion_marker.setName("l_"+ name + "_insertion");
              p1.setName("l_"+ name + "_origin");
      }
      else{
              instertion_marker.setName("r_"+ name + "_insertion");
              p1.setName("r_"+ name + "_origin");
      }
                     
              
      
      
      AxialSpring m = findMuscle(name);
     
      m.setFirstPoint(instertion_marker);
      m.setSecondPoint(p1);
      AxialSpring.setDamping (m, myMuscleDamping);
      mech.addAxialSpring(m);
      // Spring Render Props
      RenderProps.setLineRadius(m, 2.0);
      //RenderProps.setLineSlices(myAxialSprings, 8);
      RenderProps.setLineStyle(m, Renderer.LineStyle.SPINDLE);
      RenderProps.setLineColor(m, Color.WHITE);
      ((Muscle)m).setExcitationColor (Color.RED);
  ((Muscle)m).setMaxColoredExcitation (1);
      
}
 
private Muscle findMuscle(String name) {
         for (Muscle m : myMuscles) {
            if (name.compareTo(m.getName()) == 0) return m;
         }
         return null;
      }  

public void assembleMuscles() {
         /*
          * all muscle CSA values and 40 N/cm^2 constant taken from Peck 2000 Arch
          * Oral Biology
          */
         double shlpMaxForce = 66.9 / 0.7 * 0.3; // ihlp reported as 70% of muscle
         // [Peck 2000]
         double mylohyoidMaxForce = 177.0 / 100 / 2.0 * 40.0; // mylohyoid 177 mm^2
         // from
         // Buchilliard2009
         // JASA, divided
         // equally into
         // anterior and
         // posterior parts
         double geniohyoidMaxForce = 80.0 / 100 * 40.0; // geniohyoid 80 mm^2 from
         // Buchilliard2009 JASA
         double postdigMaxForce = 40.0; // same CSA as antdig from vanEijden 1997
         // Anat Rec
         double stylohyoidMaxForce = 0.39 * 40; // 0.39 cm^2 from van Eijden 1997
         // Anat Rec

         // NB - max length and opt length get overwritten in
         // updateMuscleLengthProps()
         // Skull - Jaw Muscles
         myMuscles.add(createPeckMuscle("lat", 158.0, 75.54, 95.92, 0.5)); // lat
         myMuscles.add(createPeckMuscle("ldm", 81.6, 29.07, 44.85, 0.29)); // ldm
         myMuscles.add(createPeckMuscle("lip", 66.9, 31.5, 41.5, 0.0)); // lip
         // (opener)
         myMuscles.add(createPeckMuscle("lmp", 174.8, 40.51, 50.63, 0.64)); // lmp
         myMuscles.add(createPeckMuscle("lmt", 95.6, 65.81, 93.36, 0.48)); // lmt
         myMuscles.add(createPeckMuscle("lpt", 75.6, 77.11, 101.08, 0.51)); // lpt
         myMuscles.add(createPeckMuscle("lsm", 190.4, 51.46, 66.88, 0.46)); // lsm
         myMuscles.add(createPeckMuscle("lsp", shlpMaxForce, 27.7, 37.7, 0.0)); // lsp
         // (opener)
         myMuscles.add(createPeckMuscle("rat", 158.0, 75.54, 95.92, 0.5)); // rat
         myMuscles.add(createPeckMuscle("rdm", 81.6, 29.07, 44.85, 0.29)); // rdm
         myMuscles.add(createPeckMuscle("rip", 66.9, 31.5, 41.5, 0.0)); // rip
         // (opener)
         myMuscles.add(createPeckMuscle("rmp", 174.8, 40.51, 50.63, 0.64)); // rmp
         myMuscles.add(createPeckMuscle("rmt", 95.6, 65.81, 93.36, 0.48)); // rmt
         myMuscles.add(createPeckMuscle("rpt", 75.6, 77.11, 101.08, 0.51)); // rpt
         myMuscles.add(createPeckMuscle("rsm", 190.4, 51.46, 66.88, 0.46)); // rsm
         myMuscles.add(createPeckMuscle("rsp", shlpMaxForce, 27.7, 37.7, 0.0)); // rsp
         // (opener)

         // Laryngeal Muscles (jaw-hyoid, skull-hyoid)
         myMuscles.add(createPeckMuscle("lad", 40.0, 35.1, 45.1, 0.0)); // lad
         // (opener)
         myMuscles.add(createPeckMuscle("lpd", postdigMaxForce, 35.1, 45.1,
               0.0)); // lpd
         myMuscles.add(createPeckMuscle("lam", mylohyoidMaxForce, 35.1, 45.1,
               0.0));// Left Anterior Mylohyoid
         myMuscles.add(createPeckMuscle("lpm", mylohyoidMaxForce, 35.1, 45.1,
               0.0));// Left Posterior Mylohyoid
         myMuscles.add(createPeckMuscle("lgh", geniohyoidMaxForce, 35.1, 45.1,
               0.0));// Left Geniohyoid
         myMuscles.add(createPeckMuscle("lsh", stylohyoidMaxForce, 35.1, 45.1,
               0.0));// Left Stylohyoid
         myMuscles.add(createPeckMuscle("rad", 40.0, 35.1, 45.1, 0.0)); // rad
         // (opener)
         myMuscles.add(createPeckMuscle("rpd", postdigMaxForce, 35.1, 45.1,
               0.0)); // rpd
         myMuscles.add(createPeckMuscle("ram", mylohyoidMaxForce, 35.1, 45.1,
               0.0));// Right Anterior Mylohyoid
         myMuscles.add(createPeckMuscle("rpm", mylohyoidMaxForce, 35.1, 45.1,
               0.0));// Right Posterior Mylohyoid
         myMuscles.add(createPeckMuscle("rgh", geniohyoidMaxForce, 35.1, 45.1,
               0.0));// Right Geniohyoid
         myMuscles.add(createPeckMuscle("rsh", stylohyoidMaxForce, 35.1, 45.1,
               0.0));// Right Stylohyoid

         // hyoid depressors
         myMuscles.add(createPeckMuscle("lth", 20.0, 35.1, 45.1, 0.5));// Left
         // Thyrohyoid
         myMuscles.add(createPeckMuscle("lsteh", 50.0, 35.1, 45.1, 0.5));// Left
         // Sternohyoid
         myMuscles.add(createPeckMuscle("loh", 50.0, 35.1, 45.1, 0.5));// Left
         // Omohyoid
         myMuscles.add(createPeckMuscle("rth", 20.0, 35.1, 45.1, 0.5));// Right
         // Thyrohyoid
         myMuscles.add(createPeckMuscle("rsteh", 50.0, 35.1, 45.1, 0.5));// Right
         // Sternohyoid
         myMuscles.add(createPeckMuscle("roh", 50.0, 35.1, 45.1, 0.5));// Right
         // Omohyoid

         // Laryngeal Muscles (thyroid-cricoid, crico-arytenoid, sternum)
         myMuscles.add(createPeckMuscle("lpc", 20.0, 35.1, 45.1, 0.5));// Left
         // Posterior
         // Cricoarytenoid
         myMuscles.add(createPeckMuscle("llc", 20.0, 35.1, 45.1, 0.5));// Left
         // Lateral
         // Cricoarytenoid
         myMuscles.add(createPeckMuscle("lpct", 20.0, 35.1, 45.1, 0.5));// Left
         // Posterior
         // Cricothyroid
         myMuscles.add(createPeckMuscle("lact", 20.0, 35.1, 45.1, 0.5));// Left
         // Anterior
         // Cricothyroid
         myMuscles.add(createPeckMuscle("lstet", 20.0, 35.1, 45.1, 0.5));// Left
         // Sternothyroid
         myMuscles.add(createPeckMuscle("rpc", 20.0, 35.1, 45.1, 0.5));// Right
         // Posterior
         // Cricoarytenoid
         myMuscles.add(createPeckMuscle("rlc", 20.0, 35.1, 45.1, 0.5));// Right
         // Lateral
         // Cricoarytenoid
         myMuscles.add(createPeckMuscle("rpct", 20.0, 35.1, 45.1, 0.5));// Right
         // Posterior
         // Cricothyroid
         myMuscles.add(createPeckMuscle("ract", 20.0, 35.1, 45.1, 0.5));// Right
         // Anterior
         // Cricothyroid
         myMuscles.add(createPeckMuscle("rstet", 20.0, 35.1, 45.1, 0.5));// Right
         // Sternothyroid
         myMuscles.add(createPeckMuscle("ta", 20.0, 35.1, 45.1, 0.5));// Transverse
        
    
      }

private Muscle createPeckMuscle ( String name, double maxForce, double optLen, double maxLen, double ratio) {
   Muscle m = new Muscle(name);
   double maxForceScale = 1000;
   PeckAxialMuscle mat = PeckAxialMuscle.create (
      maxForce*maxForceScale, optLen, maxLen, ratio, /*damping=*/0);
   m.setMaterial (mat);
   return m;
}
 
public void setUpMusclesLeft(MechModel mech) {
   Point3d insertion1 = new Point3d(-4.27229, 21.2955, 13.0872);
   Point3d origin1 = new Point3d(-65.14684, 165.12822, -183.33423);
   attachMuscle (mech, "lpt" , myMandibleLeft, insertion1,  origin1 );
   panel.addWidget("lpt excitation",findMuscle("lpt"), "excitation");
  
   Point3d origin2 = new Point3d(-58.569826, 189.53709, -159.02902);
   attachMuscle (mech, "lmt" , myMandibleLeft, insertion1,  origin2 );
   panel.addWidget("lmt excitation",findMuscle("lmt"), "excitation");
   
   Point3d insertion3 = new Point3d(1.07911, 0.934111, -20.3294);
   Point3d origin3 = new Point3d(-51.536661, 214.09176, -222.37601);
   attachMuscle (mech, "lsm" , myMandibleLeft, insertion3,  origin3 );
   panel.addWidget("lsm excitation",findMuscle("lsm"), "excitation");
   
   Point3d insertion4 = new Point3d(-2.73851, 0.567611, -1.038639);
   Point3d origin4 = new Point3d(-55.482663, 202.91357, -220.46544);
   attachMuscle (mech, "ldm" , myMandibleLeft, insertion4,  origin4 );
   panel.addWidget("ldm excitation",findMuscle("ldm"), "excitation");
   
   Point3d insertion5 = new Point3d(-0.72241, -6.95744, 13.5241);
   Point3d origin5 = new Point3d(-33.190282, 205.75699, -213.32137);
   attachMuscle (mech, "lsp" , myMandibleLeft, insertion5,  origin5 );
   panel.addWidget("lsp excitation",findMuscle("lsp"), "excitation");
   
   Point3d insertion6 = new Point3d(-0.471776, -6.95318, 11.1241);
   Point3d origin6 = new Point3d(-11.68072, 190.25231, -223.97783);
   attachMuscle (mech, "lip" , myMandibleLeft, insertion6,  origin6 );
   panel.addWidget("lip excitation",findMuscle("lip"), "excitation");
   
   Point3d insertion7 = new Point3d(2.81627, 16.6838, 3.19833);
   Point3d origin7 = new Point3d(-48.193902, 206.45258, -153.92632);
   attachMuscle (mech, "lat" , myMandibleLeft, insertion7,  origin7 );
   panel.addWidget("lat excitation",findMuscle("lat"), "excitation");
   
   Point3d insertion8 = new Point3d(6.07181, -2.81954, -15.4855);
   Point3d origin8 = new Point3d(-11.74333, 190.9603, -228.80142);
   attachMuscle (mech, "lmp" , myMandibleLeft, insertion8,  origin8 );
   panel.addWidget("lmp excitation",findMuscle("lmp"), "excitation");
}

public void setUpMusclesRight(MechModel mech) {
   Point3d insertion1 = new Point3d(13.6629, -17.4152, 5.6396);
   Point3d origin1 = new Point3d(58.100863, 222.81683, -224.17241);
   attachMuscle (mech, "rsm" , myMandibleRight, insertion1,  origin1 );
   panel.addWidget("rsm excitation",findMuscle("rsm"), "excitation");
  
   Point3d insertion2 = new Point3d(16.3831, -17.8704, 23.3682);
   Point3d origin2 = new Point3d(63.651532, 211.55093, -219.66616);
   attachMuscle (mech, "rdm" , myMandibleRight, insertion2,  origin2 );
   panel.addWidget("rdm excitation",findMuscle("rdm"), "excitation");
   
   Point3d insertion3 = new Point3d(14.4147, -3.00008, 41.6657);
   Point3d origin3 = new Point3d(59.913887, 192.50212, -153.9332);
   attachMuscle (mech, "rmt" , myMandibleRight, insertion3,  origin3 );
   panel.addWidget("rmt excitation",findMuscle("rmt"), "excitation");
   
   Point3d origin4 = new Point3d(61.97074, 168.10453, -180.85943);
   attachMuscle (mech, "rpt" , myMandibleRight, insertion3,  origin4 );
   panel.addWidget("rpt excitation",findMuscle("rpt"), "excitation");
   
   Point3d insertion5 = new Point3d(17.4837, -33.967, 46.2344);
   Point3d origin5 = new Point3d(43.990474, 205.46494, -211.96007);
   attachMuscle (mech, "rsp" , myMandibleRight, insertion5,  origin5 );
   panel.addWidget("rsp excitation",findMuscle("rsp"), "excitation");
   
   Point3d insertion6 = new Point3d(16.8901, -32.3504, 44.2592);
   Point3d origin6 = new Point3d(20.725576, 186.18471, -223.191);
   attachMuscle (mech, "rip" , myMandibleRight, insertion6,  origin6 );
   panel.addWidget("rip excitation",findMuscle("rip"), "excitation");
   
   Point3d insertion7 = new Point3d(7.37727, -14.996, 22.9699);
   Point3d origin7 = new Point3d(51.312092, 177.8054 ,-155.5615);
   attachMuscle (mech, "rat" , myMandibleRight, insertion7,  origin7 );
   panel.addWidget("rat excitation",findMuscle("rat"), "excitation");
   
   Point3d insertion8 = new Point3d(9.22816 ,-24.6472, 7.85935);
   Point3d origin8 = new Point3d(18.539156, 189.01125, -227.62909);
   attachMuscle (mech, "rmp" , myMandibleRight, insertion8,  origin8 );
   panel.addWidget("rmp excitation",findMuscle("rmp"), "excitation");
}

public void setUpMusclesLow(MechModel mech) {
 
   
   Point3d insertion2 = new Point3d(-11.5866, 12.8073, -22.6352);
   Point3d origin2 = new Point3d(0.25443328, 209.3767, -269.93265);
   attachMuscle (mech, "ram" , myMandibleRight, insertion2,  origin2 );
   panel.addWidget("ram excitation",findMuscle("ram"), "excitation");
   
   Point3d insertion3 = new Point3d(-21.2797, 20.2201, -25.4962);
   Point3d origin3 = new Point3d(15.179621, 206.16923, -267.8695);
   attachMuscle (mech, "rad" , myMandibleRight, insertion3,  origin3 );
   panel.addWidget("rad excitation",findMuscle("rad"), "excitation");
   
   Point3d insertion4 = new Point3d(-26.6339, 21.527, -27.3735);
   Point3d origin4 = new Point3d(-0.57146761, 210.26745, -268.82661);
   attachMuscle (mech, "rgh" , myMandibleRight, insertion4,  origin4 );
   panel.addWidget("rgh excitation",findMuscle("rgh"), "excitation");

   Point3d insertion8 = new Point3d(2.33865, -9.15197, -3.96097);
   Point3d origin8 = new Point3d(7.8201302, 207.25009, -268.61501);
   attachMuscle (mech, "rpm" , myMandibleRight, insertion8,  origin8 );
   panel.addWidget("rpm excitation",findMuscle("lmp"), "excitation");


   
}
 
private void attachPlateToDonorSegments (MechModel mech) {
      
      // just attach selected plate nodes to the donor segments
      int[] plateAttachNodes0 = new int[] {
         };
      attachFemToBody (mech, myDonor0, myMandibleLeft, plateAttachNodes0);
      
      int[] plateAttachNodes1 = new int[] {
        477, 513, 1858
      };
      attachFemToBody (mech, myDonor1, myMandibleRight, plateAttachNodes1);
   }


   private void attachFemToBody (
MechModel mech, FemModel3d fem, PointAttachable body, int[] nodeNums) {

for (int num : nodeNums) {
   mech.attachPoint (fem.getNodeByNumber(num), body);
}
}  

   /**
    * Attach an FEM model to another body (either an FEM or a rigid body) by
    * attaching all surface nodes that are within a certain distance of the
    * body's surface mesh.
    *
    * @param mech MechModel containing all the components
    * @param fem FEM model to be connected
    * @param body body to attach the FEM to. Can be a rigid body
    * or another FEM.
    * @param dist distance to the body surface for attaching nodes
    */
   private void attachFemToBody (
      MechModel mech, FemModel3d fem, PointAttachable body, double dist) {
      
      PolygonalMesh surface = null;
      if (body instanceof RigidBody) {
         surface = ((RigidBody)body).getSurfaceMesh();
      }
      else if (body instanceof FemModel3d) {
         surface = ((FemModel3d)body).getSurfaceMesh();
      }
      else {
         throw new IllegalArgumentException (
            "body is neither a rigid body nor an FEM model");
      }
      for (FemNode3d n : fem.getNodes()) {
         if (fem.isSurfaceNode (n)) {
            double d = surface.distanceToPoint (n.getPosition());
            if (d < dist) {
               mech.attachPoint (n, body);
               // set the attached points to render as red spheres
               RenderProps.setSphericalPoints (n, 0.5, Color.RED);
            }
         }
      }
   }
   
   private void attachElemToSegment (
   MechModel mech, HexElement hex, FemModel3d donorFem,
   double screwLen, double attachTol) {

   // compute centroid of the hex element
   Point3d cent = new Point3d();
   hex.computeCentroid (cent);

   // compute normal pointing toward the donor FEM. From the construction of
   // plate FEM, we know that this is given by the outward facing normal of
   // the quad face given by the first four hex nodes.
   Vector3d nrm = new Vector3d();
   FemNode3d[] nodes = hex.getNodes();
   Face.computeNormal (
      nrm, nodes[0].getPosition(), nodes[1].getPosition(),
      nodes[2].getPosition(), nodes[3].getPosition());

   // represent the screw as a cylinder with radius 1/10 of it length.
   RigidBody screw = RigidBody.createCylinder (
      null, screwLen/10, screwLen, myTitaniumDensity, 10);
   // Set the pose of the screw so that it lies along the normal starting at
   // the hex centroid.
   RigidTransform3d TSW = new RigidTransform3d ();
   TSW.p.set (cent);
   TSW.R.setZDirection (nrm);
   TSW.mulXyz (0, 0, screwLen/2);
   screw.setPose (TSW);

   mech.addRigidBody (screw); // add to the MechModel

   // attach to the screw all donor FEM nodes that are within attachTol of
   // its surface
   PolygonalMesh smesh = screw.getSurfaceMesh();
   int nattach = 0;
   for (FemNode3d n : donorFem.getNodes()) {
      if (smesh.distanceToPoint (n.getPosition()) <= attachTol) {
         mech.attachPoint (n, screw);
         nattach++;
      }
   }
   System.out.println ("screw attached attached with" + nattach + " points");
   // also attach the screw to the hex element
   mech.attachFrame (screw, hex);
}

   public RigidBody createRigidBody (
      MechModel mech, String name, String meshName, double density) {

      PolygonalMesh surface = loadMesh (meshName);
      RigidBody body = RigidBody.createFromMesh (
         name, surface, density, /*scale=*/1.0);
      // set coordinate frame to be coincident with the center of mass 
      body.centerPoseOnCenterOfMass();
      mech.addRigidBody (body);
      // set the color of the surface mesh
      RenderProps.setFaceColor (body, BONE);
      return body;
   }
   
   private PolygonalMesh loadMesh (String meshName) {
      PolygonalMesh mesh = null;
      String meshPath = myGeoDir + meshName;
      try {
         mesh = new PolygonalMesh (meshPath);
      }
      catch (IOException e) {
         System.out.println ("Can't open or load "+meshPath);
      }
      return mesh;
   }
  
   public FemModel3d createFemModel (
      MechModel mech, String name, String meshName,
      double density, double E, double nu) {

      // create the fem and set its material properties
      FemModel3d fem = new FemModel3d (name);
      fem.setDensity (density);
      fem.setMaterial (new LinearMaterial (E, nu));

      // load the triangular surface mesh and then call createFromMesh,
      // which uses tetgen to create a tetrahedral volumetric mesh:
      PolygonalMesh surface = loadMesh (meshName);
      FemFactory.createFromMesh (fem, surface, /*tetgen quality=*/1.5);

      // damping parameters are important for stabilty
      fem.setMassDamping (1.0);
      fem.setStiffnessDamping (0);

      // enable computation of nodal stresses. Do this so that stresses will be
      // computed even if they are not being rendered.
      fem.setComputeNodalStress (true);

      // turn on surface rendering and set surface color to light blue
      RenderProps.setFaceColor (fem, PALE_BLUE);
      fem.setSurfaceRendering (FemModel.SurfaceRender.Shaded);
      RenderProps.setSphericalPoints (fem, 0.35, Color.BLUE);

      mech.addModel (fem);
      return fem;
   }   
   
   public void SetUpModel(MechModel mech) {
      mech.setGravity (0, 0, 0);
      setMaxStepSize (0.001);
      mech.setStabilization (PosStabilization.GlobalStiffness);
      
   }

   /**
    * Attach an FEM model to another body (either an FEM or a rigid body) by
    * attaching all surface nodes that are within a certain distance of the
    * body's surface mesh.
    *
    * @param mech MechModel containing all the components
    * @param fem FEM model to be connected
    * @param body body to attach the FEM to. Can be a rigid body
    * or another FEM.
    * @param dist distance to the body surface for attaching nodes
    */

   /**
    * Helper method to set the interactions between donor segments and the
    * mandible segments.
    */
   private void setDonorSegmentInteractions (MechModel mech) {
      if (myUseCollisions) {
         // set the interactions using collisions
         mech.setCollisionBehavior (myDonor1, myMandibleRight, true);
         mech.setCollisionBehavior (myDonor1, myDonor0, true);
         mech.setCollisionBehavior (myDonor0, myMandibleLeft, true);
         // set collision manager render properties in case we want to render
         // contact info at some point
         CollisionManager cm = mech.getCollisionManager();
         RenderProps.setLineColor (cm, Color.GREEN);
         RenderProps.setLineRadius (cm, 0.5);
         RenderProps.setLineStyle (cm, LineStyle.SOLID_ARROW);
         RenderProps.setVisible (cm, true);        
      }
      else {
         // just connect the segments together
         double dist = 0.0001;
         attachFemToBody (mech, myDonor0, myDonor1, dist);
         attachFemToBody (mech, myDonor1, myMandibleRight, dist);
         attachFemToBody (mech, myDonor0, myMandibleLeft, dist);
      }
   }
  
   private void setupRenderProps() {
      RenderProps props = createRenderProps();

      // Particle RenderProps
      props.setPointRadius(1.0);
      props.setPointStyle(Renderer.PointStyle.SPHERE);
      //props.setPointSlices(12);
      props.setPointColor(Color.PINK);

      // Line RenderProps
      props.setLineRadius(2.0);
      //props.setLineSlices(8);
      props.setLineWidth(3);
      props.setLineStyle(Renderer.LineStyle.LINE);
      props.setLineColor(Color.WHITE);

      // Mesh RenderProps
      props.setShading(Renderer.Shading.SMOOTH);
      props.setFaceColor(new Color(1f, 0.8f, 0.6f));
      props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);

      setRenderProps(props);
}
}

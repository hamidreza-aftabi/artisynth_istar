package artisynth.istar.Prisman;

import java.awt.Color;
import java.io.*;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.*;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.StlReader;
import maspack.interpolation.Interpolation;
import maspack.interpolation.Interpolation.Order;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.*;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.materials.AnisotropicLinearMaterial;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.femmodels.*;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.mechmodels.*;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.properties.PropertyList;
import maspack.render.*;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;

public class SSMJoining extends RootModel {
   double length = 169.5;
   double width = 7;
   double height = 2;
   double repeat_len = 10;
   
   double cylinder_radius = 1.3;
   double start_buffer = 3.8;
   
   
   MechModel myMechMod;
   String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "segmentations_partial\\");
   String saveDirectory = ArtisynthPath.getSrcRelativePath(this, "segmentations_partial\\joined\\");
   
   public static PropertyList myProps =  new PropertyList (SSMJoining.class, RootModel.class);
   
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   
   static {
      myProps.add ("value * *", "a value", 0, "[-1,3] AE");
   }
   
   double v = 0;
   
   public double getValue() {
      return v;
   }

   public void setValue (double newv) {
      v = newv;
   }

   public int maxIndex(double... ds) {
      int idx = -1;
      double d= Double.NEGATIVE_INFINITY;
      for(int i = 0; i < ds.length; i++)
          if(ds[i] > d) {
              d = ds[i];
              idx = i;
          }
      return idx;
  }
  
  public int[] maxIndexN(double... ds) {
     int N = 10;
     int[] idxs = new int[N];
     int i;
     double large[] = new double[N];
     double max = 0;
     int index;
     for (int j = 0; j < idxs.length; j++) {
         max = ds[0];
         index = 0;
         for (i = 1; i < ds.length; i++) {
             if (max < ds[i]) {
                 max = ds[i];
                 index = i;
             }
         }
         large[j] = max;
         ds[index] = Integer.MIN_VALUE;
         idxs[j] = index;
     }
     return idxs;
  }
  

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   

   @SuppressWarnings("deprecation")
   public void build (String[] args) throws IOException {
      File[] oMeshFiles = new File(meshDirectory + "Defects\\").listFiles();
      File[] ssmMeshFiles = new File(meshDirectory + "Originals\\").listFiles();
      
      //PolygonalMesh oMesh = GenericModel.loadGeometry(meshDirectory, "7Remaining.stl");
      //PolygonalMesh ssmMesh = GenericModel.loadGeometry(meshDirectory, "7RemainingRecon.stl");
      
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
      
      for (int k = 0; k < oMeshFiles.length; k++) {
         PolygonalMesh oMesh = StlReader.read (oMeshFiles[k].getAbsolutePath ());
         //k%5
         System.out.println(oMeshFiles[k].getAbsolutePath ());
         System.out.println(ssmMeshFiles[k/5].getAbsolutePath ());
         PolygonalMesh ssmMesh = StlReader.read (ssmMeshFiles[k / 5].getAbsolutePath ());
         //System.out.println((oMeshFiles[k].getAbsolutePath ().substring(oMeshFiles[k].getAbsolutePath ().length() -5, oMeshFiles[k].getAbsolutePath ().length()-1)));
         
         if (oMeshFiles[k].getAbsolutePath ().substring(oMeshFiles[k].getAbsolutePath ().length() -5, oMeshFiles[k].getAbsolutePath ().length()-1).equals ("c.st")) {
            ArrayList<Face> faces = oMesh.getFaces ();
            double[] areas = new double[faces.size ()];
            for (int i = 0; i < faces.size (); i++) {
               faces.get (i).computeNormal ();
               areas[i] = faces.get (i).getPlanarArea ();
            }

            Integer idxOfLargestFace = maxIndex (areas);
            Vector3d normal = new Vector3d ();
            //faces.get (idxOfLargestFace).computeNormal(normal);
            //faces.get (idxOfLargestFace).computeNormal();
            normal = faces.get (idxOfLargestFace).getNormal ();

            Vector3d centerOfFace = new Vector3d ();
            faces.get (idxOfLargestFace).computeCentroid (centerOfFace);

            PolygonalMesh plane1Mesh = MeshFactory.createPlane (90, 85);
            RigidTransform3d initialPose = new RigidTransform3d ();

            Vector3d currentPlane1Normal = plane1Mesh.getNormal (0);

            currentPlane1Normal.transform (initialPose);

            RotationMatrix3d rotatePlane1 =
               meshHelper.rotatePlane (currentPlane1Normal, normal);

            AffineTransform3d t1 = new AffineTransform3d ();
            t1.setRotation (rotatePlane1);
            plane1Mesh.transform (t1);

            Vector3d translationVector = new Vector3d (normal);
            translationVector.negate ();
            translationVector.normalize ();
            translationVector.scale (0.001);
            centerOfFace.add (translationVector);

            meshHelper.setPlaneOrigin (plane1Mesh, centerOfFace);

            ssmMesh =
               intersector.findDifference01 (ssmMesh.clone (), plane1Mesh);

            //ssmMesh = intersector.findUnion (ssmMesh, oMesh);

            GenericMeshWriter.writeMesh (saveDirectory + oMeshFiles[k].getAbsolutePath ().substring(oMeshFiles[k].getAbsolutePath ().length() -8, oMeshFiles[k].getAbsolutePath ().length()-4) + "Joined.stl", ssmMesh);  
         }
         // 2 Faces
         else {
            System.out.println("2 face");
            PolygonalMesh[] resections = new PolygonalMesh[2];
            PolygonalMesh[] pieces = oMesh.partitionIntoConnectedMeshes ();
            for (int i = 0; i < 2; i++) {
               //For some reason the element 0 and element 1 of pieces are the same
//               ArrayList<Face> faces = pieces[i+1].getFaces ();
//               double[] areas = new double[faces.size ()];
//               for (int j = 0; j < faces.size (); j++) {
//                  faces.get (j).computeNormal ();
//                  areas[j] = faces.get (j).getPlanarArea ();
//               }
//               
//               Integer idxOfLargestFace = maxIndex (areas);
//               Vector3d normal = new Vector3d ();
//               normal = faces.get (idxOfLargestFace).getNormal ();
//
//               Vector3d centerOfFace = new Vector3d ();
//               faces.get (idxOfLargestFace).computeCentroid (centerOfFace);
               
               
               //New method to get faces
               ArrayList<Face> faces = pieces[i+1].getFaces ();
               double[] areas = new double[faces.size ()];
               for (int l = 0; l < faces.size (); l++) {
                  areas[l] = faces.get (l).getPlanarArea ();
               }
               int[] indices_to_keep = maxIndexN(areas);
               Vector3d[] normals = new Vector3d[indices_to_keep.length];
               for (int l = 0; l < indices_to_keep.length; l++) {
                  normals[l] = new Vector3d();
               }
               for (int l = 0; l < indices_to_keep.length; l++) {
                  faces.get (indices_to_keep[l]).computeNormal (normals[l]);
               }
               
               double[] count = new double[indices_to_keep.length];
               for (int l = 0; l < indices_to_keep.length; l++) {
                  for (int m = 0; m < indices_to_keep.length; m++) {
                     normals[l].normalize ();
                     normals[m].normalize ();
                     if (normals[l].dot (normals[m]) >.9999) {
                        count[l] += 1;
                     }
                  }
               }
               
               Integer idxOfCommonNormal = maxIndex (count);

//               double[] testArr=  {1.0, 2.3, 0.9, 5.9, 11.6, 2.0, 1.0};
//               int[] testIdx = maxIndexN(testArr);
//               for (int l = 0; l < testIdx.length; l++) {
//                  System.out.println(testIdx[l]);
//               }
               
               Vector3d centerOfFace = new Vector3d ();
               faces.get (indices_to_keep[idxOfCommonNormal]).computeCentroid (centerOfFace);
               
               Vector3d normal = new Vector3d (); 
               normal = faces.get (indices_to_keep[idxOfCommonNormal]).getNormal ();
               


               PolygonalMesh planeMesh = MeshFactory.createPlane (90, 85);
               RigidTransform3d initialPose = new RigidTransform3d ();

               Vector3d currentPlane1Normal = planeMesh.getNormal (0);

               currentPlane1Normal.transform (initialPose);

               RotationMatrix3d rotatePlane1 =
                  meshHelper.rotatePlane (currentPlane1Normal, normal);

               AffineTransform3d t1 = new AffineTransform3d ();
               t1.setRotation (rotatePlane1);
               planeMesh.transform (t1);

               Vector3d translationVector = new Vector3d (normal);
               translationVector.negate ();
               translationVector.normalize ();
               translationVector.scale (0.01);
               centerOfFace.add (translationVector);

               meshHelper.setPlaneOrigin (planeMesh, centerOfFace);

               //GenericMeshWriter.writeMesh (saveDirectory + "plane" + Integer.toString (i) + ".stl", planeMesh);
               ssmMesh = intersector.findDifference01 (ssmMesh, planeMesh);
               
            }
            //ssmMesh = intersector.findUnion (resections[0], resections[1]);
//            ssmMesh = intersector.findUnion (ssmMesh, oMesh);
//            GenericMeshWriter.writeMesh (saveDirectory + oMeshFiles[k].getAbsolutePath ().substring(oMeshFiles[k].getAbsolutePath ().length() -8, oMeshFiles[k].getAbsolutePath ().length()-4) + "Joined.stl", resections[0]);
//            GenericMeshWriter.writeMesh (saveDirectory + oMeshFiles[k].getAbsolutePath ().substring(oMeshFiles[k].getAbsolutePath ().length() -8, oMeshFiles[k].getAbsolutePath ().length()-4) + "Joined1.stl", resections[1]);
            GenericMeshWriter.writeMesh (saveDirectory + oMeshFiles[k].getAbsolutePath ().substring(oMeshFiles[k].getAbsolutePath ().length() -8, oMeshFiles[k].getAbsolutePath ().length()-4) + "Joined.stl", ssmMesh);
//            for (int l = 0; l <3; l++) {
//               GenericMeshWriter.writeMesh (saveDirectory + "test" + Integer.toString (l) + ".stl", pieces[l]);
//              
//            }
         } 
      }
      
      
//      GenericMeshWriter
//      .writeMesh (meshDirectory + "Plane.stl", plane1Mesh);
//      
//      Vertex3d[] vertices = faces.get (idxOfLargestFace).getTriVertices ();
//      
//      PolygonalMesh newMesh = new PolygonalMesh();
//      for (int i=0; i<3; i++) {
//         newMesh.addVertex (faces.get (idxOfLargestFace).getTriVertices ()[i]);
//      }
//      
//      int[] indicies = new int[3];
//      indicies[0] = 0;
//      indicies[1] = 1;
//      indicies[2] = 2;
//      
//      
//      newMesh.addFace (indicies);
//      //newMesh.addFace(faces.get (idxOfLargestFace).getTriVertices ());
//      
//      GenericMeshWriter
//      .writeMesh (meshDirectory + "Face.stl", newMesh);
      

      
      //Searching for smilar faces
//      ArrayList<Vector3d> normals = new ArrayList<Vector3d> ();
//      
//      
//      for (int i = 0; i < faces.size (); i++) {
//         normals.add (new Vector3d());
//         faces.get (i).computeNormal(normals.get (i));
//         normals.get (i).normalize ();
//      }
//      
//      ArrayList<Integer> count = new ArrayList<>();
//      Integer removed = 0;
//      Integer size = normals.size();
//      for (int i =0; i < size; i++) {
//         ArrayList<Integer> indicies_to_remove = new ArrayList<>();
//         for (int j = i+1; j < normals.size() - i-1; j++) {
//            double distance = normals.get (i).distance (normals.get (j));
//            if (distance < 0.1) {
//               indicies_to_remove.add(j);
//            }
//         }
//         for (Integer idx : indicies_to_remove) {
//            normals.remove(idx);
//         }
//
//         count.add (indicies_to_remove.size());
//         removed += indicies_to_remove.size();
//         if (i == size - removed) {
//            break;
//         }     
//      }
//      System.out.println(count);
   }

   @Override
   public void attach (DriverInterface driver) {
   }

   @Override
   public void detach (DriverInterface driver) {
   }

   /**
    * {@inheritDoc}
    */
   public String getAbout() {
      return "testing bending plate";
   }
}


/**
 * Copyright (c) 2020, by the Author: Rui Yang (UBC)
 */
package artisynth.istar.Rui;

import java.util.Random;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Line;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

/**
 * Constructive Solid Geometry tools
 */
public class ShapeDiameterFunction {

   // Shapira, L., Shapira, L., Shamir, A., Shamir, A., Cohen-Or, D., & Cohen-Or, D. (2008). 
   // Consistent mesh partitioning and skeletonisation using the shape diameter function. 
   // The Visual Computer, 24(4), 249-259. doi:10.1007/s00371-007-0197-5
   public static double computeShapeDiameter(Vertex3d vert, PolygonalMesh mesh, double coneAngle, int numRays) {
      
      double sdf;
      double dist = 0.0;
     // double weightSum = 0.0;        
      Vector3d nrm = new Vector3d();
      vert.computeNormal (nrm);
      Vector3d negNrm = new Vector3d();  
      negNrm.negate (nrm);            
      RotationMatrix3d R = new RotationMatrix3d();
      
      // when nrm is in an exact opposite direction
      if(negNrm == new Vector3d (0, 0, -1))
         R = new RotationMatrix3d(1, 0, 0,
                                  0, -1, 0,
                                  0, 0, -1);         
      else 
         R = computeRotation(new Vector3d (0, 0, 1), negNrm);
       
      RigidTransform3d T = new RigidTransform3d (vert.pnt.x, vert.pnt.y, vert.pnt.z);
      T.mulRotation (R);
      
      for(int i = 0; i < numRays; i++) {         
        Line ray = getRandomVectInCone(coneAngle);    
       // double angle = ray.getDirection ().angle (new Vector3d (0, 0, 1));       
        ray.transform (T);        
        dist += intersectMesh(mesh, ray, nrm);// / angle;       
       // weightSum += 1.0/angle;        
      }
      
      sdf = dist / numRays;// * weightSum);
//      System.out.println("sdf "+ sdf);
      return sdf;  
   }
   
   //https://math.stackexchange.com/questions/56784/generate-a-random-direction-within-a-cone/205589#205589
   public static Line getRandomVectInCone(double coneAngle){
      
      Random r1 = new Random();
      double rand1 = r1.nextDouble();      
      double ang = 0.5 * Math.PI * coneAngle /180; 
      double z = rand1 * (1 - Math.cos(ang)) + Math.cos(ang);
      
      Random r2 = new Random();
      double rand2 = r2.nextDouble();      
      double x = Math.sqrt(1 - Math.pow(z,2))* Math.cos(rand2 * 2 * Math.PI);
      double y = Math.sqrt(1 - Math.pow(z,2))* Math.sin(rand2 * 2 * Math.PI);
      
      Line ray = new Line(new Point3d (0, 0, 0), new Vector3d (x, y, z));      
      return (ray);      
      
   }
   
   //debugged https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
   public static RotationMatrix3d computeRotation(Vector3d vec1, Vector3d vec2) {
      RotationMatrix3d R = new RotationMatrix3d(1, 0, 0,
         0, 1, 0,
         0, 0, 1);
      Vector3d cross = new Vector3d();
      cross.cross (vec1, vec2);
      double dot = vec1.dot (vec2); 
      Matrix3d skewSymmetricCross = new Matrix3d (0, -cross.z, cross.y,
                                                  cross.z, 0, -cross.x,
                                                  -cross.y, cross.x, 0); 
      R.add (skewSymmetricCross);
      skewSymmetricCross.mul (skewSymmetricCross);
      skewSymmetricCross.scale(1.0/(1.0+dot));
      R.add(skewSymmetricCross);
          
      return R;      
   }
     
   public static double intersectMesh(PolygonalMesh mesh, Line ray, Vector3d nrm) {
      
      double dist = 0.0;
      double ang = 0.0;
      Point3d pos = new Point3d();
      BVFeatureQuery query = new BVFeatureQuery();     
      Face faceHit;
      
      do {         
         Point3d rayOff = new Point3d ();
         rayOff.set (ray.getOrigin ());
         rayOff.scaledAdd (0.001, ray.getDirection ());
         faceHit = query.nearestFaceAlongRay (pos, null, mesh, rayOff, ray.getDirection ());
         //pos = BVFeatureQuery.nearestPointAlongRay (mesh, rayOff, ray.getDirection ());
        
         if(faceHit == null) 
            break; 
         
         dist += pos.distance (ray.getOrigin ());
         ray.setOrigin (pos);
               
         //check for false intersection
         ang = faceHit.getNormal ().angle (nrm);
         
         }while(ang < Math.PI/3);     
      
      return dist;
   }
}

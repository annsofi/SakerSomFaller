using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace WindowsGame3
{
    class Physics
    {

        public void initPhysics(ref PhysicsMesh physMesh)
        {
            // Ett pappers densitet i kg/m^3
            physMesh.density = 1000;

            for (int i = 0; i < physMesh.numberOfPolygons; i++)
            {
                calculateCentroid(ref physMesh.polygonArray[i]);
                calculatePolygonMass(ref physMesh.polygonArray[i], physMesh.density);
            }

            calculateObjectMass(ref physMesh);
            //calculateObjectCenterOfMass(ref physMesh);
            //calculateInertiaTensorInverse(ref physMesh);
        }

        public void calculateCentroid(ref Polygon triangle)
        {
            Vector3 centroid = new Vector3();

            for (int i = 0; i < triangle.vertices.Length; i++)
            {
                centroid.X += triangle.vertices[i].X;
                centroid.Y += triangle.vertices[i].Y;
                centroid.Z += triangle.vertices[i].Z;
            }

            centroid.X = centroid.X / triangle.vertices.Length;
            centroid.Y = centroid.Y / triangle.vertices.Length;
            centroid.Z = centroid.Z / triangle.vertices.Length;

            triangle.centerOfMass = centroid;
            System.Diagnostics.Debug.WriteLine("Centroid: " + centroid.X + ", " + centroid.Y + ", " + centroid.Z);
        }

        public void calculatePolygonMass(ref Polygon triangle, float density)
        {
            Vector3 AB = new Vector3();
            Vector3 AC = new Vector3();

            AB = Vector3.Subtract(triangle.vertices[0], triangle.vertices[1]);
            AC = Vector3.Subtract(triangle.vertices[0], triangle.vertices[2]);

            triangle.area = (float)(0.5 * (Vector3.Cross(AB, AC).Length()));
            triangle.mass = triangle.area * density;
        }

        public void calculateObjectMass(ref PhysicsMesh physMesh)
        {
            float totalMass = 0.0f;

            for (int i = 0; i < physMesh.polygonArray.Length; i++)
            {
                totalMass += physMesh.polygonArray[i].mass;
            }
            physMesh.totalMass = totalMass;

            Console.WriteLine("total mass: " + totalMass);
        }

        public void calculateObjectCenterOfMass(ref PhysicsMesh physMesh)
        {
            Vector3 COM = new Vector3();

            for (int i = 0; i < physMesh.polygonArray.Length; i++)
            {
                COM += physMesh.polygonArray[i].centerOfMass*physMesh.polygonArray[i].mass;
            }

            COM = COM / physMesh.totalMass;
            physMesh.centerOfMass = COM;

            Console.WriteLine("COM " + COM);

            //COM i origo
            for (int i = 0; i < physMesh.polygonArray.Length; i++)
            {
                physMesh.polygonArray[i].vertices[0] = physMesh.polygonArray[i].vertices[0] - COM;
                physMesh.polygonArray[i].vertices[1] = physMesh.polygonArray[i].vertices[1] - COM;
                physMesh.polygonArray[i].vertices[2] = physMesh.polygonArray[i].vertices[2] - COM;
            }

            for (int i = 0; i < physMesh.numberOfPolygons; i++)
            {
                calculateCentroid(ref physMesh.polygonArray[i]);

            }
        }

        public void calculateInertiaTensorInverse(ref PhysicsMesh physMesh)
        {
            Matrix Ibody = new Matrix();

            for (int i = 0; i < physMesh.polygonArray.Length; i++)
            {
                //Console.WriteLine("i: " + physMesh.polygonArray[i].mass);
                Ibody.M11 += physMesh.polygonArray[i].mass * ((float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Y, 2.0) + (float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Z, 2.0));
                Ibody.M12 += physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Y));
                Ibody.M13 += physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Z));

                Ibody.M21 += physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Y));
                Ibody.M22 += physMesh.polygonArray[i].mass * ((float)Math.Pow(physMesh.polygonArray[i].centerOfMass.X, 2.0) + (float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Z, 2.0));
                Ibody.M23 += physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.Y) * (physMesh.polygonArray[i].centerOfMass.Z));

                Ibody.M31 += physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Z));
                Ibody.M32 += physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.Y) * (physMesh.polygonArray[i].centerOfMass.Y));
                Ibody.M33 += physMesh.polygonArray[i].mass * ((float)Math.Pow(physMesh.polygonArray[i].centerOfMass.X, 2.0) + (float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Y, 2.0));
            }

            physMesh.inertiaTensorInverse = Matrix.Invert(Ibody);
        }

    }
}

/*
 * Physics.cs
 * 
 * Denna klass hanterar stelkroppsdynamik samt vilka krafter som verkar
 * på stelkropparna. Den sköter offline-uträkningar som sker innan
 * simuleringen börjar, såväl som online-uträkningarna under själva
 * simuleringen.
 * 
 * Fysiken som beräknas är första ordningens differentialekvationer för
 * translation och rotation i 3 dimensioner när ett godtyckligt antal 
 * krafter påverkar kroppen.
*/

using System;
using System.Collections.Generic;
using System.Collections;
using System.Text;
using Microsoft.Xna.Framework;


class Physics
{
    public float speed = 1f;
    private float h;
    // private float det = 0;

    private Random RandomClass;
    public windSimulation windSim;
    public Boolean usingQuat = true;
    public Boolean usingGravity = true;
    public Boolean usingLinearDrag = true;
    public Boolean usingAngularDrag = true;
    public Boolean usingWind = true;
    private Vector3 oldPosition;
    private Matrix3 oldRotation;
    private Vector3 oldAngularMomentum;
    private Vector3 oldLinearMomentum;
    private Vector3 oldAngularVelocity;
    private Quaternion oldRotationQuaternion;
    private Vector3 gravity;
    private Vector3 dragForce;
    private Vector3 Normal;
    private Vector3 Velocity;
    private float p;
    private float Cd;
    private float areaPercent;
    private Matrix3 angularVelocityStar;
    private Vector3 angularVelocity;

    private Vector3 forceI;
    private Vector3 totalForce;
    private Vector3 torque;


    private Quaternion tempQuaternion;
    private Quaternion angularVelocityQuaternion;

    public Physics()
    {
        RandomClass = new Random();
        windSim = new windSimulation();

        angularVelocityStar = new Matrix3();
        angularVelocity = new Vector3();


        forceI = new Vector3();
        totalForce = new Vector3(0.0f, 0.0f, 0.0f);
        torque = new Vector3(0.0f, 0.0f, 0.0f);
    }

    // Denna funktion initierar fysiken och gör dem beräkningar som är nödvändiga
    // för att en simulering skall kunna genomföras.

    public void initPhysics(ref physicsMesh physMesh)
    {
        // Ett pappers densitet i kg/m^3
        physMesh.density = 780f + RandomClass.Next(3000);

        p = 1.21f;

        for (int i = 0; i < physMesh.numberOfPolygons; i++)
        {
            calculateCentroid(ref physMesh.polygonArray[i]);
            calculatePolygonMass(ref physMesh.polygonArray[i], physMesh.density);
        }

        calculateObjectMass(ref physMesh);
        calculateObjectCenterOfMass(ref physMesh);
        calculateInertiaTensorInverse(ref physMesh);

        physMesh.initiated = true;

    }


    public void updatePhysics(ref logicalObject logObject, ref physicsMesh physMesh, bool intScene)
    {

        if (speed < 0) speed = 0f;
        if (speed > 1) speed = 1f;
        // Console.WriteLine("phys speed: " + speed);
        if (speed != 0)
        {

            h = speed / 60f;

            Cd = logObject.Cd;

            oldPosition = logObject.Position;
            oldRotation = logObject.Rotation;
            oldAngularMomentum = logObject.angularMomentum;
            oldLinearMomentum = logObject.linearMomentum;
            oldAngularVelocity = logObject.angularVelocity;
            oldRotationQuaternion = logObject.rotationQuaternion;

            calculatePosition(ref logObject, physMesh.totalMass);


            if (usingQuat)
                calculateQuaternionRotation(ref logObject, physMesh.inertiaTensorInverse);
            else calculateMatrixRotation(ref logObject, physMesh.inertiaTensorInverse);

            if (intScene)
            {
                collisionDetection(ref logObject);
            }
            calculateMomentum(ref logObject, oldRotation, physMesh.numberOfPolygons, physMesh.polygonArray, physMesh.totalMass, oldAngularVelocity, oldLinearMomentum, oldPosition);

        }
    }


    // räknar ut polygonen triangles center of mass.
    public Vector3 calculateCentroidForce(polygon poly, Vector3 linearVelocity, Matrix3 oldRotation, Vector3 angularVelocity, Vector3 objectPosition)
    {
        gravity = new Vector3(0.0f, -9.82f * poly.mass, 0.0f);
        dragForce = calculateDragForce(poly, linearVelocity, oldRotation, angularVelocity, objectPosition);
        //Console.WriteLine("Dragforce: " + dragForce + "\n");
        if (usingGravity)
        {
            return (Vector3)(gravity + dragForce);
        }
        else
        {
            return (Vector3)(dragForce);
        }
        //return new Vector3();
    }


    public Vector3 calculateDragForce(polygon poly, Vector3 linearVelocity, Matrix3 oldRotation, Vector3 angularVelocity, Vector3 objectPosition)
    {
        Normal = Vector3.Cross(poly.vertices[0] - poly.vertices[1], poly.vertices[0] - poly.vertices[2]);

        //if (Normal.Y > 0) Normal = -1.0f * Normal;

        Velocity = new Vector3(0.0f, 0.0f, 0.0f);
        if (usingLinearDrag)
        {
            Velocity -= linearVelocity;
        }
        if (usingAngularDrag)
        {
            Velocity -= Vector3.Cross(angularVelocity, oldRotation * poly.centerOfMass);
        }
        if (usingWind)
        {
            Velocity -= windSim.returnWind(oldRotation * poly.centerOfMass + objectPosition);
        }
        //Velocity = -linearVelocity - Vector3.Cross(angularVelocity, oldRotation * poly.centerOfMass) -windSim.returnWind(oldRotation * poly.centerOfMass + objectPosition);

        //Console.WriteLine("Centroid: " + poly.centerOfMass.Length());
        areaPercent = Math.Abs(Vector3.Dot(oldRotation * Normal, Velocity) / (Normal.Length() * Velocity.Length()));
        //Console.WriteLine("poly Area: " + poly.area);
        return (p * poly.area * areaPercent * 0.5f * Cd * Vector3.Multiply(Velocity, abs(Velocity)));
    }

    public Vector3 abs(Vector3 v)
    {
        return new Vector3(Math.Abs(v.X), Math.Abs(v.Y), Math.Abs(v.Z));
    }

    public void calculateCentroid(ref polygon triangle)
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

        //Console.WriteLine("Centroid: " + centroid.X + ", " + centroid.Y + ", " + centroid.Z);
    }

    public void calculatePolygonMass(ref polygon triangle, float density)
    {
        Vector3 AB = new Vector3();
        Vector3 AC = new Vector3();

        AB = Vector3.Subtract(triangle.vertices[0], triangle.vertices[1]);
        AC = Vector3.Subtract(triangle.vertices[0], triangle.vertices[2]);

        triangle.area = (float)(0.5 * (Vector3.Cross(AB, AC).Length()));
        triangle.mass = triangle.area * 0.001f * density;
    }

    public void calculateObjectMass(ref physicsMesh physMesh)
    {
        float totalMass = 0.0f;

        for (int i = 0; i < physMesh.polygonArray.Length; i++)
        {
            totalMass += physMesh.polygonArray[i].mass;
        }
        physMesh.totalMass = totalMass;

        //Console.WriteLine("total mass: " + totalMass);
    }

    public void calculateObjectCenterOfMass(ref physicsMesh physMesh)
    {
        Vector3 COM = new Vector3();

        for (int i = 0; i < physMesh.polygonArray.Length; i++)
        {
            COM = COM + Vector3.Multiply(physMesh.polygonArray[i].centerOfMass, physMesh.polygonArray[i].mass);
        }

        COM = COM / physMesh.totalMass;

        //Console.WriteLine("COM " + COM);

        for (int i = 0; i < physMesh.polygonArray.Length; i++)
        {
            physMesh.polygonArray[i].vertices[0] = physMesh.polygonArray[i].vertices[0] - COM;
            physMesh.polygonArray[i].vertices[1] = physMesh.polygonArray[i].vertices[1] - COM;
            physMesh.polygonArray[i].vertices[2] = physMesh.polygonArray[i].vertices[2] - COM;
        }


        physMesh.centerOfMass = COM;


        for (int i = 0; i < physMesh.numberOfPolygons; i++)
        {
            calculateCentroid(ref physMesh.polygonArray[i]);

        }
    }

    public void calculateInertiaTensorInverse(ref physicsMesh physMesh)
    {
        Matrix3 Ibody = new Matrix3();

        for (int i = 0; i < physMesh.polygonArray.Length; i++)
        {
            //Console.WriteLine("i: " + physMesh.polygonArray[i].mass);
            Ibody.M11 = Ibody.M11 + physMesh.polygonArray[i].mass * ((float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Y, 2.0) + (float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Z, 2.0));
            Ibody.M12 = Ibody.M12 + physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Y));
            Ibody.M13 = Ibody.M13 + physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Z));

            Ibody.M21 = Ibody.M21 + physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Y));
            Ibody.M22 = Ibody.M22 + physMesh.polygonArray[i].mass * ((float)Math.Pow(physMesh.polygonArray[i].centerOfMass.X, 2.0) + (float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Z, 2.0));
            Ibody.M23 = Ibody.M23 + physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.Y) * (physMesh.polygonArray[i].centerOfMass.Z));

            Ibody.M31 = Ibody.M31 + physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.X) * (physMesh.polygonArray[i].centerOfMass.Z));
            Ibody.M32 = Ibody.M32 + physMesh.polygonArray[i].mass * -1 * ((physMesh.polygonArray[i].centerOfMass.Y) * (physMesh.polygonArray[i].centerOfMass.Y));
            Ibody.M33 = Ibody.M33 + physMesh.polygonArray[i].mass * ((float)Math.Pow(physMesh.polygonArray[i].centerOfMass.X, 2.0) + (float)Math.Pow(physMesh.polygonArray[i].centerOfMass.Y, 2.0));
        }

        physMesh.inertiaTensorInverse = Ibody.Inverse();
    }

    public void calculatePosition(ref logicalObject logObject, float totalMass)
    {
        //Console.WriteLine("Position: " + logObject.Position);
        logObject.Position += h * logObject.linearMomentum / totalMass;
        //Console.WriteLine("linear: " + logObject.linearMomentum);
    }

    public void calculateMatrixRotation(ref logicalObject logObject, Matrix3 inertiaTensorInverse)
    {
        angularVelocity = logObject.Rotation * inertiaTensorInverse * logObject.Rotation.Transverse() * logObject.angularMomentum;
        angularVelocityStar.Star(angularVelocity);

        logObject.Rotation += h * angularVelocityStar * logObject.Rotation;
        logObject.Rotation = logObject.Rotation.GramSchmidt();
        logObject.angularVelocity = angularVelocity;
        //Console.WriteLine("Rot");
    }

    public void calculateQuaternionRotation(ref logicalObject logObject, Matrix3 inertiaTensorInverse)
    {

        angularVelocityQuaternion = new Quaternion();
        tempQuaternion = new Quaternion();
        angularVelocity = logObject.Rotation * inertiaTensorInverse * logObject.Rotation.Transverse() * logObject.angularMomentum;
        angularVelocityStar.Star(angularVelocity);
        angularVelocityQuaternion.W = 0.0f;
        //Console.WriteLine("Quat");



        angularVelocityQuaternion.X = angularVelocity.X;
        angularVelocityQuaternion.Y = angularVelocity.Y;
        angularVelocityQuaternion.Z = angularVelocity.Z;

        tempQuaternion = Quaternion.Multiply(angularVelocityQuaternion, 0.5f);
        tempQuaternion = Quaternion.Multiply(tempQuaternion, logObject.rotationQuaternion);


        //Console.WriteLine(logObject.rotationQuaternion.Length());
        logObject.rotationQuaternion += Quaternion.Multiply(tempQuaternion, h);
        logObject.rotationQuaternion = Quaternion.Normalize(logObject.rotationQuaternion);
        logObject.Rotation.QuaternionToMatrix3(logObject.rotationQuaternion);
        logObject.angularVelocity = angularVelocity;
    }

    public void calculateMomentum(ref logicalObject logObject, Matrix3 oldRotation, int numberOfPolygons, polygon[] polygonArray, float totalMass, Vector3 angularVelocity, Vector3 oldLinearMomentum, Vector3 oldPosition)
    {

        forceI.X = forceI.Y = forceI.Z = 0.0f;
        totalForce.X = totalForce.Y = totalForce.Z = 0.0f;
        torque.X = torque.Y = torque.Z = 0.0f;

        //Console.WriteLine("Linear: " + oldLinearMomentum / totalMass);

        //Console.WriteLine("ny");

        for (int i = 0; i < numberOfPolygons; i++)
        {
            forceI = calculateCentroidForce(polygonArray[i], oldLinearMomentum / totalMass, oldRotation, angularVelocity, oldPosition);
            totalForce += forceI;
            torque += Vector3.Cross(oldRotation * polygonArray[i].centerOfMass, forceI);
            //Console.WriteLine("hoho " + Vector3.Cross(oldRotation * polygonArray[i].centerOfMass, forceI));
        }

        //Console.WriteLine("torque " + torque);


        //Console.WriteLine("Total Force: " + totalForce + " Length: " + totalForce.Length());
        logObject.linearMomentum += h * totalForce;
        //Console.WriteLine("Speed: " + logObject.linearMomentum.Length());
        logObject.angularMomentum += h * torque;
    }

    private void collisionDetection(ref logicalObject logObject)
    {
        if (logObject.Position.Y <= 1)
        {
            logObject.Position.Y = 1.0f;

            //logObject.linearMomentum.Y *= -5f;
            logObject.linearMomentum *= 0.9f;
            logObject.angularMomentum *= 0.9f;
        }

    }

    public float getSpeedValue()
    {
        return speed;
    }


}


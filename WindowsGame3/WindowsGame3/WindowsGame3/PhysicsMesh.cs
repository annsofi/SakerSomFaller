using System;
using System.Collections.Generic;
using System.Collections;
using System.Text;
using Microsoft.Xna.Framework;


    struct PhysicsMesh
    {
        public int numberOfPolygons;
        public Polygon[] polygonArray;

        public float totalMass;
        public Vector3 centerOfMass;
        public Matrix inertiaTensorInverse;
        public float density;

    }


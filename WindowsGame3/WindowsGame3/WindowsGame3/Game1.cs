using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;

namespace WindowsGame3
{
    public class Game1 : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        GraphicsDevice device;

        Effect effect;
        
        Polygon triangle;
        PhysicsMesh physMesh;

        Physics physicsEngine;

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
        }

        protected override void Initialize()
        {
            graphics.PreferredBackBufferWidth = 500;
            graphics.PreferredBackBufferHeight = 500;
            graphics.IsFullScreen = false;
            graphics.ApplyChanges();
            Window.Title = "Saker Som Faller";

            base.Initialize();
        }

        protected override void LoadContent()
        {
            spriteBatch = new SpriteBatch(GraphicsDevice);

            device = graphics.GraphicsDevice;

            effect = Content.Load<Effect>("effects");

            triangle = new Polygon();
            physMesh = new PhysicsMesh();
            physMesh.polygonArray = new Polygon[1];

            triangle.vertices[0] = new Vector3(0f, 1f, 0f);
            triangle.vertices[1] = new Vector3(1f, 1f, 0f);
            triangle.vertices[2] = new Vector3(1f, 0f, 0f);

            physMesh.polygonArray[0] = triangle;
            physMesh.numberOfPolygons = 1;

            physicsEngine = new Physics();

            //physicsEngine.initPhysics(ref physMesh);

            
        }

        protected override void UnloadContent()
        {
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();


            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            device.Clear(Color.Gray);

            effect.CurrentTechnique = effect.Techniques["Pretransformed"];

            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();

                device.DrawUserPrimitives(PrimitiveType.TriangleList, triangle.vertices, 0, 1, VertexPositionColor.VertexDeclaration);
            }

            base.Draw(gameTime);
        }
    }
}
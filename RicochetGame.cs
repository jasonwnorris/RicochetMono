using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using FarseerPhysics;
using FarseerPhysics.Common;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Contacts;
using FarseerPhysics.Dynamics.Joints;
using FarseerPhysics.Factories;

namespace RicochetMono
{
  public class RicochetGame : Game
  {
    private const int c_ScreenWidth = 1280;
    private const int c_ScreenHeight = 720;
    private const float c_Zoom = 50.0f;
    private const float c_InverseZoom = 1.0f / c_Zoom;

    private static readonly Vector2 c_HalfScreenOffset = new Vector2(-c_ScreenWidth / 2.0f, c_ScreenHeight / 2.0f);

    private GraphicsDeviceManager m_GraphicsDeviceManager;
    private World m_World;
    private Body m_PaddleBody;
    private FixedMouseJoint m_MouseJoint;
    private bool m_IsGrabbed;
    private DebugView m_DebugView;
    private Matrix m_ProjectionMatrix;
    private Matrix m_ViewMatrix;

    public RicochetGame()
    {
      m_GraphicsDeviceManager = new GraphicsDeviceManager(this);
      m_GraphicsDeviceManager.PreferredBackBufferWidth = c_ScreenWidth;
      m_GraphicsDeviceManager.PreferredBackBufferHeight = c_ScreenHeight;

      m_IsGrabbed = false;

      m_ProjectionMatrix = Matrix.CreateOrthographic(c_ScreenWidth, c_ScreenHeight, 0.001f, 1000.0f);
      m_ViewMatrix = Matrix.CreateScale(c_Zoom);

      Content.RootDirectory = "Content";

      IsMouseVisible = true;
    }

    protected override void Initialize()
    {
      m_World = new World(Vector2.Zero);

      m_DebugView = new DebugView(m_World, GraphicsDevice);
      m_DebugView.AppendFlags(DebugViewFlags.Shape);
      m_DebugView.AppendFlags(DebugViewFlags.Controllers);
      m_DebugView.AppendFlags(DebugViewFlags.Joint);
      m_DebugView.AppendFlags(DebugViewFlags.ContactPoints);
      m_DebugView.AppendFlags(DebugViewFlags.ContactNormals);

      Body ballBody = BodyFactory.CreateBody(m_World);
      ballBody.BodyType = BodyType.Dynamic;

      Fixture ballFixture = FixtureFactory.AttachCircle(0.5f, 1.0f, ballBody);
      ballFixture.Friction = 0.85f;
      ballFixture.Restitution = 0.75f;

      Body floorBody = BodyFactory.CreateBody(m_World);
      floorBody.BodyType = BodyType.Static;

      Fixture floorFixture = FixtureFactory.AttachRectangle(3.5f, 0.5f, 1.0f, new Vector2(0.0f, -5.0f), floorBody);
      floorFixture.Friction = 0.5f;
      floorFixture.Restitution = 0.75f;

      m_PaddleBody = BodyFactory.CreateBody(m_World, new Vector2(2.5f, 0.0f));
      m_PaddleBody.BodyType = BodyType.Dynamic;

      Fixture paddleFixture = FixtureFactory.AttachCircle(0.25f, 1.0f, m_PaddleBody);
      paddleFixture.Friction = 0.5f;
      paddleFixture.Restitution = 0.75f;

      Vertices vertices = new Vertices(new List<Vector2>()
      {
        new Vector2(-9.0f, 7.0f),
        new Vector2(-9.0f, -7.0f),
        new Vector2(9.0f, -7.0f),
        new Vector2(9.0f, 7.0f),
        new Vector2(-9.0f, 7.0f)
      });

      Body borderBody = BodyFactory.CreateChainShape(m_World, vertices);
      borderBody.BodyType = BodyType.Static;

      Fixture borderFixture = FixtureFactory.AttachChainShape(vertices, borderBody);
      borderFixture.Friction = 0.0f;
      borderFixture.Restitution = 1.0f;

      base.Initialize();
    }

    protected override void LoadContent()
    {
    }

    protected override void UnloadContent()
    {
      m_DebugView.Dispose();
    }

    protected override void Update(GameTime p_GameTime)
    {
      GamePadState gamePadState = GamePad.GetState(PlayerIndex.One);
      KeyboardState keyboardState = Keyboard.GetState();
      MouseState mouseState = Mouse.GetState();

      if (gamePadState.IsButtonDown(Buttons.Back) || keyboardState.IsKeyDown(Keys.Escape))
      {
        Exit();
      }

      if (mouseState.LeftButton == ButtonState.Pressed)
      {
        if (!m_IsGrabbed)
        {
          Vector2 position = TransformToWorldSpace(mouseState.Position);

          Fixture fixture = m_World.TestPoint(position);
          if (fixture != null)
          {
            m_MouseJoint = new FixedMouseJoint(fixture.Body, position);
            m_World.AddJoint(m_MouseJoint);
            m_IsGrabbed = true;
          }
        }
        else
        {
          m_MouseJoint.WorldAnchorB = TransformToWorldSpace(mouseState.Position);
        }
      }
      else if (mouseState.LeftButton == ButtonState.Released)
      {
        if (m_IsGrabbed)
        {
          m_World.RemoveJoint(m_MouseJoint);
          m_MouseJoint = null;
          m_IsGrabbed = false;
        }
      }

      m_World.Step(p_GameTime.ElapsedGameTime.Milliseconds / 1000.0f);

      base.Update(p_GameTime);
    }

    protected override void Draw(GameTime p_GameTime)
    {
      GraphicsDevice.Clear(Color.Black);

      m_DebugView.RenderDebugData(ref m_ProjectionMatrix, ref m_ViewMatrix);

      base.Draw(p_GameTime);
    }

    private Vector2 TransformToWorldSpace(Point p_Point)
    {
      return (c_HalfScreenOffset + new Vector2(p_Point.X, -p_Point.Y)) * c_InverseZoom;
    }
  }
}

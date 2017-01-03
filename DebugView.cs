using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using FarseerPhysics;
using FarseerPhysics.Collision;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Common;
using FarseerPhysics.Controllers;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Contacts;
using FarseerPhysics.Dynamics.Joints;

namespace RicochetMono
{
  public class DebugView : DebugViewBase, IDisposable
  {
    private struct ContactPoint
    {
      public Vector2 Normal;
      public Vector2 Position;
      public PointState State;
    }

    private const float c_AxisScale = 0.4f;
    private const int c_CircleSegments = 32;
    private const int c_MaxContactPoints = 2048;
    private const float c_OutlinedOpacity = 0.5f;
    private const float c_PointSize = 0.1f;
    private const double c_RadianStep = Math.PI * 2.0 / c_CircleSegments;

    public Color DefaultShapeColor = new Color(0.9f, 0.7f, 0.7f);
    public Color InactiveShapeColor = new Color(0.5f, 0.5f, 0.3f);
    public Color KinematicShapeColor = new Color(0.5f, 0.5f, 0.9f);
    public Color SleepingShapeColor = new Color(0.6f, 0.6f, 0.6f);
    public Color StaticShapeColor = new Color(0.5f, 0.9f, 0.5f);
    public Color JointColor = new Color(0.5f, 0.8f, 0.8f);
    public Color ContactPointAddColor = new Color(0.3f, 0.95f, 0.3f);
    public Color ContactPointPersistColor = new Color(0.3f, 0.3f, 0.95f);
    public Color ContactNormalColor = new Color(0.4f, 0.9f, 0.4f);
    public Color AABBColor = new Color(0.9f, 0.3f, 0.9f);

    private bool m_IsEnabled;

    private PrimitiveBatch m_PrimitiveBatch;
    private Vector2[] m_TempWorkVertices;
    private ContactPoint[] m_ContactPoints;
    private int m_ContactPointCount;

    public bool IsEnabled
    {
      get { return m_IsEnabled; }
      set { m_IsEnabled = value; }
    }

    public DebugView(World p_World, GraphicsDevice p_GraphicsDevice) : base(p_World)
    {
      m_IsEnabled = true;

      m_PrimitiveBatch = new PrimitiveBatch(p_GraphicsDevice, 1000);
      m_TempWorkVertices = new Vector2[Settings.MaxPolygonVertices];
      m_ContactPoints = new ContactPoint[c_MaxContactPoints];

      p_World.ContactManager.PreSolve += PreSolve;
    }

    public void Dispose()
    {
      World.ContactManager.PreSolve -= PreSolve;
    }

    public void RenderDebugData(ref Matrix p_ProjectionMatrix, ref Matrix p_ViewMatrix)
    {
      if (m_IsEnabled)
      {
        m_PrimitiveBatch.Begin(ref p_ProjectionMatrix, ref p_ViewMatrix);

        if ((Flags & DebugViewFlags.Shape) == DebugViewFlags.Shape)
        {
          DrawShapes();
        }

        if ((Flags & DebugViewFlags.ContactPoints) == DebugViewFlags.ContactPoints)
        {
          DrawContactPoints();
        }

        if ((Flags & DebugViewFlags.PolygonPoints) == DebugViewFlags.PolygonPoints)
        {
          DrawPolygonPoints();
        }

        if ((Flags & DebugViewFlags.Joint) == DebugViewFlags.Joint)
        {
          DrawJoints();
        }

        if ((Flags & DebugViewFlags.AABB) == DebugViewFlags.AABB)
        {
          DrawAABBs();
        }

        if ((Flags & DebugViewFlags.CenterOfMass) == DebugViewFlags.CenterOfMass)
        {
          DrawCenterOfMasses();
        }

        if ((Flags & DebugViewFlags.Controllers) == DebugViewFlags.Controllers)
        {
          DrawControllers();
        }

        m_PrimitiveBatch.End();
      }
    }

    public override void DrawPolygon(Vector2[] p_Vertices, int p_Count, float p_Red, float p_Green, float p_Blue, bool p_IsClosed = true)
    {
      DrawPolygon(p_Vertices, p_Count, new Color(p_Red, p_Green, p_Blue), p_IsClosed);
    }

    public void DrawPolygon(Vector2[] p_Vertices, int p_Count, Color p_Color, bool p_IsClosed = true)
    {
      for (int i = 0; i < p_Count - 1; ++i)
      {
        DrawSegment(p_Vertices[i], p_Vertices[i + 1], p_Color);
      }

      if (p_IsClosed)
      {
        DrawSegment(p_Vertices[p_Count - 1], p_Vertices[0], p_Color);
      }
    }

    public override void DrawSolidPolygon(Vector2[] p_Vertices, int p_Count, float p_Red, float p_Green, float p_Blue)
    {
      DrawSolidPolygon(p_Vertices, p_Count, new Color(p_Red, p_Green, p_Blue));
    }

    public void DrawSolidPolygon(Vector2[] p_Vertices, int p_Count, Color p_Color, bool p_IsOutlined = true)
    {
      if (p_Count == 2)
      {
        DrawPolygon(p_Vertices, p_Count, p_Color);
      }
      else
      {
        Color color = p_Color * (p_IsOutlined ? c_OutlinedOpacity : 1.0f);

        for (int i = 1; i < p_Count - 1; ++i)
        {
          DrawTriangle(p_Vertices[0], p_Vertices[i], p_Vertices[i + 1], color);
        }

        if (p_IsOutlined)
        {
          DrawPolygon(p_Vertices, p_Count, p_Color);
        }
      }
    }

    public override void DrawCircle(Vector2 p_Center, float p_Radius, float p_Red, float p_Green, float p_Blue)
    {
      DrawCircle(p_Center, p_Radius, new Color(p_Red, p_Green, p_Blue));
    }

    public void DrawCircle(Vector2 p_Center, float p_Radius, Color p_Color)
    {
      double theta = 0.0;
      Vector2[] vertices = new Vector2[c_CircleSegments];

      for (int i = 0; i < c_CircleSegments; ++i)
      {
        float cosTheta = (float)Math.Cos(theta);
        float sinTheta = (float)Math.Sin(theta);

        vertices[i] = p_Center + p_Radius * new Vector2(cosTheta, sinTheta);

        theta += c_RadianStep;
      }

      for (int i = 0; i < c_CircleSegments; ++i)
      {
        DrawSegment(vertices[i], vertices[(i + 1) % c_CircleSegments], p_Color);
      }
    }

    public override void DrawSolidCircle(Vector2 p_Center, float p_Radius, Vector2 p_Axis, float p_Red, float p_Green, float p_Blue)
    {
      DrawSolidCircle(p_Center, p_Radius, p_Axis, new Color(p_Red, p_Green, p_Blue));
    }

    public void DrawSolidCircle(Vector2 p_Center, float p_Radius, Vector2 p_Axis, Color p_Color)
    {
      double theta = 0.0;
      Color color = p_Color * 0.5f;
      Vector2[] vertices = new Vector2[c_CircleSegments];

      for (int i = 0; i < c_CircleSegments; ++i)
      {
        float cosTheta = (float)Math.Cos(theta);
        float sinTheta = (float)Math.Sin(theta);

        vertices[i] = p_Center + p_Radius * new Vector2(cosTheta, sinTheta);

        theta += c_RadianStep;
      }

      for (int i = 0; i < c_CircleSegments; ++i)
      {
        DrawTriangle(p_Center, vertices[i], vertices[(i + 1) % c_CircleSegments], color);
      }

      DrawCircle(p_Center, p_Radius, p_Color);
      DrawSegment(p_Center, p_Center + p_Axis * p_Radius, p_Color);
    }

    public override void DrawSegment(Vector2 p_StartPoint, Vector2 p_FinishPoint, float p_Red, float p_Green, float p_Blue)
    {
      DrawSegment(p_StartPoint, p_FinishPoint, new Color(p_Red, p_Green, p_Blue));
    }

    public void DrawSegment(Vector2 p_StartPoint, Vector2 p_FinishPoint, Color p_Color)
    {
      m_PrimitiveBatch.AddLineSegment(p_StartPoint, p_FinishPoint, p_Color);
    }

    public void DrawTriangle(Vector2 p_PointA, Vector2 p_PointB, Vector2 p_PointC, Color p_Color)
    {
      m_PrimitiveBatch.AddTriangle(p_PointA, p_PointB, p_PointC, p_Color);
    }

    public override void DrawTransform(ref Transform p_Transform)
    {
      Vector2 pointCenter = p_Transform.p;
      Vector2 pointX = pointCenter + c_AxisScale * p_Transform.q.GetXAxis();
      Vector2 pointY = pointCenter + c_AxisScale * p_Transform.q.GetYAxis();

      DrawSegment(pointCenter, pointX, Color.Red);
      DrawSegment(pointCenter, pointY, Color.Green);
    }

    private void DrawPoint(Vector2 p_Point, float p_Size, Color p_Color)
    {
      float halfSize = p_Size / 2.0f;

      Vector2[] vertices = new Vector2[4]
      {
        p_Point + new Vector2(-halfSize, -halfSize),
        p_Point + new Vector2(halfSize, -halfSize),
        p_Point + new Vector2(halfSize, halfSize),
        p_Point + new Vector2(-halfSize, halfSize)
      };

      DrawSolidPolygon(vertices, 4, p_Color, true);
    }

    private void DrawShapes()
    {
      foreach (Body body in World.BodyList)
      {
        Transform transform;
        body.GetTransform(out transform);

        foreach (Fixture fixture in body.FixtureList)
        {
          if (!body.Enabled)
          {
            DrawShape(fixture, transform, InactiveShapeColor);
          }
          else if (body.BodyType == BodyType.Static)
          {
            DrawShape(fixture, transform, StaticShapeColor);
          }
          else if (body.BodyType == BodyType.Kinematic)
          {
            DrawShape(fixture, transform, KinematicShapeColor);
          }
          else if (!body.Awake)
          {
            DrawShape(fixture, transform, SleepingShapeColor);
          }
          else
          {
            DrawShape(fixture, transform, DefaultShapeColor);
          }
        }
      }
    }

    private void DrawShape(Fixture p_Fixture, Transform p_Transform, Color p_Color)
    {
      switch (p_Fixture.Shape.ShapeType)
      {
        case ShapeType.Circle:
          {
            CircleShape circleShape = (CircleShape)p_Fixture.Shape;

            Vector2 center = MathUtils.Mul(ref p_Transform, circleShape.Position);
            Vector2 axis = MathUtils.Mul(p_Transform.q, Vector2.UnitX);

            DrawSolidCircle(center, circleShape.Radius, axis, p_Color);
          }
          break;
        case ShapeType.Polygon:
          {
            PolygonShape polygonShape = (PolygonShape)p_Fixture.Shape;

            for (int i = 0; i < polygonShape.Vertices.Count; ++i)
            {
              m_TempWorkVertices[i] = MathUtils.Mul(ref p_Transform, polygonShape.Vertices[i]);
            }

            DrawSolidPolygon(m_TempWorkVertices, polygonShape.Vertices.Count, p_Color);
          }
          break;
        case ShapeType.Edge:
          {
            EdgeShape edgeShape = (EdgeShape)p_Fixture.Shape;

            Vector2 vertexA = MathUtils.Mul(ref p_Transform, edgeShape.Vertex1);
            Vector2 vertexB = MathUtils.Mul(ref p_Transform, edgeShape.Vertex2);

            DrawSegment(vertexA, vertexB, p_Color);
          }
          break;
        case ShapeType.Chain:
          {
            ChainShape chainShape = (ChainShape)p_Fixture.Shape;

            for (int i = 0; i < chainShape.Vertices.Count - 1; ++i)
            {
              Vector2 vertexA = MathUtils.Mul(ref p_Transform, chainShape.Vertices[i]);
              Vector2 vertexB = MathUtils.Mul(ref p_Transform, chainShape.Vertices[i + 1]);

              DrawSegment(vertexA, vertexB, p_Color);
            }
          }
          break;
      }
    }

    private void DrawContactPoints()
    {
      for (int i = 0; i < m_ContactPointCount; ++i)
      {
        ContactPoint contactPoint = m_ContactPoints[i];

        switch (contactPoint.State)
        {
          case PointState.Add:
            DrawPoint(contactPoint.Position, c_PointSize, ContactPointAddColor);
            break;
          case PointState.Persist:
            DrawPoint(contactPoint.Position, c_PointSize, ContactPointPersistColor);
            break;
        }

        if ((Flags & DebugViewFlags.ContactNormals) == DebugViewFlags.ContactNormals)
        {
          Vector2 pointA = contactPoint.Position;
          Vector2 pointB = pointA + c_AxisScale * contactPoint.Normal;

          DrawSegment(pointA, pointB, ContactNormalColor);
        }
      }

      m_ContactPointCount = 0;
    }

    private void DrawPolygonPoints()
    {
      foreach (Body body in World.BodyList)
      {
        foreach (Fixture fixture in body.FixtureList)
        {
          PolygonShape polygon = fixture.Shape as PolygonShape;
          if (polygon != null)
          {
            Transform transform;
            body.GetTransform(out transform);

            for (int i = 0; i < polygon.Vertices.Count; ++i)
            {
              DrawPoint(MathUtils.Mul(ref transform, polygon.Vertices[i]), c_PointSize, Color.Red);
            }
          }
        }
      }
    }

    private void DrawJoints()
    {
      foreach (Joint joint in World.JointList)
      {
        DrawJoint(joint);
      }
    }

    private void DrawJoint(Joint joint)
    {
      if (joint.Enabled)
      {
        Body bodyA = joint.BodyA;
        Body bodyB = joint.BodyB;

        Transform transformA;
        bodyA.GetTransform(out transformA);

        Vector2 positionA = transformA.p;
        Vector2 positionB = Vector2.Zero;

        if (!joint.IsFixedType())
        {
          Transform transformB;
          bodyB.GetTransform(out transformB);

          positionB = transformB.p;
        }

        Vector2 anchorA = joint.WorldAnchorA;
        Vector2 anchorB = joint.WorldAnchorB;

        switch (joint.JointType)
        {
          case JointType.Distance:
            DrawSegment(anchorA, anchorB, JointColor);
            break;
          case JointType.Pulley:
            PulleyJoint pulleyJoint = (PulleyJoint)joint;
            Vector2 pointA = bodyA.GetWorldPoint(pulleyJoint.LocalAnchorA);
            Vector2 pointB = bodyB.GetWorldPoint(pulleyJoint.LocalAnchorB);
            DrawSegment(anchorA, anchorB, JointColor);
            DrawSegment(anchorA, pointA, JointColor);
            DrawSegment(anchorB, pointB, JointColor);
            break;
          case JointType.FixedMouse:
            DrawPoint(anchorA, 0.5f, Color.LimeGreen);
            DrawSegment(anchorA, anchorB, Color.LightGray);
            break;
          case JointType.Revolute:
            DrawSegment(positionA, anchorA, JointColor);
            DrawSegment(anchorA, anchorB, JointColor);
            DrawSegment(positionB, anchorB, JointColor);
            DrawSolidCircle(anchorB, c_PointSize, Vector2.Zero, Color.Red);
            DrawSolidCircle(anchorA, c_PointSize, Vector2.Zero, Color.Blue);
            break;
          case JointType.FixedAngle:
            // Should not draw anything.
            break;
          case JointType.FixedRevolute:
            DrawSegment(positionA, anchorA, JointColor);
            DrawSolidCircle(anchorA, c_PointSize, Vector2.Zero, Color.Pink);
            break;
          case JointType.FixedLine:
            DrawSegment(positionA, anchorA, JointColor);
            DrawSegment(anchorA, anchorB, JointColor);
            break;
          case JointType.FixedDistance:
            DrawSegment(positionA, anchorA, JointColor);
            DrawSegment(anchorA, anchorB, JointColor);
            break;
          case JointType.FixedPrismatic:
            DrawSegment(positionA, anchorA, JointColor);
            DrawSegment(anchorA, anchorB, JointColor);
            break;
          case JointType.Gear:
            DrawSegment(positionA, positionB, JointColor);
            break;
          default:
            DrawSegment(positionA, anchorA, JointColor);
            DrawSegment(anchorA, anchorB, JointColor);
            DrawSegment(positionB, anchorB, JointColor);
            break;
        }
      }
    }

    private void DrawAABBs()
    {
      IBroadPhase broadPhase = World.ContactManager.BroadPhase;

      foreach (Body body in World.BodyList)
      {
        if (body.Enabled)
        {
          foreach (Fixture fixture in body.FixtureList)
          {
            for (int i = 0; i < fixture.ProxyCount; ++i)
            {
              AABB aabb;
              broadPhase.GetFatAABB(fixture.Proxies[i].ProxyId, out aabb);

              DrawAABB(ref aabb, AABBColor);
            }
          }
        }
      }
    }

    private void DrawAABB(ref AABB p_AABB, Color p_Color)
    {
      Vector2[] vertices = new Vector2[4] {
        new Vector2(p_AABB.LowerBound.X, p_AABB.LowerBound.Y),
        new Vector2(p_AABB.UpperBound.X, p_AABB.LowerBound.Y),
        new Vector2(p_AABB.UpperBound.X, p_AABB.UpperBound.Y),
        new Vector2(p_AABB.LowerBound.X, p_AABB.UpperBound.Y)
      };

      DrawPolygon(vertices, 4, p_Color);
    }

    private void DrawCenterOfMasses()
    {
      foreach (Body body in World.BodyList)
      {
        Transform transform;
        body.GetTransform(out transform);

        transform.p = body.WorldCenter;

        DrawTransform(ref transform);
      }
    }

    private void DrawControllers()
    {
      foreach (Controller controller in World.ControllerList)
      {
        BuoyancyController buoyancy = controller as BuoyancyController;
        if (buoyancy != null)
        {
          AABB container = buoyancy.Container;

          DrawAABB(ref container, Color.LightBlue);
        }
      }
    }

    private void PreSolve(Contact p_Contact, ref Manifold p_OldManifold)
    {
      if ((Flags & DebugViewFlags.ContactPoints) == DebugViewFlags.ContactPoints)
      {
        Manifold manifold = p_Contact.Manifold;

        if (manifold.PointCount > 0)
        {
          FixedArray2<PointState> stateA;
          FixedArray2<PointState> stateB;
          Collision.GetPointStates(out stateA, out stateB, ref p_OldManifold, ref manifold);

          FixedArray2<Vector2> points;
          Vector2 normal;
          p_Contact.GetWorldManifold(out normal, out points);

          for (int i = 0; i < manifold.PointCount && m_ContactPointCount < c_MaxContactPoints; ++i)
          {
            if (p_Contact.FixtureA == null)
            {
              m_ContactPoints[i] = new ContactPoint();
            }

            ContactPoint contactPoint = m_ContactPoints[m_ContactPointCount];
            contactPoint.Position = points[i];
            contactPoint.Normal = normal;
            contactPoint.State = stateB[i];

            m_ContactPoints[m_ContactPointCount] = contactPoint;

            ++m_ContactPointCount;
          }
        }
      }
    }
  }
}

using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace RicochetMono
{
  public class PrimitiveBatch : IDisposable
  {
    private const int c_DefaultBufferSize = 500;
    private const float c_Depth = -0.1f;

    private GraphicsDevice m_GraphicsDevice;
    private BasicEffect m_BasicEffect;
    private bool m_IsReady;
    private bool m_IsDisposed;
    private VertexPositionColor[] m_LineSegmentVertices;
    private int m_LineSegmentVertexCount;
    private VertexPositionColor[] m_TriangleVertices;
    private int m_TriangleVertexCount;

    public bool IsReady
    {
      get { return m_IsReady; }
    }

    public PrimitiveBatch(GraphicsDevice p_GraphicsDevice, int p_BufferSize = c_DefaultBufferSize)
    {
      if (p_GraphicsDevice == null)
      {
        throw new ArgumentNullException("GraphicsDevice");
      }

      m_GraphicsDevice = p_GraphicsDevice;

      m_TriangleVertices = new VertexPositionColor[p_BufferSize - p_BufferSize % 3];
      m_LineSegmentVertices = new VertexPositionColor[p_BufferSize - p_BufferSize % 2];

      m_BasicEffect = new BasicEffect(p_GraphicsDevice);
      m_BasicEffect.VertexColorEnabled = true;
    }

    public void Dispose()
    {
      Dispose(true);

      GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool p_IsDisposing)
    {
      if (p_IsDisposing && !m_IsDisposed)
      {
        if (m_BasicEffect != null)
        {
          m_BasicEffect.Dispose();
        }

        m_IsDisposed = true;
      }
    }

    public void Begin(ref Matrix p_ProjectionMatrix, ref Matrix p_ViewMatrix)
    {
      if (m_IsReady)
      {
        throw new InvalidOperationException("End must be called before Begin can be called again.");
      }

      m_BasicEffect.Projection = p_ProjectionMatrix;
      m_BasicEffect.View = p_ViewMatrix;
      m_BasicEffect.CurrentTechnique.Passes[0].Apply();

      m_IsReady = true;
    }

    public void AddLineSegment(Vector2 p_VertexA, Vector2 p_VertexB, Color p_Color)
    {
      if (!m_IsReady)
      {
        throw new InvalidOperationException("Begin must be called before primitives can be added.");
      }

      if (m_LineSegmentVertexCount >= m_LineSegmentVertices.Length)
      {
        FlushLines();
      }

      AddVertex(p_VertexA, p_Color, PrimitiveType.LineList);
      AddVertex(p_VertexB, p_Color, PrimitiveType.LineList);
    }

    public void AddTriangle(Vector2 p_VertexA, Vector2 p_VertexB, Vector2 p_VertexC, Color p_Color)
    {
      if (!m_IsReady)
      {
        throw new InvalidOperationException("Begin must be called before primitives can be added.");
      }

      if (m_TriangleVertexCount >= m_TriangleVertices.Length)
      {
        FlushTriangles();
      }

      AddVertex(p_VertexA, p_Color, PrimitiveType.TriangleList);
      AddVertex(p_VertexB, p_Color, PrimitiveType.TriangleList);
      AddVertex(p_VertexC, p_Color, PrimitiveType.TriangleList);
    }

    public void End()
    {
      if (!m_IsReady)
      {
        throw new InvalidOperationException("Begin must be called before End can be called.");
      }

      m_GraphicsDevice.RasterizerState = RasterizerState.CullNone;
      m_GraphicsDevice.DepthStencilState = DepthStencilState.Default;
      m_GraphicsDevice.SamplerStates[0] = SamplerState.AnisotropicClamp;

      FlushTriangles();
      FlushLines();

      m_IsReady = false;
    }

    private void AddVertex(Vector2 p_Vertex, Color p_Color, PrimitiveType p_PrimitiveType)
    {
      switch (p_PrimitiveType)
      {
        case PrimitiveType.LineList:
          m_LineSegmentVertices[m_LineSegmentVertexCount].Position = new Vector3(p_Vertex, c_Depth);
          m_LineSegmentVertices[m_LineSegmentVertexCount].Color = p_Color;
          ++m_LineSegmentVertexCount;
          break;
        case PrimitiveType.TriangleList:
          m_TriangleVertices[m_TriangleVertexCount].Position = new Vector3(p_Vertex, c_Depth);
          m_TriangleVertices[m_TriangleVertexCount].Color = p_Color;
          ++m_TriangleVertexCount;
          break;
      }
    }

    private void FlushLines()
    {
      if (m_LineSegmentVertexCount >= 2)
      {
        m_GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, m_LineSegmentVertices, 0, m_LineSegmentVertexCount / 2);

        m_LineSegmentVertexCount = 0;
      }
    }

    private void FlushTriangles()
    {
      if (m_TriangleVertexCount >= 3)
      {
        m_GraphicsDevice.DrawUserPrimitives(PrimitiveType.TriangleList, m_TriangleVertices, 0, m_TriangleVertexCount / 3);

        m_TriangleVertexCount = 0;
      }
    }
  }
}

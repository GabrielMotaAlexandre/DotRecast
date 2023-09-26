using System;
using System.Numerics;

using static DotRecast.Recast.Toolset.Gizmos.RcGizmoHelper;

namespace DotRecast.Recast.Toolset.Gizmos
{
    public class RcCapsuleGizmo : IRcGizmoMeshFilter
    {
        public readonly float[] vertices;
        public readonly int[] triangles;
        public readonly float[] center;
        public readonly float[] gradient;

        public RcCapsuleGizmo(Vector3 start, Vector3 end, float radius)
        {
            center = new float[]
            {
                0.5f * (start.X + end.X), 0.5f * (start.Y + end.Y),
                0.5f * (start.Z + end.Z)
            };
            Vector3 axis = new(end.X - start.X, end.Y - start.Y, end.Z - start.Z);
            Vector3[] normals = new Vector3[3];
            normals[1] = new Vector3(end.X - start.X, end.Y - start.Y, end.Z - start.Z);
            Vector3Extensions.Normalize(ref normals[1]);
            normals[0] = GetSideVector(axis);
            normals[2] = Vector3.Zero;
            Vector3Extensions.Cross(ref normals[2], normals[0], normals[1]);
            Vector3Extensions.Normalize(ref normals[2]);
            triangles = GenerateSphericalTriangles();
            var trX = new Vector3(normals[0].X, normals[1].X, normals[2].X);
            var trY = new Vector3(normals[0].Y, normals[1].Y, normals[2].Y);
            var trZ = new Vector3(normals[0].Z, normals[1].Z, normals[2].Z);
            float[] spVertices = GenerateSphericalVertices();
            float halfLength = 0.5f * axis.Length();
            vertices = new float[spVertices.Length];
            gradient = new float[spVertices.Length / 3];
            Vector3 v = new();
            for (int i = 0; i < spVertices.Length; i += 3)
            {
                float offset = (i >= spVertices.Length / 2) ? -halfLength : halfLength;
                float x = radius * spVertices[i];
                float y = radius * spVertices[i + 1] + offset;
                float z = radius * spVertices[i + 2];
                vertices[i] = x * trX.X + y * trX.Y + z * trX.Z + center[0];
                vertices[i + 1] = x * trY.X + y * trY.Y + z * trY.Z + center[1];
                vertices[i + 2] = x * trZ.X + y * trZ.Y + z * trZ.Z + center[2];
                v.X = vertices[i] - center[0];
                v.Y = vertices[i + 1] - center[1];
                v.Z = vertices[i + 2] - center[2];
                Vector3Extensions.Normalize(ref v);
                gradient[i / 3] = Math.Clamp(0.57735026f * (v.X + v.Y + v.Z), -1, 1);
            }
        }

        private static Vector3 GetSideVector(Vector3 axis)
        {
            Vector3 side = new(1, 0, 0);
            if (axis.X > 0.8)
            {
                side = new Vector3(0, 0, 1);
            }

            Vector3 forward = new();
            Vector3Extensions.Cross(ref forward, side, axis);
            Vector3Extensions.Cross(ref side, axis, forward);
            Vector3Extensions.Normalize(ref side);
            return side;
        }
    }
}
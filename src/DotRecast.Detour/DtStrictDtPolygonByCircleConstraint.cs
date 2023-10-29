using System;
using System.Numerics;

namespace DotRecast.Detour
{
    /**
     * Calculate the intersection between a polygon and a circle. A dodecagon is used as an approximation of the circle.
     */
    public class DtStrictDtPolygonByCircleConstraint : IDtPolygonByCircleConstraint
    {
        private const int CIRCLE_SEGMENTS = 12;
        private static readonly float[] UnitCircle = MakeUnitCircle();

        public static readonly IDtPolygonByCircleConstraint Shared = new DtStrictDtPolygonByCircleConstraint();

        private DtStrictDtPolygonByCircleConstraint()
        {
        }

        private static float[] MakeUnitCircle()
        {
            var temp = new float[CIRCLE_SEGMENTS * 3];
            for (int i = 0; i < CIRCLE_SEGMENTS; i++)
            {
                double a = i * MathF.PI * 2 / CIRCLE_SEGMENTS;
                temp[3 * i] = MathF.Cos(a);
                temp[3 * i + 1] = 0;
                temp[3 * i + 2] = (float)-Math.Sin(a);
            }

            return temp;
        }

        public ReadOnlySpan<float> Apply(ReadOnlySpan<float> verts, Vector3 center, float radius)
        {
            float radiusSqr = radius * radius;
            int outsideVertex = -1;
            for (int pv = 0; pv < verts.Length; pv += 3)
            {
                if (Vector3Extensions.Dist2DSqr(center, verts.UnsafeAs<float, Vector3>(pv)) > radiusSqr)
                {
                    outsideVertex = pv;
                    break;
                }
            }

            if (outsideVertex == -1)
            {
                // polygon inside circle
                return verts;
            }

            var qCircle = Circle(center, radius);
            var intersection = ConvexConvexIntersection.Intersect(verts, qCircle);
            if (intersection is null && DtUtils.PointInPolygon(center, verts, verts.Length / 3))
            {
                // circle inside polygon
                return qCircle;
            }

            return intersection;
        }


        private static float[] Circle(Vector3 center, float radius)
        {
            float[] circle = new float[12 * 3];
            for (int i = 0; i < CIRCLE_SEGMENTS * 3; i += 3)
            {
                circle[i] = UnitCircle[i] * radius + center.X;
                circle[i + 1] = center.Y;
                circle[i + 2] = UnitCircle[i + 2] * radius + center.Z;
            }

            return circle;
        }
    }
}
using System;
using System.Numerics;

namespace DotRecast.Detour
{
    /**
     * Calculate the intersection between a polygon and a circle. A dodecagon is used as an approximation of the circle.
     */
    public struct DtStrictDtPolygonByCircleConstraint : IDtPolygonByCircleConstraint
    {
        private const int CIRCLE_SEGMENTS = 12;
        private static readonly Vector3[] UnitCircle = MakeUnitCircle();

        public static readonly DtStrictDtPolygonByCircleConstraint Shared = new();

        private static Vector3[] MakeUnitCircle()
        {
            var temp = new Vector3[CIRCLE_SEGMENTS];
            for (int i = 0; i < CIRCLE_SEGMENTS; i++)
            {
                var a = i * Math.PI * 2 / CIRCLE_SEGMENTS;
                temp[i] = new Vector3((float)Math.Cos(a), 0, (float)-Math.Sin(a));
            }

            return temp;
        }

        public readonly ReadOnlySpan<Vector3> Apply(ReadOnlySpan<Vector3> verts, Vector3 center, float radius)
        {
            float radiusSqr = radius * radius;
            int outsideVertex = -1;
            for (int pv = 0; pv < verts.Length; pv++)
            {
                if (Vector3Extensions.Dist2DSqr(center, verts[pv]) > radiusSqr)
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
            if (intersection == default && DtUtils.PointInPolygon(center, verts, verts.Length))
            {
                // circle inside polygon
                return qCircle;
            }

            return intersection;
        }


        private static Vector3[] Circle(Vector3 center, float radius)
        {
            var circle = new Vector3[12];
            for (int i = 0; i < CIRCLE_SEGMENTS; i++)
            {
                circle[i] = center + UnitCircle[i] * radius;
            }

            return circle;
        }
    }
}
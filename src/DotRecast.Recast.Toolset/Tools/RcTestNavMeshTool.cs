using System;
using System.Collections.Generic;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcTestNavMeshTool : IRcToolable
    {
        public const int MAX_POLYS = 256;
        public const int MAX_SMOOTH = 2048;


        public RcTestNavMeshTool()
        {
        }

        public string GetName()
        {
            return "Test Navmesh";
        }

        public static DtStatus FindFollowPath(DtNavMesh navMesh, DtNavMeshQuery navQuery, long startRef, long endRef, Vector3 startPt, Vector3 endPt, IDtQueryFilter filter, bool enableRaycast,
            ref List<long> polys, ref List<Vector3> smoothPath)
        {
            if (startRef is 0 || endRef is 0)
            {
                polys?.Clear();
                smoothPath?.Clear();

                return DtStatus.DT_FAILURE;
            }

            polys ??= new List<long>();
            smoothPath ??= new List<Vector3>();

            polys.Clear();
            smoothPath.Clear();

            var opt = new DtFindPathOption(enableRaycast ? DtNavMeshQuery.DT_FINDPATH_ANY_ANGLE : 0, float.MaxValue);
            navQuery.FindPath(startRef, endRef, startPt, endPt, filter, ref polys, opt);
            if (0 >= polys.Count)
                return DtStatus.DT_FAILURE;

            // Iterate over the path to find smooth path on the detail mesh surface.
            navQuery.ClosestPointOnPoly(startRef, startPt, out var iterPos, out var _);
            navQuery.ClosestPointOnPoly(polys[^1], endPt, out var targetPos, out var _);

            float STEP_SIZE = 0.5f;
            float SLOP = 0.01f;

            smoothPath.Clear();
            smoothPath.Add(iterPos);
            var visited = new List<long>();

            // Move towards target a small advancement at a time until target reached or
            // when ran out of memory to store the path.
            while (0 < polys.Count && smoothPath.Count < MAX_SMOOTH)
            {
                // Find location to steer towards.
                if (!PathUtils.GetSteerTarget(navQuery, iterPos, targetPos, SLOP,
                        polys, out var steerPos, out var steerPosFlag, out var steerPosRef))
                {
                    break;
                }

                bool endOfPath = (steerPosFlag & DtNavMeshQuery.DT_STRAIGHTPATH_END) != 0;
                bool offMeshConnection = (steerPosFlag & DtNavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;

                // Find movement delta.
                Vector3 delta = steerPos - iterPos;
                float len = MathF.Sqrt(Vector3.Dot(delta, delta));
                // If the steer target is end of path or off-mesh link, do not move past the location.
                if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
                {
                    len = 1;
                }
                else
                {
                    len = STEP_SIZE / len;
                }

                Vector3 moveTgt = iterPos + delta * len;

                // Move
                navQuery.MoveAlongSurface(polys[0], iterPos, moveTgt, filter, out var result, ref visited);

                iterPos = result;

                polys = PathUtils.MergeCorridorStartMoved(polys, visited);
                polys = PathUtils.FixupShortcuts(polys, navQuery);

                var status = navQuery.GetPolyHeight(polys[0], result, out var h);
                if (status.Succeeded())
                {
                    iterPos.Y = h;
                }

                // Handle end of path and off-mesh links when close enough.
                if (endOfPath && PathUtils.InRange(iterPos, steerPos, SLOP, 1f))
                {
                    // Reached end of path.
                    iterPos = targetPos;
                    if (smoothPath.Count < MAX_SMOOTH)
                    {
                        smoothPath.Add(iterPos);
                    }

                    break;
                }
                else if (offMeshConnection && PathUtils.InRange(iterPos, steerPos, SLOP, 1f))
                {
                    // Reached off-mesh connection.
                    Vector3 startPos = Vector3.Zero;
                    Vector3 endPos = Vector3.Zero;

                    // Advance the path up to and over the off-mesh connection.
                    long prevRef = 0;
                    long polyRef = polys[0];
                    int npos = 0;
                    while (npos < polys.Count && polyRef != steerPosRef)
                    {
                        prevRef = polyRef;
                        polyRef = polys[npos];
                        npos++;
                    }

                    polys = polys.GetRange(npos, polys.Count - npos);

                    // Handle the connection.
                    var status2 = navMesh.GetOffMeshConnectionPolyEndPoints(prevRef, polyRef, ref startPos, ref endPos);
                    if (status2.Succeeded())
                    {
                        if (smoothPath.Count < MAX_SMOOTH)
                        {
                            smoothPath.Add(startPos);
                            // Hack to make the dotted path not visible during off-mesh connection.
                            if ((smoothPath.Count & 1) != 0)
                            {
                                smoothPath.Add(startPos);
                            }
                        }

                        // Move position at the other side of the off-mesh link.
                        iterPos = endPos;
                        navQuery.GetPolyHeight(polys[0], iterPos, out var eh);
                        iterPos.Y = eh;
                    }
                }

                // Store results.
                if (smoothPath.Count < MAX_SMOOTH)
                {
                    smoothPath.Add(iterPos);
                }
            }

            return DtStatus.DT_SUCCSESS;
        }

        public static DtStatus FindStraightPath(DtNavMeshQuery navQuery, long startRef, long endRef, Vector3 startPt, Vector3 endPt, IDtQueryFilter filter, bool enableRaycast,
            ref List<long> polys, ref List<StraightPathItem> straightPath, int straightPathOptions)
        {
            if (startRef is 0 || endRef is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            polys ??= new List<long>();
            straightPath ??= new List<StraightPathItem>();

            polys.Clear();
            straightPath.Clear();

            var opt = new DtFindPathOption(enableRaycast ? DtNavMeshQuery.DT_FINDPATH_ANY_ANGLE : 0, float.MaxValue);
            navQuery.FindPath(startRef, endRef, startPt, endPt, filter, ref polys, opt);

            if (0 >= polys.Count)
                return DtStatus.DT_FAILURE;

            // In case of partial path, make sure the end point is clamped to the last polygon.
            var epos = new Vector3(endPt.X, endPt.Y, endPt.Z);
            if (polys[^1] != endRef)
            {
                var result = navQuery.ClosestPointOnPoly(polys[^1], endPt, out var closest, out var _);
                if (result.Succeeded())
                {
                    epos = closest;
                }
            }

            navQuery.FindStraightPath(startPt, epos, polys, ref straightPath, MAX_POLYS, straightPathOptions);

            return DtStatus.DT_SUCCSESS;
        }

        public static DtStatus InitSlicedFindPath(DtNavMeshQuery navQuery, long startRef, long endRef, Vector3 startPos, Vector3 endPos, IDtQueryFilter filter, bool enableRaycast)
        {
            if (startRef is 0 || endRef is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            return navQuery.InitSlicedFindPath(startRef, endRef, startPos, endPos, filter,
                enableRaycast ? DtNavMeshQuery.DT_FINDPATH_ANY_ANGLE : 0,
                float.MaxValue
            );
        }

        public static DtStatus UpdateSlicedFindPath(DtNavMeshQuery navQuery, int maxIter, long endRef, Vector3 startPos, Vector3 endPos,
            ref List<long> path, ref List<StraightPathItem> straightPath)
        {
            navQuery.UpdateSlicedFindPath(maxIter, out _);
            var status = navQuery.Status;

            if (status.Succeeded() is false)
            {
                return status;
            }

            navQuery.FinalizeSlicedFindPath(path);

            straightPath?.Clear();
            if (path != null)
            {
                // In case of partial path, make sure the end point is clamped to the last polygon.
                Vector3 epos = endPos;
                if (path[^1] != endRef)
                {
                    var result = navQuery.ClosestPointOnPoly(path[^1], endPos, out var closest, out var _);
                    if (result.Succeeded())
                    {
                        epos = closest;
                    }
                }

                straightPath = new List<StraightPathItem>(MAX_POLYS);
                navQuery.FindStraightPath(startPos, epos, path, ref straightPath, MAX_POLYS, DtNavMeshQuery.DT_STRAIGHTPATH_ALL_CROSSINGS);
            }

            return DtStatus.DT_SUCCSESS;
        }


        public static DtStatus Raycast(DtNavMeshQuery navQuery, long startRef, long endRef, Vector3 startPos, Vector3 endPos, IDtQueryFilter filter,
            ref List<long> polys, ref List<StraightPathItem> straightPath, ref Vector3 hitPos, ref Vector2 hitNormal, ref bool hitResult)
        {
            if (startRef is 0 || endRef is 0)
            {
                polys?.Clear();
                straightPath?.Clear();

                return DtStatus.DT_FAILURE;
            }

            var status = navQuery.Raycast(startRef, startPos, endPos, filter, 0, 0, out var rayHit);
            if (!status.Succeeded())
            {
                return status;
            }

            // results ...
            polys = rayHit.path;

            if (rayHit.t > 1)
            {
                // No hit
                hitPos = endPos;
                hitResult = false;
            }
            else
            {
                // Hit
                hitPos = Vector3.Lerp(startPos, endPos, rayHit.t);
                hitNormal = rayHit.hitNormal;
                hitResult = true;
            }

            // Adjust height.
            if (rayHit.path.Count > 0)
            {
                var result = navQuery.GetPolyHeight(rayHit.path[^1], hitPos, out var h);
                if (result.Succeeded())
                {
                    hitPos.Y = h;
                }
            }

            straightPath ??= new List<StraightPathItem>();
            straightPath.Clear();
            straightPath.Add(new StraightPathItem(startPos, 0, 0));
            straightPath.Add(new StraightPathItem(hitPos, 0, 0));

            return status;
        }

        public static DtStatus FindDistanceToWall(DtNavMeshQuery navQuery, long startRef, Vector3 spos, float maxRadius, IDtQueryFilter filter,
            ref float hitDist, ref Vector3 hitPos, ref Vector2 hitNormal)
        {
            if (0 == startRef)
            {
                return DtStatus.DT_FAILURE;
            }

            var status = navQuery.FindDistanceToWall(startRef, spos, maxRadius, filter,
                out var tempHitDist, out var tempHitPos, out var tempHitNormal);

            if (status.Succeeded())
            {
                hitDist = tempHitDist;
                hitPos = tempHitPos;
                hitNormal = tempHitNormal;
            }

            return status;
        }


        public static DtStatus FindPolysAroundCircle(DtNavMeshQuery navQuery, long startRef, long endRef, Vector3 spos, Vector3 epos, IDtQueryFilter filter, ref List<long> resultRef, ref List<long> resultParent)
        {
            if (startRef is 0 || endRef is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            float dx = epos.X - spos.X;
            float dz = epos.Z - spos.Z;
            float dist = MathF.Sqrt(dx * dx + dz * dz);

            List<long> tempResultRefs = new();
            List<long> tempParentRefs = new();
            List<float> tempCosts = new();
            var status = navQuery.FindPolysAroundCircle(startRef, spos, dist, filter, ref tempResultRefs, ref tempParentRefs, ref tempCosts);
            if (status.Succeeded())
            {
                resultRef = tempResultRefs;
                resultParent = tempParentRefs;
            }

            return status;
        }

        public static DtStatus FindLocalNeighbourhood(DtNavMeshQuery navQuery, long startRef, Vector3 spos, float radius, IDtQueryFilter filter,
            ref List<long> resultRef, ref List<long> resultParent)
        {
            if (startRef is 0)
            {
                resultRef?.Clear();
                resultParent?.Clear();
                return DtStatus.DT_FAILURE;
            }

            resultRef ??= new List<long>();
            resultParent ??= new List<long>();

            resultRef.Clear();
            resultParent.Clear();

            var status = navQuery.FindLocalNeighbourhood(startRef, spos, radius, filter, ref resultRef, ref resultParent);
            return status;
        }


        public static DtStatus FindPolysAroundShape(DtNavMeshQuery navQuery, float agentHeight, long startRef, long endRef, Vector3 spos, Vector3 epos, IDtQueryFilter filter,
            ref List<long> resultRefs, ref List<long> resultParents, ref Vector3[] queryPoly)
        {
            if (startRef is 0 || endRef is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            float nx = (epos.Z - spos.Z) * 0.25f;
            float nz = -(epos.X - spos.X) * 0.25f;

            var tempQueryPoly = new Vector3[4];
            tempQueryPoly[0].X = spos.X + nx * 1.2f;
            tempQueryPoly[0].Y = spos.Y + agentHeight / 2;
            tempQueryPoly[0].Z = spos.Z + nz * 1.2f;

            tempQueryPoly[1].X = spos.X - nx * 1.3f;
            tempQueryPoly[1].Y = spos.Y + agentHeight / 2;
            tempQueryPoly[1].Z = spos.Z - nz * 1.3f;

            tempQueryPoly[2].X = epos.X - nx * 0.8f;
            tempQueryPoly[2].Y = epos.Y + agentHeight / 2;
            tempQueryPoly[2].Z = epos.Z - nz * 0.8f;

            tempQueryPoly[3].X = epos.X + nx;
            tempQueryPoly[3].Y = epos.Y + agentHeight / 2;
            tempQueryPoly[3].Z = epos.Z + nz;

            var tempResultRefs = new List<long>();
            var tempResultParents = new List<long>();
            var tempCosts = new List<float>();
            var status = navQuery.FindPolysAroundShape(startRef, tempQueryPoly, filter, ref tempResultRefs, ref tempResultParents, ref tempCosts);
            if (status.Succeeded())
            {
                resultRefs = tempResultRefs;
                resultParents = tempResultParents;
                queryPoly = tempQueryPoly;
            }

            return status;
        }

        public static DtStatus FindRandomPointAroundCircle(DtNavMeshQuery navQuery, long startRef, long endRef, Vector3 spos, Vector3 epos, IDtQueryFilter filter, bool constrainByCircle, int count,
            ref List<Vector3> points)
        {
            if (startRef is 0 || endRef is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            float dx = epos.X - spos.X;
            float dz = epos.Z - spos.Z;
            float dist = MathF.Sqrt(dx * dx + dz * dz);

            IDtPolygonByCircleConstraint constraint = constrainByCircle
                ? DtStrictDtPolygonByCircleConstraint.Shared
                : DtNoOpDtPolygonByCircleConstraint.Shared;

            var frand = new FRand();
            int prevCnt = points.Count;

            points = new List<Vector3>();
            while (0 < count && points.Count < prevCnt + count)
            {
                var status = navQuery.FindRandomPointAroundCircle(startRef, spos, dist, filter, frand, constraint,
                    out _, out var randomPt);

                if (status.Succeeded())
                {
                    points.Add(randomPt);
                }
            }

            return DtStatus.DT_SUCCSESS;
        }
    }
}
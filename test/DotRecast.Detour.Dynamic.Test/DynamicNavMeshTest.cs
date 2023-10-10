using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Threading.Tasks;
using DotRecast.Core;
using DotRecast.Detour.Dynamic.Test.Io;
using NUnit.Framework;

namespace DotRecast.Detour.Dynamic.Test;

[Parallelizable]
public class DynamicNavMeshTest
{
    private static readonly Vector3 START_POS = new(70.87453f, 0.0010070801f, 86.69021f);
    private static readonly Vector3 END_POS = new(-50.22061f, 0.0010070801f, -70.761444f);
    private static readonly Vector3 EXTENT = new(0.1f, 0.1f, 0.1f);
    private static readonly Vector3 SPHERE_POS = new(45.381645f, 0.0010070801f, 52.68981f);


    [Test]
    public void E2eTest()
    {
        byte[] bytes = RcResources.Load("test_tiles.voxels");
        using var ms = new MemoryStream(bytes);
        using var br = new BinaryReader(ms);

        // load voxels from file
        DtVoxelFileReader reader = new(DtVoxelTileLZ4ForTestCompressor.Shared);
        DtVoxelFile f = reader.Read(br);
        // create dynamic navmesh
        DtDynamicNavMesh mesh = new(f);
        // build navmesh asynchronously using multiple threads
        Task<bool> future = mesh.Build(Task.Factory);
        // wait for build to complete
        bool _ = future.Result;

        // create new query
        DtNavMeshQuery query = new(mesh.NavMesh());
        IDtQueryFilter filter = new DtQueryDefaultFilter();

        // find path
        query.FindNearestPoly(START_POS, EXTENT, filter, out var startRef, out var startPt, out var _);
        query.FindNearestPoly(END_POS, EXTENT, filter, out var endRef, out var endPt, out var _);

        var path = new List<long>();
        query.FindPath(startRef, endRef, startPt, endPt, filter, ref path, DtFindPathOption.AnyAngle);
        // check path length without any obstacles
        Assert.That(path.Count, Is.EqualTo(16));

        // place obstacle
        IDtCollider colldier = new DtSphereCollider(SPHERE_POS, 20, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GROUND, 0.1f);
        long colliderId = mesh.AddCollider(colldier);

        // update navmesh asynchronously
        future = mesh.Update(Task.Factory);
        // wait for update to complete
        _ = future.Result;
        // create new query
        query = new DtNavMeshQuery(mesh.NavMesh());

        // find path again
        query.FindNearestPoly(START_POS, EXTENT, filter, out startRef, out startPt, out var _);
        query.FindNearestPoly(END_POS, EXTENT, filter, out endRef, out endPt, out var _);
        query.FindPath(startRef, endRef, startPt, endPt, filter, ref path, DtFindPathOption.AnyAngle);

        // check path length with obstacles
        Assert.That(path.Count, Is.EqualTo(19));
        // remove obstacle
        mesh.RemoveCollider(colliderId);
        // update navmesh asynchronously
        future = mesh.Update(Task.Factory);
        // wait for update to complete
        _ = future.Result;
        // create new query
        query = new DtNavMeshQuery(mesh.NavMesh());

        // find path one more time
        query.FindNearestPoly(START_POS, EXTENT, filter, out startRef, out startPt, out var _);
        query.FindNearestPoly(END_POS, EXTENT, filter, out endRef, out endPt, out var _);
        query.FindPath(startRef, endRef, startPt, endPt, filter, ref path, DtFindPathOption.AnyAngle);

        // path length should be back to the initial value
        Assert.That(path.Count, Is.EqualTo(16));
    }
}
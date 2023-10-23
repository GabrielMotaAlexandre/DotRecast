using System;
using System.Collections.Generic;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Detour.Crowd;
using DotRecast.Recast.Toolset.Builder;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcCrowdAgentProfilingTool : IRcToolable
    {
        private readonly RcCrowdAgentProfilingToolConfig _cfg;

        private DtCrowdConfig _crowdCfg;
        private DtCrowd crowd;
        private readonly DtCrowdAgentConfig _agCfg;

        private DtNavMesh navMesh;

        private FRand rnd;
        private readonly List<DtPolyPoint> _polyPoints;
        private long crowdUpdateTime;

        public RcCrowdAgentProfilingTool()
        {
            _cfg = new RcCrowdAgentProfilingToolConfig();
            _agCfg = new DtCrowdAgentConfig();
            _polyPoints = new List<DtPolyPoint>();
        }

        public string GetName()
        {
            return "Crowd Agent Profiling";
        }

        public RcCrowdAgentProfilingToolConfig GetToolConfig()
        {
            return _cfg;
        }

        public DtCrowdAgentConfig GetCrowdConfig()
        {
            return _agCfg;
        }

        public DtCrowd GetCrowd()
        {
            return crowd;
        }

        public void Setup(float maxAgentRadius, DtNavMesh nav)
        {
            navMesh = nav;
            if (nav != null)
            {
                _crowdCfg = new DtCrowdConfig(maxAgentRadius);
            }
        }

        private DtCrowdAgentParams GetAgentParams(float agentRadius, float agentHeight, float agentMaxAcceleration, float agentMaxSpeed)
        {
            DtCrowdAgentParams ap = new()
            {
                radius = agentRadius,
                height = agentHeight,
                maxAcceleration = agentMaxAcceleration,
                maxSpeed = agentMaxSpeed
            };
            ap.collisionQueryRange = ap.radius * 12f;
            ap.pathOptimizationRange = ap.radius * 30f;
            ap.updateFlags = _agCfg.GetUpdateFlags();
            ap.obstacleAvoidanceType = _agCfg.obstacleAvoidanceType;
            ap.separationWeight = _agCfg.separationWeight;
            return ap;
        }

        private DtStatus GetMobPosition(DtNavMeshQuery navquery, IDtQueryFilter filter, out Vector3 randomPt)
        {
            return navquery.FindRandomPoint(filter, rnd, out _, out randomPt);
        }

        private DtStatus GetVillagerPosition(DtNavMeshQuery navquery, IDtQueryFilter filter, out Vector3 randomPt)
        {
            randomPt = Vector3.Zero;

            if (0 >= _polyPoints.Count)
                return DtStatus.DT_FAILURE;

            int zone = (int)(rnd.Next() * _polyPoints.Count);

            return navquery.FindRandomPointWithinCircle(_polyPoints[zone].refs, _polyPoints[zone].pt, _cfg.zoneRadius, filter, rnd,
                out _, out randomPt);
        }

        private void CreateZones()
        {
            _polyPoints.Clear();
            IDtQueryFilter filter = new DtQueryDefaultFilter();
            DtNavMeshQuery navquery = new(navMesh);
            for (int i = 0; i < _cfg.numberOfZones; i++)
            {
                float zoneSeparation = _cfg.zoneRadius * _cfg.zoneRadius * 16;
                for (int k = 0; k < 100; k++)
                {
                    var status = navquery.FindRandomPoint(filter, rnd, out var randomRef, out var randomPt);
                    if (status.Succeeded())
                    {
                        bool valid = true;
                        foreach (var zone in _polyPoints)
                        {
                            if (Vector3.DistanceSquared(zone.pt, randomPt) < zoneSeparation)
                            {
                                valid = false;
                                break;
                            }
                        }

                        if (valid)
                        {
                            _polyPoints.Add(new DtPolyPoint(randomRef, randomPt));
                            break;
                        }
                    }
                }
            }
        }

        private void CreateCrowd()
        {
            crowd = new DtCrowd(_crowdCfg, navMesh, __ => new DtQueryDefaultFilter(
                SampleAreaModifications.SAMPLE_POLYFLAGS_ALL,
                SampleAreaModifications.SAMPLE_POLYFLAGS_DISABLED,
                new float[] { 1f, 10f, 1f, 1f, 2f, 1.5f })
            );

            DtObstacleAvoidanceParams option = new(crowd.GetObstacleAvoidanceParams(0))
            {
                // Low (11)
                velBias = 0.5f,
                adaptiveDivs = 5,
                adaptiveRings = 2,
                adaptiveDepth = 1
            };
            crowd.SetObstacleAvoidanceParams(0, option);
            // Medium (22)
            option.velBias = 0.5f;
            option.adaptiveDivs = 5;
            option.adaptiveRings = 2;
            option.adaptiveDepth = 2;
            crowd.SetObstacleAvoidanceParams(1, option);
            // Good (45)
            option.velBias = 0.5f;
            option.adaptiveDivs = 7;
            option.adaptiveRings = 2;
            option.adaptiveDepth = 3;
            crowd.SetObstacleAvoidanceParams(2, option);
            // High (66)
            option.velBias = 0.5f;
            option.adaptiveDivs = 7;
            option.adaptiveRings = 3;
            option.adaptiveDepth = 3;
            crowd.SetObstacleAvoidanceParams(3, option);
        }

        public void StartProfiling(float agentRadius, float agentHeight, float agentMaxAcceleration, float agentMaxSpeed)
        {
            if (null == navMesh)
                return;

            rnd = new FRand(_cfg.randomSeed);
            CreateCrowd();
            CreateZones();
            DtNavMeshQuery navquery = new(navMesh);
            IDtQueryFilter filter = new DtQueryDefaultFilter();
            for (int i = 0; i < _cfg.agents; i++)
            {
                float tr = rnd.Next();
                RcCrowdAgentType type = RcCrowdAgentType.MOB;
                float mobsPcnt = _cfg.percentMobs / 100f;
                if (tr > mobsPcnt)
                {
                    tr = rnd.Next();
                    float travellerPcnt = _cfg.percentTravellers / 100f;
                    if (tr > travellerPcnt)
                    {
                        type = RcCrowdAgentType.VILLAGER;
                    }
                    else
                    {
                        type = RcCrowdAgentType.TRAVELLER;
                    }
                }

                var status = DtStatus.DT_FAILURE;
                var randomPt = Vector3.Zero;
                switch (type)
                {
                    case RcCrowdAgentType.MOB:
                        status = GetMobPosition(navquery, filter, out randomPt);
                        break;
                    case RcCrowdAgentType.VILLAGER:
                        status = GetVillagerPosition(navquery, filter, out randomPt);
                        break;
                    case RcCrowdAgentType.TRAVELLER:
                        status = GetVillagerPosition(navquery, filter, out randomPt);
                        break;
                }

                if (status.Succeeded())
                {
                    AddAgent(randomPt, type, agentRadius, agentHeight, agentMaxAcceleration, agentMaxSpeed);
                }
            }
        }

        public void Update(float dt)
        {
            long startTime = RcFrequency.Ticks;
            if (crowd != null)
            {
                crowd.Config().pathQueueSize = _cfg.pathQueueSize;
                crowd.Config().maxFindPathIterations = _cfg.maxIterations;
                crowd.Update(dt, null);
            }

            long endTime = RcFrequency.Ticks;
            if (crowd != null)
            {
                DtNavMeshQuery navquery = new(navMesh);
                IDtQueryFilter filter = new DtQueryDefaultFilter();
                foreach (DtCrowdAgent ag in crowd.GetActiveAgents())
                {
                    if (NeedsNewTarget(ag))
                    {
                        RcCrowdAgentData crowAgentData = (RcCrowdAgentData)ag.option.userData;
                        switch (crowAgentData.type)
                        {
                            case RcCrowdAgentType.MOB:
                                MoveMob(navquery, filter, ag, crowAgentData);
                                break;
                            case RcCrowdAgentType.VILLAGER:
                                MoveVillager(navquery, filter, ag, crowAgentData);
                                break;
                            case RcCrowdAgentType.TRAVELLER:
                                MoveTraveller(navquery, filter, ag, crowAgentData);
                                break;
                        }
                    }
                }
            }

            crowdUpdateTime = (endTime - startTime) / TimeSpan.TicksPerMillisecond;
        }

        private void MoveMob(DtNavMeshQuery navquery, IDtQueryFilter filter, DtCrowdAgent ag, RcCrowdAgentData crowAgentData)
        {
            // Move somewhere
            var status = navquery.FindNearestPoly(ag.npos, crowd.GetQueryExtents(), filter, out var nearestRef, out _, out _);
            if (status.Succeeded())
            {
                status = navquery.FindRandomPointAroundCircle(nearestRef, crowAgentData.home, _cfg.zoneRadius * 2f, filter, rnd,
                    out var randomRef, out var randomPt);
                if (status.Succeeded())
                {
                    DtCrowd.RequestMoveTarget(ag, randomRef, randomPt);
                }
            }
        }

        private void MoveVillager(DtNavMeshQuery navquery, IDtQueryFilter filter, DtCrowdAgent ag, RcCrowdAgentData crowAgentData)
        {
            // Move somewhere close
            var status = navquery.FindNearestPoly(ag.npos, crowd.GetQueryExtents(), filter, out var nearestRef, out _, out _);
            if (status.Succeeded())
            {
                status = navquery.FindRandomPointAroundCircle(nearestRef, crowAgentData.home, _cfg.zoneRadius * 0.2f, filter, rnd,
                    out var randomRef, out var randomPt);
                if (status.Succeeded())
                {
                    DtCrowd.RequestMoveTarget(ag, randomRef, randomPt);
                }
            }
        }

        private void MoveTraveller(DtNavMeshQuery navquery, IDtQueryFilter filter, DtCrowdAgent ag, RcCrowdAgentData crowAgentData)
        {
            // Move to another zone
            List<DtPolyPoint> potentialTargets = new();
            foreach (var zone in _polyPoints)
            {
                if (Vector3.DistanceSquared(zone.pt, ag.npos) > _cfg.zoneRadius * _cfg.zoneRadius)
                {
                    potentialTargets.Add(zone);
                }
            }

            if (0 < potentialTargets.Count)
            {
                potentialTargets.Shuffle();
                DtCrowd.RequestMoveTarget(ag, potentialTargets[0].refs, potentialTargets[0].pt);
            }
        }

        private static bool NeedsNewTarget(DtCrowdAgent ag)
        {
            if (ag.TargetState == DtMoveRequestState.DT_CROWDAGENT_TARGET_NONE
                || ag.TargetState == DtMoveRequestState.DT_CROWDAGENT_TARGET_FAILED)
            {
                return true;
            }

            if (ag.TargetState == DtMoveRequestState.DT_CROWDAGENT_TARGET_VALID)
            {
                float dx = ag.targetPos.X - ag.npos.X;
                float dy = ag.targetPos.Y - ag.npos.Y;
                float dz = ag.targetPos.Z - ag.npos.Z;
                return dx * dx + dy * dy + dz * dz < 0.3f;
            }

            return false;
        }

        private DtCrowdAgent AddAgent(Vector3 p, RcCrowdAgentType type, float agentRadius, float agentHeight, float agentMaxAcceleration, float agentMaxSpeed)
        {
            DtCrowdAgentParams ap = GetAgentParams(agentRadius, agentHeight, agentMaxAcceleration, agentMaxSpeed);
            ap.userData = new RcCrowdAgentData(type, p);
            return crowd.AddAgent(p, ap);
        }

        public void UpdateAgentParams()
        {
            if (crowd != null)
            {
                foreach (DtCrowdAgent ag in crowd.GetActiveAgents())
                {
                    DtCrowdAgentParams option = new()
                    {
                        radius = ag.option.radius,
                        height = ag.option.height,
                        maxAcceleration = ag.option.maxAcceleration,
                        maxSpeed = ag.option.maxSpeed,
                        collisionQueryRange = ag.option.collisionQueryRange,
                        pathOptimizationRange = ag.option.pathOptimizationRange,
                        queryFilterType = ag.option.queryFilterType,
                        userData = ag.option.userData,
                        updateFlags = _agCfg.GetUpdateFlags(),
                        obstacleAvoidanceType = _agCfg.obstacleAvoidanceType,
                        separationWeight = _agCfg.separationWeight
                    };
                    DtCrowd.UpdateAgentParameters(ag, option);
                }
            }
        }

        public long GetCrowdUpdateTime()
        {
            return crowdUpdateTime;
        }
    }
}
using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.AI;

namespace Xacce.Susanin.Runtime
{
    [Flags]
    public enum StraightPathFlags
    {
        Start = 0x01, // The vertex is the start position.
        End = 0x02, // The vertex is the end position.
        OffMeshConnection = 0x04 // The vertex is start of an off-mesh link.
    }

    [BurstCompile]
    public static class StraightPath
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Perp2D(Vector3 u, Vector3 v)
        {
            return u.z * v.x - u.x * v.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Swap(ref Vector3 a, ref Vector3 b)
        {
            (a, b) = (b, a);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SegmentSegmentCPA(out float3 c0, out float3 c1, in float3 p0, in float3 p1, in float3 q0, in float3 q1, out bool result)
        {
            var u = p1 - p0;
            var v = q1 - q0;
            var w0 = p0 - q0;

            float a = math.dot(u, u);
            float b = math.dot(u, v);
            float c = math.dot(v, v);
            float d = math.dot(u, w0);
            float e = math.dot(v, w0);

            float den = (a * c - b * b);
            float sc, tc;

            if (den == 0)
            {
                sc = 0;
                tc = d / b;

                // todo: handle b = 0 (=> a and/or c is 0)
            }
            else
            {
                sc = (b * e - c * d) / (a * c - b * b);
                tc = (a * e - b * d) / (a * c - b * b);
            }

            c0 = math.lerp(p0, p1, sc);
            c1 = math.lerp(q0, q1, tc);

            result = den != 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int RetracePortals(ref NavMeshQuery query, in int startIndex, in int endIndex, NativeSlice<PolygonId> path, int n, in float3 termPos,
            ref NativeArray<NavMeshLocation> straightPath, ref NativeArray<StraightPathFlags> straightPathFlags, int maxStraightPath)
        {
            for (var k = startIndex; k < endIndex - 1; ++k)
            {
                var type1 = query.GetPolygonType(path[k]);
                var type2 = query.GetPolygonType(path[k + 1]);
                if (type1 != type2)
                {
                    Vector3 l, r;
                    query.GetPortalPoints(path[k], path[k + 1], out l, out r);
                    float3 cpa1, cpa2;
                    SegmentSegmentCPA(out cpa1, out cpa2, l, r, straightPath[n - 1].position, termPos, out _);
                    straightPath[n] = query.CreateLocation(cpa1, path[k + 1]);

                    straightPathFlags[n] = (type2 == NavMeshPolyTypes.OffMeshConnection) ? StraightPathFlags.OffMeshConnection : 0;
                    if (++n == maxStraightPath)
                    {
                        return maxStraightPath;
                    }
                }
            }
            straightPath[n] = query.CreateLocation(termPos, path[endIndex]);
            straightPathFlags[n] = query.GetPolygonType(path[endIndex]) == NavMeshPolyTypes.OffMeshConnection ? StraightPathFlags.OffMeshConnection : 0;
            return ++n;
        }

        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindStraightPath(ref NavMeshQuery query, in float3 startPos, in float3 endPos, in NativeSlice<PolygonId> path, in int pathSize,
            ref NativeArray<NavMeshLocation> straightPath, ref NativeArray<StraightPathFlags> straightPathFlags, ref NativeArray<float> vertexSide,
            in int maxStraightPath, ref int straightPathCount, out bool result)
        {
            if (!query.IsValid(path[0]))
            {
                straightPath[0] = new NavMeshLocation(); // empty terminator
                result = false;
                return; // | PathQueryStatus.InvalidParam;
            }

            straightPath[0] = query.CreateLocation(startPos, path[0]);

            straightPathFlags[0] = StraightPathFlags.Start;

            var apexIndex = 0;
            var n = 1;

            if (pathSize > 1)
            {
                var startPolyWorldToLocal = query.PolygonWorldToLocalMatrix(path[0]);

                var apex = (Vector3)startPolyWorldToLocal.MultiplyPoint(startPos);
                var left = new Vector3(0, 0, 0); // Vector3.zero accesses a static readonly which does not work in burst yet
                var right = new Vector3(0, 0, 0);
                var leftIndex = -1;
                var rightIndex = -1;

                for (var i = 1; i <= pathSize; ++i)
                {
                    var polyWorldToLocal = query.PolygonWorldToLocalMatrix(path[apexIndex]);

                    Vector3 vl, vr;
                    if (i == pathSize)
                    {
                        vl = vr = polyWorldToLocal.MultiplyPoint(endPos);
                    }
                    else
                    {
                        var success = query.GetPortalPoints(path[i - 1], path[i], out vl, out vr);
                        if (!success)
                        {
                            result = false;
                            return; // | PathQueryStatus.InvalidParam;
                        }

                        vl = polyWorldToLocal.MultiplyPoint(vl);
                        vr = polyWorldToLocal.MultiplyPoint(vr);
                    }

                    vl = vl - apex;
                    vr = vr - apex;

                    // Ensure left/right ordering
                    if (Perp2D(vl, vr) < 0)
                        Swap(ref vl, ref vr);

                    // Terminate funnel by turning
                    if (Perp2D(left, vr) < 0)
                    {
                        var polyLocalTransform = query.PolygonLocalToWorldMatrix(path[apexIndex]);
                        var termPos = polyLocalTransform.MultiplyPoint(apex + left);

                        n = RetracePortals(ref query, apexIndex, leftIndex, path, n, termPos, ref straightPath, ref straightPathFlags, maxStraightPath);
                        if (vertexSide.Length > 0)
                        {
                            vertexSide[n - 1] = -1;
                        }

                        //Debug.Log("LEFT");

                        if (n == maxStraightPath)
                        {
                            straightPathCount = n;
                            result = true;
                            return; // | PathQueryStatus.BufferTooSmall;
                        }

                        apex = polyWorldToLocal.MultiplyPoint(termPos);
                        left = float3.zero;
                        right = float3.zero;
                        i = apexIndex = leftIndex;
                        continue;
                    }
                    if (Perp2D(right, vl) > 0)
                    {
                        var polyLocalTransform = query.PolygonLocalToWorldMatrix(path[apexIndex]);
                        var termPos = polyLocalTransform.MultiplyPoint(apex + right);

                        n = RetracePortals(ref query, apexIndex, rightIndex, path, n, termPos, ref straightPath, ref straightPathFlags, maxStraightPath);
                        if (vertexSide.Length > 0)
                        {
                            vertexSide[n - 1] = 1;
                        }

                        //Debug.Log("RIGHT");

                        if (n == maxStraightPath)
                        {
                            straightPathCount = n;
                            result = true;
                            return; // | PathQueryStatus.BufferTooSmall;
                        }

                        apex = polyWorldToLocal.MultiplyPoint(termPos);
                        left = float3.zero;
                        right = float3.zero;
                        i = apexIndex = rightIndex;
                        continue;
                    }

                    // Narrow funnel
                    if (Perp2D(left, vl) >= 0)
                    {
                        left = vl;
                        leftIndex = i;
                    }
                    if (Perp2D(right, vr) <= 0)
                    {
                        right = vr;
                        rightIndex = i;
                    }
                }
            }

            // Remove the the next to last if duplicate point - e.g. start and end positions are the same
            // (in which case we have get a single point)
            if (n > 0 && (straightPath[n - 1].position == (Vector3)endPos))
                n--;

            n = RetracePortals(ref query, apexIndex, pathSize - 1, path, n, endPos, ref straightPath, ref straightPathFlags, maxStraightPath);
            if (vertexSide.Length > 0)
            {
                vertexSide[n - 1] = 0;
            }

            if (n == maxStraightPath)
            {
                straightPathCount = n;
                result = true;
                return;
            }

            // Fix flag for final path point
            straightPathFlags[n - 1] = StraightPathFlags.End;

            straightPathCount = n;
            result = true;
        }
    }
}
/*
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Buffers;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using DotRecast.Core;

namespace System.Numerics
{
    public static class Vector3Extensions
    {
        public static Vector3 Of(float[] f, int idx)
        {
            return new Vector3(f[idx + 0], f[idx + 1], f[idx + 2]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Set(this ref Vector3 vector, float[] @in, int i)
        {
            vector.X = @in[i];
            vector.Y = @in[i + 1];
            vector.Z = @in[i + 2];
        }

        /// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
        /// @param[in] u A vector [(x, y, z)]
        /// @param[in] v A vector [(x, y, z)]
        /// @return The dot product on the xz-plane.
        ///
        /// The vectors are projected onto the xz-plane, so the y-values are
        /// ignored.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot2D(this ref Vector3 vector, Vector3 v)
        {
            return vector.X * v.X + vector.Z * v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot2D(this ref Vector3 vector, float[] v, int vi)
        {
            return vector.X * v[vi] + vector.Z * v[vi + 2];
        }

        /// Normalizes the vector.
        /// @param[in,out] v The vector to normalize. [(x, y, z)]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Normalize(this ref Vector3 vector)
        {
            vector = Vector3.Normalize(vector);
        }

        public const float EPSILON = 1e-6f;

        /// Normalizes the vector if the length is greater than zero.
        /// If the magnitude is zero, the vector is unchanged.
        /// @param[in,out]	v	The vector to normalize. [(x, y, z)]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SafeNormalize(this ref Vector3 vector)
        {
            float sqMag = vector.LengthSquared();
            if (sqMag > EPSILON)
            {
                float inverseMag = 1.0f / (float)Math.Sqrt(sqMag);
                vector.X *= inverseMag;
                vector.Y *= inverseMag;
                vector.Z *= inverseMag;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(this ref Vector3 vector, float[] @in, int i)
        {
            vector.Min(new Vector3(@in[i], @in[i + 1], @in[i + 2]));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(this ref Vector3 vector, Vector3 b)
        {
            vector = Vector3.Min(vector, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(this ref Vector3 vector, Vector3 b)
        {
            vector = Vector3.Max(vector, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(this ref Vector3 vector, float[] @in, int i)
        {
            vector.Max(new Vector3(@in[i], @in[i + 1], @in[i + 2]));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(float[] v1, float[] v2)
        {
            return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(float[] v1, Vector3 v2)
        {
            return v1[0] * v2.X + v1[1] * v2.Y + v1[2] * v2.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float PerpXZ(Vector3 a, Vector3 b)
        {
            return (a.X * b.Z) - (a.Z * b.X);
        }

        /// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
        /// @param[out] dest The result vector. [(x, y, z)]
        /// @param[in] v1 The base vector. [(x, y, z)]
        /// @param[in] v2 The vector to scale and add to @p v1. [(x, y, z)]
        /// @param[in] s The amount to scale @p v2 by before adding to @p v1.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Mad(Vector3 v1, Vector3 v2, float s)
        {
            return v1 + v2 * s;
        }

        /// Performs a linear interpolation between two vectors. (@p v1 toward @p
        /// v2)
        /// @param[out] dest The result vector. [(x, y, x)]
        /// @param[in] v1 The starting vector.
        /// @param[in] v2 The destination vector.
        /// @param[in] t The interpolation factor. [Limits: 0 <= value <= 1.0]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Lerp(float[] verts, int v1, int v2, float t)
        {
            return new Vector3(
                verts[v1 + 0] + (verts[v2 + 0] - verts[v1 + 0]) * t,
                verts[v1 + 1] + (verts[v2 + 1] - verts[v1 + 1]) * t,
                verts[v1 + 2] + (verts[v2 + 2] - verts[v1 + 2]) * t
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dist2D(Vector3 v1, Vector3 v2)
        {
            float dx = v2.X - v1.X;
            float dz = v2.Z - v1.Z;
            return (float)Math.Sqrt(dx * dx + dz * dz);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dist2DSqr(float[] v1, float[] v2)
        {
            float dx = v2[0] - v1[0];
            float dz = v2[2] - v1[2];
            return dx * dx + dz * dz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dist2DSqr(Vector3 v1, Vector3 v2)
        {
            float dx = v2.X - v1.X;
            float dz = v2.Z - v1.Z;
            return dx * dx + dz * dz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dist2DSqr(Vector3 p, float[] verts, int i)
        {
            float dx = verts[i] - p.X;
            float dz = verts[i + 2] - p.Z;
            return dx * dx + dz * dz;
        }

        /// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
        /// @param[in] u The LHV vector [(x, y, z)]
        /// @param[in] v The RHV vector [(x, y, z)]
        /// @return The dot product on the xz-plane.
        ///
        /// The vectors are projected onto the xz-plane, so the y-values are
        /// ignored.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Perp2D(Vector3 u, Vector3 v)
        {
            return u.Z * v.X - u.X * v.Z;
        }

        /// Checks that the specified vector's components are all finite.
        /// @param[in] v A point. [(x, y, z)]
        /// @return True if all of the point's components are finite, i.e. not NaN
        /// or any of the infinities.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsFinite(Vector3 v)
        {
            return float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
        }

        /// Checks that the specified vector's 2D components are finite.
        /// @param[in] v A point. [(x, y, z)]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsFinite2D(Vector3 v)
        {
            return float.IsFinite(v.X) && float.IsFinite(v.Z);
        }

        public static void Copy(ref Vector3 @out, float[] @in, int i)
        {
            Copy(ref @out, 0, @in, i);
        }

        public static void Copy(float[] @out, int n, float[] @in, int m)
        {
            @out[n] = @in[m];
            @out[n + 1] = @in[m + 1];
            @out[n + 2] = @in[m + 2];
        }

        public static void Copy(float[] @out, int n, Vector3 @in, int m)
        {
            @out[n] = @in[m];
            @out[n + 1] = @in[m + 1];
            @out[n + 2] = @in[m + 2];
        }

        public static void Copy(ref Vector3 @out, int n, float[] @in, int m)
        {
            @out[n] = @in[m];
            @out[n + 1] = @in[m + 1];
            @out[n + 2] = @in[m + 2];
        }

        public static void Add(ref Vector3 e0, Vector3 a, float[] verts, int i)
        {
            e0.X = a.X + verts[i];
            e0.Y = a.Y + verts[i + 1];
            e0.Z = a.Z + verts[i + 2];
        }


        public static void Sub(ref Vector3 e0, float[] verts, int i, int j)
        {
            e0.X = verts[i] - verts[j];
            e0.Y = verts[i + 1] - verts[j + 1];
            e0.Z = verts[i + 2] - verts[j + 2];
        }


        public static void Sub(ref Vector3 e0, Vector3 i, float[] verts, int j)
        {
            e0.X = i.X - verts[j];
            e0.Y = i.Y - verts[j + 1];
            e0.Z = i.Z - verts[j + 2];
        }


        public static void Cross(float[] dest, float[] v1, float[] v2)
        {
            dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
            dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
            dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
        }

        public static void Cross(ref Vector3 dest, Vector3 v1, Vector3 v2)
        {
            dest.X = v1.Y * v2.Z - v1.Z * v2.Y;
            dest.Y = v1.Z * v2.X - v1.X * v2.Z;
            dest.Z = v1.X * v2.Y - v1.Y * v2.X;
        }

        public static TTo ReadAsUnsafe<T, TTo>(this T[] array, int index = 0) where T : struct where TTo : struct
        {
            return Unsafe.Add(ref Unsafe.As<T[], Pinnable<TTo>>(ref array).Data, index);
        }

        [StructLayout(LayoutKind.Sequential)]
        private sealed class Pinnable<T>
        {
            public T Data;
        }
    }
}
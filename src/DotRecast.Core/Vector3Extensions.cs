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

using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace System.Numerics
{
    public static class Vector3Extensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 AsVector2XZ(this Vector3 vector)
        {
            return new Vector2(vector.X, vector.Z);
        }

        public static Vector3 Of(float[] f, int idx)
        {
            return f.GetUnsafe(idx).UnsafeAs<float, Vector3>();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Set(this ref Vector3 vector, float[] @in, int i)
        {
            vector = @in.GetUnsafe(i).UnsafeAs<float, Vector3>();
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
                vector /= MathF.Sqrt(sqMag);
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
        public static float PerpXZ(Vector3 a, Vector3 b)
        {
            return (a.X * b.Z) - (a.Z * b.X);
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
            ref readonly Vector3 vector1 = ref verts.GetReference().UnsafeAdd(v1).UnsafeAs<float, Vector3>();
            ref readonly Vector3 vector2 = ref verts.GetReference().UnsafeAdd(v2).UnsafeAs<float, Vector3>();

            return vector1 + (vector2 - vector1) * t;
            //return Vector3.Lerp(vector1, vector2, t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dist2D(in Vector3 v1, in Vector3 v2)
        {
            float dx = v2.X - v1.X;
            float dz = v2.Z - v1.Z;
            return (float)MathF.Sqrt(dx * dx + dz * dz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dist2DSqr(in Vector3 v1, in Vector3 v2)
        {
            float dx = v2.X - v1.X;
            float dz = v2.Z - v1.Z;
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
        public static float Perp2D(in Vector3 u, in Vector3 v)
        {
            return u.Z * v.X - u.X * v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Perp2D(in Vector2 u, in Vector2 v)
        {
            return u.Y * v.X - u.X * v.Y;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Copy(float[] @out, int n, float[] @in, int m)
        {
            @out.GetUnsafe(n).UnsafeAs<float, Vector3>() = @in.GetUnsafe(m).UnsafeAs<float, Vector3>();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Copy(Span<float> @out, int n, Span<float> @in, int m)
        {
            @out.GetUnsafe(n).UnsafeAs<float, Vector3>() = @in.GetUnsafe(m).UnsafeAs<float, Vector3>();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Sub(ref Vector3 e0, float[] verts, int i, int j)
        {
            e0 = verts.GetUnsafe(i).UnsafeAs<float, Vector3>() - verts.GetUnsafe(j).UnsafeAs<float, Vector3>();
        }

        public static void Cross(ref Vector3 dest, Vector3 v1, Vector3 v2)
        {
            dest = Vector3.Cross(v1, v2);
        }
    }

    public static class UnsafeExtensions
    {
        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T GetUnsafe<T>(this T[] array, int index)
        {
            return ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(array), index);
        }

        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T GetUnsafe<T>(this Span<T> array, int index)
        {
            return ref Unsafe.Add(ref MemoryMarshal.GetReference(array), index);
        }

        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T GetReference<T>(this T[] array)
        {
            return ref MemoryMarshal.GetArrayDataReference(array);
        }

        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TTo UnsafeAs<T, TTo>(this T[] array) where T : struct where TTo : struct
        {
            return ref array.UnsafeAs<T, TTo>(0);
        }

        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TTo UnsafeAs<T, TTo>(this T[] array, int index) where T : struct where TTo : struct
        {
            return ref Unsafe.As<T, TTo>(ref array.GetReference()).UnsafeAdd(index);
        }

        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TTo UnsafeAs<T, TTo>(this Span<T> array, int index) where T : struct where TTo : struct
        {
            return ref Unsafe.As<T, TTo>(ref MemoryMarshal.GetReference(array)).UnsafeAdd(index);
        }

        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TTo UnsafeAs<T, TTo>(ref this T value, int index = 0) where T : struct where TTo : struct
        {
            return ref Unsafe.Add(ref Unsafe.As<T, TTo>(ref value), index);
        }

        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T UnsafeAdd<T>(ref this T value, int offset) where T : struct
        {
            return ref Unsafe.Add(ref value, offset);
        }
    }

#pragma warning disable CS8981 // The type name only contains lower-cased ascii characters. Such names may become reserved for the language.
#pragma warning disable IDE1006 // Naming Styles
    public static class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static T select<T>(this T value1, T value2, bool select)
        {
            return select ? value2 : value1;
        }
    }
#pragma warning restore IDE1006 // Naming Styles
#pragma warning restore CS8981 // The type name only contains lower-cased ascii characters. Such names may become reserved for the language.
}

namespace UnityEngine
{
    public readonly struct Vector2Int
    {
        public readonly int x;
        public readonly int y;

        public Vector2Int(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
    }
}
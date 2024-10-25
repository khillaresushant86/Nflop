/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "vector.h"

bool vector2Equal(const vector2_t *a, const vector2_t *b)
{
    return (a->x == b->x) && (a->y == b->y);
}

vector2_t *vector2Zero(vector2_t *v)
{
    v->x = 0.0f;
    v->y = 0.0f;

    return v;
}

vector2_t *vector2Add(vector2_t *result, const vector2_t *a, const vector2_t *b)
{
    result->x = a->x + b->x;
    result->y = a->y + b->y;

    return result;
}

vector2_t *vector2Sub(vector2_t *result, const vector2_t *a, const vector2_t *b)
{
    result->x = a->x - b->x;
    result->y = a->y - b->y;

    return result;
}

vector2_t *vector2Scale(vector2_t *result, const vector2_t *v, const float k)
{
    result->x = v->x * k;
    result->y = v->y * k;

    return result;
}

float vector2Dot(const vector2_t *a, const vector2_t *b)
{
    return a->x * b->x + a->y * b->y;
}

float vector2Cross(const vector2_t *a, const vector2_t *b)
{
    return a->x * b->y - a->y * b->x;
}

float vector2NormSq(const vector2_t *v)
{
    return vector2Dot(v, v);
}

float vector2Norm(const vector2_t *v)
{
    return sqrtf(vector2NormSq(v));
}

vector2_t *vector2Normalize(vector2_t *result, const vector2_t *v)
{
    const float normSq = vector2NormSq(v);

    if (normSq > 0.0f) {
        return vector2Scale(result, v, 1.0f / sqrtf(normSq));
    } else {
        return vector2Zero(result);
    }
}

bool vector3Equal(const vector3_t *a, const vector3_t *b)
{
    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z);
}

vector3_t *vector3Zero(vector3_t *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;

    return v;
}

vector3_t *vector3Add(vector3_t *result, const vector3_t *a, const vector3_t *b)
{
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;

    return result;
}

vector3_t *vector3Sub(vector3_t *result, const vector3_t *a, const vector3_t *b)
{
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;

    return result;
}

vector3_t *vector3Scale(vector3_t *result, const vector3_t *v, const float k)
{
    result->x = v->x * k;
    result->y = v->y * k;
    result->z = v->z * k;

    return result;
}

float vector3Dot(const vector3_t *a, const vector3_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

vector3_t *vector3Cross(vector3_t *result, const vector3_t *a, const vector3_t *b)
{
    vector3_t tmp;

    tmp.x = a->y * b->z - a->z * b->y;
    tmp.y = a->z * b->x - a->x * b->z;
    tmp.z = a->x * b->y - a->y * b->x;

    *result = tmp;

    return result;
}

float vector3NormSq(const vector3_t *v)
{
    return vector3Dot(v, v);
}

float vector3Norm(const vector3_t *v)
{
    return sqrtf(vector3NormSq(v));
}

vector3_t *vector3Normalize(vector3_t *result, const vector3_t *v)
{
    const float normSq = vector3NormSq(v);

    if (normSq > 0) {  // Hopefully sqrt(nonzero) is quite large
        return vector3Scale(result, v, 1.0f / sqrtf(normSq));
    } else {
        return vector3Zero(result);
    }
}

vector3_t *matrixVectorMul(vector3_t * result, const matrix33_t *mat, const vector3_t *v)
{
    const vector3_t tmp = *v;

    result->x = mat->xx * tmp.x + mat->xy * tmp.y + mat->xz * tmp.z;
    result->y = mat->yx * tmp.x + mat->yy * tmp.y + mat->yz * tmp.z;
    result->z = mat->zx * tmp.x + mat->zy * tmp.y + mat->zz * tmp.z;

    return result;
}

vector3_t *matrixTrnVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v)
{
    const vector3_t tmp = *v;

    result->x = mat->xx * tmp.x + mat->yx * tmp.y + mat->zx * tmp.z;
    result->y = mat->xy * tmp.x + mat->yy * tmp.y + mat->zy * tmp.z;
    result->z = mat->xz * tmp.x + mat->yz * tmp.y + mat->zz * tmp.z;

    return result;
}

matrix33_t *buildRotationMatrix(matrix33_t *result, const fp_angles_t *rpy)
{
    const float cosx = cos_approx(rpy->angles.roll);
    const float sinx = sin_approx(rpy->angles.roll);
    const float cosy = cos_approx(rpy->angles.pitch);
    const float siny = sin_approx(rpy->angles.pitch);
    const float cosz = cos_approx(rpy->angles.yaw);
    const float sinz = sin_approx(rpy->angles.yaw);

    const float coszcosx = cosz * cosx;
    const float sinzcosx = sinz * cosx;
    const float coszsinx = sinx * cosz;
    const float sinzsinx = sinx * sinz;

    result->xx = cosz * cosy;
    result->xy = -cosy * sinz;
    result->xz = siny;
    result->yx = sinzcosx + (coszsinx * siny);
    result->yy = coszcosx - (sinzsinx * siny);
    result->yz = -sinx * cosy;
    result->zx = (sinzsinx) - (coszcosx * siny);
    result->zy = (coszsinx) + (sinzcosx * siny);
    result->zz = cosy * cosx;

    return result;
}

vector3_t *applyRotationMatrix(vector3_t *v, const matrix33_t *rotationMatrix)
{
    return matrixTrnVectorMul(v, rotationMatrix, v);
}

matrix33_t *yawToRotationMatrixZ(matrix33_t *result, const float yaw)
{
    const float sinYaw = sin_approx(yaw);
    const float cosYaw = cos_approx(yaw);

    result->xx = cosYaw;
    result->xy = -sinYaw;
    result->xz = 0.0f;
    result->yx = sinYaw;
    result->yy = cosYaw;
    result->yz = 0.0f;
    result->zx = 0.0f;
    result->zy = 0.0f;
    result->zz = 1.0f;

    return result;
}
